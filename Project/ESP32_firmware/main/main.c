/*
	Take a picture and Publish it via Web Socket.

	This code is in the Public Domain (or CC0 licensed, at your option.)

	Unless required by applicable law or agreed to in writing, this
	software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
	CONDITIONS OF ANY KIND, either express or implied.

	I based from here:
	https://github.com/espressif/esp-idf/tree/master/examples/protocols/http_server/ws_echo_server
*/

// Includes
#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "esp_system.h"
#include "esp_event.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "nvs_flash.h"
#include "esp_vfs.h"
#include "esp_spiffs.h"

#include "esp_camera.h"
#include "camera_pin.h"

#include "esp_wifi.h"
#include "esp_websocket_client.h"

#include "nmea_parser.h"
#include "mdns.h"
// #include <cJSON.h>
// #include "protocol_examples_common.h"


// Defines ********************************************************************
#define DEVICE_ID 1
#define WIFI_SSID "Nasmes_INF"
#define WIFI_PSWD "167294381"

#define MDNS_HOSTNAME "dashcam-config"

#define WEBSOCKET_URI "ws://192.168.62.244"
#define WEBSOCKET_PORT 8765

#define VIDEO_FPS 1 //Hz
#define JPEG_QUALITY 24

#define TIMELINE_UPDATE_DELAY		10 /*updates from GPS module*/

/* Event bits for events in the program:
 * 0 - Connected to WIFI
 * 1 - Websocket connection established
 * 2 - Device is registered to an account and ready to transfer data
 * */
#define WIFI_INITIALIZED_BIT		BIT0
#define WIFI_RETRY_BIT					BIT1
#define WIFI_CONNECTED_BIT 			BIT2

#define WS_INITIALIZED_BIT			BIT3
#define WS_CONNECTED_BIT 				BIT4
#define DEVICE_LOGIN_RETRY_BIT 	BIT5
#define DEVICE_LOGGED_IN_BIT	 	BIT6
#define VIDEO_RECORD_BIT				BIT7
#define TIMELINE_UPLOAD_BIT			BIT8

#define WIFI_REQ_CONNECT_BIT		BIT9
#define WIFI_REQ_DISCONNECT_BIT BIT10
#define WS_REQ_CONNECT_BIT			BIT11
#define WS_REQ_DISCONNECT_BIT		BIT12
#define LOGIN_REQ_BIT						BIT13
#define LOGOUT_REQ_BIT					BIT14


#define TIME_ZONE (+7)   //Hanoi Time
#define YEAR_BASE (2000) //date in GPS starts from 2000


// Global Variables ***********************************************************
static const char *TAG = "DASHCAM_SYSTEM";

char *wifi_ssid;
char *wifi_pswd;

static EventGroupHandle_t app_event_group;

esp_websocket_client_handle_t client = NULL;
SemaphoreHandle_t ws_sema;

// Position data {lat, long}
float pos[2] = {0.0};
SemaphoreHandle_t pos_sema;
uint8_t gps_update_count = TIMELINE_UPDATE_DELAY;


// Functions prototypes *******************************************************
// Other functions 
static void log_error_if_nonzero(const char *message, int error_code);
static void read_config_from_nvs();

// Camera functions
static void camera_init();

// Wifi functions
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static void wifi_init(uint8_t mode);
static void wifi_connect(char *ssid, char *pswd);
static void wifi_disconnect();

// mDNS functions
void initialize_mdns(void);

// SPIFFS functions
esp_err_t mountSPIFFS(char * partition_label, char * base_path);
static void printSPIFFS(char * path);

// Websocket server
esp_err_t start_webserver(void);

// Websocket client
static void websocket_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
static void websocket_connect(char *uri, int port);
static void websocket_disconnect();

// GPS
/**
 * @brief GPS Event Handler
 *
 * @param event_handler_arg handler specific arguments
 * @param event_base event base, here is fixed to ESP_NMEA_EVENT
 * @param event_id event id
 * @param event_data event specific arguments
 */
static void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void gps_init();

// Server Authorizing
static uint8_t device_login();
static void device_logout();

// App Tasks
static void task_app_control();
static void task_video_record();
static void task_timeline_record();

// Main ***********************************************************************
void app_main(void)
{
	app_event_group = xEventGroupCreate();
	ws_sema = xSemaphoreCreateMutex();
	pos_sema = xSemaphoreCreateMutex();
	xEventGroupSetBits(app_event_group,
										 WIFI_REQ_CONNECT_BIT | WIFI_RETRY_BIT
										 | WS_REQ_CONNECT_BIT
										 | LOGIN_REQ_BIT | DEVICE_LOGIN_RETRY_BIT
										 | 0);

	// Initialize NVS
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	// Mount SPIFFS
	ret = mountSPIFFS(/*partition_label =*/ "storage", /*base_path =*/ "/spiffs");
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "mountSPIFFS fail");
		while(1) { vTaskDelay(1); }
	}
	printSPIFFS("/spiffs/");

	// Initialize peripherals
	camera_init();
	gps_init();

	// Initilize WiFi
	read_config_from_nvs();
	wifi_init(WIFI_MODE_STA);

	// Initialize mdns
	initialize_mdns();

	/* Start the server */
	start_webserver();

	// Start tasks
	xTaskCreate(task_video_record, "video_rec", 4096, NULL, 2, NULL);
	xTaskCreate(task_timeline_record, "timeline_rec", 1000, NULL, 3, NULL);
	xTaskCreate(task_app_control, "control", 3072, NULL, 2, NULL);

}


// Function declarations ******************************************************
// Other functions 
static void log_error_if_nonzero(const char *message, int error_code)
{
	if (error_code != 0) {
		ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
	}
}

static void read_config_from_nvs()
{
	wifi_ssid = malloc((strlen(WIFI_SSID) + 1) * sizeof(char));
	wifi_pswd = malloc((strlen(WIFI_PSWD) + 1) * sizeof(char));
	strcpy(wifi_ssid, WIFI_SSID);
	strcpy(wifi_pswd, WIFI_PSWD);
}

// Camera functions
static void camera_init(){
	//static camera_config_t camera_config = {
	camera_config_t camera_config = {
		.pin_pwdn = CAM_PIN_PWDN,
		.pin_reset = CAM_PIN_RESET,
		.pin_xclk = CAM_PIN_XCLK,
		.pin_sscb_sda = CAM_PIN_SIOD,
		.pin_sscb_scl = CAM_PIN_SIOC,

		.pin_d7 = CAM_PIN_D7,
		.pin_d6 = CAM_PIN_D6,
		.pin_d5 = CAM_PIN_D5,
		.pin_d4 = CAM_PIN_D4,
		.pin_d3 = CAM_PIN_D3,
		.pin_d2 = CAM_PIN_D2,
		.pin_d1 = CAM_PIN_D1,
		.pin_d0 = CAM_PIN_D0,
		.pin_vsync = CAM_PIN_VSYNC,
		.pin_href = CAM_PIN_HREF,
		.pin_pclk = CAM_PIN_PCLK,

		//XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
		.xclk_freq_hz = 20000000,
		.ledc_timer = LEDC_TIMER_0,
		.ledc_channel = LEDC_CHANNEL_0,

		.pixel_format = PIXFORMAT_JPEG, //YUV422,GRAYSCALE,RGB565,JPEG
		.frame_size = FRAMESIZE_VGA, //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

		.jpeg_quality = JPEG_QUALITY, //0-63 lower number means higher quality
		.fb_count = 1, //if more than one, i2s runs in continuous mode. Use only with JPEG
		.grab_mode = CAMERA_GRAB_WHEN_EMPTY,
		.fb_location = CAMERA_FB_IN_PSRAM
	};

	esp_err_t err = esp_camera_init(&camera_config);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Camera Init Failed");
		while(1) { vTaskDelay(1); }
	}
}

// Wifi functions
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
	if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
		esp_wifi_connect();
	}
	else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
		xEventGroupClearBits(app_event_group, WIFI_CONNECTED_BIT);
		if (xEventGroupGetBits(app_event_group) & DEVICE_LOGGED_IN_BIT){
			xEventGroupClearBits(app_event_group, DEVICE_LOGGED_IN_BIT);
			// Retry logging in later
			if (xEventGroupGetBits(app_event_group) & DEVICE_LOGIN_RETRY_BIT)
				xEventGroupSetBits(app_event_group, LOGIN_REQ_BIT);
		}

		if (xEventGroupGetBits(app_event_group) & WIFI_RETRY_BIT){
			ESP_LOGI(TAG, "retrying to connect to the AP");
			esp_wifi_connect();
		}
	}
	else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
		ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
		ESP_LOGI(TAG, "connected to ap, got ip:" IPSTR, IP2STR(&event->ip_info.ip));
		xEventGroupSetBits(app_event_group, WIFI_CONNECTED_BIT);
	}
}

static void wifi_init(uint8_t mode)
{
	// If WIFI is initialized already, warn and stop
	if (xEventGroupGetBits(app_event_group) & WIFI_INITIALIZED_BIT){
		ESP_LOGW(TAG, "WIFI STA already initialized!");
		return;
	}
	ESP_LOGI(TAG,"ESP-IDF esp_netif");
	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	esp_netif_t *netif = esp_netif_create_default_wifi_sta();
	assert(netif);

#if CONFIG_STATIC_IP

	ESP_LOGI(TAG, "CONFIG_STATIC_IP_ADDRESS=[%s]",CONFIG_STATIC_IP_ADDRESS);
	ESP_LOGI(TAG, "CONFIG_STATIC_GW_ADDRESS=[%s]",CONFIG_STATIC_GW_ADDRESS);
	ESP_LOGI(TAG, "CONFIG_STATIC_NM_ADDRESS=[%s]",CONFIG_STATIC_NM_ADDRESS);

	/* Stop DHCP client */
	ESP_ERROR_CHECK(esp_netif_dhcpc_stop(netif));
	ESP_LOGI(TAG, "Stop DHCP Services");

	/* Set STATIC IP Address */
	esp_netif_ip_info_t ip_info;
	memset(&ip_info, 0 , sizeof(esp_netif_ip_info_t));
	ip_info.ip.addr = ipaddr_addr(CONFIG_STATIC_IP_ADDRESS);
	ip_info.netmask.addr = ipaddr_addr(CONFIG_STATIC_NM_ADDRESS);
	ip_info.gw.addr = ipaddr_addr(CONFIG_STATIC_GW_ADDRESS);;
	esp_netif_set_ip_info(netif, &ip_info);

	/*
	I referred from here.
	https://www.esp32.com/viewtopic.php?t=5380

	if we should not be using DHCP (for example we are using static IP addresses),
	then we need to instruct the ESP32 of the locations of the DNS servers manually.
	Google publicly makes available two name servers with the addresses of 8.8.8.8 and 8.8.4.4.
	*/

	ip_addr_t d;
	d.type = IPADDR_TYPE_V4;
	d.u_addr.ip4.addr = 0x08080808; //8.8.8.8 dns
	dns_setserver(0, &d);
	d.u_addr.ip4.addr = 0x08080404; //8.8.4.4 dns
	dns_setserver(1, &d);

#endif // CONFIG_STATIC_IP

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));

	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
	ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
	ESP_ERROR_CHECK(esp_wifi_set_mode(mode));

	xEventGroupSetBits(app_event_group, WIFI_INITIALIZED_BIT);

	ESP_LOGI(TAG, "wifi_init finished.");
}

static void wifi_connect(char *ssid, char *pswd)
{
	// If WIFI STA is connected, warn and stop
	if (xEventGroupGetBits(app_event_group) & WIFI_CONNECTED_BIT){
		ESP_LOGW(TAG, "WIFI STA already connected!");
		return;
	}
	wifi_config_t wifi_config = {
		.sta = {
			.ssid = "0"
		}
	};
	strcpy((char *)wifi_config.sta.ssid, ssid);
	strcpy((char *)wifi_config.sta.password, pswd);

	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());
}

static void wifi_disconnect()
{
	EventBits_t event_bits = xEventGroupGetBits(app_event_group);
	// If WIFI STA is connected, disconnect and stop it
	if (event_bits & WIFI_CONNECTED_BIT){
		// Store WIFI retry setting
		event_bits &= WIFI_RETRY_BIT;

		// Clear retry bit, so WIFI wont reconnect automatically
		xEventGroupClearBits(app_event_group, WIFI_RETRY_BIT);
		
		// Disconnect and stop WIFI
		esp_wifi_disconnect();
		esp_wifi_stop();

		// Update WIFI status
		xEventGroupClearBits(app_event_group, WIFI_CONNECTED_BIT);

		// Restore retry setting
		if (event_bits){
			xEventGroupClearBits(app_event_group, WIFI_RETRY_BIT);
		}
	}
}

// mDNS functions
void initialize_mdns(void)
{
	//initialize mDNS
	ESP_ERROR_CHECK( mdns_init() );
	//set mDNS hostname (required if you want to advertise services)
	ESP_ERROR_CHECK( mdns_hostname_set(MDNS_HOSTNAME) );
	ESP_LOGI(TAG, "mdns hostname set to: [%s]", MDNS_HOSTNAME);

	//initialize service
	ESP_ERROR_CHECK( mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0) );

#if 0
	//set default mDNS instance name
	ESP_ERROR_CHECK( mdns_instance_name_set("ESP32 with mDNS") );
#endif
}

// SPIFFS functions
esp_err_t mountSPIFFS(char * partition_label, char * base_path) {
	ESP_LOGI(TAG, "Initializing SPIFFS file system");

	esp_vfs_spiffs_conf_t conf = {
		.base_path = base_path,
		.partition_label = partition_label,
		.max_files = 5,
		.format_if_mount_failed = true
	};

	// Use settings defined above to initialize and mount SPIFFS filesystem.
	// Note: esp_vfs_spiffs_register is an all-in-one convenience function.
	esp_err_t ret = esp_vfs_spiffs_register(&conf);
	if (ret != ESP_OK) {
		if (ret == ESP_FAIL) {
			ESP_LOGE(TAG, "Failed to mount or format filesystem");
		} else if (ret == ESP_ERR_NOT_FOUND) {
			ESP_LOGE(TAG, "Failed to find SPIFFS partition");
		} else {
			ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
		}
		return ret;
	}

	size_t total = 0, used = 0;
	ret = esp_spiffs_info(partition_label, &total, &used);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
	} else {
		ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
	}
	ESP_LOGI(TAG, "Mount SPIFFS filesystem");
	return ret;
}

static void printSPIFFS(char * path) {
	DIR* dir = opendir(path);
	assert(dir != NULL);
	while (true) {
		struct dirent*pe = readdir(dir);
		if (!pe) break;
		ESP_LOGI(__FUNCTION__,"d_name=%s d_ino=%d d_type=%x", pe->d_name,pe->d_ino, pe->d_type);
	}
	closedir(dir);
}

// Websocket client
static void websocket_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
	esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;
	switch (event_id) {
		case WEBSOCKET_EVENT_CONNECTED:{
			ESP_LOGI(TAG, "WEBSOCKET_EVENT_CONNECTED");
			// Set status
			xEventGroupSetBits(app_event_group, WS_CONNECTED_BIT);
			break;
		}
		case WEBSOCKET_EVENT_DISCONNECTED:{
			ESP_LOGI(TAG, "WEBSOCKET_EVENT_DISCONNECTED");
			// Clear status
			xEventGroupClearBits(app_event_group, WS_CONNECTED_BIT | VIDEO_RECORD_BIT);
			
			if (xEventGroupGetBits(app_event_group) & DEVICE_LOGGED_IN_BIT){
				xEventGroupClearBits(app_event_group, DEVICE_LOGGED_IN_BIT);
				// Retry logging in later
				if (xEventGroupGetBits(app_event_group) & DEVICE_LOGIN_RETRY_BIT)
					xEventGroupSetBits(app_event_group, LOGIN_REQ_BIT);
			}

			log_error_if_nonzero("HTTP status code",  data->error_handle.esp_ws_handshake_status_code);
			if (data->error_handle.error_type == WEBSOCKET_ERROR_TYPE_TCP_TRANSPORT) {
					log_error_if_nonzero("reported from esp-tls", data->error_handle.esp_tls_last_esp_err);
					log_error_if_nonzero("reported from tls stack", data->error_handle.esp_tls_stack_err);
					log_error_if_nonzero("captured as transport's socket errno",  data->error_handle.esp_transport_sock_errno);
			}
			break;
		}
		case WEBSOCKET_EVENT_CLOSED:{
			ESP_LOGI(TAG, "WEBSOCKET_EVENT_CLOSED");
			// Clear status
			xEventGroupClearBits(app_event_group, WS_CONNECTED_BIT | DEVICE_LOGGED_IN_BIT | VIDEO_RECORD_BIT);
			break;
		}
		case WEBSOCKET_EVENT_DATA:{
			ESP_LOGI(TAG, "WEBSOCKET_EVENT_DATA");
			ESP_LOGI(TAG, "Received opcode=%d", data->op_code);
			if (data->op_code == 0x08 && data->data_len == 2) {
					ESP_LOGW(TAG, "Received closed message with code=%d", 256 * data->data_ptr[0] + data->data_ptr[1]);
			}
			else if (data->data_ptr[0] == 'i' && data->data_len == 1){
				ESP_LOGI(TAG, "Logged into server successfully");
				xEventGroupSetBits(app_event_group, DEVICE_LOGGED_IN_BIT);
			}
			else if (data->data_ptr[0] == 'u' && data->data_len == 1){
				ESP_LOGI(TAG, "Log in failed: Device is not registered");
				xEventGroupClearBits(app_event_group, DEVICE_LOGGED_IN_BIT | VIDEO_RECORD_BIT);
				
				if (xEventGroupGetBits(app_event_group) & DEVICE_LOGIN_RETRY_BIT)
					xEventGroupSetBits(app_event_group, LOGIN_REQ_BIT);
			}
			else if (data->data_ptr[0] == 'c' && data->data_len == 1){
				ESP_LOGI(TAG, "Logged out due to changes in device's ownership");
				xEventGroupClearBits(app_event_group, DEVICE_LOGGED_IN_BIT | VIDEO_RECORD_BIT);

				if (xEventGroupGetBits(app_event_group) & DEVICE_LOGIN_RETRY_BIT)
					xEventGroupSetBits(app_event_group, LOGIN_REQ_BIT);
			}
			else if (data->data_ptr[0] == 'o' && data->data_len == 1){
				ESP_LOGI(TAG, "Logged out of server!");
				xEventGroupClearBits(app_event_group, DEVICE_LOGGED_IN_BIT | VIDEO_RECORD_BIT);
			}
			else {
					ESP_LOGW(TAG, "Received=%.*s\n\n", data->data_len, (char *)data->data_ptr);
			}

			// If received data contains json structure it succeed to parse
			// cJSON *root = cJSON_Parse(data->data_ptr);
			// if (root) {
			// 		for (int i = 0 ; i < cJSON_GetArraySize(root) ; i++) {
			// 				cJSON *elem = cJSON_GetArrayItem(root, i);
			// 				cJSON *id = cJSON_GetObjectItem(elem, "id");
			// 				cJSON *name = cJSON_GetObjectItem(elem, "name");
			// 				ESP_LOGW(TAG, "Json={'id': '%s', 'name': '%s'}", id->valuestring, name->valuestring);
			// 		}
			// 		cJSON_Delete(root);
			// }

			ESP_LOGW(TAG, "Total payload length=%d, data_len=%d, current payload offset=%d\r\n", data->payload_len, data->data_len, data->payload_offset);

			break;
		}
		case WEBSOCKET_EVENT_ERROR:{
			ESP_LOGI(TAG, "WEBSOCKET_EVENT_ERROR");
			log_error_if_nonzero("HTTP status code",  data->error_handle.esp_ws_handshake_status_code);
			if (data->error_handle.error_type == WEBSOCKET_ERROR_TYPE_TCP_TRANSPORT) {
					log_error_if_nonzero("reported from esp-tls", data->error_handle.esp_tls_last_esp_err);
					log_error_if_nonzero("reported from tls stack", data->error_handle.esp_tls_stack_err);
					log_error_if_nonzero("captured as transport's socket errno",  data->error_handle.esp_transport_sock_errno);
			}
			break;
		}
	}
}

static void websocket_connect(char *uri, int port)
{
	// If websocket already initialized, send a warning message, then exit
	// User need to disconnect properly before hand
	if (xSemaphoreTake(ws_sema, portMAX_DELAY)){
		if (xEventGroupGetBits(app_event_group) & WS_INITIALIZED_BIT){
			ESP_LOGW(TAG, "Websocket client already initialized!");
			return;
		}
		
		esp_websocket_client_config_t websocket_cfg = {
			.disable_auto_reconnect = false,
			.uri = uri,
			.port = port,
		};

#if CONFIG_WS_OVER_TLS_MUTUAL_AUTH
		/* Configuring client certificates for mutual authentification */
		extern const char cacert_start[] asm("_binary_ca_cert_pem_start"); // CA certificate
		extern const char cert_start[] asm("_binary_client_cert_pem_start"); // Client certificate
		extern const char cert_end[]   asm("_binary_client_cert_pem_end");
		extern const char key_start[] asm("_binary_client_key_pem_start"); // Client private key
		extern const char key_end[]   asm("_binary_client_key_pem_end");

		websocket_cfg.cert_pem = cacert_start;
		websocket_cfg.client_cert = cert_start;
		websocket_cfg.client_cert_len = cert_end - cert_start;
		websocket_cfg.client_key = key_start;
		websocket_cfg.client_key_len = key_end - key_start;

#elif CONFIG_WS_OVER_TLS_SERVER_AUTH
		extern const char cacert_start[] asm("_binary_ca_certificate_public_domain_pem_start"); // CA cert of wss://echo.websocket.event, modify it if using another server
		websocket_cfg.cert_pem = cacert_start;
#endif

#if CONFIG_WS_OVER_TLS_SKIP_COMMON_NAME_CHECK
		websocket_cfg.skip_cert_common_name_check = true;
#endif

		client = esp_websocket_client_init(&websocket_cfg);
		esp_websocket_register_events(client, WEBSOCKET_EVENT_ANY, websocket_event_handler, (void *)client);
		xEventGroupSetBits(app_event_group, WS_INITIALIZED_BIT);

		ESP_LOGI(TAG, "Websocket waiting for WIFI");

		xEventGroupWaitBits(app_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
		ESP_LOGI(TAG, "Connecting to %s:%d...", websocket_cfg.uri, websocket_cfg.port);
		esp_websocket_client_start(client);
		xSemaphoreGive(ws_sema);
	}
}

static void websocket_disconnect()
{
	// Take ws mutex to stop video and timeline tasks
	if (xSemaphoreTake(ws_sema, portMAX_DELAY)){
		EventBits_t event_bits = xEventGroupGetBits(app_event_group);
		// If websocket is connected, send a close message
		if (event_bits & WS_CONNECTED_BIT){
			esp_websocket_client_close(client, portMAX_DELAY);
			xEventGroupClearBits(app_event_group, WS_CONNECTED_BIT);
		}
		// If websocket is initialized, stop and destroy it
		if (event_bits & WS_INITIALIZED_BIT){
			esp_websocket_client_stop(client);
			esp_websocket_client_destroy(client);
			xEventGroupClearBits(app_event_group, WS_INITIALIZED_BIT);
		}
		// Give mutex
		xSemaphoreGive(ws_sema);
	}
}

// GPS
static void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
	gps_t *gps = NULL;
	switch (event_id) {
	case GPS_UPDATE:
		// Make it count 5 times before call timeline task
		if (--gps_update_count){
			break;
		}

		gps = (gps_t *)event_data;
		/* print information parsed from GPS statements */
		ESP_LOGI(TAG, "%d/%d/%d %d:%d:%d => \r\n"
						"\t\t lat:  %.05f°N\r\n"
						"\t\t long: %.05f°E\r\n"
						"\tNum of Satellines: %u",
						gps->date.year + YEAR_BASE, gps->date.month, gps->date.day,
						gps->tim.hour + TIME_ZONE, gps->tim.minute, gps->tim.second,
						gps->latitude, gps->longitude, gps->sats_in_use);
		if (xSemaphoreTake(pos_sema, 10 / portTICK_PERIOD_MS)){
			// Restart count if updating succeed
			gps_update_count = TIMELINE_UPDATE_DELAY;

			// Update Position data
			pos[0] = gps->latitude;
			pos[1] = gps->longitude;

			// Trigger event
			xEventGroupSetBits(app_event_group, TIMELINE_UPLOAD_BIT);
			xSemaphoreGive(pos_sema);
		}
		else{
			// If fail to update, then set to 1, so it will update next time.
			gps_update_count = 1;
		}

		break;
	case GPS_UNKNOWN:
		/* print unknown statements */
		// ESP_LOGW(TAG, "Unknown statement:%s", (char *)event_data);
		break;
	default:
			break;
	}
}

static void gps_init()
{
	/* NMEA parser configuration */
	nmea_parser_config_t config = NMEA_PARSER_CONFIG_DEFAULT();
	/* init NMEA parser library */
	nmea_parser_handle_t nmea_hdl = nmea_parser_init(&config);
	/* register event handler for NMEA parser library */
	nmea_parser_add_handler(nmea_hdl, gps_event_handler, NULL);
}

// Server Authorizing
static uint8_t device_login(){
	// Message header
	char data_header[2] = {'i', DEVICE_ID};

	EventBits_t status_bit = WS_CONNECTED_BIT | WIFI_CONNECTED_BIT;

	// Check if device status is okay and hasn't logged in
	if ((xEventGroupGetBits(app_event_group) & (status_bit | DEVICE_LOGGED_IN_BIT)) == (status_bit | (!DEVICE_LOGGED_IN_BIT))){
		// Take mutex
		if(xSemaphoreTake(ws_sema, portMAX_DELAY)){
			// Send log in message to server
			esp_websocket_client_send_bin(client, data_header, 2, portMAX_DELAY);

			// Give mutex
			xSemaphoreGive(ws_sema);
		}
		return 1;
	}
	return 0;
}

static void device_logout(){
	// Message header
	char data_header[2] = {'o', DEVICE_ID};

	EventBits_t status_bit = WS_CONNECTED_BIT | WIFI_CONNECTED_BIT;

	// Check if device status is okay and has logged in
	if ((xEventGroupGetBits(app_event_group) & (status_bit | DEVICE_LOGGED_IN_BIT)) == (status_bit | DEVICE_LOGGED_IN_BIT)){
		if (xSemaphoreTake(ws_sema, portMAX_DELAY)){
			// Send log out message to server
			esp_websocket_client_send_bin(client, data_header, 2, portMAX_DELAY);

			// Wait until logged out successfully
			while (xEventGroupGetBits(app_event_group) & DEVICE_LOGGED_IN_BIT){
				vTaskDelay(10 / portTICK_PERIOD_MS);
			}

			// Clear status bits to stop video and timeline tasks
			xEventGroupClearBits(app_event_group, VIDEO_RECORD_BIT | TIMELINE_UPLOAD_BIT);
			xSemaphoreGive(ws_sema);
		}
	}
}

// App Tasks
static void task_video_record()
{
	// Task TAG
	char TAG[] = "Video_task";

	// Timing vars
	TickType_t loop_time = 0, last_run = xTaskGetTickCount();
	int capture_delay = 1000 / VIDEO_FPS;
	
	// Event bits
	EventBits_t status_bits = WIFI_CONNECTED_BIT | WS_CONNECTED_BIT | DEVICE_LOGGED_IN_BIT;

	// Cam buffer
	camera_fb_t *fb = NULL;

	// Data buffer
	uint8_t *data_buf;
	uint32_t data_len;
	
	while (1){
		xEventGroupWaitBits(app_event_group, VIDEO_RECORD_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
		
		// Check if app status is correct and websocket sema is free
		if (xSemaphoreTake(ws_sema, portMAX_DELAY)){
			if ((xEventGroupGetBits(app_event_group) & (status_bits | VIDEO_RECORD_BIT)) == (status_bits | VIDEO_RECORD_BIT))
			{				
				// Pace the capture rate to match frequency Max
				xTaskDelayUntil(&last_run, pdMS_TO_TICKS(capture_delay));
				loop_time = pdTICKS_TO_MS(xTaskGetTickCount() - loop_time);
				ESP_LOGI(TAG, "Video loop time: %d", (int)loop_time);
				loop_time = last_run;

				// Capture image with camera
				fb = esp_camera_fb_get();
				if (!fb){
					ESP_LOGE(TAG, "Camera Capture failed.");
					continue;
				}

				// Create message frame
				data_len = 2 + fb->len;
				data_buf = (uint8_t *) malloc(data_len * sizeof(uint8_t));
				// Edit header
				data_buf[0] = 'v';
				data_buf[1] = DEVICE_ID;
				// Move data from camera buffer
				memcpy(data_buf + 2, fb->buf, fb->len);

				// Free camera buffer
				esp_camera_fb_return(fb);

				// Send image to server
				if (esp_websocket_client_is_connected(client)){
					esp_websocket_client_send_bin(client, (char *) data_buf, data_len, portMAX_DELAY);

					ESP_LOGI(TAG, "Data length: %d", fb->len);
				}
				else {
					ESP_LOGE(TAG, "Websocket not connected.");
				}
				
				// Free data buffer
				free(data_buf);
				data_len = 0;
			}
			// Give mutex
			xSemaphoreGive(ws_sema);
		}

		vTaskDelay(1);
	}
}

static void task_timeline_record()
{
	// Task TAG
	char TAG[] = "Timeline_task";
	
	// Event bits
	EventBits_t status_bits = WIFI_CONNECTED_BIT | WS_CONNECTED_BIT | DEVICE_LOGGED_IN_BIT;

	// Message header
	char data_header[2] = {'t', DEVICE_ID};

	while (1){
		xEventGroupWaitBits(app_event_group, status_bits | TIMELINE_UPLOAD_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
		// Take semas
		if (xSemaphoreTake(ws_sema, portMAX_DELAY)){
			if (xSemaphoreTake(pos_sema, portMAX_DELAY)){
				// Check if app status is correct
				if ((xEventGroupGetBits(app_event_group) & (status_bits | TIMELINE_UPLOAD_BIT)) == (status_bits | TIMELINE_UPLOAD_BIT)){
					// Send current position to server
					if (esp_websocket_client_is_connected(client)){
						esp_websocket_client_send_bin_partial(client, data_header, 2, portMAX_DELAY);
						esp_websocket_client_send_cont_msg(client, (char *) pos, 8, portMAX_DELAY);
						esp_websocket_client_send_fin(client, portMAX_DELAY);

						ESP_LOGI(TAG, "Current Position uploaded");
					}
					else {
						ESP_LOGE(TAG, "Websocket not connected.");
					}
					// Release resources
					xEventGroupClearBits(app_event_group, TIMELINE_UPLOAD_BIT);
				}
				xSemaphoreGive(pos_sema);
			}
			xSemaphoreGive(ws_sema);
		}

		vTaskDelay(1);
	}
}

static void task_app_control()
{
	// Task TAG
	char TAG[] = "Control_task";
	
	// Event bits
	EventBits_t event_bits,
							request_bits = WIFI_REQ_CONNECT_BIT | WIFI_REQ_DISCONNECT_BIT
													 | WS_REQ_CONNECT_BIT | WS_REQ_DISCONNECT_BIT
													 | LOGIN_REQ_BIT | LOGOUT_REQ_BIT;

	TickType_t last_login_attempt = xTaskGetTickCount();

	while (1){
		event_bits = xEventGroupWaitBits(app_event_group, request_bits, pdFALSE, pdFALSE, portMAX_DELAY);
		
		// Logout request
		if (event_bits & LOGOUT_REQ_BIT){
			ESP_LOGI(TAG, "Logging out ...");
			// Logout
			device_logout();

			// Clear request bit
			xEventGroupClearBits(app_event_group, LOGOUT_REQ_BIT);
			event_bits = xEventGroupGetBits(app_event_group);
		}

		// WS disconnect request
		if (event_bits & WS_REQ_DISCONNECT_BIT){
			// If device is still logged in, skip until it finish logging out
			if ((event_bits & DEVICE_LOGGED_IN_BIT) == false){
				ESP_LOGI(TAG, "Closing websocket client ...");
				// Disconnect websocket
				websocket_disconnect();

				xEventGroupClearBits(app_event_group, WS_REQ_DISCONNECT_BIT);
				event_bits = xEventGroupGetBits(app_event_group);
			}
		}

		// Wifi disconnect request
		if (event_bits & WIFI_REQ_DISCONNECT_BIT){
			// If device is logged in or websocket still alive, skip until those are closed
			if ((event_bits & (WS_CONNECTED_BIT | DEVICE_LOGGED_IN_BIT)) == false){
				ESP_LOGI(TAG, "Disconnecting WIFI STA ...");
				// Disconnect WIFI STA
				wifi_disconnect();

				xEventGroupClearBits(app_event_group, WIFI_REQ_DISCONNECT_BIT);
				event_bits = xEventGroupGetBits(app_event_group);
			}
		}

		// Wifi connect request
		if (event_bits & WIFI_REQ_CONNECT_BIT){
			// If WIFI is already connected, create an event to disconnect it first
			if (event_bits & WIFI_CONNECTED_BIT){
				xEventGroupSetBits(app_event_group, WIFI_REQ_DISCONNECT_BIT);
			}
			else {
				ESP_LOGI(TAG, "Connecting to WIFI ...");
				// Call connect function
				wifi_connect(wifi_ssid, wifi_pswd);
				// Clear request bit
				xEventGroupClearBits(app_event_group, WIFI_REQ_CONNECT_BIT);
			}
		}

		// WS connect request
		if (event_bits & WS_REQ_CONNECT_BIT){
			// If websocket connection is alive, create an event to close it first.
			if (event_bits & WS_CONNECTED_BIT){
				xEventGroupSetBits(app_event_group, WS_REQ_DISCONNECT_BIT);
			}
			else {
				ESP_LOGI(TAG, "Connecting to server ...");
				// Call connect function
				websocket_connect(WEBSOCKET_URI, WEBSOCKET_PORT);
				// Clear request bit
				xEventGroupClearBits(app_event_group, WS_REQ_CONNECT_BIT);
			}
		}

		// Login request
		if (event_bits & LOGIN_REQ_BIT){
			// Pace device retry frequency to make sure it doesn't litter server
			if (pdTICKS_TO_MS(xTaskGetTickCount() - last_login_attempt) > 5000){
				ESP_LOGI(TAG, "Logging in to server ...");
				last_login_attempt = xTaskGetTickCount();
				// Login
				if (device_login()){
					// Clear request bit if login was not cancelled by wrong condition
					xEventGroupClearBits(app_event_group, LOGIN_REQ_BIT);
				}
			}
		}

		vTaskDelay(1);
	}
}