// Includes *******************************************************************
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
#include "esp_mac.h"

#include "nvs_flash.h"
#include "esp_vfs.h"
#include "esp_spiffs.h"

#include "esp_camera.h"
#include "camera_pin.h"

#include "esp_wifi.h"
#include "esp_websocket_client.h"

#include "nmea_parser.h"
#include "mdns.h"


// Defines ********************************************************************
#define DEVICE_ID 1
#define AP_SSID   "C1G1_DASHCAM_SYSTEM"
#define AP_PSWD   "doan10diem"
#define WIFI_SSID "Nasmes_INF"
#define WIFI_PSWD "167294381"

#define MDNS_HOSTNAME "dashcam-config"

#define WEBSOCKET_URI "ws://192.168.62.244"
#define WEBSOCKET_PORT 8765

#define VIDEO_FPS 5 //Hz
#define JPEG_QUALITY 24

#define GPS_UART_PIN 3
#define TIMELINE_UPDATE_DELAY 10 /*updates from GPS module*/

/* Event bits for events in the program:
 * 0 - Connected to WIFI
 * 1 - Websocket connection established
 * 2 - Device is registered to an account and ready to transfer data
 * */
#define WIFI_INITIALIZED_BIT		BIT0
#define WIFI_RECON_BIT					BIT1
#define WIFI_CONNECTED_BIT 			BIT2

#define WS_INITIALIZED_BIT			BIT3
#define WS_CONNECTED_BIT 				BIT4
#define DEVICE_RELOG_BIT        BIT5
#define DEVICE_LOGGED_IN_BIT	 	BIT6
#define VIDEO_RECORD_BIT				BIT7
#define VIDEO_STOP_BIT					BIT8
#define TIMELINE_UPLOAD_BIT			BIT9

#define WIFI_REQ_CONNECT_BIT		BIT10
#define WIFI_REQ_DISCONNECT_BIT BIT11
#define WS_REQ_CONNECT_BIT			BIT12
#define WS_REQ_DISCONNECT_BIT		BIT13
#define DEVICE_REQ_LOGIN_BIT    BIT14
#define DEVICE_REQ_LOGOUT_BIT		BIT15


#define TIME_ZONE (+7)   //Hanoi Time
#define YEAR_BASE (2000) //date in GPS starts from 2000

// Variables ******************************************************************
extern EventGroupHandle_t app_event_group;

extern nvs_handle_t nvs_config;

// Functions ******************************************************************
static void wifi_read_config_from_nvs(char **ssid, char **pswd)
{
	size_t buff_len = 0;

	// Free old buffer (user better do it themselves or provide clean pointers)
	if (*ssid) free(*ssid);
	if (*pswd) free(*pswd);

	// Get SSID length
	ESP_ERROR_CHECK(nvs_get_str(nvs_config, "wifi_ssid", NULL, &buff_len));
	if (buff_len > 1){
		// Allocate memory and get SSID string
		*ssid = malloc(buff_len * sizeof(char));
		ESP_ERROR_CHECK(nvs_get_str(nvs_config, "wifi_ssid", *ssid, &buff_len));
	}
	else{
		// If SSID is empty, use system default SSID, also write it to NVS
		*ssid = malloc((strlen(WIFI_SSID) + 1) * sizeof(char));
		strcpy(*ssid, WIFI_SSID);
		ESP_ERROR_CHECK(nvs_set_str(nvs_config, "wifi_ssid", WIFI_SSID));
	}
	
	// Get password, same procedure
	ESP_ERROR_CHECK(nvs_get_str(nvs_config, "wifi_pswd", NULL, &buff_len));
	if (buff_len > 8){
		*pswd = malloc(buff_len * sizeof(char));
		ESP_ERROR_CHECK(nvs_get_str(nvs_config, "wifi_pswd", *pswd, &buff_len));
	}
	else{
		*pswd = malloc((strlen(WIFI_PSWD) + 1) * sizeof(char));
		strcpy(*pswd, WIFI_PSWD);
		ESP_ERROR_CHECK(nvs_set_str(nvs_config, "wifi_pswd", WIFI_PSWD));
	}

	ESP_ERROR_CHECK(nvs_commit(nvs_config));
}

static void websocket_read_config_from_nvs(char **uri, int *port)
{
	size_t buff_len = 0;

	// Free old buffer (user better do it themselves or provide clean pointers)
	if (*uri) free(*uri);

	// Get URI length
	ESP_ERROR_CHECK(nvs_get_str(nvs_config, "ws_uri", NULL, &buff_len));
	if (buff_len > 1){
		// Allocate memory and get URI string
		*uri = malloc(buff_len * sizeof(char));
		ESP_ERROR_CHECK(nvs_get_str(nvs_config, "ws_uri", *uri, &buff_len));
	}
	else{
		// If URI is empty, use system default URI, also write it to NVS
		*uri = malloc((strlen(WEBSOCKET_URI) + 1) * sizeof(char));
		strcpy(*uri, WEBSOCKET_URI);
		ESP_ERROR_CHECK(nvs_set_str(nvs_config, "ws_uri", WEBSOCKET_URI));
		ESP_ERROR_CHECK(nvs_commit(nvs_config));
	}

	// Get port number from NVS (probably safe, its just an int value)
	ESP_ERROR_CHECK(nvs_get_i32(nvs_config, "ws_port", (int32_t*) port));
}