/*
	Take a picture and Publish it via Web Socket.

	This code is in the Public Domain (or CC0 licensed, at your option.)

	Unless required by applicable law or agreed to in writing, this
	software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
	CONDITIONS OF ANY KIND, either express or implied.

	I based from here:
	https://github.com/espressif/esp-idf/tree/master/examples/protocols/http_server/ws_echo_server
*/

#include <main.h>
#include <cJSON.h>

#include <sys/stat.h>
#include <mbedtls/base64.h>
#include "esp_http_server.h"

static const char *TAG = "WS_SERVER";

typedef struct async_resp_arg {
	httpd_handle_t hd;
	int fd;
} async_resp_arg_t;

static void task_client_ui_update(void *arg)
{
	int count = 0;
	// Task TAG
	char TAG[] = "Web_UI_task";

	// Client info
	async_resp_arg_t *client_arg;
	client_arg = arg;
	if (client_arg == NULL){
		return;
	}
	
	// Event bits
	EventBits_t cur_status, old_status = BIT31; // Assign absurd value to make it run first time 

	// Data buffers
	uint8_t *data_buf = NULL;
	uint8_t wf_st, sv_st, vid_st, recon, relog;
	char *ssid = NULL;
	char *pswd = NULL;
	char *uri = NULL;
	int port = 0;
	
	// JSON
	cJSON *jsData = NULL;

	while (httpd_ws_get_fd_info(client_arg->hd, client_arg->fd) == HTTPD_WS_CLIENT_WEBSOCKET)
	{
		count++;
		cur_status = xEventGroupGetBits(app_event_group);
		if(cur_status != old_status){
			// Prepare data
			wf_st = (cur_status & WIFI_CONNECTED_BIT) != 0;
			sv_st = (cur_status & WS_CONNECTED_BIT) ? ((cur_status & WS_CONNECTED_BIT) ? 2 : 1) : 0;
			vid_st = (cur_status & VIDEO_RECORD_BIT) != 0;
			ESP_ERROR_CHECK(nvs_get_u8(nvs_config, "wifi_recon", &recon));
			ESP_ERROR_CHECK(nvs_get_u8(nvs_config, "relog", &relog));
			wifi_read_config_from_nvs(&ssid, &pswd);
			websocket_read_config_from_nvs(&uri, &port);

			// Create JSON
			ESP_LOGI(TAG, "Creating JSON data");
			jsData = cJSON_CreateObject();
			cJSON_AddNumberToObject(jsData, "id", DEVICE_ID);
			cJSON_AddNumberToObject(jsData, "wf_st", wf_st);
			cJSON_AddNumberToObject(jsData, "sv_st", sv_st);
			cJSON_AddNumberToObject(jsData, "vid_st", vid_st);
			cJSON_AddStringToObject(jsData, "ssid", ssid);
			cJSON_AddStringToObject(jsData, "pswd", pswd);
			cJSON_AddNumberToObject(jsData, "recon", recon);
			cJSON_AddStringToObject(jsData, "uri", uri);
			cJSON_AddNumberToObject(jsData, "port", port);
			cJSON_AddNumberToObject(jsData, "relog", relog);

			// Send to UI
			if (data_buf){
				free(data_buf);
				data_buf = NULL;
			}
			data_buf = (uint8_t*) cJSON_Print(jsData);

			httpd_ws_frame_t ws_pkt;
    	memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
			ws_pkt.payload = data_buf;
			ws_pkt.len = strlen((char*) data_buf);
			ws_pkt.type = HTTPD_WS_TYPE_TEXT;

			ESP_LOGI(TAG, "Sending JSON data, len: %d bytes", ws_pkt.len);
			httpd_ws_send_frame_async(client_arg->hd, client_arg->fd, &ws_pkt);

			// Clean up
			cJSON_Delete(jsData);
			free(ssid);
			free(pswd);
			free(uri);
			ssid = NULL;
			pswd = NULL;
			uri = NULL;
		}
		old_status = cur_status;
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
	ESP_LOGW(TAG, "Client UI updater exit after %d run", count);
	vTaskDelete(NULL);
}

/*static esp_err_t camera_capture(char * FileName, size_t *pictureSize)
{
	//clear internal queue
	//for(int i=0;i<2;i++) {
	for(int i=0;i<1;i++) {
		camera_fb_t * fb = esp_camera_fb_get();
		ESP_LOGI(TAG, "fb->len=%d", fb->len);
		esp_camera_fb_return(fb);
	}

	//acquire a frame
	camera_fb_t * fb = esp_camera_fb_get();
	if (!fb) {
		ESP_LOGE(TAG, "Camera Capture Failed");
		return ESP_FAIL;
	}

	//replace this with your own function
	//process_image(fb->width, fb->height, fb->format, fb->buf, fb->len);
	FILE* f = fopen(FileName, "wb");
	if (f == NULL) {
		ESP_LOGE(TAG, "Failed to open file for writing");
		return ESP_FAIL; 
	}
	fwrite(fb->buf, fb->len, 1, f);
	ESP_LOGI(TAG, "fb->len=%d", fb->len);
	*pictureSize = (size_t)fb->len;
	fclose(f);
	
	//return the frame buffer back to the driver for reuse
	esp_camera_fb_return(fb);

	return ESP_OK;
}*/

// Calculate the size after conversion to base64
// http://akabanessa.blog73.fc2.com/blog-entry-83.html
/*int32_t calcBase64EncodedSize(int origDataSize)
{
	// Number of blocks in 6-bit units (rounded up in 6-bit units)
	int32_t numBlocks6 = ((origDataSize * 8) + 5) / 6;
	// Number of blocks in units of 4 characters (rounded up in units of 4 characters)
	int32_t numBlocks4 = (numBlocks6 + 3) / 4;
	// Number of characters without line breaks
	int32_t numNetChars = numBlocks4 * 4;
	// Size considering line breaks every 76 characters (line breaks are "\ r \ n")
	//return numNetChars + ((numNetChars / 76) * 2);
	return numNetChars;
}*/

/*esp_err_t Image2Base64(char * imageFileName, size_t base64_buffer_len, uint8_t * base64_buffer)
{
	struct stat st;
	if (stat(imageFileName, &st) != 0) {
		ESP_LOGE(TAG, "[%s] not found", imageFileName);
		return ESP_FAIL;
	}
	ESP_LOGI(TAG, "%s st.st_size=%ld", imageFileName, st.st_size);

	// Allocate image memory
	unsigned char*	image_buffer = NULL;
	size_t image_buffer_len = st.st_size;
	image_buffer = malloc(image_buffer_len);
	if (image_buffer == NULL) {
		ESP_LOGE(TAG, "malloc fail. image_buffer_len %d", image_buffer_len);
		return ESP_FAIL;
	}

	// Read image file
	FILE * fp_image = fopen(imageFileName,"rb");
	if (fp_image == NULL) {
		ESP_LOGE(TAG, "[%s] fopen fail.", imageFileName);
		free(image_buffer);
		return ESP_FAIL;
	}
	for (int i=0;i<st.st_size;i++) {
		fread(&image_buffer[i], sizeof(char), 1, fp_image);
	}
	fclose(fp_image);

	// Convert from JPEG to BASE64
	size_t encord_len;
	esp_err_t ret = mbedtls_base64_encode(base64_buffer, base64_buffer_len, &encord_len, image_buffer, st.st_size);
	ESP_LOGI(TAG, "mbedtls_base64_encode=%d encord_len=%d", ret, encord_len);

	free(image_buffer);
	return ESP_OK;
}
*/

static esp_err_t ws_handler(httpd_req_t *req)
{
	if (req->method == HTTP_GET) {
		ESP_LOGI(TAG, "Handshake done, the new connection was opened");
		return ESP_OK;
	}

	// Get message (dynamic allocate) ===========================================
	httpd_ws_frame_t ws_pkt;
	uint8_t *buf = NULL;
	memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
	ws_pkt.type = HTTPD_WS_TYPE_TEXT;

	// First, get the message len. Set max_len = 0 here
	esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "httpd_ws_recv_frame failed to get frame len with %d", ret);
		return ret;
	}
	ESP_LOGI(TAG, "frame len is %d", ws_pkt.len);

	// Check to see if message is not blank
	if (ws_pkt.len) {
		// Allocate memory for buffer. Set max_len = ws_pkt.len + 1 since we expect a string and need NULL terminator
		buf = calloc(1, ws_pkt.len + 1);
		if (buf == NULL) {
			ESP_LOGE(TAG, "Failed to calloc memory for buf");
			return ESP_ERR_NO_MEM;
		}
		ws_pkt.payload = buf;

		// Get the actual message. Set max_len = ws_pkt.len to get the frame payload */
		ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
		if (ret != ESP_OK) {
			ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
			free(buf);
			return ret;
		}
		ESP_LOGI(TAG, "Got packet with message: [%.*s]", ws_pkt.len, ws_pkt.payload);
	}
	ESP_LOGI(TAG, "Packet final: %d", ws_pkt.final);
	ESP_LOGI(TAG, "Packet fragmented: %d", ws_pkt.fragmented);
	ESP_LOGI(TAG, "Packet type: %d", ws_pkt.type);

// 	char imageFileName[256];
// 	//strcpy(imageFileName, "/spiffs/esp32.jpeg");
// 	strcpy(imageFileName, "/spiffs/capture.jpeg");
//
// 	// Delete local file
// 	struct stat statBuf;
// 	if (stat(imageFileName, &statBuf) == 0) {
// 		// Delete it if it exists
// 		unlink(imageFileName);
// 		ESP_LOGI(TAG, "Delete Local file");
// 	}
//
// #if CONFIG_ENABLE_FLASH
// 	// Flash Light ON
// 	gpio_set_level(CONFIG_GPIO_FLASH, 1);
// #endif
//
// 	// Save Picture to Local file
// 	int retryCounter = 0;
// 	while(1) {
// 		size_t pictureSize;
// 		ret = camera_capture(imageFileName, &pictureSize);
// 		ESP_LOGI(TAG, "camera_capture=%d",ret);
// 		ESP_LOGI(TAG, "pictureSize=%d",pictureSize);
// 		if (ret != ESP_OK) continue;
// 		if (stat(imageFileName, &statBuf) == 0) {
// 			ESP_LOGI(TAG, "st_size=%d", (int)statBuf.st_size);
// 			if (statBuf.st_size == pictureSize) break;
// 			retryCounter++;
// 			ESP_LOGI(TAG, "Retry capture %d",retryCounter);
// 			if (retryCounter > 10) {
// 				ESP_LOGE(TAG, "Retry over for capture");
// 				break;
// 			}
// 			vTaskDelay(1000);
// 		}
// 	} // end while
//
// #if CONFIG_ENABLE_FLASH
// 	// Flash Light OFF
// 	gpio_set_level(CONFIG_GPIO_FLASH, 0);
// #endif
//
// 	// Get Image size
// 	struct stat st;
// 	if (stat(imageFileName, &st) != 0) {
// 		ESP_LOGE(TAG, "[%s] not found", imageFileName);
// 		return ESP_FAIL;
// 	}
// 	ESP_LOGI(TAG, "%s st.st_size=%ld", imageFileName, st.st_size);
//
// 	// Get Base64 size
// 	int32_t base64Size = calcBase64EncodedSize(st.st_size);
// 	ESP_LOGI(TAG, "base64Size=%"PRIi32, base64Size);
//
// 	// Allocate Base64 buffer
// 	// You have to use calloc. It doesn't work with malloc.
// 	uint8_t *base64_buffer = NULL;
// 	size_t base64_buffer_len = base64Size + 1;
// 	base64_buffer = calloc(1, base64_buffer_len);
// 	if (base64_buffer == NULL) {
// 		ESP_LOGE(TAG, "calloc fail. base64_buffer_len %d", base64_buffer_len);
// 		return ESP_FAIL;
// 	}
// 	memset(base64_buffer, 0, base64_buffer_len);
//
// 	// Convert from Image to Base64
// 	//ret = Image2Base64("/spiffs/esp32.jpeg", base64_buffer_len, base64_buffer);
// 	ret = Image2Base64(imageFileName, base64_buffer_len, base64_buffer);
// 	ESP_LOGI(TAG, "Image2Base64=%d", ret);
// 	if (ret != ESP_OK) {
// 		free(base64_buffer);
// 		return ret;
// 	}
//
// 	// Send by WebSocket
// 	ws_pkt.payload = base64_buffer;
// 	ws_pkt.len = base64Size;
// 	ret = httpd_ws_send_frame(req, &ws_pkt);
// 	if (ret != ESP_OK) {
// 		ESP_LOGE(TAG, "httpd_ws_send_frame failed with %d", ret);
// 	}
// 	free(base64_buffer);

	// Processing logics ========================================================
	
	if (ws_pkt.type == HTTPD_WS_TYPE_TEXT)
	switch (ws_pkt.payload[0])
	{
		// If UI is accessing for the first time
		case 'a':{
			ESP_LOGI(TAG, "New user from Web UI");
			async_resp_arg_t *client_args = malloc(sizeof(async_resp_arg_t));
			if (client_args == NULL){
				return ESP_ERR_NO_MEM;
			}
			client_args->hd = req->handle;
			client_args->fd = httpd_req_to_sockfd(req);

			xTaskCreate(task_client_ui_update, NULL, 2048, (void*)client_args, 3, NULL);
			break;
		}
		
		// If UI request video record function
		case 'v':{
			ESP_LOGI(TAG, "Video toggle from Web UI");
			if(xEventGroupGetBits(app_event_group) & VIDEO_RECORD_BIT){
				xEventGroupSetBits(app_event_group, VIDEO_STOP_BIT);
			}
			else{
				xEventGroupSetBits(app_event_group, VIDEO_RECORD_BIT);
			}
			break;
		}

		// If UI request to disconnect, set events to disconnect
		case 'd':{
			ESP_LOGI(TAG, "Disconnect request from Web UI");
			xEventGroupSetBits(app_event_group, WIFI_REQ_DISCONNECT_BIT | WS_REQ_DISCONNECT_BIT | DEVICE_REQ_LOGOUT_BIT);
			xEventGroupClearBits(app_event_group, WIFI_REQ_CONNECT_BIT | WS_REQ_CONNECT_BIT | DEVICE_REQ_LOGIN_BIT);
			break;
		}
		
		// Else, probably JSON and request to connect
		default:{
			ESP_LOGI(TAG, "Got new config from Web UI");
			// Create JSON object and parse data, ignore if it's not
			cJSON *jsonObj = cJSON_Parse((char*) ws_pkt.payload);
			cJSON *cursor;
			if (jsonObj == NULL){
				ESP_LOGW(TAG, "JSON parse failed!");
				cJSON_Delete(jsonObj);
				break;
			}

			// Update system config
			cursor = cJSON_GetObjectItemCaseSensitive(jsonObj, "ssid");
			if (cJSON_IsString(cursor)){
				if (cursor->valuestring != NULL){
					ESP_ERROR_CHECK(nvs_set_str(nvs_config, "wifi_ssid", cursor->valuestring));
				}
			}
			cursor = cJSON_GetObjectItemCaseSensitive(jsonObj, "pswd");
			if (cJSON_IsString(cursor)){
				if (cursor->valuestring != NULL){
					if (strlen(cursor->valuestring) > 7)
						ESP_ERROR_CHECK(nvs_set_str(nvs_config, "wifi_pswd", cursor->valuestring));
				}
			}
			cursor = cJSON_GetObjectItemCaseSensitive(jsonObj, "uri");
			if (cJSON_IsString(cursor)){
				if (cursor->valuestring != NULL){
					ESP_ERROR_CHECK(nvs_set_str(nvs_config, "ws_uri", cursor->valuestring));
				}
			}
			cursor = cJSON_GetObjectItemCaseSensitive(jsonObj, "port");
			if (cJSON_IsNumber(cursor)){
				ESP_ERROR_CHECK(nvs_set_i32(nvs_config, "ws_port", (int32_t) cursor->valuedouble));
			}
			cursor = cJSON_GetObjectItemCaseSensitive(jsonObj, "recon");
			if (cJSON_IsNumber(cursor)){
				ESP_ERROR_CHECK(nvs_set_u8(nvs_config, "wifi_recon", (uint8_t) cursor->valuedouble));
			}
			cursor = cJSON_GetObjectItemCaseSensitive(jsonObj, "relog");
			if (cJSON_IsNumber(cursor)){
				ESP_ERROR_CHECK(nvs_set_u8(nvs_config, "relog", (uint8_t) cursor->valuedouble));
			}
			ESP_ERROR_CHECK(nvs_commit(nvs_config));

			// Clean up
			cJSON_Delete(jsonObj);

			// Set events to connect
			xEventGroupSetBits(app_event_group, WIFI_REQ_CONNECT_BIT | WS_REQ_CONNECT_BIT | DEVICE_REQ_LOGIN_BIT);
			xEventGroupClearBits(app_event_group, WIFI_REQ_DISCONNECT_BIT | WS_REQ_DISCONNECT_BIT | DEVICE_REQ_LOGOUT_BIT);

			break;
			}
	}

	// Clean up =================================================================
	free(buf);
	return ret;
}

static esp_err_t http_handler(httpd_req_t *req)
{
	/* Send a simple response */
	const char resp[] =
	"<!DOCTYPE html><html lang=\"en\"><head> <meta charset=\"UTF-8\"> <meta http-equiv=\"X-UA-Compatible\" content=\"IE=edge\"> <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\"> <title>Dashcam config</title></head><body onload=\"on_load();\"> <h1>Dashcam device config UI</h1> <h5>CE232 Project - Class 1 Group 1</h5> <hr> <div id=\"openning\"><h2>Connecting to device ...</h2></div> <div id=\"Config\" style=\"display: none;\"> <h3>Device status</h3> <table> <tr> <td> <span><b>Device ID:</b></span> <span id=\"id\"></span> </td> </tr> <tr> <td><b>WIFI status:</b></td> <td><b><span id=\"wf_st\"></span></b></td> </tr> <tr> <td><b>Service status:</b></td> <td><b><span id=\"sv_st\"></span></b></td> </tr> <tr> <td><b>Video status:</b></td> <td><b><span id=\"vid_st\"></span></b></td> </tr> </table> <input id=\"vid_but\" type=\"button\" onclick=\"on_vid_but();\"> <hr> <h3>WIFI setting</h3> <table> <tr> <td><label for=\"ssid\">SSID:</label></td> <td><input id=\"ssid\" type=\"text\"></td> </tr> <tr> <td><label for=\"pswd\">Password:</label></td> <td><input id=\"pswd\" type=\"text\"></td> </tr> </table> <label for=\"recon\">Auto reconnect</label> <input id=\"recon\" type=\"checkbox\"> <h3>WebSocket Client setting</h3> <table> <tr> <td><label for=\"uri\">URI: </label></td> <td><input id=\"uri\" type=\"text\"></td> </tr> <tr> <td><label for=\"port\">Port: </label></td> <td><input id=\"port\" type=\"text\"></td> </tr> </table> <label for=\"relog\">Auto login</label> <input id=\"relog\" type=\"checkbox\"> <br> <input id=\"cf_but\" type=\"button\" onclick=\"on_cf_but();\"> </div> <div id=\"closed\" style=\"display: none;\"><h2>Lost connect! Check wifi and refresh this page.</h2></div></body><script> var ws = null; function on_load() { ws = new WebSocket(\"ws://dashcam-config.local/ws\"); ws.binaryType = \'arraybuffer\'; ws.onopen = on_connect; ws.onmessage = on_message; ws.onerror = on_error; ws.onclose = on_error; } function on_connect() { ws.send(\"a\"); } function on_vid_but() { ws.send(\"v\"); } function on_cf_but() { if (document.getElementById(\"sv_st\").value > 0 || document.getElementById(\"wf_st\").value > 0){ ws.send(\"d\"); } else{ ws.send(JSON.stringify( { \"ssid\": document.getElementById(\"ssid\").value, \"pswd\": document.getElementById(\"pswd\").value, \"recon\": (+document.getElementById(\"recon\").checked), \"uri\": document.getElementById(\"uri\").value, \"port\": new Number(document.getElementById(\"port\").value), \"relog\": (+document.getElementById(\"relog\").checked) } )); } } function on_message(recv_data) { try { const data = JSON.parse(recv_data.data); var wf_st = document.getElementById(\"wf_st\"); var sv_st = document.getElementById(\"sv_st\"); var vid_st = document.getElementById(\"vid_st\"); var ssid = document.getElementById(\"ssid\"); var pswd = document.getElementById(\"pswd\"); var recon = document.getElementById(\"recon\"); var uri = document.getElementById(\"uri\"); var port = document.getElementById(\"port\"); var relog = document.getElementById(\"relog\"); document.getElementById(\"id\").textContent = data.id; wf_st.value = data.wf_st; sv_st.value = data.sv_st; vid_st.value = data.vid_st; wf_st.textContent = wf_st.value > 0 ? \"Connected\" : \"Disconnected\"; wf_st.style.color = wf_st.value > 0 ? \"green\" : \"crimson\"; sv_st.textContent = sv_st.value == 2 ? \"Logged in\" : (sv_st.value == 1 ? \"Connected - Not logged in\" : \"Not connected\"); sv_st.style.color = sv_st.value == 2 ? \"green\" : (sv_st.value == 1 ? \"darkorange\" : \"crimson\"); vid_st.textContent = vid_st.value > 0 ? \"Recording\" : \"Idle\"; vid_st.style.color = vid_st.value > 0 ? \"blue\" : \"black\"; document.getElementById(\"cf_but\").value = (wf_st.value > 0 || sv_st.value > 0) ? \"Disconnect\" : \"Connect\"; document.getElementById(\"vid_but\").value = vid_st.value > 0 ? \"Stop recording\" : \"Start recording\"; var val = wf_st.value > 0 || sv_st.value > 0 || vid_st.value > 0; ssid.disabled = val; pswd.disabled = val; recon.disabled = val; uri.disabled = val; port.disabled = val; relog.disabled = val; document.getElementById(\"cf_but\").disabled = vid_st.value > 0; document.getElementById(\"vid_but\").disabled = wf_st.value != 1 || sv_st.value != 2; if (val){ ssid.value = data.ssid; recon.checked = data.recon; uri.value = data.uri; port.value = data.port; relog.checked = data.relog; } document.getElementById(\'openning\').style.display = \'none\'; document.getElementById(\'Config\').style.display = \'inline\'; } catch (e) { return; } } function on_error(recv_data) { console.log(\"connection lost\"); document.getElementById(\'Config\').style.display = \'none\'; document.getElementById(\'closed\').style.display = \'inline\'; }</script></html>";
	httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
	return ESP_OK;
}

/* Function to start the web server */
esp_err_t start_webserver(void)
{
	httpd_handle_t server = NULL;
	httpd_config_t config = HTTPD_DEFAULT_CONFIG();
	config.stack_size = 8192;

	// Start the httpd server
	ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
	if (httpd_start(&server, &config) != ESP_OK) {
		ESP_LOGE(TAG, "Failed to starting server!");
		return ESP_FAIL;
	}

	// Registering http handler (to get index.html)
	ESP_LOGI(TAG, "Registering HTTP URI handlers");
	httpd_uri_t http = {
		.uri		= "/",
		.method		= HTTP_GET,
		.handler	= http_handler,
		.user_ctx	= NULL
	};
	httpd_register_uri_handler(server, &http);

	// Registering the ws handler
	ESP_LOGI(TAG, "Registering WS URI handlers");
	httpd_uri_t ws = {
		.uri		= "/ws",
		.method		= HTTP_GET,
		.handler	= ws_handler,
		.user_ctx	= NULL,
		.is_websocket = true,
	};
	httpd_register_uri_handler(server, &ws);

	return ESP_OK;
}
