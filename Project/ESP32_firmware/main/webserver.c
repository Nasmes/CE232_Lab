/*
	Take a picture and Publish it via Web Socket.

	This code is in the Public Domain (or CC0 licensed, at your option.)

	Unless required by applicable law or agreed to in writing, this
	software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
	CONDITIONS OF ANY KIND, either express or implied.

	I based from here:
	https://github.com/espressif/esp-idf/tree/master/examples/protocols/http_server/ws_echo_server
*/

#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <sys/stat.h>
#include <mbedtls/base64.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs.h"

#include "esp_camera.h"

#include "esp_http_server.h"

static const char *TAG = "WS_SERVER";

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
	// Echo back
	ret = httpd_ws_send_frame(req, &ws_pkt);
	if (ret != ESP_OK) {
			ESP_LOGE(TAG, "httpd_ws_send_frame failed with %d", ret);
	}

	// Clean up =================================================================
	free(buf);
	return ret;
}

static esp_err_t http_handler(httpd_req_t *req)
{
	/* Send a simple response */
	const char resp[] =
	"\
	<html>\
		<head>\
		</head>\
		<body onload=\"on_load();\">\
			<div id=\"connect\">\
				<input type=\"button\" value=\"connect server\" onclick=\"on_connect_server();\">\
				<input type=\"text\" id=\"connect_input\" name=\"connect_input\" value=\"dashcam-config\">\
			</div>\
			<div id=\"take\">\
				<input type=\"button\" value=\"take picture\" onclick=\"on_take_picture();\">\
			</div>\
			<div id=\"fail\">\
				Server connection fail.\
			</div>\
			<div>\
				<canvas id=\"canvas_image\" width=\"640\" height=\"480\"></canvas>\
			</div>\
		</body>\
		<script>\
			var web_socket = null;\
			function on_load()\
			{\
				document.getElementById(\'take\').style.display = \'none\';\
				document.getElementById(\'fail\').style.display = \'none\';\
			}\
			function on_connect_server()\
			{\
				var text = document.getElementById(\'connect_input\');\
				console.log(text.value);\
				var connection = \"ws://\" + text.value + \":80/ws\";\
				console.log(connection);\
				web_socket = new WebSocket(connection);\
				web_socket.binaryType = \'arraybuffer\';\
				web_socket.onmessage = on_message;\
				web_socket.onerror = on_error;\
				document.getElementById(\'take\').style.display = \'inline\';\
				document.getElementById(\'connect\').style.display = \'none\';\
			};\
			function on_take_picture()\
			{\
				var text_input = \"capture\";\
				web_socket.send(text_input);\
			}\
			function on_error(recv_data)\
			{\
				console.log(\"on_error\");\
				document.getElementById(\'take\').style.display = \'none\';\
				document.getElementById(\'fail\').style.display = \'inline\';\
			}\
			function on_message(recv_data)\
			{\
				console.log(\"on_message\");\
				console.log(recv_data.data);\
				console.log(typeof(recv_data.data));\
				console.log(recv_data.data.length);\
				var recv_image_data = new Uint8Array(recv_data.data);\
				var canvas_image = document.getElementById(\'canvas_image\');\
				var ctx = canvas_image.getContext(\'2d\');\
				var image = new Image();\
				image.src = \'data:image/jpeg;base64,\' + recv_data.data;\
				image.onload = function() {\
					ctx.drawImage(image, 0, 0);\
				}\
			}\
		</script>\
	</html>\
	";
	httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
	return ESP_OK;
}

/* Function to start the web server */
esp_err_t start_webserver(void)
{
	httpd_handle_t server = NULL;
	httpd_config_t config = HTTPD_DEFAULT_CONFIG();

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
		.is_websocket = true
	};
	httpd_register_uri_handler(server, &ws);

	return ESP_OK;
}
