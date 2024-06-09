set(SRC_FILES "esp_websocket_client.c" "main.c" "webserver.c") # Define source files
set(INCLUDE_DIRS ".") # Define include directories
set(EMBED_FILES "") # Initialize an empty list for files to embed

# Conditionally append files to the list based on configuration
if(CONFIG_WS_OVER_TLS_MUTAL_AUTH)
    list(APPEND EMBED_FILES
        "certs/client_cert.pem"
        "certs/ca_cert.pem"
        "certs/client_key.pem")
endif()

# For testing purpose we are using CA of wss://echo.websocket.events
if(CONFIG_WS_OVER_TLS_SERVER_AUTH)
    list(APPEND EMBED_FILES
        "certs/ca_certificate_public_domain.pem")
endif()

idf_component_register(SRCS "nmea_parser.c" "${SRC_FILES}"
                    INCLUDE_DIRS "${INCLUDE_DIRS}"
                    EMBED_TXTFILES "${EMBED_FILES}"
                    REQUIRED_IDF_TARGETS esp32)
