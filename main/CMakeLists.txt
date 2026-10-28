idf_component_register(
  SRCS "main.cpp"
  INCLUDE_DIRS "."
  EMBED_FILES "provisioning-bg.jpg"
  REQUIRES esp_wifi esp_event esp_netif nvs_flash esp_http_client esp_http_server
           mbedtls
)