#include <algorithm>
#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_crt_bundle.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "NimBLEDevice.h"

#include "usb/cdc_acm_host.h"
#include "usb/cdc_host_types.h"
#include "usb/usb_host.h"
#include "usb/vcp_ch34x.h"
#include "usb/vcp_cp210x.h"
#include "usb/vcp_ftdi.h"

#include "gateway_config.h"

extern "C" {
extern const uint8_t provisioning_bg_start[] asm("_binary_provisioning_bg_jpg_start");
extern const uint8_t provisioning_bg_end[] asm("_binary_provisioning_bg_jpg_end");
}

namespace {

constexpr char TAG[] = "DSF-GATEWAY";
constexpr int USB_HOST_PRIORITY = 20;
constexpr int USB_TX_TIMEOUT_MS = 1000;
constexpr int MODBUS_POLL_INTERVAL_MS = 2000;
constexpr size_t RX_CAPACITY = 256;
constexpr uint8_t USB_HUB_DEVICE_CLASS = 0x09;

constexpr char NUS_SERVICE_UUID[] = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
constexpr char NUS_RX_UUID[]      = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";
constexpr char NUS_TX_UUID[]      = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";

constexpr EventBits_t WIFI_CONNECTED_BIT = BIT0;
constexpr EventBits_t WIFI_FAILED_BIT = BIT1;
constexpr char NVS_NAMESPACE[] = "dsf_gateway";
constexpr char NVS_WIFI_SSID[] = "wifi_ssid";
constexpr char NVS_WIFI_PASSWORD[] = "wifi_password";
constexpr char NVS_API_KEY[] = "api_key";
constexpr gpio_num_t SETUP_BUTTON_GPIO = GPIO_NUM_0;

struct SensorReading {
  float temperature_c = 0;
  float moisture_percent = 0;
  uint16_t ec_us_cm = 0;
  float ph = 0;
  uint16_t nitrogen_mg_kg = 0;
  uint16_t phosphorus_mg_kg = 0;
  uint16_t potassium_mg_kg = 0;
  uint16_t fertility = 0;
  bool has_fertility = false;
  bool valid = false;
  uint32_t sequence = 0;
};

struct ProvisioningConfig {
  std::string wifi_ssid;
  std::string wifi_password;
  std::string api_key;
};

enum class AppMessageId {
  UsbConnected,
  UsbDisconnected,
};

struct AppMessage {
  AppMessageId id;
  uint16_t vid;
  uint16_t pid;
};

QueueHandle_t app_queue = nullptr;
SemaphoreHandle_t cdc_mutex = nullptr;
SemaphoreHandle_t reading_mutex = nullptr;
EventGroupHandle_t wifi_events = nullptr;
httpd_handle_t provisioning_server = nullptr;

cdc_acm_dev_hdl_t cdc_device = nullptr;
NimBLECharacteristic *ble_tx_characteristic = nullptr;

uint8_t rx_buffer[RX_CAPACITY] = {};
size_t rx_size = 0;
SensorReading latest_reading;
volatile uint8_t detected_register_count = 0;

// Một số cảm biến JXBS-3001 không có 7 thanh ghi liền nhau:
// block 0x0000/4 chứa ẩm, nhiệt độ, EC, pH và block 0x001E/3 chứa N/P/K.
// Hai cờ này chỉ được dùng trong callback USB, sau đó ghép thành một reading.
SensorReading split_reading;
bool split_environment_received = false;
bool split_nutrients_received = false;

bool nvs_read_string(nvs_handle_t handle, const char *key, std::string &out) {
  size_t length = 0;
  if (nvs_get_str(handle, key, nullptr, &length) != ESP_OK || length < 2) {
    return false;
  }
  std::string value(length, '\0');
  if (nvs_get_str(handle, key, value.data(), &length) != ESP_OK) return false;
  value.resize(length > 0 ? length - 1 : 0);
  out = value;
  return !out.empty();
}

bool load_provisioning_config(ProvisioningConfig &config) {
  nvs_handle_t handle = 0;
  if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) return false;
  const bool loaded =
      nvs_read_string(handle, NVS_WIFI_SSID, config.wifi_ssid) &&
      nvs_read_string(handle, NVS_WIFI_PASSWORD, config.wifi_password) &&
      nvs_read_string(handle, NVS_API_KEY, config.api_key);
  nvs_close(handle);
  return loaded;
}

bool save_provisioning_config(const ProvisioningConfig &config) {
  nvs_handle_t handle = 0;
  if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) return false;
  esp_err_t error = nvs_set_str(handle, NVS_WIFI_SSID, config.wifi_ssid.c_str());
  if (error == ESP_OK) error = nvs_set_str(handle, NVS_WIFI_PASSWORD, config.wifi_password.c_str());
  if (error == ESP_OK) error = nvs_set_str(handle, NVS_API_KEY, config.api_key.c_str());
  if (error == ESP_OK) error = nvs_commit(handle);
  nvs_close(handle);
  return error == ESP_OK;
}

bool setup_button_is_held() {
  gpio_config_t button_config = {};
  button_config.pin_bit_mask = 1ULL << SETUP_BUTTON_GPIO;
  button_config.mode = GPIO_MODE_INPUT;
  button_config.pull_up_en = GPIO_PULLUP_ENABLE;
  button_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
  button_config.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&button_config);
  vTaskDelay(pdMS_TO_TICKS(50));
  return gpio_get_level(SETUP_BUTTON_GPIO) == 0;
}

uint16_t modbus_crc16(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) ? static_cast<uint16_t>((crc >> 1U) ^ 0xA001U)
                       : static_cast<uint16_t>(crc >> 1U);
    }
  }
  return crc;
}

size_t build_modbus_poll(uint8_t out[8], uint16_t start_register,
                         uint8_t register_count) {
  out[0] = DSF_MODBUS_ADDRESS;
  out[1] = 0x03;
  out[2] = static_cast<uint8_t>(start_register >> 8);
  out[3] = static_cast<uint8_t>(start_register & 0xFF);
  out[4] = 0;
  out[5] = register_count;
  const uint16_t crc = modbus_crc16(out, 6);
  out[6] = static_cast<uint8_t>(crc & 0xFF);
  out[7] = static_cast<uint8_t>(crc >> 8);
  return 8;
}

uint16_t read_register(const uint8_t *frame, int index) {
  const size_t offset = 3 + static_cast<size_t>(index) * 2;
  return static_cast<uint16_t>(
      (static_cast<uint16_t>(frame[offset]) << 8) | frame[offset + 1]);
}

enum class ModbusFrameKind {
  Invalid,
  Full,
  Environment,
  Nutrients,
};

ModbusFrameKind parse_modbus_frame(const uint8_t *frame, size_t length,
                                   SensorReading &out) {
  if (length < 11 || frame[0] != DSF_MODBUS_ADDRESS || frame[1] != 0x03) {
    return ModbusFrameKind::Invalid;
  }

  const uint8_t byte_count = frame[2];
  const size_t expected_length = static_cast<size_t>(byte_count) + 5;
  if (length != expected_length ||
      (byte_count != 0x06 && byte_count != 0x08 &&
       byte_count != 0x0E && byte_count != 0x10)) {
    return ModbusFrameKind::Invalid;
  }

  const uint16_t expected_crc = modbus_crc16(frame, length - 2);
  const uint16_t received_crc =
      static_cast<uint16_t>(frame[length - 2]) |
      (static_cast<uint16_t>(frame[length - 1]) << 8);
  if (expected_crc != received_crc) return ModbusFrameKind::Invalid;

  out = SensorReading{};
  // Register order used by the supported probes is:
  // moisture, temperature, EC, pH, N, P, K, fertility.
  if (byte_count >= 0x08) {
    out.moisture_percent = read_register(frame, 0) / 10.0F;
    out.temperature_c = static_cast<int16_t>(read_register(frame, 1)) / 10.0F;
    out.ec_us_cm = read_register(frame, 2);
    out.ph = read_register(frame, 3) / 10.0F;
  }
  if (byte_count >= 0x0E) {
    out.nitrogen_mg_kg = read_register(frame, 4);
    out.phosphorus_mg_kg = read_register(frame, 5);
    out.potassium_mg_kg = read_register(frame, 6);
    out.has_fertility = byte_count == 0x10;
    out.fertility = out.has_fertility ? read_register(frame, 7) : 0;
  } else if (byte_count == 0x06) {
    out.nitrogen_mg_kg = read_register(frame, 0);
    out.phosphorus_mg_kg = read_register(frame, 1);
    out.potassium_mg_kg = read_register(frame, 2);
  }

  // Dải vật lý của cảm biến đang được Sensor App hỗ trợ.
  if (byte_count == 0x08 || byte_count >= 0x0E) {
    if (out.temperature_c < -40 || out.temperature_c > 85 ||
        out.moisture_percent < 0 || out.moisture_percent > 100 ||
        out.ec_us_cm > 2000 || out.ph < 0 || out.ph > 14) {
      return ModbusFrameKind::Invalid;
    }
  }

  // Payload mặc định khi đầu dò chưa cắm vào đất, không phải phép đo thật.
  if (byte_count >= 0x08 &&
      out.temperature_c <= -19.5F &&
      out.moisture_percent >= 99.5F &&
      out.ec_us_cm == 0 &&
      out.ph == 4.0F &&
      out.nitrogen_mg_kg == 0 &&
      out.phosphorus_mg_kg == 0 &&
      out.potassium_mg_kg == 0) {
    ESP_LOGW(TAG, "Cam bien chua san sang/khong cam vao dat");
    return ModbusFrameKind::Invalid;
  }

  if (byte_count == 0x10 || byte_count == 0x0E) {
    detected_register_count = byte_count == 0x10 ? 8 : 7;
    out.valid = true;
    return ModbusFrameKind::Full;
  }
  if (byte_count == 0x08) return ModbusFrameKind::Environment;
  return ModbusFrameKind::Nutrients;
}

void reading_to_ble_json(const SensorReading &reading, char *out, size_t capacity) {
  std::snprintf(
      out,
      capacity,
      "{\"temperature\":%.1f,\"moisture\":%.1f,\"ec\":%u,"
      "\"ph\":%.1f,\"nitrogen\":%u,\"phosphorus\":%u,"
      "\"potassium\":%u,\"fertility\":%u}\n",
      reading.temperature_c,
      reading.moisture_percent,
      reading.ec_us_cm,
      reading.ph,
      reading.nitrogen_mg_kg,
      reading.phosphorus_mg_kg,
      reading.potassium_mg_kg,
      reading.fertility);
}

void reading_to_server_json(const SensorReading &reading, char *out, size_t capacity) {
  std::snprintf(
      out,
      capacity,
      "{\"soilMoisture\":%.1f,\"soilTemperature\":%.1f,"
      "\"soilPh\":%.1f,\"soilEc\":%u,"
      "\"depths\":[{\"depth\":15,\"moisture\":%.1f,"
      "\"temperature\":%.1f,\"ec\":%u,\"ph\":%.1f,"
      "\"n\":%u,\"p\":%u,\"k\":%u}]}",
      reading.moisture_percent,
      reading.temperature_c,
      reading.ph,
      reading.ec_us_cm,
      reading.moisture_percent,
      reading.temperature_c,
      reading.ec_us_cm,
      reading.ph,
      reading.nitrogen_mg_kg,
      reading.phosphorus_mg_kg,
      reading.potassium_mg_kg);
}

void publish_reading(const SensorReading &parsed) {
  SensorReading reading = parsed;
  xSemaphoreTake(reading_mutex, portMAX_DELAY);
  reading.sequence = latest_reading.sequence + 1;
  latest_reading = reading;
  xSemaphoreGive(reading_mutex);

  char json[256];
  reading_to_ble_json(reading, json, sizeof(json));
  ESP_LOGI(TAG, "Reading #%lu: %s",
           static_cast<unsigned long>(reading.sequence), json);

  if (ble_tx_characteristic != nullptr) {
    // Sensor App ghép các notification vào bộ đệm cho tới dấu xuống dòng.
    // Chia 18 byte để vẫn chạy với MTU BLE mặc định 23 byte.
    const auto *bytes = reinterpret_cast<const uint8_t *>(json);
    const size_t length = std::strlen(json);
    constexpr size_t safe_payload = 18;
    for (size_t offset = 0; offset < length; offset += safe_payload) {
      const size_t chunk_length = std::min(safe_payload, length - offset);
      if (!ble_tx_characteristic->notify(bytes + offset, chunk_length)) {
        ESP_LOGD(TAG, "BLE chua co client dang nhan notification");
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(12));
    }
  }
}

void consume_rx_buffer() {
  while (rx_size >= 5) {
    size_t start = 0;
    while (start < rx_size &&
           !(rx_buffer[start] == DSF_MODBUS_ADDRESS &&
             start + 1 < rx_size &&
             (rx_buffer[start + 1] == 0x03 ||
              rx_buffer[start + 1] == 0x83))) {
      ++start;
    }

    if (start > 0) {
      std::memmove(rx_buffer, rx_buffer + start, rx_size - start);
      rx_size -= start;
    }
    if (rx_size < 5) return;

    // Log exception frames instead of silently discarding them.  This makes
    // a wrong register map / slave address visible in the serial monitor.
    if (rx_buffer[1] == 0x83) {
      const uint16_t expected_crc = modbus_crc16(rx_buffer, 3);
      const uint16_t received_crc =
          static_cast<uint16_t>(rx_buffer[3]) |
          (static_cast<uint16_t>(rx_buffer[4]) << 8);
      if (expected_crc == received_crc) {
        ESP_LOGW(TAG, "Cam bien tu choi Modbus: ma loi 0x%02X",
                 rx_buffer[2]);
        std::memmove(rx_buffer, rx_buffer + 5, rx_size - 5);
        rx_size -= 5;
        continue;
      }
    }

    if (rx_buffer[1] != 0x03) {
      std::memmove(rx_buffer, rx_buffer + 1, --rx_size);
      continue;
    }

    const uint8_t byte_count = rx_buffer[2];
    if (byte_count != 0x06 && byte_count != 0x08 &&
        byte_count != 0x0E && byte_count != 0x10) {
      std::memmove(rx_buffer, rx_buffer + 1, --rx_size);
      continue;
    }

    const size_t frame_length = static_cast<size_t>(byte_count) + 5;
    if (rx_size < frame_length) return;

    SensorReading parsed;
    const ModbusFrameKind kind =
        parse_modbus_frame(rx_buffer, frame_length, parsed);
    if (kind == ModbusFrameKind::Full) {
      split_environment_received = false;
      split_nutrients_received = false;
      ESP_LOGI(TAG, "Nhan khung Modbus day du: %u thanh ghi", byte_count / 2);
      publish_reading(parsed);
    } else if (kind == ModbusFrameKind::Environment) {
      split_reading.moisture_percent = parsed.moisture_percent;
      split_reading.temperature_c = parsed.temperature_c;
      split_reading.ec_us_cm = parsed.ec_us_cm;
      split_reading.ph = parsed.ph;
      split_environment_received = true;
      ESP_LOGI(TAG, "Nhan block 0x0000: am=%.1f nhiet=%.1f EC=%u pH=%.1f",
               parsed.moisture_percent, parsed.temperature_c,
               parsed.ec_us_cm, parsed.ph);
    } else if (kind == ModbusFrameKind::Nutrients) {
      split_reading.nitrogen_mg_kg = parsed.nitrogen_mg_kg;
      split_reading.phosphorus_mg_kg = parsed.phosphorus_mg_kg;
      split_reading.potassium_mg_kg = parsed.potassium_mg_kg;
      split_reading.has_fertility = false;
      split_reading.fertility = 0;
      split_nutrients_received = true;
      ESP_LOGI(TAG, "Nhan block 0x001E: N=%u P=%u K=%u",
               parsed.nitrogen_mg_kg, parsed.phosphorus_mg_kg,
               parsed.potassium_mg_kg);
    } else {
      ESP_LOGW(TAG, "Khung Modbus sai CRC/dinh dang; bo 1 byte");
      std::memmove(rx_buffer, rx_buffer + 1, --rx_size);
      continue;
    }

    if (split_environment_received && split_nutrients_received) {
      split_reading.valid = true;
      publish_reading(split_reading);
      split_environment_received = false;
      split_nutrients_received = false;
    }

    std::memmove(rx_buffer, rx_buffer + frame_length, rx_size - frame_length);
    rx_size -= frame_length;
  }
}

bool handle_usb_rx(const uint8_t *data, size_t data_length, void *) {
  if (data_length == 0) return true;

  const size_t free_space = RX_CAPACITY - rx_size;
  if (data_length > free_space) {
    ESP_LOGW(TAG, "Tran bo dem USB; xoa khung cu");
    rx_size = 0;
  }

  const size_t copy_length = std::min(data_length, RX_CAPACITY - rx_size);
  std::memcpy(rx_buffer + rx_size, data, copy_length);
  rx_size += copy_length;
  consume_rx_buffer();
  return true;
}

void handle_usb_event(const cdc_acm_host_dev_event_data_t *event, void *) {
  if (event->type == CDC_ACM_HOST_DEVICE_DISCONNECTED && app_queue != nullptr) {
    const AppMessage message{AppMessageId::UsbDisconnected, 0, 0};
    xQueueSend(app_queue, &message, 0);
  } else if (event->type == CDC_ACM_HOST_ERROR) {
    ESP_LOGE(TAG, "USB CDC error: %d", event->data.error);
  }
}

void handle_new_usb_device(usb_device_handle_t usb_device) {
  const usb_device_desc_t *descriptor = nullptr;
  if (usb_host_get_device_descriptor(usb_device, &descriptor) != ESP_OK) return;

  if (descriptor->bDeviceClass == USB_HUB_DEVICE_CLASS) {
    ESP_LOGI(TAG, "Da nhan USB hub: VID=0x%04X PID=0x%04X; cho cam bien o cong downstream",
             descriptor->idVendor, descriptor->idProduct);
    return;
  }

  ESP_LOGI(TAG, "Thiet bi USB: VID=0x%04X PID=0x%04X class=0x%02X",
           descriptor->idVendor, descriptor->idProduct,
           descriptor->bDeviceClass);
  if (app_queue != nullptr) {
    const AppMessage message{
        AppMessageId::UsbConnected, descriptor->idVendor, descriptor->idProduct};
    xQueueSend(app_queue, &message, 0);
  }
}

cdc_acm_dev_hdl_t open_usb_serial(uint16_t vid, uint16_t pid) {
  cdc_acm_host_device_config_t config = {
      .connection_timeout_ms = 5000,
      .out_buffer_size = 64,
      .in_buffer_size = 512,
      .event_cb = handle_usb_event,
      .data_cb = handle_usb_rx,
      .user_arg = nullptr,
  };

  cdc_acm_dev_hdl_t handle = nullptr;
  esp_err_t error = ESP_FAIL;
  switch (vid) {
    case FTDI_VID:
      error = ftdi_vcp_open(pid, 0, &config, &handle);
      break;
    case NANJING_QINHENG_MICROE_VID:
      error = ch34x_vcp_open(pid, 0, &config, &handle);
      break;
    case SILICON_LABS_VID:
      error = cp210x_vcp_open(pid, 0, &config, &handle);
      break;
    default:
      error = cdc_acm_host_open(vid, pid, 0, &config, &handle);
      break;
  }

  if (error != ESP_OK) {
    ESP_LOGE(TAG, "Khong mo duoc USB VID=0x%04X PID=0x%04X: %s",
             vid, pid, esp_err_to_name(error));
    return nullptr;
  }

  cdc_acm_line_coding_t line_coding = {
      .dwDTERate = DSF_SENSOR_BAUD_RATE,
      .bCharFormat = 0,
      .bParityType = 0,
      .bDataBits = 8,
  };
  error = cdc_acm_host_line_coding_set(handle, &line_coding);
  if (error != ESP_OK && error != ESP_ERR_NOT_SUPPORTED) {
    ESP_LOGW(TAG, "Khong dat duoc 9600 8N1: %s", esp_err_to_name(error));
  }

  cdc_acm_host_set_control_line_state(handle, true, false);
  ESP_LOGI(TAG, "Da mo cam bien USB o %d baud, 8N1", DSF_SENSOR_BAUD_RATE);
  return handle;
}

void usb_host_task(void *main_task) {
  const usb_host_config_t host_config = {
      .skip_phy_setup = false,
      .intr_flags = ESP_INTR_FLAG_LOWMED,
  };
  ESP_ERROR_CHECK(usb_host_install(&host_config));

  const cdc_acm_host_driver_config_t driver_config = {
      .driver_task_stack_size = 4096,
      .driver_task_priority = USB_HOST_PRIORITY + 1,
      .xCoreID = 0,
      .new_dev_cb = handle_new_usb_device,
  };
  ESP_ERROR_CHECK(cdc_acm_host_install(&driver_config));
  xTaskNotifyGive(static_cast<TaskHandle_t>(main_task));

  while (true) {
    uint32_t event_flags = 0;
    usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
  }
}

void modbus_poll_task(void *) {
  uint8_t query[8];
  struct PollQuery {
    uint16_t start_register;
    uint8_t register_count;
    const char *label;
  };
  constexpr PollQuery probe_queries[] = {
      {DSF_MODBUS_START_REG, DSF_MODBUS_REGISTER_COUNT, "full 8"},
      {DSF_MODBUS_START_REG, 7, "full 7"},
      {DSF_MODBUS_START_REG, DSF_MODBUS_ENV_REGISTER_COUNT, "block am/n hiet/EC/pH"},
      {DSF_MODBUS_NPK_START_REG, DSF_MODBUS_NPK_REGISTER_COUNT, "block N/P/K"},
  };
  size_t probe_index = 0;

  while (true) {
    PollQuery query_config;
    if (detected_register_count == 7 || detected_register_count == 8) {
      query_config = {DSF_MODBUS_START_REG, detected_register_count,
                      detected_register_count == 8 ? "full 8" : "full 7"};
    } else {
      query_config = probe_queries[probe_index];
      probe_index = (probe_index + 1) % (sizeof(probe_queries) / sizeof(probe_queries[0]));
    }

    const size_t query_length = build_modbus_poll(
        query, query_config.start_register, query_config.register_count);
    xSemaphoreTake(cdc_mutex, portMAX_DELAY);
    cdc_acm_dev_hdl_t handle = cdc_device;
    if (handle != nullptr) {
      ESP_LOGI(TAG, "Gui poll: reg=0x%04X count=%u (%s)",
               query_config.start_register, query_config.register_count,
               query_config.label);
      const esp_err_t error = cdc_acm_host_data_tx_blocking(
          handle, query, query_length, USB_TX_TIMEOUT_MS);
      if (error != ESP_OK) {
        ESP_LOGW(TAG, "Gui lenh Modbus that bai: %s", esp_err_to_name(error));
      }
    }
    xSemaphoreGive(cdc_mutex);
    vTaskDelay(pdMS_TO_TICKS(MODBUS_POLL_INTERVAL_MS));
  }
}

class GatewayServerCallbacks final : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *, NimBLEConnInfo &) override {
    ESP_LOGI(TAG, "Dien thoai da ket noi BLE");
  }

  void onDisconnect(NimBLEServer *server, NimBLEConnInfo &, int) override {
    ESP_LOGI(TAG, "Dien thoai da ngat BLE");
    server->startAdvertising();
  }
};

void init_ble() {
  uint8_t mac[6] = {};
  esp_read_mac(mac, ESP_MAC_BT);
  char name[24];
  std::snprintf(name, sizeof(name), "DURIAN-SENSOR-%02X%02X", mac[4], mac[5]);

  NimBLEDevice::init(name);
  // Set the GAP/GATT device name explicitly as well as the advertising name.
  // iOS may display the Generic Access name instead of the scan-response name.
  NimBLEDevice::setDeviceName(name);
  NimBLEDevice::setMTU(247);

  NimBLEServer *server = NimBLEDevice::createServer();
  server->setCallbacks(new GatewayServerCallbacks());

  NimBLEService *service = server->createService(NUS_SERVICE_UUID);
  ble_tx_characteristic = service->createCharacteristic(
      NUS_TX_UUID, NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);
  service->createCharacteristic(
      NUS_RX_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  service->start();

  NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
  advertising->addServiceUUID(NUS_SERVICE_UUID);
  advertising->setName(name);
  advertising->enableScanResponse(true);
  advertising->start();
  ESP_LOGI(TAG, "BLE san sang: %s", name);
}

int hex_digit(char value) {
  if (value >= '0' && value <= '9') return value - '0';
  if (value >= 'a' && value <= 'f') return value - 'a' + 10;
  if (value >= 'A' && value <= 'F') return value - 'A' + 10;
  return -1;
}

std::string url_decode(const std::string &encoded) {
  std::string decoded;
  decoded.reserve(encoded.size());
  for (size_t i = 0; i < encoded.size(); ++i) {
    if (encoded[i] == '+') {
      decoded.push_back(' ');
    } else if (encoded[i] == '%' && i + 2 < encoded.size()) {
      const int high = hex_digit(encoded[i + 1]);
      const int low = hex_digit(encoded[i + 2]);
      if (high >= 0 && low >= 0) {
        decoded.push_back(static_cast<char>((high << 4) | low));
        i += 2;
      } else {
        decoded.push_back(encoded[i]);
      }
    } else {
      decoded.push_back(encoded[i]);
    }
  }
  return decoded;
}

bool get_form_field(const std::string &body, const char *field,
                    std::string &value) {
  const std::string prefix = std::string(field) + "=";
  size_t start = 0;
  while (start < body.size()) {
    const size_t end = body.find('&', start);
    const size_t length = end == std::string::npos ? body.size() - start : end - start;
    if (body.compare(start, prefix.size(), prefix) == 0) {
      value = url_decode(body.substr(start + prefix.size(), length - prefix.size()));
      return true;
    }
    if (end == std::string::npos) break;
    start = end + 1;
  }
  return false;
}

constexpr char PROVISIONING_PAGE[] = R"html(
<!doctype html><html lang="vi"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="theme-color" content="#064e3b">
<title>Cấu hình Durian Smart Sensor</title>
<style>
*{box-sizing:border-box}html,body{margin:0;min-height:100%;font-family:Arial,sans-serif}
body{min-height:100vh;padding:238px 16px 28px;background:#dcefdc url('/background.jpg')
center top/cover fixed no-repeat;color:#fff}
.panel{max-width:480px;margin:0 auto;padding:20px;border:1px solid rgba(255,255,255,.3);
border-radius:18px;background:rgba(3,31,19,.92);box-shadow:0 16px 48px rgba(0,0,0,.48);
backdrop-filter:blur(5px)}
.eyebrow{margin:0 0 5px;color:#86efac;font-size:12px;font-weight:800;letter-spacing:.12em;
text-transform:uppercase}.title{margin:0;font-size:24px;line-height:1.2;color:#fff}
.subtitle{margin:8px 0 16px;color:#d1fae5;font-size:14px;line-height:1.5}
.note{padding:12px 13px;border:1px solid rgba(134,239,172,.35);border-radius:10px;
background:rgba(6,78,59,.72);color:#ecfdf5;font-size:13px;line-height:1.5}
label{display:block;margin-top:16px;color:#fff;font-size:14px;font-weight:700}
.hint{display:block;margin-top:4px;color:#a7f3d0;font-size:11px;font-weight:400}
input{display:block;width:100%;margin-top:7px;padding:13px 12px;border:2px solid #d1fae5;
border-radius:10px;background:#fff;color:#10251a;font-size:16px;outline:none}
input:focus{border-color:#4ade80;box-shadow:0 0 0 3px rgba(74,222,128,.24)}
input::placeholder{color:#64748b}input[readonly]{border-color:#86efac;background:#ecfdf5;
color:#065f46;font-weight:700}
button{width:100%;margin-top:22px;padding:14px;border:0;border-radius:10px;
background:#22c55e;color:#062d19;font-size:16px;font-weight:800;box-shadow:0 4px 0 #15803d}
button:active{transform:translateY(2px);box-shadow:0 2px 0 #15803d}
.privacy{margin:14px 0 0;color:#bbf7d0;font-size:11px;line-height:1.45;text-align:center}
@media(max-height:760px){body{padding-top:190px;background-position:center top}.panel{padding:17px}}
</style></head><body><main class="panel">
<p class="eyebrow">Durian Smart Sensor 8 in 1</p>
<h1 class="title">Cấu hình kết nối vườn</h1>
<p class="subtitle">Nhập Wi-Fi tại vườn để thiết bị tự động gửi dữ liệu lên Durian Smart Farm.</p>
<div class="note" id="setup-note">Thiết bị chỉ cần cấu hình một lần. Thông tin được lưu an toàn trong gateway.</div>
<form method="post" action="/save">
<label for="ssid">Tên Wi-Fi của vườn
<span class="hint">Chọn đúng mạng Wi-Fi có Internet tại vị trí đặt cảm biến.</span></label>
<input id="ssid" name="ssid" maxlength="32" required autocomplete="off" placeholder="Ví dụ: WiFi Vuon Sau Rieng">
<label for="password">Mật khẩu Wi-Fi
<span class="hint">Để trống nếu mạng Wi-Fi không có mật khẩu.</span></label>
<input id="password" name="password" type="password" maxlength="64" autocomplete="off" placeholder="Nhập mật khẩu Wi-Fi">
<label for="api-key">Mã kết nối thiết bị
<span class="hint">Mã này được tự động điền khi cấu hình từ tem QR.</span></label>
<input id="api-key" name="api_key" maxlength="64" required autocomplete="off" placeholder="Mã API của thiết bị">
<button type="submit">Lưu cấu hình và kết nối</button></form>
<p class="privacy">Gateway chỉ sử dụng thông tin này để kết nối Wi-Fi và gửi dữ liệu cảm biến của vườn.</p>
</main>
<script>
const key=new URLSearchParams(location.search).get("api_key");
if(key&&/^[A-Za-z0-9_-]{4,64}$/.test(key)){
  const input=document.getElementById("api-key");
  input.value=key;input.readOnly=true;
  document.getElementById("setup-note").textContent=
    "Đã nhận mã thiết bị từ tem QR. Bạn chỉ cần nhập tên và mật khẩu Wi-Fi của vườn.";
}
</script></body></html>
)html";

esp_err_t provisioning_background_handler(httpd_req_t *request) {
  const size_t image_size =
      static_cast<size_t>(provisioning_bg_end - provisioning_bg_start);
  httpd_resp_set_type(request, "image/jpeg");
  httpd_resp_set_hdr(request, "Cache-Control", "public, max-age=86400");
  return httpd_resp_send(
      request, reinterpret_cast<const char *>(provisioning_bg_start), image_size);
}

esp_err_t provisioning_index_handler(httpd_req_t *request) {
  httpd_resp_set_type(request, "text/html; charset=utf-8");
  return httpd_resp_send(request, PROVISIONING_PAGE, HTTPD_RESP_USE_STRLEN);
}

esp_err_t provisioning_save_handler(httpd_req_t *request) {
  if (request->content_len <= 0 || request->content_len > 1024) {
    httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "Form khong hop le");
    return ESP_FAIL;
  }

  std::string body(static_cast<size_t>(request->content_len), '\0');
  size_t received = 0;
  while (received < body.size()) {
    const int count = httpd_req_recv(
        request, body.data() + received, body.size() - received);
    if (count <= 0) {
      httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "Khong doc duoc form");
      return ESP_FAIL;
    }
    received += static_cast<size_t>(count);
  }
  body.resize(received);

  ProvisioningConfig config;
  if (!get_form_field(body, "ssid", config.wifi_ssid) ||
      !get_form_field(body, "password", config.wifi_password) ||
      !get_form_field(body, "api_key", config.api_key) ||
      config.wifi_ssid.empty() || config.api_key.empty()) {
    httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                        "Can nhap SSID va API key");
    return ESP_FAIL;
  }

  if (!save_provisioning_config(config)) {
    httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                        "Khong luu duoc cau hinh");
    return ESP_FAIL;
  }

  const char response[] =
      "<html><meta charset=\"utf-8\"><body><h2>Đã lưu cấu hình</h2>"
      "<p>Gateway đang khởi động lại. Hãy kết nối điện thoại về Wi-Fi "
      "của vườn rồi mở Sensor App.</p></body></html>";
  httpd_resp_set_type(request, "text/html; charset=utf-8");
  httpd_resp_send(request, response, HTTPD_RESP_USE_STRLEN);
  vTaskDelay(pdMS_TO_TICKS(1200));
  esp_restart();
  return ESP_OK;
}

void start_provisioning_portal() {
  ESP_LOGW(TAG, "Chua co cau hinh. Mo Wi-Fi %s de cai dat",
           DSF_PROVISION_AP_SSID);
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_ap();

  wifi_init_config_t wifi_init = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&wifi_init));

  wifi_config_t ap_config = {};
  std::strncpy(reinterpret_cast<char *>(ap_config.ap.ssid),
               DSF_PROVISION_AP_SSID, sizeof(ap_config.ap.ssid) - 1);
  ap_config.ap.ssid_len = std::strlen(DSF_PROVISION_AP_SSID);
  ap_config.ap.channel = 1;
  ap_config.ap.max_connection = 4;
  ap_config.ap.authmode = WIFI_AUTH_OPEN;
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  httpd_config_t server_config = HTTPD_DEFAULT_CONFIG();
  server_config.max_uri_handlers = 4;
  ESP_ERROR_CHECK(httpd_start(&provisioning_server, &server_config));
  const httpd_uri_t index_uri = {
      .uri = "/", .method = HTTP_GET, .handler = provisioning_index_handler,
      .user_ctx = nullptr};
  const httpd_uri_t save_uri = {
      .uri = "/save", .method = HTTP_POST, .handler = provisioning_save_handler,
      .user_ctx = nullptr};
  const httpd_uri_t background_uri = {
      .uri = "/background.jpg", .method = HTTP_GET,
      .handler = provisioning_background_handler, .user_ctx = nullptr};
  ESP_ERROR_CHECK(httpd_register_uri_handler(provisioning_server, &index_uri));
  ESP_ERROR_CHECK(httpd_register_uri_handler(provisioning_server, &save_uri));
  ESP_ERROR_CHECK(
      httpd_register_uri_handler(provisioning_server, &background_uri));

  ESP_LOGI(TAG, "Ket noi Wi-Fi %s, mo http://192.168.4.1",
           DSF_PROVISION_AP_SSID);
  while (true) vTaskDelay(pdMS_TO_TICKS(1000));
}

void wifi_event_handler(void *, esp_event_base_t event_base, int32_t event_id,
                        void *) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    xEventGroupClearBits(wifi_events, WIFI_CONNECTED_BIT);
    xEventGroupSetBits(wifi_events, WIFI_FAILED_BIT);
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    xEventGroupClearBits(wifi_events, WIFI_FAILED_BIT);
    xEventGroupSetBits(wifi_events, WIFI_CONNECTED_BIT);
    ESP_LOGI(TAG, "Wi-Fi da ket noi");
  }
}

bool init_wifi(const ProvisioningConfig &provisioning) {
  wifi_events = xEventGroupCreate();
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&init_config));
  ESP_ERROR_CHECK(esp_event_handler_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, nullptr));
  ESP_ERROR_CHECK(esp_event_handler_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, nullptr));

  wifi_config_t wifi_config = {};
  std::strncpy(reinterpret_cast<char *>(wifi_config.sta.ssid),
               provisioning.wifi_ssid.c_str(), sizeof(wifi_config.sta.ssid) - 1);
  std::strncpy(reinterpret_cast<char *>(wifi_config.sta.password),
               provisioning.wifi_password.c_str(),
               sizeof(wifi_config.sta.password) - 1);
  wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());
  return true;
}

bool upload_to_server(const SensorReading &reading, const std::string &api_key) {
  char body[384];
  reading_to_server_json(reading, body, sizeof(body));

  esp_http_client_config_t config = {};
  config.url = DSF_API_URL;
  config.method = HTTP_METHOD_POST;
  config.timeout_ms = 15000;
  config.crt_bundle_attach = esp_crt_bundle_attach;

  esp_http_client_handle_t client = esp_http_client_init(&config);
  if (client == nullptr) return false;

  esp_http_client_set_header(client, "Content-Type", "application/json");
  esp_http_client_set_header(client, "x-api-key", api_key.c_str());
  esp_http_client_set_post_field(client, body, std::strlen(body));

  const esp_err_t error = esp_http_client_perform(client);
  const int status = error == ESP_OK ? esp_http_client_get_status_code(client) : 0;
  esp_http_client_cleanup(client);

  if (error == ESP_OK && status >= 200 && status < 300) {
    ESP_LOGI(TAG, "Da gui du lieu len Durian Smart Farm (HTTP %d)", status);
    return true;
  }

  ESP_LOGW(TAG, "Upload that bai: %s, HTTP %d", esp_err_to_name(error), status);
  return false;
}

void wifi_upload_task(void *) {
  ProvisioningConfig provisioning;
  if (!load_provisioning_config(provisioning) || !init_wifi(provisioning)) {
    vTaskDelete(nullptr);
    return;
  }

  uint32_t last_uploaded_sequence = 0;
  uint32_t reconnect_delay_ms = 2000;
  while (true) {
    EventBits_t bits = xEventGroupGetBits(wifi_events);
    if ((bits & WIFI_CONNECTED_BIT) == 0) {
      ESP_LOGW(TAG, "Wi-Fi mat ket noi; thu lai sau %lu giay",
               static_cast<unsigned long>(reconnect_delay_ms / 1000));
      // Wait for reconnection instead of sleeping blindly. A Wi-Fi event can
      // restore the connection during the backoff; calling esp_wifi_connect()
      // afterward while already connected produces a misleading driver warning.
      const EventBits_t wait_bits = xEventGroupWaitBits(
          wifi_events, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE,
          pdMS_TO_TICKS(reconnect_delay_ms));
      if ((wait_bits & WIFI_CONNECTED_BIT) != 0) {
        reconnect_delay_ms = 2000;
        continue;
      }
      xEventGroupClearBits(wifi_events, WIFI_FAILED_BIT);
      const esp_err_t connect_error = esp_wifi_connect();
      if (connect_error != ESP_OK && connect_error != ESP_ERR_WIFI_CONN) {
        ESP_LOGW(TAG, "Khong goi lai duoc Wi-Fi: %s",
                 esp_err_to_name(connect_error));
      }
      reconnect_delay_ms = std::min<uint32_t>(reconnect_delay_ms * 2, 60000);
      continue;
    }
    reconnect_delay_ms = 2000;

    SensorReading reading;
    xSemaphoreTake(reading_mutex, portMAX_DELAY);
    reading = latest_reading;
    xSemaphoreGive(reading_mutex);

    if (reading.valid && reading.sequence != last_uploaded_sequence) {
      if (upload_to_server(reading, provisioning.api_key)) {
        last_uploaded_sequence = reading.sequence;
      }
    }

    // Thức dậy ngay khi rớt Wi-Fi thay vì ngủ cứng suốt 5 phút.
    xEventGroupWaitBits(
        wifi_events, WIFI_FAILED_BIT, pdFALSE, pdFALSE,
        pdMS_TO_TICKS(DSF_WIFI_UPLOAD_INTERVAL_MS));
  }
}

void init_nvs() {
  esp_err_t error = nvs_flash_init();
  if (error == ESP_ERR_NVS_NO_FREE_PAGES ||
      error == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    error = nvs_flash_init();
  }
  ESP_ERROR_CHECK(error);
}

}  // namespace

extern "C" void app_main() {
  ESP_LOGI(TAG, "Khoi dong DSF ESP32-S3 USB Gateway");
  ESP_LOGI(TAG, "Cam sensor vao cong USB&OTG; nap/log qua cong CH343P");
  ESP_LOGI(TAG, "Ly do reset ESP32 (esp_reset_reason): %d",
           static_cast<int>(esp_reset_reason()));
#if CONFIG_USB_HOST_HUBS_SUPPORTED
  ESP_LOGI(TAG, "Cau hinh build: USB hub ngoai 1 tang ENABLED");
#else
  ESP_LOGE(TAG, "Cau hinh build: USB hub DISABLED; xoa sdkconfig va build lai");
#endif

  init_nvs();

  // BLE cần được phát ngay cả khi thiết bị chưa được provision Wi-Fi/API.
  // Trước đây init_ble() nằm sau nhánh này, nên firmware mới nạp lần đầu
  // chỉ mở AP DURIAN-SMART-SENSOR và không thể được phát hiện bằng BLE.
  init_ble();

  ProvisioningConfig provisioning;
  if (setup_button_is_held() || !load_provisioning_config(provisioning)) {
    start_provisioning_portal();
    return;
  }

  ESP_LOGI(TAG, "Da tim thay cau hinh Wi-Fi/API key trong NVS");
  app_queue = xQueueCreate(8, sizeof(AppMessage));
  cdc_mutex = xSemaphoreCreateMutex();
  reading_mutex = xSemaphoreCreateMutex();
  configASSERT(app_queue && cdc_mutex && reading_mutex);

  TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
  configASSERT(xTaskCreate(usb_host_task, "usb_host", 6144, current_task,
                           USB_HOST_PRIORITY, nullptr) == pdPASS);
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  configASSERT(xTaskCreate(modbus_poll_task, "modbus_poll", 4096, nullptr,
                           10, nullptr) == pdPASS);
  configASSERT(xTaskCreate(wifi_upload_task, "wifi_upload", 8192, nullptr,
                           8, nullptr) == pdPASS);

  while (true) {
    AppMessage message;
    if (xQueueReceive(app_queue, &message, portMAX_DELAY) != pdTRUE) continue;

    if (message.id == AppMessageId::UsbConnected) {
      xSemaphoreTake(cdc_mutex, portMAX_DELAY);
      if (cdc_device == nullptr) {
        cdc_device = open_usb_serial(message.vid, message.pid);
      }
      xSemaphoreGive(cdc_mutex);
    } else if (message.id == AppMessageId::UsbDisconnected) {
      xSemaphoreTake(cdc_mutex, portMAX_DELAY);
      if (cdc_device != nullptr) {
        cdc_acm_host_close(cdc_device);
        cdc_device = nullptr;
      }
      rx_size = 0;
      xSemaphoreGive(cdc_mutex);

      xSemaphoreTake(reading_mutex, portMAX_DELAY);
      latest_reading.valid = false;
      xSemaphoreGive(reading_mutex);
      detected_register_count = 0;
      split_reading = SensorReading{};
      split_environment_received = false;
      split_nutrients_received = false;

      ESP_LOGW(TAG, "Cam bien USB da rut");
    }
  }
}