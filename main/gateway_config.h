#pragma once

#define DSF_API_URL "https://duriansmartfarm.com/api/sensor/data"
#define DSF_PROVISION_AP_SSID "DURIAN-SMART-SENSOR"

// Cảm biến 7/8 thông số mà Sensor App đang hỗ trợ.
#define DSF_MODBUS_ADDRESS       0x01
#define DSF_MODBUS_START_REG     0x0000
#define DSF_MODBUS_REGISTER_COUNT 8
#define DSF_MODBUS_ENV_REGISTER_COUNT 4
#define DSF_MODBUS_NPK_START_REG 0x001E
#define DSF_MODBUS_NPK_REGISTER_COUNT 3
#define DSF_SENSOR_BAUD_RATE     9600

// Gửi lên máy chủ mỗi 5 phút; BLE vẫn cập nhật sau mỗi lần đo hợp lệ.
#define DSF_WIFI_UPLOAD_INTERVAL_MS (5 * 60 * 1000)