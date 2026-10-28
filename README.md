# DSF ESP32-S3 USB Gateway

Firmware dành cho board **ESP32-S3-WROOM-1-N16R8** có hai cổng USB như ảnh:

- Cổng **USB&OTG**: cắm cảm biến USB.
- Cổng **USB-to-serial CH343P**: nạp firmware, xem log và cấp nguồn thử nghiệm.

Firmware thực hiện đồng thời:

1. ESP32-S3 làm USB Host cho cảm biến CDC/CH34x/CP210x/FTDI, nối trực tiếp
   hoặc qua **một tầng USB hub có nguồn riêng**.
2. Gửi Modbus RTU FC03 mỗi 2 giây, `9600 8N1`, địa chỉ mặc định `0x01`.
3. Đọc được cả hai bản đồ thanh ghi:
   - bản liền: `0x0000` với 7 hoặc 8 thanh ghi;
   - JXBS-3001 chuẩn: `0x0000` với 4 thanh ghi (ẩm/nhiệt/EC/pH) và
     `0x001E` với 3 thanh ghi (N/P/K), sau đó ghép thành một lần đo.
4. Phát JSON bằng BLE Nordic UART, đúng profile Sensor App đang hỗ trợ.
5. Gửi dữ liệu qua Wi-Fi lên Durian Smart Farm mỗi 5 phút.

## 1. Phần mềm cần cài

- Visual Studio Code
- Extension **Espressif IDF**
- ESP-IDF 5.4 trở lên
- Cáp USB-C có truyền dữ liệu

Không dùng Arduino IDE cho bản này. USB Host CDC cùng nhiều driver USB-UART
được hỗ trợ chính thức và ổn định hơn trong ESP-IDF.

## 2. Cấu hình lần đầu (provisioning)

Không cần sửa Wi-Fi hoặc API key trong mã nguồn. Một firmware dùng được cho
nhiều thiết bị.

Sau lần nạp đầu tiên, gateway sẽ phát một mạng Wi-Fi cấu hình:

```text
DURIAN-SMART-SENSOR
```

Nếu thiết bị có tem QR do Admin Durian Smart Farm phát hành:

1. Quét QR, đăng nhập và nhận thiết bị vào vườn.
2. Kết nối điện thoại vào Wi-Fi `DURIAN-SMART-SENSOR`.
3. Quay lại màn hình hướng dẫn và bấm **Mở cấu hình Wi-Fi**.
4. API key sẽ được điền tự động. Chỉ cần nhập tên và mật khẩu Wi-Fi của vườn,
   rồi nhấn **Lưu và kết nối**.

Nếu cấu hình thủ công:

1. Kết nối vào Wi-Fi `DURIAN-SMART-SENSOR`.
2. Nếu điện thoại báo mạng không có Internet, chọn vẫn duy trì kết nối.
3. Mở trình duyệt và vào:

```text
http://192.168.4.1
```

4. Nhập tên và mật khẩu Wi-Fi của vườn.
5. Nhập API key của thiết bị trong trang **Cài đặt → Thiết bị cảm biến**
   của Durian Smart Farm.
6. Nhấn **Lưu và kết nối**.

ESP32 sẽ lưu thông tin vào bộ nhớ NVS rồi tự khởi động lại. Từ lần sau, nó
tự kết nối Wi-Fi và gửi dữ liệu.

Muốn cấu hình lại một thiết bị đã cài, giữ nút **BOOT** trong lúc nhấn **RST**
và giữ BOOT khoảng một giây. Thiết bị sẽ phát lại `DURIAN-SMART-SENSOR`.

## 3. Build và nạp

Mở terminal ESP-IDF tại thư mục firmware:

```bash
rm -f sdkconfig
rm -rf build
idf.py set-target esp32s3
idf.py build
idf.py -p COM_PORT flash monitor
```

Trên PowerShell của Windows, nếu thư mục firmware đã từng được build, hãy đóng
monitor rồi xóa cấu hình build cũ trước:

```powershell
Remove-Item sdkconfig -ErrorAction SilentlyContinue
Remove-Item build -Recurse -Force -ErrorAction SilentlyContinue
idf.py set-target esp32s3
idf.py build
idf.py -p COM7 flash monitor
```

Việc xóa `sdkconfig` và `build` chỉ tạo lại cấu hình/build trên máy tính; không
xóa Wi-Fi/API key đã lưu trong NVS của ESP32. Khi khởi động, log phải ghi
`Cau hinh build: USB hub ngoai 1 tang ENABLED`. Nếu ghi `DISABLED`, chưa nên
kiểm tra hub cho tới khi build lại đúng cấu hình.

Ví dụ Windows:

```bash
idf.py -p COM7 flash monitor
```

Ví dụ macOS:

```bash
idf.py -p /dev/cu.wchusbserialXXXX flash monitor
```

Nếu không vào chế độ nạp:

1. Giữ nút **BOOT**.
2. Nhấn rồi thả **RST**.
3. Thả **BOOT**.
4. Chạy lại lệnh `flash`.

## 4. Thứ tự cắm dây

1. Chưa cắm cảm biến.
2. Cắm máy tính vào cổng **CH343P**.
3. Nạp firmware và mở monitor.
4. Cấp nguồn 5V ổn định cho board.
5. Nếu nối trực tiếp, cắm cảm biến qua cáp OTG vào cổng **USB&OTG**.
6. Nếu dùng hub có nguồn:
   - cấp 5V cho đúng cổng **Power/PD input** của hub;
   - cắm cảm biến vào USB-A downstream của hub;
   - cắm dây upstream liền của hub vào cổng **USB&OTG** của ESP32-S3.
   Máy tính chỉ cấp điện vào cổng Power/PD của hub, không được trở thành host
   dữ liệu đồng thời với ESP32-S3.

Log đúng:

```text
Da nhan USB hub: VID=0x.... PID=0x....; cho cam bien o cong downstream
Thiet bi USB: VID=0x1A86 PID=0x7523 class=0xFF
Da mo cam bien USB o 9600 baud, 8N1
Reading #1: {"temperature":28.6,...}
BLE san sang: DURIAN-SENSOR-AB12
Wi-Fi da ket noi
Da gui du lieu len Durian Smart Farm (HTTP 200)
```

## 5. Kết nối Sensor App

1. Bật Bluetooth và quyền thiết bị ở gần trên điện thoại.
2. Mở Sensor App.
3. Chọn kết nối BLE.
4. Chọn thiết bị tên `DURIAN-SENSOR-xxxx`.

Gateway tự phát JSON sau mỗi khung Modbus hợp lệ, nên Sensor App không cần cắm
cáp vào cảm biến.

Đây là **BLE Nordic UART**, không phải Bluetooth Classic. Vì vậy thiết bị có thể
không xuất hiện trong danh sách ghép đôi Bluetooth thông thường của Android/iOS.
Hãy tìm thiết bị trong Sensor App hoặc một ứng dụng BLE scanner như nRF Connect.
Gateway cũng phát BLE ngay khi mới nạp firmware, trước cả bước cấu hình Wi-Fi;
trong giai đoạn đó thiết bị vẫn chưa có dữ liệu cảm biến để gửi.

Firmware dùng bảng phân vùng `partitions.csv` với vùng ứng dụng 4 MB và cấu hình
flash 16 MB dành cho module ESP32-S3-WROOM-1-N16R8. Sau khi thay đổi
`sdkconfig.defaults` hoặc bảng phân vùng, phải xóa `sdkconfig` và thư mục `build`
trước khi build lại; `idf.py fullclean` một mình không đặt lại các giá trị đã
lưu trong `sdkconfig`.

## 6. Nguồn ngoài vườn

Cảm biến USB lấy 5V từ cổng OTG khi nối trực tiếp. Nhiều board không cấp đủ
dòng từ cổng OTG khi chỉ chạy pin. Khi thử nghiệm nên dùng nguồn **5V tối thiểu
2A** hoặc USB hub OTG có cấp nguồn. Firmware bật hỗ trợ **một tầng hub ngoài**;
không xâu chuỗi nhiều hub.

Không nối hai nguồn 5V đối đầu nhau và không đặt board trần ngoài trời. Dùng hộp
IP65, đầu cáp chống nước, cầu chì nhỏ ở nguồn vào và bộ hạ áp ổn định.

## 7. Nếu không có dữ liệu

Gửi lại toàn bộ ba dòng đầu xuất hiện sau khi cắm cảm biến:

```text
Da nhan USB hub: VID=... (nếu dùng hub)
Thiet bi USB: VID=...
Da mo ... / Khong mo ...
Khung Modbus ... / Reading ...
```

Nếu USB mở được nhưng không có `Reading`, có thể cảm biến dùng địa chỉ Modbus
khác `0x01`, sai baudrate/nguồn, hoặc cáp OTG không phải cáp dữ liệu. Firmware
mới cũng ghi rõ từng trường hợp:

```text
Gui poll: reg=0x0000 count=4 (block am/nhiet/EC/pH)
Nhan block 0x0000: am=72.0 nhiet=30.6 EC=1320 pH=6.9
Gui poll: reg=0x001E count=3 (block N/P/K)
Nhan block 0x001E: N=102 P=34 K=310
Reading #1: ...
```

Nếu thấy `Cam bien tu choi Modbus: ma loi ...` thì gateway đã nhận được phản
hồi, nhưng địa chỉ/bản đồ thanh ghi chưa đúng. Nếu chỉ thấy `Gui poll` mà
không thấy `Nhan block`, `Reading` hoặc lỗi Modbus, cần kiểm tra nguồn 5V
OTG, cáp USB-C có truyền dữ liệu và đúng cổng `USB&OTG`. Với cảm biến là
RS485 trần, không thể cắm trực tiếp bằng cáp USB-C: phải có bộ chuyển
USB-RS485 tương thích CH34x/CP210x/FTDI.

Nếu chỉ thấy dòng `Da nhan USB hub` nhưng không thấy `Thiet bi USB:
VID=0x1A86 PID=0x7523`, kiểm tra lại cổng cấp nguồn riêng của hub, dây upstream
và thử đổi cổng USB-A trên hub. Hub phải hoạt động như USB hub dữ liệu, không
chỉ là bộ chia nguồn.

Các giá trị địa chỉ và bản đồ mặc định nằm trong `main/gateway_config.h`;
địa chỉ cảm biến mặc định là `0x01`, baudrate là `9600 8N1`.
