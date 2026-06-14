# LEOXI-LIDARTRACKING — cài & test trên Windows

Bê bản `.exe` sang máy Windows. Làm theo thứ tự dưới đây.

## 1. Cài app
- Tải artifact `leoxi-lidartracking-windows` từ GitHub → **Actions** → lần build mới nhất.
- Trong đó có 2 file:
  - `LEOXI-LIDARTRACKING-0.1.0-x64.exe` — **bản cài (NSIS)**, chạy để cài vào máy.
  - `LEOXI-LIDARTRACKING-0.1.0-x64.exe` (portable) — chạy thẳng không cần cài.
- App **chưa ký số** → Windows hiện cảnh báo *"Windows protected your PC"* → bấm
  **More info → Run anyway**. (Bình thường, do chưa mua chứng chỉ ký.)

## 2. NDI — KHÔNG cần cài gì thêm để app phát NDI
- `Processing.NDI.Lib.x64.dll` đã được **đóng sẵn trong app** (CI tự tải lúc build).
  App phát nguồn NDI tên **`LidarBridge-Mapping`** ngay, không cần cài NDI Tools.
- **TouchDesigner** đã có sẵn NDI bên trong → nhận nguồn được luôn.
- (Tùy chọn) Muốn xem nguồn bằng **NDI Video Monitor** thì cài **NDI Tools for Windows**
  (miễn phí) — chỉ để giám sát, không bắt buộc.

## 3. Driver cho LiDAR (BẮT BUỘC khi cắm cảm biến thật)
App nói chuyện trực tiếp với RPLIDAR, nhưng Windows cần driver USB↔UART để thấy
cổng COM:
- **RPLIDAR A1/A2/A3** (đầu USB hãng) dùng chip **Silicon Labs CP2102** →
  cài **"CP210x Universal Windows Driver"** (trang Silicon Labs).
- Nếu cáp/đầu dùng chip **CH340/CH341** → cài **driver CH340** (WCH).
- Kiểm tra: **Device Manager → Ports (COM & LPT)** phải thấy
  `Silicon Labs CP210x USB to UART Bridge (COMx)`.
- Trong app: **CONNECTION → SERIAL → COM PORT** chọn đúng cổng đó.
- **BAUDRATE** theo model: A1/A2 = `115200`, A3/S1 = `256000` (sai baud sẽ không ra điểm).

## 4. TouchDesigner
- Cài TouchDesigner trên máy Windows (máy đích).
- Thêm **NDI In TOP** → param **Source Name** chọn `LidarBridge-Mapping`.
- Nếu dropdown trống (app + TD cùng máy): điền **Extra Search IPs = `127.0.0.1`** → Enter.

## 5. Firewall (quan trọng cho NDI + OSC)
- Lần đầu chạy app, Windows Firewall sẽ hỏi → **Allow access** (ít nhất mạng **Private**).
- NDI dùng mDNS (UDP 5353) + TCP để dò nguồn; OSC dùng UDP cổng `7000`.
- Nếu app và TD ở 2 máy khác nhau → phải cùng một mạng LAN.

## 6. Nếu app không mở được
- Cài **Microsoft Visual C++ Redistributable (x64)** (runtime native module).
- Chạy bản **portable** để loại trừ lỗi cài đặt.

## Test nhanh (không cần LiDAR)
1. Mở app → **COM PORT = SIM** → **CONNECT** → **STREAMING**.
2. Bấm **START NDI**.
3. Trong TD: NDI In TOP → chọn `LidarBridge-Mapping` → thấy point cloud chuyển động.
4. OSC: TD **OSC In CHOP**, Network Port `7000` → thấy các kênh `/lidar/...`.
