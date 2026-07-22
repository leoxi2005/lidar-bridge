# HANDOFF — LiDAR Bridge (RPLIDAR + point cloud)

> Mục đích: mở **session Claude Code MỚI** và cho nó đọc **chỉ file này** thay vì
> `--resume` session cũ (session cũ sống hơn 1 tháng → mỗi tin nhắn tốn ~$1–1.5
> chỉ để đọc lại lịch sử). Đọc file này xong là đủ context để làm tiếp.

## App là gì
Electron app cầu nối LiDAR → point cloud → OSC/NDI cho touch-wall (chọn cửa tương tác).
Chạy: `cd app && npm start`. Version hiện tại: **v5.7** (xem `git log`).

**v5.5–v5.7 (Hokuyo):** thêm hỗ trợ Hokuyo UST-10LX/20LX (Ethernet/SCIP 2.0) song song RPLIDAR —
driver `hokuyo.js`, nhánh fusion, UI chọn brand, network auto-detect (2 pha: TCP knock → SCIP identify),
`tools/hokuyo-emu.js` để test không cần phần cứng. v5.7 = bản đã review kỹ + fix 7 lỗi (auto-detect bỏ sót
sensor, probe false-positive/không bật laser, `_pending` cross-talk, buffer cap, `_mode`, preset khôi phục brand).
Đã verify: output OSC + normalize 0-1 GIỐNG HỆT RPLIDAR (pipeline sensor-agnostic); background subtract +
persist qua preset chạy đúng cho Hokuyo. Release build cả macOS/.dmg + Windows/.exe qua GitHub Actions (`release.yml`, tag `v*`).

## Kiến trúc (file chính, đọc khi cần)
- `app/main/main.js` — process chính: quản lý surface, connect sensor, auto-reconnect, fusion đa cảm biến.
- `app/main/rplidar.js` — driver RPLIDAR (giao thức nhị phân Slamtec).
- `app/main/hokuyo.js` — **MỚI**: driver Hokuyo UST-10LX/20LX (SCIP 2.0 qua TCP). Cùng interface EventEmitter với RPLidar (event `scan`/`status`/`info`/`error`, node `{angle, distMm, quality}`) → pipeline/OSC/fusion dùng lại y nguyên.
- `app/main/pipeline.js` — xử lý scan → toạ độ, background-subtract, tracking.
- `app/renderer/renderer.js` — UI + hiển thị point cloud.
- `app/main/osc.js`, `app/main/ndi.js` — output.

## HỖ TRỢ HOKUYO — ĐÃ XONG (v5.5)
Driver `hokuyo.js` (266 dòng) + gắn vào cả 3 điểm tạo sensor trong `main.js`:
- ✅ auto-reconnect (`lastConnectCfg.hokuyo ? new Hokuyo() : new RPLidar()`)
- ✅ connect chính (`isHokuyo ? new Hokuyo() : new RPLidar()`, `config.brand==='hokuyo'`)
- ✅ **nhánh FUSION** (`openFusionSensor`): `d.brand==='hokuyo' ? new Hokuyo() : …` — watchdog auto-reconnect dùng lại `openFusionSensor(s.cfg)` nên Hokuyo treo cũng tự nối lại.
- ✅ **UI**: dropdown BRAND (RPLIDAR / HOKUYO) trong panel CONNECTION. Chọn HOKUYO → ẩn SERIAL/scan-mode, ép NETWORK, mặc định `192.168.0.10:10940`. Field `brand` chảy vào cả single-connect lẫn fusion device payload.

## TEST KHÔNG CẦN PHẦN CỨNG — emulator
`tools/hokuyo-emu.js` = Hokuyo UST-10LX giả, nói SCIP 2.0 qua TCP (QT/SCIP2.0/PP/BM/MD),
stream cảnh động (tường 4m + người đi qua lại) ~10Hz. Đã verify driver thật đọc đúng:
1081 điểm/scan, angle + distance chuẩn.
```bash
node tools/hokuyo-emu.js 10940      # sensor 1
node tools/hokuyo-emu.js 10941      # sensor 2 (test FUSION 2 con)
cd app && npm start                 # BRAND=HOKUYO, IP 127.0.0.1, PORT 10940 → CONNECT
```
Test nhanh chỉ driver (không mở app): `node tools/hokuyo-emu.js & node -e '…new Hokuyo().connect({host:"127.0.0.1",port:10940})…'`

### Còn lại khi có phần cứng thật (mốt qua Bali)
- Connect Hokuyo UST thật qua Ethernet (IP thật, thường 192.168.0.10), xác nhận point cloud + OSC ra.
- Chỉnh transform/pose cho khớp tường; thử FUSION RPLIDAR + Hokuyo chung.

## Git đang dở
- Modified: `app/main/main.js`  (đang gắn Hokuyo)
- Untracked: `app/main/hokuyo.js`  (driver mới, chưa commit)

## Quy tắc tiết kiệm credits khi làm tiếp
- KHÔNG `--resume` session cũ (16a1f823…). Mở session mới, đọc file này.
- Xong 1 task → `/clear`. Đừng để session sống nhiều ngày.
- Việc cơ học (sửa driver, chỉnh param) → dùng model Sonnet (`/model`); để Opus cho debug khó.
- Hạn chế screenshot; nếu cần thì downscale, chụp xong xử lý rồi clear.
