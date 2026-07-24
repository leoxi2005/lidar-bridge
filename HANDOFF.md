# HANDOFF — LEOXI-LIDARTRACKING (LiDAR Bridge)

> **Mục đích:** mở **session Claude Code MỚI** cho đọc **CHỈ file này** thay vì `--resume`
> session cũ (session sống lâu → mỗi tin nhắn tốn nhiều token chỉ để đọc lại lịch sử).
> Đọc xong file này là đủ context làm tiếp.

---

## ▶️ BẮT ĐẦU SESSION MỚI (tiết kiệm token)
**Bước 1 — mở Claude Code ĐÚNG trong folder dự án** (Terminal):
```
cd /Users/macos/Downloads/design_handoff_lidar_bridge
claude
```
**Bước 2 — dán đúng câu này** (đã ghi full path để chắc chắn tìm thấy file dù mở ở đâu):
```
Đọc /Users/macos/Downloads/design_handoff_lidar_bridge/HANDOFF.md để nắm context
(đừng đọc git log / lịch sử cũ, đừng --resume session cũ), rồi làm tiếp: <MÔ TẢ VIỆC CẦN LÀM>
```
Quy tắc giữ token thấp:
- KHÔNG `--resume` session cũ. Mỗi lần mở session mới + đọc HANDOFF.md.
- Xong 1 task → `/clear`. Đừng để 1 session sống nhiều ngày.
- Hạn chế screenshot; cần thì chụp xong xử lý rồi bỏ.
- Commit repo này **KHÔNG kèm** dòng `Co-Authored-By`.
- Sau khi làm xong việc lớn → cập nhật lại chính file HANDOFF.md này rồi push.

---

## App là gì
Electron app cầu nối **LiDAR → point cloud → OSC/NDI** cho **touch-wall** (chạm tường mở "cửa" tương tác).
Chạy dev: `cd app && npm start`. **Version hiện tại: v5.8.0** (đã release trên GitHub).
Repo GitHub: `github.com/leoxi2005/lidar-bridge` (gh CLI đã auth `leoxi2005`).

## Kiến trúc (file chính)
- `app/main/main.js` — process chính: quản lý surface, connect sensor, auto-reconnect, **fusion đa cảm biến**, gửi OSC.
- `app/main/rplidar.js` — driver RPLIDAR (nhị phân Slamtec, USB serial, 360°).
- `app/main/hokuyo.js` — driver **Hokuyo UST-10LX/20LX** (SCIP 2.0 qua TCP, Ethernet, 270°). Cùng interface
  EventEmitter với RPLidar (`scan`/`status`/`info`/`error`, node `{angle, distMm, quality}`) → pipeline/OSC/fusion dùng lại y nguyên.
- `app/main/pipeline.js` — scan → toạ độ world → **background-subtract** → cluster/track → zones. **Không phân biệt loại sensor.**
- `app/main/homography.js` — `computeH`/`applyH`: warp 4 góc → toạ độ chuẩn hoá u,v ∈ [0,1].
- `app/renderer/renderer.js` — UI + hiển thị point cloud.
- `app/main/osc.js`, `app/main/ndi.js` — output.
- `tools/hokuyo-emu.js` — **Hokuyo giả** để test không cần phần cứng.

---

## TRẠNG THÁI: Hokuyo + zonecal — ĐÃ XONG & RELEASE (v5.5 → v5.8)
- **v5.5** — driver `hokuyo.js` + gắn vào cả 3 điểm tạo sensor (single-connect / auto-reconnect / **fusion**);
  UI dropdown BRAND (RPLIDAR/HOKUYO), chọn HOKUYO → ép NETWORK, mặc định `192.168.0.10:10940`.
- **v5.6** — **network auto-detect** cho Hokuyo (quét LAN tìm UST trên `:10940`).
- **v5.7** — review kỹ + **fix 7 lỗi**: auto-detect bỏ sót sensor (đổi sang **2 pha**: TCP knock → SCIP identify,
  quét /24 quanh mọi IP của PC + fallback `192.168.0.`), probe false-positive/không bật laser (`identify()` có validate),
  `_pending` cross-talk (so echo lệnh), buffer overflow guard, `_mode` sai property, preset khôi phục brand.
  Thêm workflow `.github/workflows/release.yml`.
- **v5.8** — OSC **`/zonecal`** (xem mục OSC dưới). Door Portals **v1.0.2** đã đọc format này.

**Đã verify (test thực nghiệm với emulator, không phần cứng):**
- Output OSC + normalize 0-1 **GIỐNG HỆT RPLIDAR** (pipeline sensor-agnostic) → **file design TouchDesigner chạy nguyên**.
- Background subtract chạy đúng cho Hokuyo; **preset lưu + tự khôi phục nền** (không phải chụp lại sau khi load).
- Preset lưu đủ: devices + brand + IP + warp + **zones** + background baseline + OSC config + surfaces.
- Fusion 5 con + auto-detect 2 pha + reconnect: OK.

---

## 📡 OSC OUTPUT SPEC (để khớp app đọc, vd Door Portals)
Gửi OSC/UDP tới IP:port ở panel OUTPUT (mặc định `127.0.0.1:7000`). **prefix** = ô **"OSC/"** cạnh tên surface
(KHÔNG phải tên "Mặt N"); **slug** = tên zone. Mỗi tường đặt 1 prefix (`tuong1`..`tuong5`), mỗi cửa 1 zone (`cua1`,`cua2`).

**1. Chạm/thả theo zone (dùng cho cửa):**
```
/<prefix>/zone/<slug>          <int>   1 = có người trong zone, 0 = rời   (vd /tuong1/zone/cua1)
/<prefix>/zone/<slug>/count    <int>   số điểm chạm trong zone
/<prefix>/zone/<slug>/dwell    <float> giây đã chạm (làm "giữ 2s mới mở")
```
**2. Toạ độ điểm chạm thô (nếu cần vị trí liên tục):**
```
/<prefix>/count      <int>              số điểm chạm
/<prefix>/pN/on      <int> 1
/<prefix>/pN/x       <float> 0..1       0=trái, 1=phải (dọc chiều dài tường)
/<prefix>/pN/y       <float> 0..1       cao/thấp (xem lưu ý fy dưới)
/<prefix>/pN/id      <int>              id theo dõi
```
**3. Calibration vị trí zone (MỚI v5.8 — để app hiển thị vẽ overlay ô cửa):**
```
/zonecal/<prefix>/<slug>   fx0 fx1 fy0 fy1     4× FLOAT "f", 0..1, ~1Hz
   fx0,fx1 = bbox ngang (0=trái,1=phải);  fy0,fy1 = bbox cao
```
Ví dụ bắt thật: `/zonecal/tuong2/cua1  0.10 0.30 0.333 0.889`

**⚠️ Điều kiện & lưu ý:**
- x,y,fx,fy chỉ chuẩn 0..1 khi đã **WARP 4 góc = 4 góc mặt tường** + bật **"apply normalized output"**.
- `fy` dùng **cùng hệ v với `/pN/y`** → zone overlay luôn khớp với chạm thật. Chiều dọc (0=sàn hay 0=đỉnh) do
  cách xếp 4 góc warp; mặc định đang **đỉnh=0/sàn=1**. Cần đổi thì hàm `emitZonecal()` trong `main.js` (chưa làm option lật).
- `/zonecal` chỉ gửi khi đang stream (đã connect). Nếu cần gửi lúc chưa connect → phải thêm (chưa có).

---

## TEST KHÔNG CẦN PHẦN CỨNG — emulator
`tools/hokuyo-emu.js` = Hokuyo UST-10LX giả (SCIP 2.0 qua TCP). Hiện mô phỏng **phòng chữ nhật** (ray-cast 4 tường
+ 1 người đi) — chứng minh point cloud ra **hình phòng thật, không phải vòng tròn**.
```bash
node tools/hokuyo-emu.js 10940   # 1 sensor
node tools/hokuyo-emu.js 10941   # thêm con nữa để test FUSION
cd app && npm start              # THÊM THỦ CÔNG → HOKUYO → 127.0.0.1:10940 → CONNECT (auto-detect bỏ qua loopback)
```
Test app đầy đủ (fusion/preset/zonecal) mình hay dùng: cài tạm `npm i --no-save playwright-core` trong `app/`,
viết script `_electron.launch(...)` điều khiển + bắt OSC bằng `dgram` cổng 7000, xong `rm -rf app/node_modules/playwright-core`.
(Máy Mac này arm64, KHÔNG có wine → không build Windows local; app đóng gói build trên GitHub Actions.)

**Chưa làm (đã đề xuất):** đổi emulator sang cảnh **touch-wall** (tường phẳng + "ngón tay" chạm ở 4 vị trí trái/phải-thấp/cao)
+ app dựng sẵn 5 surface để user **tập** WARP→subtract→zones→OSC trước khi qua Bali.

---

## 🚀 RELEASE (build Mac + Windows cho user tải)
Workflow `.github/workflows/release.yml` trigger bởi **tag `v*`** → build trên cloud runner (macos + windows,
tự rebuild native serialport/koffi, Windows tự tải NDI DLL) → tạo **GitHub Release** đính kèm `.dmg` + `.exe` (installer + portable).
Cắt bản mới:
```bash
# sửa xong + verify:
# 1) bump version trong app/package.json  (vd "5.8.0" -> "5.9.0")
git add -A && git commit -m "vX.Y: <mô tả>"      # KHÔNG kèm Co-Authored-By
git push origin main
git tag vX.Y.Z && git push origin vX.Y.Z          # -> kích hoạt release.yml
gh run list --limit 3                             # theo dõi
gh release view vX.Y.Z --json assets --jq '.assets[].name'   # xác nhận đủ dmg + 2 exe
```
App **không ký số**: macOS chuột phải → Open lần đầu; Windows SmartScreen → More info → Run anyway. dmg là **arm64** (Apple Silicon).
Bản đang live: **v5.8.0** — https://github.com/leoxi2005/lidar-bridge/releases

---

## 🏗️ SETUP THỰC TẾ Ở BALI (5 Hokuyo, phòng ngũ giác, touch-wall)
**Phần cứng:** 5× Hokuyo UST-10LX/20LX (Ethernet, KHÔNG USB), mỗi con **IP riêng** (`192.168.0.10`..`.14`, đổi bằng tool Hokuyo
vì mới ra hộp đều là `.10` → cắm chung là đụng IP). PC NIC cùng subnet (`192.168.0.100/24`). Switch ≥6 cổng.
**Nguồn riêng cho mỗi con (~12V), KHÔNG PoE.** Lắp mỗi con **giữa cạnh 1 mặt tường**, lát cắt **áp sát & song song mặt tường**
(cách ~2–5cm); Hokuyo 270° → xoay cho **vùng mù 90° quay ra ngoài** (nhìn point cloud thấy "múi trống" → vặn cho múi trống ra ngoài tường).

**Trong app:**
1. **AUTO-DETECT** → tự tìm cả 5 Hokuyo trên LAN (loopback/emulator thì không thấy — dùng THÊM THỦ CÔNG).
2. Bật ON cả 5 → **FUSION**.
3. Mỗi Mặt: đặt **OSC/** = `tuong1`..`tuong5`; **WARP** kéo 4 góc khớp mặt tường; **background subtract** chụp tường trống; vẽ **zones** (cửa) đặt tên `cua1`,`cua2`.
4. OUTPUT: OSC host = IP máy Door Portals, port 7000, bật **normalize**.
5. **Save preset** → mở lại có sẵn hết (sensor+IP+warp+zones+nền+OSC), chỉ tinh chỉnh warp theo tường thật.

**Concept đã giải thích cho user (khỏi giải thích lại):**
- Hokuyo là LiDAR **2D**: quét **1 mặt phẳng mỏng**, KHÔNG thấy sàn/trần. Hình point cloud = hình vật cắt qua mặt phẳng đó.
- Lát cắt **dọc sát tường** → cho **cả x (ngang) lẫn y (cao)** trên mặt tường → dùng được điều kiện độ cao.
- Khác RPLIDAR: Ethernet thay USB, 270° thay 360° (phải xoay đúng chiều). Output/normalize/OSC thì **y hệt**.

---

## Việc còn có thể làm tiếp (nếu user yêu cầu)
- Dựng **touch-wall emulator + app 5-surface** để tập (đã đề xuất, chưa làm).
- Thêm option **lật fy** cho `/zonecal` nếu Door Portals cần 0=sàn cứng.
- Gửi `/zonecal` cả khi chưa connect (hiện chỉ gửi lúc stream).
- Build **universal dmg** nếu cần chạy Mac Intel (hiện chỉ arm64).
