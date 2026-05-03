# QR Hedefleri ve Kamera Kayıt Sistemi Kılavuzu

Bu döküman; projeye eklenen QR kod sahnelerini, uçağın fiziksel kamera konfigürasyonunu ve `scripts/camera_recorder.py` dosyasının teknik çalışma prensiplerini detaylandırır.

---

## 1. Sistem Gereksinimleri ve Ön Hazırlık

Sistemin donanım hızlandırmalı sıkıştırma ve canlı yayın yapabilmesi için aşağıdaki bileşenlerin kurulu olması şarttır:

*   **Donanım:** NVIDIA GPU (NVENC desteği için RTX 40 serisi önerilir).
*   **Docker:** RTMP yayını için MediaMTX konteynerinin çalışıyor olması gerekir.
*   **ROS 2:** Gazebo verilerinin aktarımı için ROS 2 ortamı yüklü olmalıdır.
*   **Tarayıcı:** Chrome/Firefox üzerinden HLS akışını izlemek için "Native HLS Playback" eklentisi gereklidir.

## 2. QR Kod Hedefleri ve Saha Modelleri

QR hedefleri `worlds/zephyr_runway.sdf` dosyasında statik modeller olarak tanımlanmıştır. Bu modeller, farklı zemin ve ışık koşullarında QR kod tespiti yapılmasına olanak sağlar:

*   **Modeller:** `qr_target`, `qr_toprak`, `qr_beton`, `qr_karisik`, `qr_parlak`, `qr_golge`, `qr_karsiaydinlatma`, `qr_kismigolge`, `qr_egimli`, `qr_engelli`.
*   **Yapı:** Her hedef, `materials/textures/qr_code.png` görselini içeren bir ana yüzey ile kuzey, güney, doğu ve batı yönlerinde destek plakalarına sahiptir.

Aşağıdaki kod parçaları, QR modellerinin sahneye nasıl eklendiğini ve yapılarını gösterir.

### 2.1. QR Model Yapısı

`models/qr_target/model.sdf` içindeki temel QR hedef yapısı şu şekildedir:

```xml
<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="qr_target">
    <static>true</static>

    <link name="link">
      <visual name="visual">
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>2 2</size>
          </plane>
        </geometry>
        <material>
          <diffuse>1 1 1 1</diffuse>
          <specular>0 0 0 1</specular>
          <pbr>
            <metal>
              <albedo_map>materials/textures/qr_code.png</albedo_map>
            </metal>
          </pbr>
        </material>
      </visual>
    </link>

    <link name="plate_north">
      <pose>0 2.0606 1.0606 -0.785398 0 0</pose>
      <visual name="visual">
        <geometry>
          <box><size>2 0.05 3</size></box>
        </geometry>
        <material>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.3 0.3 0.3 1</diffuse>
        </material>
      </visual>
    </link>

    <link name="plate_south">
      <pose>0 -2.0606 1.0606 0.785398 0 0</pose>
      <visual name="visual">
        <geometry>
          <box><size>2 0.05 3</size></box>
        </geometry>
        <material>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.3 0.3 0.3 1</diffuse>
        </material>
      </visual>
    </link>

    <link name="plate_east">
      <pose>2.0606 0 1.0606 0 0.785398 0</pose>
      <visual name="visual">
        <geometry>
          <box><size>0.05 2 3</size></box>
        </geometry>
        <material>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.3 0.3 0.3 1</diffuse>
        </material>
      </visual>
    </link>

    <link name="plate_west">
      <pose>-2.0606 0 1.0606 0 -0.785398 0</pose>
      <visual name="visual">
        <geometry>
          <box><size>0.05 2 3</size></box>
        </geometry>
        <material>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.3 0.3 0.3 1</diffuse>
        </material>
      </visual>
    </link>

  </model>
</sdf>
```

### 2.2. QR Modellerinin Sahneye Eklenmesi

`worlds/zephyr_runway.sdf` içinde QR hedefler şu şekilde yerleştirilir:

```xml
<include><uri>model://qr_target</uri><pose>-29 500 0.01 0 0 0</pose></include>
<include><uri>model://qr_toprak</uri><pose>-300 300 0.01 0 0 0</pose></include>
<include><uri>model://qr_beton</uri><pose>500 -200 0.01 0 0 0</pose></include>
<include><uri>model://qr_karisik</uri><pose>-500 -200 0.01 0 0 0</pose></include>
<include><uri>model://qr_parlak</uri><pose>0 700 0.01 0 0 0</pose></include>
<include><uri>model://qr_golge</uri><pose>400 600 0.01 0 0 0</pose></include>
<include><uri>model://qr_karsiaydinlatma</uri><pose>-400 600 0.01 0 0 0</pose></include>
<include><uri>model://qr_kismigolge</uri><pose>200 -400 0.01 0 0 0</pose></include>
<include><uri>model://qr_egimli</uri><pose>-200 -400 0.01 0 0 0</pose></include>
<include><uri>model://qr_engelli</uri><pose>600 400 0.01 0 0 0</pose></include>
```

## 3. Uçağa Kamera Eklenmesi ve Fiziksel Yapılandırma

Zephyr uçağına eklenen kamera, `models/zephyr_with_ardupilot/model.sdf` içinde yeni bir `camera_link` ve bu linki uçağın gövdesine sabitleyen `camera_joint` ile tanımlanmıştır.

Kamera yapılandırması aşağıdaki gibi eklenmiştir:

```xml
<link name="camera_link">
  <pose>0 -0.35 0.08 0 0.10 -1.57</pose>
  <inertial>
    <mass>0.001</mass>
    <inertia>
      <ixx>1e-7</ixx>
      <iyy>1e-7</iyy>
      <izz>1e-7</izz>
    </inertia>
  </inertial>

  <visual name="camera_visual">
    <geometry>
      <box>
        <size>0.02 0.02 0.02</size>
      </box>
    </geometry>
    <material>
      <ambient>0.1 0.1 0.1 1</ambient>
      <diffuse>0.1 0.1 0.1 1</diffuse>
    </material>
  </visual>

  <sensor name="front_camera" type="camera">
    <pose>0 0 0 0 0 0</pose>
    <update_rate>30</update_rate>
    <visualize>false</visualize>
    <topic>/zephyr/camera/front/image_raw</topic>
    <camera>
      <horizontal_fov>1.57</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>3000</far>
      </clip>
    </camera>
  </sensor>
</link>

<joint name="camera_joint" type="fixed">
  <parent>zephyr::wing</parent>
  <child>camera_link</child>
</joint>
```

*   **Bağlantı:** `zephyr::wing` üzerine sabit olarak bağlandı.
*   **Kütle:** `0.001 kg` olarak ayarlandı.
*   **Render:** `visualize=false` ile ek görselleştirme kapatıldı.

## 4. `scripts/camera_recorder.py` Teknik İşleyişi

Bu betik, ROS 2 topic'ini dinleyerek aşağıdaki işlemleri gerçekleştirir:

1.  **Tampon Bellek (Buffer):** Son 5 saniyelik görüntüyü sürekli RAM'de tutar.
2.  **Canlı Yayın (Streaming):** `h264_nvenc` kodlayıcısı ile tarayıcı uyumlu RTMP yayını üretir.
3.  **Ultra Sıkıştırma:** `hevc_nvenc` kullanarak kayıt dosyaları üretir.

### 4.1. Temel Parametreler

```python
SERVER_RTMP_URL = "rtmp://127.0.0.1:1935/live/test"
SAVE_FOLDER = os.path.expanduser("~/Kayit")
FPS = 30
BUFFER_SECONDS = 5
WIDTH, HEIGHT = 1280, 720
frame_buffer = collections.deque(maxlen=FPS * BUFFER_SECONDS)
```

### 4.2. ROS 2 Aboneliği ve Frame İşleme

```python
class GazeboCameraNode(Node):
    def __init__(self):
        super().__init__("gazebo_camera_listener")
        self.subscription = self.create_subscription(
            Image,
            "/zephyr/camera/front/image_raw",
            self.image_callback,
            10,
        )
        self.streaming = False

    def image_callback(self, msg: Image):
        global stream_proc, WIDTH, HEIGHT

        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
        if msg.encoding == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        if not self.streaming:
            WIDTH, HEIGHT = msg.width, msg.height
            stream_proc = start_ffmpeg_stream(WIDTH, HEIGHT)
            self.streaming = True
            print("[SİSTEM] Yayın başladı!")

        timestamp_text = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        cv2.putText(frame, timestamp_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        frame_buffer.append(frame.copy())

        try:
            if stream_proc and stream_proc.stdin:
                stream_proc.stdin.write(frame.tobytes())
                stream_proc.stdin.flush()
        except:
            pass
```

### 4.3. RTMP Yayını Başlatma

```python
def start_ffmpeg_stream(width: int, height: int):
    command = [
        "ffmpeg", "-y",
        "-loglevel", "error",
        "-f", "rawvideo", "-vcodec", "rawvideo",
        "-pix_fmt", "bgr24", "-s", f"{width}x{height}", "-r", str(FPS),
        "-i", "-",
        "-c:v", "h264_nvenc",
        "-pix_fmt", "yuv420p",
        "-preset", "p1",
        "-tune", "ull",
        "-profile:v", "main",
        "-g", "30",
        "-f", "flv",
        SERVER_RTMP_URL,
    ]
    return subprocess.Popen(command, stdin=subprocess.PIPE)
```

### 4.4. Olay Sonrası Kayıt

```python
def save_event_clip(clip_frames, event_name):
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{event_name}_{timestamp_str}.mp4"
    local_path = os.path.join(SAVE_FOLDER, filename)

    print(f"\n[SİSTEM] {event_name} sıkıştırılarak kaydediliyor...")

    if not clip_frames:
        return

    height, width = clip_frames[0].shape[:2]

    command = [
        "ffmpeg", "-y",
        "-f", "rawvideo", "-vcodec", "rawvideo",
        "-pix_fmt", "bgr24", "-s", f"{width}x{height}", "-r", str(FPS),
        "-i", "-",
        "-c:v", "hevc_nvenc",
        "-preset", "p6",
        "-rc", "vbr",
        "-cq", "32",
        "-pix_fmt", "yuv420p",
        local_path,
    ]

    try:
        proc = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=subprocess.DEVNULL)
        for frame in clip_frames:
            proc.stdin.write(frame.tobytes())
        proc.stdin.close()
        proc.wait()
        print(f"[TAMAMLANDI] Sıkıştırılmış dosya: {filename}")
    except Exception as e:
        print(f"[HATA] Kayıt hatası: {e}")
```

### 4.5. Kullanıcı Komut Döngüsü

```python
def main():
    if not os.path.exists(SAVE_FOLDER):
        os.makedirs(SAVE_FOLDER)

    rclpy.init()
    camera_node = GazeboCameraNode()
    thread = threading.Thread(target=rclpy.spin, args=(camera_node,), daemon=True)
    thread.start()

    try:
        print("\n--- TEKNOFEST KAYIT PANELİ ---")
        print("'k' -> Kamikaze | 'i' -> Tespit | 'q' -> Çıkış\n")
        while True:
            cmd = input("Komut: ").lower()
            if cmd == "k":
                clip = list(frame_buffer)
                threading.Thread(target=save_event_clip, args=(clip, "KAMIKAZE"), daemon=True).start()
            elif cmd == "i":
                clip = list(frame_buffer)
                threading.Thread(target=save_event_clip, args=(clip, "TESPIT"), daemon=True).start()
            elif cmd == "q":
                break
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
```

## 5. Kurulum ve Çalıştırma Adımları

Sistemi ayağa kaldırmak için aşağıdaki komutları sırasıyla uygulayın:

1.  **Docker Sunucusu (MediaMTX):**
    ```bash
    sudo docker run -d --name rtmp-server -p 1935:1935 -p 8888:8888 -p 8889:8889 bluenviron/mediamtx
    ```
2.  **Gazebo Simülasyonu:**
    ```bash
    gz sim -v4 -r worlds/zephyr_runway.sdf
    ```
3.  **ROS 2 Köprüsü (Bridge) - KRİTİK:**
    ```bash
    ros2 run ros_gz_bridge parameter_bridge /zephyr/camera/front/image_raw@sensor_msgs/msg/Image[gz.msgs.Image
    ```
4.  **Kayıt Aracını Başlatma:**
    ```bash
    python3 scripts/camera_recorder.py
    ```

## 6. Kullanım Komutları ve Kayıtlar

Program çalışırken terminal üzerinden şu komutlar verilebilir:

*   **k** : Son 5 saniyeyi `KAMIKAZE_<zaman>.mp4` olarak kaydeder.
*   **i** : Son 5 saniyeyi `TESPIT_<zaman>.mp4` olarak kaydeder.
*   **q** : Uygulamadan güvenli çıkış yapar.

**Kayıt Konumu:** Dosyalar otomatik olarak `~/Kayit` klasöründe depolanır.

---

## Bilinen Hatalar ve Çözümleri

*   **Görüntü Siyah/Gelmiyor:** FFmpeg komutunda `-g 30` (Keyframe) ve `-pix_fmt yuv420p` parametrelerinin olduğunu kontrol edin.
*   **I/O Error:** MediaMTX konteynerinin çalıştığını ve 1935 portunun boş olduğunu `netstat -tuln | grep 1935` ile kontrol edin.

Not: Eğer NVIDIA ekran kartı olmayan bir bilgisayarda çalıştırmanız gerekirse, kod içindeki nvenc kısımlarını libx264 (yayın için) ve libx265 (kayıt için) olarak değiştirmeniz yeterlidir.