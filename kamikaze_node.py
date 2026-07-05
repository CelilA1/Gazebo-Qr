#!/usr/bin/env python3

import os
import math
import time
import threading
import collections
import requests
import subprocess
from datetime import datetime
from enum import Enum, auto

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, AttitudeTarget, OverrideRCIn, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import Image  
from std_msgs.msg import String

# ==============================================================================
# ─── YAPILANDIRMA VE PARAMETRELER (MODÜLER AYARLAR) ───────────────────────────
# ==============================================================================

# İrtifa ve Mesafe Ayarları
DIVE_ALTITUDE_M         = 105.0   # Dalışa başlama irtifası
SAFE_ALTITUDE_M         = 75.0    # Recovery sonrası tırmanılacak güvenli irtifa
RECOVERY_ALTITUDE_M     = 53.0    # Dalıştan çıkış (Recovery tetiklenme) irtifası
DIVE_ENTRY_DISTANCE_M   = 100.0   # QR koda kalan dalış kapısı mesafesi
CLIMB_ALT_TOLERANCE_M   = 4.0     # Hedef irtifaya ulaşma toleransı (Hata payı)
AWAY_DISTANCE_M         = 200.0   # QR koda çok yakınsa arkaya uzaklaşma mesafesi
CORRIDOR_TOLERANCE_M    = 30.0    # Giriş koridoruna ve hattına uyum toleransı

# Uçuş Dinamiği ve Açı Ayarları
DIVE_ANGLE_DEG          = 45.0    # Nominal dalış açısı (Pitch)
CLIMB_PITCH_DEG         = -8.0    # Tırmanma esnasındaki pitch açısı
RECOVERY_MAX_PITCH_DEG  = -12.0   # Recovery esnasındaki maksimum tırmanma açısı
MAX_ROLL_RATE_DEG_S     = 35.0    # Saniyede yapılabilecek maksimum roll değişimi
MAX_PITCH_RATE_DEG_S    = 25.0    # Saniyede yapılabilecek maksimum pitch değişimi

# Motor (Thrust) Ayarları
CRUISE_SPEED_MS         = 16.0    # Seyir hızı
THRUST_CLIMB            = 0.85    # Tırmanma esnasındaki gaz yüzdesi
THRUST_POSITION         = 0.65    # Konumlanma esnasındaki baz gaz yüzdesi
THRUST_DIVE             = 0.15    # Dalış esnasındaki motor gaz yüzdesi
THRUST_RECOVERY_BASE    = 0.65    # Recovery başlangıç baz gaz yüzdesi

# Yapay Zeka ve Kamera Ayarları
AI_PROCESS_PERIOD_S     = 0.08    # QR kod işleme periyodu (0.08 sn = ~12 FPS)
CAMERA_TIMEOUT_S        = 1.0     # Kamera kesinti/timeout algılama süresi
DEAD_RECKONING_TIMEOUT_S= 0.5     # QR kod kaybolduktan sonra yedek rotaya geçiş süresi
DIVE_INIT_DURATION_S    = 3.5     # DIVE_INIT fazının (burun dikme) süresi
LOITER_STABILITY_TIME_S = 3.0     # Görev bitimi LOITER modunda kalma emniyet süresi
VIDEO_TIMEOUT_S         = 15.0    # FFmpeg asenkron yazıcı timeout süresi

# Kontrolcü (PID) Kazançları
PID_KP_ROLL,  PID_KI_ROLL,  PID_KD_ROLL  = 0.35, 0.005, 0.05
PID_KP_PITCH, PID_KI_PITCH, PID_KD_PITCH = 0.35, 0.005, 0.05

# Sunucu ve Kayıt Ayarları
CONTEST_SERVER_URL      = "http://127.0.0.1:8080/api/kamikaze"
SERVER_RTMP_URL         = "rtmp://127.0.0.1:1935/live/test"
SAVE_CLIP               = True
SAVE_FOLDER             = os.path.expanduser("~/Kayit")
FPS                     = 30

VIDEO_TIMEOUT_S         = 15.0    # FFmpeg asenkron yazıcı timeout süresi
PRE_DIVE_SECONDS        = 5.0     # Dalış bitiş anından öncesi (kayıt penceresi)
POST_DIVE_SECONDS       = 2.0     # Dalış bitiş anından sonrası (kayıt penceresi)
BUFFER_MAX_SECONDS      = 8.0     # Sürekli tampon kaç saniye tutsun

QR_TARGETS = {
    1:  {"name": "qr_target",            "x": -29.0,  "y": 500.0,  "z": 0.01},
    2:  {"name": "qr_toprak",            "x": -300.0, "y": 300.0,  "z": 0.01},
    3:  {"name": "qr_beton",             "x": 500.0,  "y": -200.0, "z": 0.01},
    4:  {"name": "qr_karisik",           "x": -500.0, "y": -200.0, "z": 0.01},
    5:  {"name": "qr_parlak",            "x": 0.0,    "y": 700.0,  "z": 0.01},
    6:  {"name": "qr_golge",             "x": 400.0,  "y": 600.0,  "z": 0.01},
    7:  {"name": "qr_karsiaydinlatma",   "x": -400.0, "y": 600.0,  "z": 0.01},
    8:  {"name": "qr_kismigolge",        "x": 200.0,  "y": -400.0, "z": 0.01},
    9:  {"name": "qr_egimli",            "x": -200.0, "y": -400.0, "z": 0.01},
    10: {"name": "qr_engelli",           "x": 600.0,  "y": 400.0,  "z": 0.01}
}

# ==============================================================================
# ─── GÖREV DURUM MAKİNESİ VE YARDIMCI SINIFLAR ────────────────────────────────
# ==============================================================================

class MissionPhase(Enum):
    IDLE            = auto()
    ARM_TAKEOFF     = auto()
    CLIMB           = auto()
    POSITION        = auto()
    DIVE_INIT       = auto()
    DIVE            = auto()
    RECOVERY        = auto()
    DONE            = auto()


class PID:
    def __init__(self, kp, ki, kd, out_min=-1.0, out_max=1.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self._integral  = 0.0
        self._prev_err  = 0.0
        self._last_t    = None

    def reset(self):
        self._integral = 0.0
        self._prev_err = 0.0
        self._last_t   = None

    def update(self, error: float) -> float:
        now = time.time()
        dt  = (now - self._last_t) if self._last_t else 0.033
        self._last_t     = now
        self._integral  += error * dt
        derivative       = (error - self._prev_err) / max(dt, 1e-6)
        self._prev_err   = error
        out = self.kp * error + self.ki * self._integral + self.kd * derivative
        return max(self.out_min, min(self.out_max, out))


class KamikazeNode(Node):
    def __init__(self):
        super().__init__("kamikaze_node")

        self.target_x = QR_TARGETS[1]["x"]
        self.target_y = QR_TARGETS[1]["y"]
        self.target_z = QR_TARGETS[1]["z"]
        self.target_name = QR_TARGETS[1]["name"]

        self.phase            = MissionPhase.IDLE
        self.current_pose     = PoseStamped()
        self.current_velocity = TwistStamped()
        self.mav_state        = State()
        self.armed            = False
        self.mode             = ""
        
        self._mode_requested  = False
        self._arm_requested   = False

        self.qr_detected  = False
        self.qr_content   = None
        self.qr_bbox      = None
        
        self.frame_buffer = collections.deque(maxlen=10) 
        self.frame_history = collections.deque()
        self.frame_lock     = threading.Lock()
        self.dive_end_event_t    = None
        self.post_dive_recording = False
        self.dive_frames_snapshot = [] 
        
        self.video_triggered = False
        self.recovery_trigger_t = None 
        self.menu_triggered_for_done = False

        self.last_seen_qr_t         = 0.0
        self.dead_reckoning_timeout = DEAD_RECKONING_TIMEOUT_S

        self.camera_active       = False
        self.last_camera_frame_t = 0.0
        self.last_qr_process_t   = 0.0
        self.last_log_print_t    = 0.0

        self.pid_roll  = PID(PID_KP_ROLL,  PID_KI_ROLL,  PID_KD_ROLL,  -30, 30)
        self.pid_pitch = PID(PID_KP_PITCH, PID_KI_PITCH, PID_KD_PITCH, -30, 30)
        self.cam_w = 800
        self.cam_h = 800

        self.virtual_wp_x = 0.0
        self.virtual_wp_y = 0.0

        self.moving_away  = False
        self.away_bearing = 0.0

        self.track_start_x     = 0.0
        self.track_start_y     = 0.0
        self.route_initialized = False

        self.dive_bearing_deg     = None
        self.recovery_bearing_deg = None

        self.dive_start_line_x    = 0.0
        self.dive_start_line_y    = 0.0
        self.dive_vector_x        = 0.0
        self.dive_vector_y        = 0.0

        self._last_sent_roll_deg  = 0.0
        self._last_sent_pitch_deg = 0.0
        self._last_setpoint_time  = None

        self._stream_proc = None
        self._loiter_start_time = None

        # ==============================================================================
        # ─── GÜNCEL KESİN ÇÖZÜM: HİBRİT QoS PROFİLLERİ ───────────────────────────────
        # ==============================================================================
        # Kamera için tam eşleşen RELIABLE profil
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # MAVROS için zorunlu olan BEST_EFFORT profil (Uyuşmazlık hatasını çözen kısım)
        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        reliable_qos = QoSProfile(depth=10) # Durum mesajları için standart profil

        # Her aboneyi kendi ait olduğu doğru profile bağlıyoruz
        self.create_subscription(Image,        "/zephyr/camera/front/image_raw",        self._cb_camera,  camera_qos)
        self.create_subscription(State,        "/mavros/state",                         self._cb_state,   reliable_qos)
        self.create_subscription(PoseStamped,  "/mavros/local_position/pose",           self._cb_pose,    mavros_qos)
        self.create_subscription(TwistStamped, "/mavros/local_position/velocity_local", self._cb_vel,     mavros_qos)
        self.create_subscription(String,       "/kamikaze/command",                     self._cb_command, reliable_qos)

        self.pub_setpoint_pos = self.create_publisher(PositionTarget, "/mavros/setpoint_raw/local",    10)
        self.pub_setpoint_att = self.create_publisher(AttitudeTarget, "/mavros/setpoint_raw/attitude", 10)
        self.pub_status       = self.create_publisher(String,         "/kamikaze/status",              10)
        self.pub_rc_override  = self.create_publisher(OverrideRCIn,  "/mavros/rc/override",           10)

        self.cli_arm     = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.cli_mode    = self.create_client(SetMode,     "/mavros/set_mode")

        self.create_timer(0.05, self._control_loop)
        self.create_timer(0.1,  self._publish_heartbeat_setpoint)
        self.create_timer(0.5,  self._check_and_print_menu_callback)
        self.create_timer(0.2,  self._check_camera_timeout)

        self._qrdet   = None
        self._qreader = None
        self._load_qr_libs()

        self._worker_thread_active = False
        self._qr_worker_thread = None

        self.get_logger().info("🎯 Kamikaze Node başlatıldı. '/kamikaze/command' topic'ine 'START_X' gönderin.")

    def _start_ffmpeg_stream(self, width: int, height: int):
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

    def _load_qr_libs(self):
        try:
            from qrdet import QRDetector
            self._qrdet = QRDetector()
            self.get_logger().info("✅ qrdet kütüphanesi yüklendi.")
        except ImportError:
            self.get_logger().warn("⚠️  qrdet bulunamadı → fallback: cv2.QRCodeDetector")

        try:
            from qreader import QReader
            self._qreader = QReader()
            self.get_logger().info("✅ qreader kütüphanesi yüklendi.")
        except ImportError:
            self.get_logger().warn("⚠️  qreader bulunamadı → fallback: cv2.QRCodeDetector")

    def _reset_mission_variables(self):
        self.qr_detected         = False
        self.qr_content          = None
        self.qr_bbox             = None
        self.last_seen_qr_t      = 0.0
        self.camera_active       = False
        self.last_camera_frame_t = time.time()
        self.video_triggered     = False
        self.recovery_trigger_t  = None
        self.menu_triggered_for_done = False
        self._png_match_saved = False
        self._png_false_saved = False
        
        with self.frame_lock:
            self.frame_history.clear()
            self.dive_frames_snapshot.clear()
        self.dive_end_event_t    = None
        self.post_dive_recording = False
            
        self.route_initialized   = False
        self.dive_bearing_deg    = None
        self.recovery_bearing_deg = None
        self._mode_requested     = False
        self._arm_requested      = False
        self._loiter_start_time  = None
        
        self._last_sent_roll_deg  = 0.0
        self._last_sent_pitch_deg = 0.0
        self._last_setpoint_time  = None

        if hasattr(self, 'qr_latched'):
            self.qr_latched = False
            delattr(self, 'qr_latched')

        self.dive_start_line_x    = 0.0
        self.dive_start_line_y    = 0.0
        self.dive_vector_x        = 0.0
        self.dive_vector_y        = 0.0

        self.pid_roll.reset()
        self.pid_pitch.reset()
        if hasattr(self, '_done_logged'):
            delattr(self, '_done_logged')
        self.get_logger().info(f"♻️ Tüm uçuş değişkenleri sıfırlandı. Yeni Hedef: {self.target_name}")

    def _check_camera_timeout(self):
        if self.last_camera_frame_t > 0.0:
            if (time.time() - self.last_camera_frame_t) > CAMERA_TIMEOUT_S:
                self.camera_active = False

    def _cb_state(self, msg: State):
        self.mav_state = msg
        self.armed     = msg.armed
        self.mode      = msg.mode

    def _cb_pose(self, msg: PoseStamped):
        self.current_pose = msg

    def _cb_vel(self, msg: TwistStamped):
        self.current_velocity = msg

    def _cb_camera(self, msg: Image):
        try:
            raw_data = np.frombuffer(msg.data, dtype=np.uint8)
            encoding = msg.encoding.lower()
            
            if encoding in ("rgb8", "bgr8"):
                frame = raw_data.reshape((msg.height, msg.width, 3))
                if encoding == "rgb8":
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            elif encoding in ("rgba8", "bgra8"):
                frame = raw_data.reshape((msg.height, msg.width, 4))
                if encoding == "rgba8":
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
                else:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            elif "mono" in encoding or encoding == "8uc1":
                frame = raw_data.reshape((msg.height, msg.width, 1))
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            else:
                expected_elements = msg.height * msg.width * 3
                if raw_data.size >= expected_elements:
                    frame = raw_data[:expected_elements].reshape((msg.height, msg.width, 3))
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                else:
                    frame = raw_data.reshape((msg.height, msg.width, -1))
                    if frame.shape[2] == 4:
                        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
                    elif frame.shape[2] == 1:
                        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        except Exception as e:
            self.get_logger().error(f"🎥 Kamera imaj çözümleme hatası: {e}")
            return

        self.camera_active       = True
        self.last_camera_frame_t = time.time()
        self.cam_h, self.cam_w   = frame.shape[:2]
        
        if self._stream_proc is None:
            self._stream_proc = self._start_ffmpeg_stream(self.cam_w, self.cam_h)

        # ─── CANLI YAYIN VE VİDEO İÇİN BOUNDING BOX ÇİZİMİ ────────────────────────────
        now = time.time()
        is_qr_valid = (now - self.last_seen_qr_t) <= self.dead_reckoning_timeout

        if is_qr_valid and self.qr_bbox is not None:
            cx, cy, bw, bh = self.qr_bbox
            # İşçi thread 2 kat büyük resimde (w*2, h*2) çalıştığı için 
            # Orijinal ekrana çizim yaparken koordinatları 2'ye bölüyoruz.
            cx_orig, cy_orig = cx // 2, cy // 2
            bw_orig, bh_orig = bw // 2, bh // 2

            x1 = max(0, cx_orig - bw_orig // 2)
            y1 = max(0, cy_orig - bh_orig // 2)
            x2 = min(self.cam_w, cx_orig + bw_orig // 2)
            y2 = min(self.cam_h, cy_orig + bh_orig // 2)

            # Ekrana yeşil kareyi ve anlık durumu basıyoruz
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
            status_lbl = f"QR LOCK: {self.qr_content if self.qr_content else 'TRACKING'}"
            cv2.putText(frame, status_lbl, (x1, max(20, y1 - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        # ──────────────────────────────────────────────────────────────────────────────

        timestamp_text = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        cv2.putText(frame, timestamp_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        if self.phase in (MissionPhase.DIVE_INIT, MissionPhase.DIVE, MissionPhase.RECOVERY):
            with self.frame_lock:
                self.frame_history.append((now, frame.copy()))
                cutoff = now - BUFFER_MAX_SECONDS
                while self.frame_history and self.frame_history[0][0] < cutoff:
                    self.frame_history.popleft()

        try:
            if self._stream_proc and self._stream_proc.stdin:
                self._stream_proc.stdin.write(frame.tobytes())
                self._stream_proc.stdin.flush()
        except:
            pass

        if self.phase in (MissionPhase.POSITION, MissionPhase.DIVE_INIT, MissionPhase.DIVE, MissionPhase.RECOVERY):
            if (now - self.last_qr_process_t) >= AI_PROCESS_PERIOD_S:
                self.frame_buffer.append(frame)
                self.last_qr_process_t = now

    def _qr_worker_loop(self):
        while rclpy.ok() and self._worker_thread_active:
            if not self.frame_buffer:
                time.sleep(0.02)
                continue
            frame = self.frame_buffer.pop()
            self._detect_qr(frame)

    def _cb_command(self, msg: String):
        raw_cmd = msg.data.strip().upper()

        if raw_cmd.startswith("START_"):
            try:
                target_idx = int(raw_cmd.split("_")[1])
                if target_idx in QR_TARGETS:
                    self.target_x    = QR_TARGETS[target_idx]["x"]
                    self.target_y    = QR_TARGETS[target_idx]["y"]
                    self.target_z    = QR_TARGETS[target_idx]["z"]
                    self.target_name = QR_TARGETS[target_idx]["name"]
                else:
                    self.get_logger().error("⚠️ Geçersiz hedef indeksi!")
                    return
            except Exception as e:
                self.get_logger().error(f"Komut ayrıştırma hatası: {e}")
                return

            if not self.mav_state.connected:
                self.get_logger().error("⚠️ SITL (MAVROS) bağlantısı yok!")
                return

            self.get_logger().info(f"🚀 Kamikaze görevi BAŞLATILIYOR -> Hedef: {self.target_name}")
            self._reset_mission_variables()

            if not self._worker_thread_active:
                self._worker_thread_active = True
                self._qr_worker_thread = threading.Thread(target=self._qr_worker_loop, daemon=True)
                self._qr_worker_thread.start()
                self.get_logger().info("🧠 [Sistem] Yapay Zeka Worker Thread'i uyandırıldı.")

            rc_msg = OverrideRCIn()
            rc_msg.channels = [0] * 18
            self.pub_rc_override.publish(rc_msg)

            self.track_start_x = self._x()
            self.track_start_y = self._y()
            self.route_initialized = True

            bearing = self._bearing_to_target()
            initial_dist = self._dist_to_target_xy()

            if initial_dist < AWAY_DISTANCE_M:
                self.moving_away  = True
                self.away_bearing = (bearing + math.pi) % (2 * math.pi) - math.pi
                self.get_logger().info(f"⚠️ QR koda çok yakın başlanıldı ({initial_dist:.1f} m)! Arkaya uzaklaşılıyor...")
            else:
                self.moving_away = False

            self._set_phase(MissionPhase.ARM_TAKEOFF)

        elif raw_cmd == "ABORT":
            if self.phase not in (MissionPhase.IDLE, MissionPhase.DONE):
                self.get_logger().warn("🛑 ABORT alındı! Sistem sıfırlanıyor...")
                
                if SAVE_CLIP and len(self.frame_history) > 0 and not self.video_triggered:
                    self.video_triggered = True
                    with self.frame_lock:
                        clip_snapshot = [f.copy() for (t_, f) in self.frame_history]
                    t = threading.Thread(target=self._save_clip_worker, args=(clip_snapshot, "ABORT_CLIP"), daemon=True)
                    t.start()

                self._set_mode("LOITER")
                self._set_phase(MissionPhase.DONE)

    def _detect_qr(self, frame: np.ndarray):
        h, w = frame.shape[:2]
        resized = cv2.resize(frame, (w * 2, h * 2), interpolation=cv2.INTER_LINEAR)
        gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
        
        # Kontrast sınırını dengeli tutuyoruz
        clahe = cv2.createCLAHE(clipLimit=1.8, tileGridSize=(8, 8))
        equalized = clahe.apply(gray)
        processed_frame = cv2.cvtColor(equalized, cv2.COLOR_GRAY2BGR)

        bbox    = None
        content = None

        if self._qrdet is not None:
            try:
                detections = self._qrdet.detect(image=processed_frame, is_bgr=True)
                if detections:
                    det = detections[0]
                    x1, y1, x2, y2 = (int(det["bbox_xyxy"][i]) for i in range(4))
                    bbox = ((x1 + x2) // 2, (y1 + y2) // 2, x2 - x1, y2 - y1)
            except Exception:
                pass

        if bbox is None:
            try:
                detector = cv2.QRCodeDetector()
                data, pts, _ = detector.detectAndDecode(processed_frame)
                if pts is not None:
                    pts = pts.astype(int).reshape(-1, 2)
                    cx  = int(pts[:, 0].mean())
                    cy  = int(pts[:, 1].mean())
                    w_b = int(pts[:, 0].max() - pts[:, 0].min())
                    h_b = int(pts[:, 1].max() - pts[:, 1].min())
                    bbox    = (cx, cy, w_b, h_b)
                    content = data if data else None
            except Exception:
                pass

        if bbox is not None and content is None:
            if self._qreader is not None:
                try:
                    results = self._qreader.detect_and_decode(image=processed_frame, is_bgr=True)
                    if results:
                        content = results[0]
                except Exception:
                    pass

        # ─── 1. REZİLLİK ÖNLEYİCİ BOYUT FİLTRESİ ───────────────────────────────
        if bbox is not None:
            w_b, h_b = bbox[2], bbox[3]
            if w_b > (w * 2 * 0.40) or h_b > (h * 2 * 0.40):
                self.get_logger().warn("⚠️ Devasa sahte kutu engellendi (Muhtemelen pist çizgisi).")
                bbox = None
                content = None

        # ─── 2. DETECT VE MATCH DURUMLARINA GÖRE AYRI PNG KAYITLARI ────────────
        is_qr_valid = (time.time() - self.last_seen_qr_t) <= self.dead_reckoning_timeout

        if bbox is not None:
            cx, cy, bw, bh = bbox
            cx_orig, cy_orig = cx // 2, cy // 2
            bw_orig, bh_orig = bw // 2, bh // 2

            x1 = max(0, cx_orig - bw_orig // 2)
            y1 = max(0, cy_orig - bh_orig // 2)
            x2 = min(w, cx_orig + bw_orig // 2)
            y2 = min(h, cy_orig + bh_orig // 2)

            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            os.makedirs(SAVE_FOLDER, exist_ok=True)

            if content is not None:
                # DURUM A: Hem kutu var hem de içerik okundu (Gerçek Hedef)
                self.qr_bbox        = bbox
                self.last_seen_qr_t = time.time()

                if not hasattr(self, '_png_match_saved') or not self._png_match_saved:
                    try:
                        snap_frame = frame.copy()
                        cv2.rectangle(snap_frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
                        cv2.putText(snap_frame, f"MATCH: {content}", (x1, max(20, y1 - 10)), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        
                        img_path = os.path.join(SAVE_FOLDER, f"MATCH_{self.target_name}_{timestamp}.png")
                        cv2.imwrite(img_path, snap_frame)
                        self.get_logger().info(f"🎯 Gerçek QR başarıyla çözüldü ve mühürlendi: {img_path}")
                        self._png_match_saved = True
                    except Exception as e:
                        self.get_logger().error(f"❌ MATCH PNG hatası: {e}")
            else:
                # DURUM B: Sadece kutu var, içerik OKUNAMADI (Senin istediğin sahte/hatalı tespit anı)
                if not hasattr(self, '_png_false_saved') or not self._png_false_saved:
                    try:
                        snap_frame = frame.copy()
                        # Yanıltıcı kutu olduğunu belli etmek için kırmızı veya sarı da çizebilirsin, 
                        # ama istersen yeşil kalsın. Fark etmesi için kırmızı (0, 0, 255) yaptım:
                        cv2.rectangle(snap_frame, (x1, y1), (x2, y2), (0, 0, 255), 3)
                        cv2.putText(snap_frame, "LOCK: DECODING_FAILED", (x1, max(20, y1 - 10)), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                        
                        img_path = os.path.join(SAVE_FOLDER, f"FALSE_LOCK_{self.target_name}_{timestamp}.png")
                        cv2.imwrite(img_path, snap_frame)
                        self.get_logger().warn(f"📸 Sahte kilitlenme (Sadece Detection) PNG olarak kaydedildi: {img_path}")
                        self._png_false_saved = True
                    except Exception as e:
                        self.get_logger().error(f"❌ FALSE PNG hatası: {e}")

                # İçerik olmadığı için sahte kutuyu otopilota beslemiyoruz, rotayı koruyoruz
                if not is_qr_valid:
                    self.qr_bbox = None
        else:
            if not is_qr_valid:
                self.qr_bbox = None

        # ─── 3. METİN OKUNURSA SUNUCUYA RAPORLAMA BÖLÜMÜ ───────────────────────
        if content and not self.qr_detected:
            self.qr_detected = True
            self.qr_content  = content
            self.get_logger().info(f"✅ QR KOD İÇERİĞİ OKUNDU: {content}")
            self._report_to_server(content)

    def _report_to_server(self, text: str):
        try:
            payload = {"qr_data": text, "team": "Sema Aviation", "timestamp": str(time.time())}
            requests.post(CONTEST_SERVER_URL, json=payload, timeout=0.5)
        except Exception:
            pass

    def _alt(self) -> float:
        return self.current_pose.pose.position.z

    def _x(self) -> float:
        return self.current_pose.pose.position.x

    def _y(self) -> float:
        return self.current_pose.pose.position.y

    def _speed(self) -> float:
        v = self.current_velocity.twist.linear
        return math.sqrt(v.x**2 + v.y**2 + v.z**2)

    def _dist_to_target_xy(self) -> float:
        dx = self.current_pose.pose.position.x - self.target_x
        dy = self.current_pose.pose.position.y - self.target_y
        return math.sqrt(dx**2 + dy**2)

    def _bearing_to_target(self) -> float:
        dx = self.target_x - self.current_pose.pose.position.x
        dy = self.target_y - self.current_pose.pose.position.y
        return math.atan2(dy, dx)

    def _get_dive_entry_wp_xy(self) -> tuple:
        bearing = self._bearing_to_target()
        wp_x = self.target_x - DIVE_ENTRY_DISTANCE_M * math.cos(bearing)
        wp_y = self.target_y - DIVE_ENTRY_DISTANCE_M * math.sin(bearing)
        return wp_x, wp_y

    def _dist_to_dive_entry_wp(self) -> float:
        wpx, wpy = self._get_dive_entry_wp_xy()
        dx = self.current_pose.pose.position.x - wpx
        dy = wpy - self.current_pose.pose.position.y
        return math.sqrt(dx**2 + dy**2)

    def _get_heading_error_to_target(self) -> float:
        q = self.current_pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)
        err = self._bearing_to_target() - current_yaw
        return (err + math.pi) % (2 * math.pi) - math.pi

    def _heading_error_from_bearing(self, bearing_rad: float) -> float:
        q = self.current_pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)
        err = bearing_rad - current_yaw
        return (err + math.pi) % (2 * math.pi) - math.pi

    def _phase_elapsed(self) -> float:
        return time.time() - self.phase_start_t

    def _set_phase(self, phase: MissionPhase):
        self.get_logger().info(f"  ▶ Faz: {self.phase.name} → {phase.name}")
        self.phase         = phase
        self.phase_start_t = time.time()

        if phase == MissionPhase.POSITION:
            self.route_initialized = False

        elif phase == MissionPhase.DIVE_INIT:
            self._set_mode("GUIDED_NOGPS")
            self.dive_bearing_deg = math.degrees(self._bearing_to_target())
            
            self.dive_start_line_x = self._x()
            self.dive_start_line_y = self._y()
            bearing_rad = self._bearing_to_target()
            self.dive_vector_x = math.cos(bearing_rad)
            self.dive_vector_y = math.sin(bearing_rad)
            self.get_logger().info(f"⚡ Mod: GUIDED_NOGPS | Yedek Vektör Hattı Kilitlendi: {self.dive_bearing_deg:.1f}°")

        elif phase == MissionPhase.DIVE:
            self.pid_roll.reset()
            self.pid_pitch.reset()
            self.dive_start_t = time.time()
            self._set_mode("GUIDED_NOGPS")

        elif phase == MissionPhase.RECOVERY:
            self._set_mode("GUIDED")
            self.recovery_bearing_deg = math.degrees(self._bearing_to_target())
            self.recovery_trigger_t  = time.time()
            self.dive_end_event_t    = time.time()
            self.post_dive_recording = True 
            
            with self.frame_lock:
                self.dive_frames_snapshot = [f for f in self.frame_history]
            self.get_logger().info(f"📸 [Snapshot] Dalış bitti! {len(self.dive_frames_snapshot)} adet geçmiş frame kilitlendi.")

    def _publish_position_setpoint(self, x: float, y: float, z: float):
        now = time.time()
        if not hasattr(self, '_last_pos_sp_t'):
            self._last_pos_sp_t = 0.0
        if (now - self._last_pos_sp_t) < 0.2:
            return
        self._last_pos_sp_t = now

        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = (
            PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
        )
        msg.position.x = float(x)
        msg.position.y = float(y)
        msg.position.z = -float(z)
        self.pub_setpoint_pos.publish(msg)

    def _publish_attitude_setpoint(self, roll_deg: float, pitch_deg: float,
                                   yaw_deg: float, thrust: float = 0.0):
        now = time.time()
        if self._last_setpoint_time is None:
            dt = 0.05  
        else:
            dt = now - self._last_setpoint_time
            
        self._last_setpoint_time = now

        max_roll_step  = MAX_ROLL_RATE_DEG_S * dt
        max_pitch_step = MAX_PITCH_RATE_DEG_S * dt

        roll_diff = roll_deg - self._last_sent_roll_deg
        if abs(roll_diff) > max_roll_step:
            roll_deg = self._last_sent_roll_deg + math.copysign(max_roll_step, roll_diff)

        pitch_diff = pitch_deg - self._last_sent_pitch_deg
        if abs(pitch_diff) > max_pitch_step:
            pitch_deg = self._last_sent_pitch_deg + math.copysign(max_pitch_step, pitch_diff)

        self._last_sent_roll_deg  = roll_deg
        self._last_sent_pitch_deg = pitch_deg

        msg = AttitudeTarget()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.type_mask = (
            AttitudeTarget.IGNORE_ROLL_RATE |
            AttitudeTarget.IGNORE_PITCH_RATE |
            AttitudeTarget.IGNORE_YAW_RATE
        )
        roll  = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw   = math.radians(yaw_deg)

        cr = math.cos(roll  / 2); sr = math.sin(roll  / 2)
        cp = math.cos(pitch / 2); sp = math.sin(pitch / 2)
        cy = math.cos(yaw   / 2); sy = math.sin(yaw   / 2)

        msg.orientation.w = cr * cp * cy + sr * sp * sy
        msg.orientation.x = sr * cp * cy - cr * sp * sy
        msg.orientation.y = cr * sp * cy + sr * cp * sy
        msg.orientation.z = cr * cp * sy - sr * sp * cy
        msg.thrust = float(max(0.0, min(1.0, thrust)))
        self.pub_setpoint_att.publish(msg)

    def _publish_heartbeat_setpoint(self):
        if self.phase in (MissionPhase.IDLE, MissionPhase.DONE):
            self._publish_position_setpoint(
                self._x(), self._y(), max(self._alt(), 2.0)
            )

    def _set_mode(self, mode: str):
        if self.mode == mode:
            self._mode_requested = False
            return True
        if self._mode_requested: 
            return False 
        if not self.cli_mode.service_is_ready():
            return False
        
        self._mode_requested = True
        req = SetMode.Request()
        req.custom_mode = mode
        
        future = self.cli_mode.call_async(req)
        def cb(f):
            self._mode_requested = False
        future.add_done_callback(cb)
        return True

    def _arm(self, arm: bool = True):
        if self.armed == arm:
            self._arm_requested = False
            return True
        if self._arm_requested: 
            return False 
        if not self.cli_arm.service_is_ready():
            return False
        
        self._arm_requested = True
        req = CommandBool.Request()
        req.value = arm
        
        future = self.cli_arm.call_async(req)
        def cb(f):
            self._arm_requested = False
        future.add_done_callback(cb)
        return True

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)
        self.get_logger().info(text)

    def _camera_pid_corrections(self):
        if self.qr_bbox is None:
            return 0.0, 0.0

        cx, cy, bw, bh = self.qr_bbox
        err_x = (cx - self.cam_w / 2.0) / (self.cam_w / 2.0)
        err_y = (cy - self.cam_h / 2.0) / (self.cam_h / 2.0)

        half_fov_h = 39.0
        half_fov_v = half_fov_h * (self.cam_h / self.cam_w)

        delta_roll  = self.pid_roll.update(err_x)
        delta_pitch = self.pid_pitch.update(err_y)

        speed = self._speed()
        speed_factor = CRUISE_SPEED_MS / max(speed, 10.0)

        return delta_roll * half_fov_h * speed_factor, delta_pitch * half_fov_v * speed_factor

    def _control_loop(self):
        phase = self.phase
        now = time.time()

        if self.post_dive_recording and not self.video_triggered:
            if (now - self.dive_end_event_t) >= POST_DIVE_SECONDS:
                self.video_triggered = True
                self.get_logger().info("💾 [MÜHÜR] Dalış sonrası 2 saniyelik süre tamamlandı. Klip kesiliyor...")

                window_start = self.dive_end_event_t - PRE_DIVE_SECONDS

                clip_snapshot = [f.copy() for (t_, f) in self.dive_frames_snapshot if t_ >= window_start]
                
                with self.frame_lock:
                    additional_frames = [f.copy() for (t_, f) in self.frame_history if t_ > self.dive_end_event_t]
                
                clip_snapshot.extend(additional_frames)
                self.get_logger().info(f"🎬 Toplam {len(clip_snapshot)} frame diske yazılmak üzere hazırlanıyor.")

                t = threading.Thread(
                    target=self._save_clip_worker,
                    args=(clip_snapshot, "KAMIKAZE_DIVE_COMPLETE"),
                    daemon=True
                )
                t.start()

        if phase in (MissionPhase.IDLE, MissionPhase.DONE):
            if phase == MissionPhase.DONE and not hasattr(self, '_done_logged'):
                self._publish_status(f"🏁 GÖREV SONLANDI! QR: {self.qr_content or 'OKUNAMADI'}")
                self._done_logged = True
                self.menu_triggered_for_done = True
            return

        elif phase == MissionPhase.ARM_TAKEOFF:
            if (now - self.last_log_print_t) >= 0.4:
                self._publish_status("📍 Faz: ARM_TAKEOFF")
                self.last_log_print_t = now

            if self._phase_elapsed() < 2.0:
                self._publish_position_setpoint(self.track_start_x, self.track_start_y, DIVE_ALTITUDE_M)
                return

            if self.mode != "GUIDED":
                self._set_mode("GUIDED")
                self._publish_position_setpoint(self.track_start_x, self.track_start_y, DIVE_ALTITUDE_M)
                return

            if not self.armed:
                self._arm(True)
                self._publish_position_setpoint(self.track_start_x, self.track_start_y, DIVE_ALTITUDE_M)
                return

            if self.armed and self.mode == "GUIDED":
                self._set_phase(MissionPhase.CLIMB)

        elif phase == MissionPhase.CLIMB:
            target_alt = DIVE_ALTITUDE_M
            if (now - self.last_log_print_t) >= 0.4:
                self._publish_status(f"📍 Faz: CLIMB | İrtifa: {self._alt():.1f}m → {target_alt}m")
                self.last_log_print_t = now
                
            self._publish_position_setpoint(self.track_start_x, self.track_start_y, target_alt)

            heading_err = self._get_heading_error_to_target()
            roll_corr   = max(-20.0, min(20.0, -math.degrees(heading_err) * 1.2))

            self._publish_attitude_setpoint(roll_corr, CLIMB_PITCH_DEG, math.degrees(self._bearing_to_target()), THRUST_CLIMB)

            if self._alt() >= target_alt - CLIMB_ALT_TOLERANCE_M:
                self._set_phase(MissionPhase.POSITION)
            return

        elif phase == MissionPhase.POSITION:
            dist_to_qr    = self._dist_to_target_xy()
            dist_to_entry = self._dist_to_dive_entry_wp()
            raw_alt_error = self._alt() - DIVE_ALTITUDE_M
            alt_err       = abs(raw_alt_error)

            if self.moving_away:
                q = self.current_pose.pose.orientation
                siny_cosp = 2 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
                current_yaw = math.atan2(siny_cosp, cosy_cosp)
                away_err    = (self.away_bearing - current_yaw + math.pi) % (2 * math.pi) - math.pi
                roll_cmd    = max(-25.0, min(25.0, -math.degrees(away_err) * 0.8))

                if raw_alt_error > 1.0:
                    pitch_cmd  = max(-10.0, -raw_alt_error * 0.8)
                    thrust_cmd = max(0.35, THRUST_POSITION - (raw_alt_error * 0.05))
                else:
                    pitch_cmd  = min(2.5, -raw_alt_error * 0.3)
                    thrust_cmd = THRUST_POSITION

                if (now - self.last_log_print_t) >= 0.4:
                    self._publish_status(f"📍 Faz: POSITION | Uzaklaşılıyor | Mesafe: {dist_to_qr:.1f} m")
                    self.last_log_print_t = now
                    
                self._publish_attitude_setpoint(roll_cmd, pitch_cmd, math.degrees(self.away_bearing), thrust_cmd)

                if dist_to_qr >= AWAY_DISTANCE_M:
                    self.moving_away   = False
                    self.route_initialized = False 
                return

            tx, ty = self._get_dive_entry_wp_xy()

            if not self.route_initialized:
                bearing = self._bearing_to_target()
                self.track_start_x = self.target_x - 300.0 * math.cos(bearing)
                self.track_start_y = self.target_y - 300.0 * math.sin(bearing)
                self.route_initialized = True

            total_dx = tx - self.track_start_x
            total_dy = ty - self.track_start_y
            line_len = math.sqrt(total_dx**2 + total_dy**2)

            if line_len > 1e-3:
                u_dx = self._x() - self.track_start_x
                u_dy = self._y() - self.track_start_y
                t    = (u_dx * total_dx + u_dy * total_dy) / (line_len**2)
                t    = max(0.0, min(1.0, t))
                proj_x = self.track_start_x + t * total_dx
                proj_y = self.track_start_y + t * total_dy
                look_ahead_dist   = 120.0
                self.virtual_wp_x = proj_x + (look_ahead_dist * (total_dx / line_len))
                self.virtual_wp_y = proj_y + (look_ahead_dist * (total_dy / line_len))
            else:
                self.virtual_wp_x = tx
                self.virtual_wp_y = ty

            dx_to_wp      = self.virtual_wp_x - self._x()
            dy_to_wp      = self.virtual_wp_y - self._y()
            bearing_to_wp = math.atan2(dy_to_wp, dx_to_wp)

            q = self.current_pose.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
            current_yaw    = math.atan2(siny_cosp, cosy_cosp)
            wp_heading_err = (bearing_to_wp - current_yaw + math.pi) % (2 * math.pi) - math.pi
            roll_cmd       = max(-30.0, min(30.0, -math.degrees(wp_heading_err) * 0.7))

            if raw_alt_error > 1.0:
                pitch_cmd  = max(-6.0, -raw_alt_error * 0.5)
                thrust_cmd = max(0.45, THRUST_POSITION - (raw_alt_error * 0.02))
            else:
                pitch_cmd  = min(2.0, -raw_alt_error * 0.2)
                thrust_cmd = THRUST_POSITION

            if (now - self.last_log_print_t) >= 0.4:
                self._publish_status(f"📍 Faz: POSITION | Giriş Kapısına: {dist_to_entry:.1f}m | İrtifa: {self._alt():.1f}m")
                self.last_log_print_t = now
                
            self._publish_attitude_setpoint(roll_cmd, pitch_cmd, math.degrees(bearing_to_wp), thrust_cmd)

            koridora_girdi = (dist_to_entry <= CORRIDOR_TOLERANCE_M)
            hattin_icinde   = (90.0 <= dist_to_qr <= (DIVE_ENTRY_DISTANCE_M + CORRIDOR_TOLERANCE_M))
            
            if (koridora_girdi or hattin_icinde) and alt_err < 10.0:
                self.get_logger().info("🚀 Giriş koridoru ve 10m tolerans doğrulandı. Dalış tetiklendi.")
                self._set_phase(MissionPhase.DIVE_INIT)

        elif phase == MissionPhase.DIVE_INIT:
            elapsed = self._phase_elapsed()
            self._set_mode("GUIDED_NOGPS")

            if (now - self.last_log_print_t) >= 0.4:
                self._publish_status(f"📍 Faz: DIVE_INIT | Süre: {elapsed:.1f}s")
                self.last_log_print_t = now

            bearing_deg  = self.dive_bearing_deg
            bearing_rad  = math.radians(bearing_deg)
            t_ratio      = min(elapsed / DIVE_INIT_DURATION_S, 1.0)
            pitch_target = DIVE_ANGLE_DEG * t_ratio
            thrust_ramp  = max(0.0, 0.8 * (1.0 - t_ratio))

            is_qr_valid = (time.time() - self.last_seen_qr_t) <= self.dead_reckoning_timeout
            if is_qr_valid:
                roll_corr, _ = self._camera_pid_corrections()
            else:
                v_dx = self._x() - self.dive_start_line_x
                v_dy = self._y() - self.dive_start_line_y
                cross_track_error = v_dx * self.dive_vector_y - v_dy * self.dive_vector_x
                heading_err = self._heading_error_from_bearing(bearing_rad)
                roll_corr   = max(-15.0, min(15.0, -math.degrees(heading_err) * 0.7 + (cross_track_error * 0.4)))

            self._publish_attitude_setpoint(roll_corr, pitch_target, bearing_deg, thrust_ramp)

            if elapsed >= DIVE_INIT_DURATION_S:
                self._set_phase(MissionPhase.DIVE)

        elif phase == MissionPhase.DIVE:
            elapsed = time.time() - self.dive_start_t if self.dive_start_t else 0.0
            alt     = self._alt()
            self._set_mode("GUIDED_NOGPS")

            if self.qr_detected or (time.time() - self.last_seen_qr_t) < 0.1:
                if not hasattr(self, 'qr_latched') or not self.qr_latched:
                    self.get_logger().info("🎯 [LATCH] QR kod konumu mühürlendi!")
                    self.qr_latched = True

            is_qr_valid = (time.time() - self.last_seen_qr_t) <= self.dead_reckoning_timeout

            if (now - self.last_log_print_t) >= 0.4:
                status_text = "ANLIK KİLİTLİ" if is_qr_valid else ("HAFIZADAN TAKİP" if getattr(self, 'qr_latched', False) else "VEKTÖR HATTI AKTİF")
                self._publish_status(f"📍 Faz: DIVE | İrtifa: {alt:.1f}m | QR: {status_text}")
                self.last_log_print_t = now

            if is_qr_valid:
                roll_corr, pitch_corr = self._camera_pid_corrections()
                bearing_deg = math.degrees(self._bearing_to_target())
            elif getattr(self, 'qr_latched', False):
                dx = self.target_x - self._x()
                dy = self.target_y - self._y()
                yatay_mesafe = math.sqrt(dx**2 + dy**2)
                bearing_deg = math.degrees(math.atan2(dy, dx))
                
                pitch_corr = max(-15.0, min(15.0, math.degrees(math.atan2(max(0.1, alt - self.target_z), yatay_mesafe)) - DIVE_ANGLE_DEG)) if yatay_mesafe > 2.0 else 0.0
                heading_err = self._heading_error_from_bearing(math.radians(bearing_deg))
                roll_corr   = max(-15.0, min(15.0, -math.degrees(heading_err) * 0.8))
            else:
                bearing_deg = self.dive_bearing_deg
                v_dx = self._x() - self.dive_start_line_x
                v_dy = self._y() - self.dive_start_line_y
                cross_track_error = v_dx * self.dive_vector_y - v_dy * self.dive_vector_x
                heading_err = self._heading_error_from_bearing(math.radians(bearing_deg))
                roll_corr   = max(-15.0, min(15.0, -math.degrees(heading_err) * 0.7 + (cross_track_error * 0.5)))
                pitch_corr  = 0.0

            self._publish_attitude_setpoint(roll_corr, DIVE_ANGLE_DEG + pitch_corr, bearing_deg, THRUST_DIVE)

            if alt <= RECOVERY_ALTITUDE_M or elapsed > 20.0:
                self._set_phase(MissionPhase.RECOVERY)

        elif phase == MissionPhase.RECOVERY:
            elapsed = self._phase_elapsed()
            alt     = self._alt()

            tirmanma_bitti = (alt >= SAFE_ALTITUDE_M - 2.0 or elapsed >= 12.0)

            if not tirmanma_bitti:
                if self.mode != "GUIDED":
                    self._set_mode("GUIDED")
                
                if self.mode in ("GUIDED", "GUIDED_NOGPS"):
                    heading_err = self._heading_error_from_bearing(math.radians(self.recovery_bearing_deg))
                    roll_corr   = max(-10.0, min(10.0, -math.degrees(heading_err) * 0.5))
                    smooth_factor    = math.sin(min(max(0.0, SAFE_ALTITUDE_M - alt) / (SAFE_ALTITUDE_M - RECOVERY_ALTITUDE_M), 1.0) * (math.pi / 2))

                    self._publish_attitude_setpoint(roll_corr, RECOVERY_MAX_PITCH_DEG * max(smooth_factor, 0.35), self.recovery_bearing_deg, THRUST_RECOVERY_BASE + (0.25 * smooth_factor))
            else:
                if self.mode != "LOITER":
                    self._set_mode("LOITER")
                    self._loiter_start_time = None  
                else:
                    if self._loiter_start_time is None:
                        self._loiter_start_time = time.time()
                        self.get_logger().info("⏳ [Emniyet] Pixhawk LOITER onayladı. Olduğu yerde dönüyor...")
                    
                    if (time.time() - self._loiter_start_time) >= LOITER_STABILITY_TIME_S:
                        self._set_phase(MissionPhase.DONE)

            if (now - self.last_log_print_t) >= 0.4:
                self._publish_status(f"📍 Faz: RECOVERY | İrtifa: {alt:.1f}m | Mod: {self.mode}")
                self.last_log_print_t = now

    def _check_and_print_menu_callback(self):
        if self.menu_triggered_for_done:
            print_menu()
            self.menu_triggered_for_done = False

    def _save_clip_worker(self, raw_frames: list, event_name: str):
        self.get_logger().info(f"🔍 [DEBUG] _save_clip_worker çağrıldı, frame sayısı: {len(raw_frames)}")
        if not raw_frames:
            self.get_logger().warn("⚠️ [DEBUG] raw_frames boş, kayıt yapılmıyor.")
            return
        try:
            os.makedirs(SAVE_FOLDER, exist_ok=True)
            h, w = raw_frames[0].shape[:2]
            local_path = os.path.join(SAVE_FOLDER, f"{event_name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4")
            self.get_logger().info(f"💾 [DEBUG] Hedef dosya: {local_path} | boyut: {w}x{h}")

            command = [
                "ffmpeg", "-y", "-f", "rawvideo", "-vcodec", "rawvideo",
                "-pix_fmt", "bgr24", "-s", f"{w}x{h}", "-r", str(FPS),
                "-i", "-", "-c:v", "hevc_nvenc", "-preset", "p4", "-rc", "vbr", "-cq", "28", "-pix_fmt", "yuv420p", local_path
            ]
            proc = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=subprocess.PIPE)

            write_error = None
            try:
                for f in raw_frames:
                    if f is not None and f.size > 0:
                        proc.stdin.write(f.tobytes())
                proc.stdin.close()
                proc.wait(timeout=VIDEO_TIMEOUT_S)
            except Exception as e:
                write_error = e
                stderr_out = proc.stderr.read().decode(errors="ignore") if proc.stderr else ""
                self.get_logger().error(f"❌ [DEBUG] nvenc yazma hatası: {e} | ffmpeg stderr: {stderr_out[:500]}")

            self.get_logger().info(f"🔍 [DEBUG] nvenc returncode: {proc.returncode}")

            if proc.returncode != 0:
                self.get_logger().warn("⚠️ [DEBUG] nvenc başarısız, libx264 fallback deneniyor...")
                fallback_cmd = [
                    "ffmpeg", "-y", "-f", "rawvideo", "-vcodec", "rawvideo",
                    "-pix_fmt", "bgr24", "-s", f"{w}x{h}", "-r", str(FPS),
                    "-i", "-", "-c:v", "libx264", "-preset", "ultrafast", "-pix_fmt", "yuv420p", local_path
                ]
                proc_fb = subprocess.Popen(fallback_cmd, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
                try:
                    for f in raw_frames:
                        if f is not None and f.size > 0:
                            proc_fb.stdin.write(f.tobytes())
                    proc_fb.stdin.close()
                    proc_fb.wait(timeout=VIDEO_TIMEOUT_S)
                except Exception as e:
                    stderr_out = proc_fb.stderr.read().decode(errors="ignore") if proc_fb.stderr else ""
                    self.get_logger().error(f"❌ [DEBUG] libx264 yazma hatası: {e} | ffmpeg stderr: {stderr_out[:500]}")

                self.get_logger().info(f"🔍 [DEBUG] libx264 returncode: {proc_fb.returncode}")

            if os.path.exists(local_path) and os.path.getsize(local_path) > 0:
                self.get_logger().info(f"✅ Video başarıyla kaydedildi: {local_path}")
            else:
                self.get_logger().error(f"❌ [DEBUG] Dosya oluşmadı ya da boş: {local_path}")

        except Exception as e:
            self.get_logger().error(f"❌ Video yazma hatası (genel): {e}")


def print_menu():
    print("\n╔══════════════════════════════════════════════════════════╗")
    print("║                SEMA AVIATION - KAMİKAZE                  ║")
    print("╠══════════════════════════════════════════════════════════╣")
    print("║  s → Görevi BAŞLAT (Hedef Seçimi ile)                    ║")
    print("║  a → GÖREVİ İPTAL ET (ABORT)                             ║")
    print("║  q → Programdan çık                                      ║")
    print("╚══════════════════════════════════════════════════════════╝\n")

def print_target_sub_menu():
    print("\n🎯 --- QR HEDEF LİSTESİ ---")
    for key, val in QR_TARGETS.items():
        print(f"  [{key}] -> {val['name']} (X: {val['x']}, Y: {val['y']})")
    print("───────────────────────────")

def keyboard_loop(node):
    pub = node.create_publisher(String, "/kamikaze/command", 10)
    print_menu()

    while rclpy.ok():
        try:
            cmd = input("Komut: ").strip().lower()
        except EOFError:
            break

        msg = String()
        if cmd == "s":
            print_target_sub_menu()
            try:
                target_choice = input("Hedef No Seçin (1-10): ").strip()
                target_idx    = int(target_choice)
                if target_idx in QR_TARGETS:
                    msg.data = f"START_{target_idx}"
                else:
                    print("⚠️ Geçersiz numara!")
                    continue
            except ValueError:
                print("⚠️ Sayısal değer girin!")
                continue
        elif cmd == "a":
            msg.data = "ABORT"
        elif cmd == "q":
            rclpy.shutdown()
            break
        else:
            continue
        pub.publish(msg)

def main(args=None):
    if not os.path.exists(SAVE_FOLDER): 
        os.makedirs(SAVE_FOLDER)
        
    rclpy.init(args=args)
    node = KamikazeNode()

    t = threading.Thread(target=keyboard_loop, args=(node,), daemon=True)
    t.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
