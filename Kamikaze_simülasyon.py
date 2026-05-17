#!/usr/bin/env python3
import os
import math
import time
import threading
import collections
import requests
from datetime import datetime
from enum import Enum, auto

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# MAVROS mesaj tipleri
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3
from mavros_msgs.msg import State, AttitudeTarget, OverrideRCIn, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Header, Bool, String

# ─── YAPILANDIRMA ─────────────────────────────────────────────────────────────

# Görev parametreleri (Rapor §4.2)
DIVE_ALTITUDE_M       = 105.0   # A noktası irtifası [m]
SAFE_ALTITUDE_M       = 105.0   # Güvenli kalkış irtifası
RECOVERY_ALTITUDE_M   = 25   # C noktası / güvenli minimum irtifa [m]
DIVE_HORIZONTAL_M     = 129.31  # A noktasının hedefe yatay uzaklığı [m]
DIVE_ANGLE_DEG        = 45.0    # Dalış açısı [derece]
INIT_SPEED_MS         = 15.0    # A noktasındaki başlangıç hızı [m/s]
CRUISE_SPEED_MS       = 16.0    # Seyir hızı [m/s]
MAX_DIVE_SPEED_MS     = 36.62   # Teorik azami dalış hızı [m/s]
DIVE_RADIUS_M         = 27.31   # A→B dönüş yarıçapı [m]
RECOVERY_RADIUS_M     = 71.81   # C noktası toparlanma yarıçapı [m]

# Simülasyon QR hedef koordinatı
QR_TARGET_X = -29.0
QR_TARGET_Y = 500.0
QR_TARGET_Z =  0.01

# Kamera PID kazançları (piksel hatası → açı hatası)
PID_KP_ROLL  = 0.35  
PID_KI_ROLL  = 0.005
PID_KD_ROLL  = 0.05
PID_KP_PITCH = 0.35
PID_KI_PITCH = 0.005
PID_KD_PITCH = 0.05

# Sunucu (yarışma)
CONTEST_SERVER_URL = "http://127.0.0.1:8080/api/kamikaze"

# Kayıt Çıktı Konumu
SAVE_CLIP = True
SAVE_FOLDER = os.path.expanduser("~/Kayit")

# ─── GÖREV DURUM MAKİNESİ ─────────────────────────────────────────────────────

class MissionPhase(Enum):
    IDLE        = auto()
    ARM_TAKEOFF = auto()
    CLIMB       = auto()
    POSITION    = auto()
    DIVE_INIT   = auto()
    DIVE        = auto()
    RECOVERY    = auto()
    DONE        = auto()
    ABORT       = auto()

# ─── YARDIMCI: PID ────────────────────────────────────────────────────────────

class PID:
    def __init__(self, kp, ki, kd, out_min=-1.0, out_max=1.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self._integral = 0.0
        self._prev_err = 0.0
        self._last_t   = None

    def reset(self):
        self._integral = 0.0
        self._prev_err = 0.0
        self._last_t   = None

    def update(self, error: float) -> float:
        now = time.time()
        dt  = (now - self._last_t) if self._last_t else 0.033
        self._last_t = now
        self._integral  += error * dt
        derivative       = (error - self._prev_err) / max(dt, 1e-6)
        self._prev_err   = error
        out = self.kp * error + self.ki * self._integral + self.kd * derivative
        return max(self.out_min, min(self.out_max, out))

# ─── ANA NODE ─────────────────────────────────────────────────────────────────

class KamikazeNode(Node):
    def __init__(self):
        super().__init__("kamikaze_node")

        # ── Durum değişkenleri
        self.phase            = MissionPhase.IDLE
        self.current_pose     = PoseStamped()
        self.current_velocity = TwistStamped()
        self.mav_state        = State()
        self.armed            = False
        self.mode             = ""
        self.last_mode_req_t  = 0.0
        self.last_arm_req_t   = 0.0

        # QR
        self.qr_detected   = False
        self.qr_content    = None
        self.qr_bbox       = None          
        self.frame_buffer  = collections.deque(maxlen=90)  
        self.dive_frames   = []

        # PID kontrolcüler (kamera merkezleme)
        self.pid_roll  = PID(PID_KP_ROLL,  PID_KI_ROLL,  PID_KD_ROLL,  -30, 30)
        self.pid_pitch = PID(PID_KP_PITCH, PID_KI_PITCH, PID_KD_PITCH, -30, 30)
        self.cam_w = 640
        self.cam_h = 480

        # Zaman damgaları
        self.phase_start_t = time.time()
        self.dive_start_t  = None

        # Durak noktaları
        self.target_a_x = 0.0
        self.target_a_y = 0.0
        self.virtual_wp_x = 0.0
        self.virtual_wp_y = 0.0
        
        # Uzaklaşma mekanizması değişkenleri
        self.moving_away = False

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        reliable_qos = QoSProfile(depth=10)

        # Subscriber'lar
        self.create_subscription(State, "/mavros/state", self._cb_state, reliable_qos)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose", self._cb_pose, sensor_qos)
        self.create_subscription(TwistStamped, "/mavros/local_position/velocity_local", self._cb_vel, sensor_qos)
        self.create_subscription(Image, "/zephyr/camera/front/image_raw", self._cb_camera, sensor_qos)
        self.create_subscription(String, "/kamikaze/command", self._cb_command, reliable_qos)

        # Publisher'lar
        self.pub_setpoint_pos = self.create_publisher(PositionTarget, "/mavros/setpoint_raw/local", 10)
        self.pub_setpoint_att = self.create_publisher(AttitudeTarget, "/mavros/setpoint_raw/attitude", 10)
        self.pub_status = self.create_publisher(String, "/kamikaze/status", 10)
        self.pub_rc_override = self.create_publisher(OverrideRCIn, "/mavros/rc/override", 10)

        # Servisler
        self.cli_arm      = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.cli_mode     = self.create_client(SetMode,     "/mavros/set_mode")
        self.cli_takeoff  = self.create_client(CommandTOL,  "/mavros/cmd/takeoff")

        # Zamanlayıcılar
        self.create_timer(0.05, self._control_loop)
        self.create_timer(0.1, self._publish_heartbeat_setpoint)

        self._qrdet = None
        self._qreader = None
        self._load_qr_libs()

        self.get_logger().info("🎯 Kamikaze Node başlatıldı. '/kamikaze/command' topic'ine 'START' gönderin.")

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

    def _cb_state(self, msg: State):
        self.mav_state = msg
        self.armed     = msg.armed
        self.mode      = msg.mode

    def _cb_pose(self, msg: PoseStamped):
        self.current_pose = msg

    def _cb_vel(self, msg: TwistStamped):
        self.current_velocity = msg

    def _cb_camera(self, msg: Image):
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
        if msg.encoding == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        self.cam_h, self.cam_w = frame.shape[:2]
        self.frame_buffer.append(frame.copy())

        if self.phase in (MissionPhase.DIVE, MissionPhase.DIVE_INIT):
            self.dive_frames.append(frame.copy())
            self._detect_qr(frame)

    def _cb_command(self, msg: String):
        cmd = msg.data.strip().upper()
        if cmd == "START" and self.phase == MissionPhase.IDLE:
            if not self.mav_state.connected:
                self.get_logger().error("⚠️ SITL (MAVROS) bağlantısı yok!")
                return

            self.get_logger().info("🚀 Kamikaze görevi BAŞLATILDI.")
            
            rc_msg = OverrideRCIn()
            rc_msg.channels = [0] * 18
            self.pub_rc_override.publish(rc_msg)

            bearing = self._bearing_to_target()
            
            # L1 Navigasyonunun rotayı bozmadan uçağı düz getirmesi için
            # sanal waypoint'i QR hedefine göre 300 metre ileri atıyoruz.
            self.virtual_wp_x = QR_TARGET_X + 300.0 * math.cos(bearing)
            self.virtual_wp_y = QR_TARGET_Y + 300.0 * math.sin(bearing)

            # Eğer uçak QR koduna 200 metereden yakınsa otonom uzaklaşmayı tetikle
            initial_dist = self._dist_to_target_xy()
            if initial_dist < 200.0:
                self.moving_away = True
                self.get_logger().info(f"⚠️ QR koda çok yakın başlanıldı ({initial_dist:.1f} m)! Güvenli hizalanma için arkaya doğru uzaklaşılıyor...")
            else:
                self.moving_away = False
            
            self._set_phase(MissionPhase.ARM_TAKEOFF)
        elif cmd == "ABORT":
            self.get_logger().warn("🛑 ABORT komutu alındı!")
            self._set_phase(MissionPhase.ABORT)

    def _detect_qr(self, frame: np.ndarray):
        if self.qr_detected:
            return  

        bbox = None
        content = None

        if self._qrdet is not None:
            detections = self._qrdet.detect(image=frame, is_bgr=True)
            if detections:
                det = detections[0]
                x1, y1, x2, y2 = (int(det["bbox_xyxy"][i]) for i in range(4))
                bbox = ((x1 + x2) // 2, (y1 + y2) // 2, x2 - x1, y2 - y1)

        if bbox is None:
            detector = cv2.QRCodeDetector()
            data, pts, _ = detector.detectAndDecode(frame)
            if pts is not None:
                pts = pts.astype(int).reshape(-1, 2)
                cx = int(pts[:, 0].mean())
                cy = int(pts[:, 1].mean())
                w  = int(pts[:, 0].max() - pts[:, 0].min())
                h  = int(pts[:, 1].max() - pts[:, 1].min())
                bbox = (cx, cy, w, h)
                content = data if data else None

        self.qr_bbox = bbox

        if bbox is not None and content is None:
            if self._qreader is not None:
                try:
                    results = self._qreader.detect_and_decode(image=frame, is_bgr=True)
                    if results:
                        content = results[0]
                except Exception as e:
                    self.get_logger().debug(f"qreader hata: {e}")

        if content:
            self.qr_detected = True
            self.qr_content  = content
            self.get_logger().info(f"✅ QR KOD ÇÖZÜLDÜ: {content}")
            self._report_to_server(content)

    def _report_to_server(self, qr_content: str):
        payload = {
            "team_id": "755335",
            "qr_data": qr_content,
            "timestamp": datetime.utcnow().isoformat(),
            "altitude": self._alt(),
        }
        def _send():
            try:
                r = requests.post(CONTEST_SERVER_URL, json=payload, timeout=3)
                self.get_logger().info(f"📡 Sunucu yanıtı: {r.status_code}")
            except Exception as e:
                self.get_logger().warn(f"Sunucu gönderim hatası: {e}")
        threading.Thread(target=_send, daemon=True).start()

    def _alt(self) -> float:
        return self.current_pose.pose.position.z

    def _x(self) -> float:
        return self.current_pose.pose.position.x

    def _y(self) -> float:
        return self.current_pose.pose.position.z # local_position odom izdüşümü

    def _speed(self) -> float:
        v = self.current_velocity.twist.linear
        return math.sqrt(v.x**2 + v.y**2 + v.z**2)

    def _dist_to_target_xy(self) -> float:
        dx = self.current_pose.pose.position.x - QR_TARGET_X
        dy = self.current_pose.pose.position.y - QR_TARGET_Y
        return math.sqrt(dx**2 + dy**2)

    def _bearing_to_target(self) -> float:
        dx = QR_TARGET_X - self.current_pose.pose.position.x
        dy = QR_TARGET_Y - self.current_pose.pose.position.y
        return math.atan2(dy, dx)

    def _bearing_to_point(self, px: float, py: float) -> float:
        return math.atan2(py - self.current_pose.pose.position.y, px - self.current_pose.pose.position.x)

    def _get_heading_error(self) -> float:
        q = self.current_pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)
        target_bearing = self._bearing_to_target()
        err = target_bearing - current_yaw
        return (err + math.pi) % (2 * math.pi) - math.pi

    def _phase_elapsed(self) -> float:
        return time.time() - self.phase_start_t

    def _set_phase(self, phase: MissionPhase):
        self.get_logger().info(f"  ▶ Faz: {self.phase.name} → {phase.name}")
        self.phase       = phase
        self.phase_start_t = time.time()
        
        if phase == MissionPhase.DIVE:
            self.pid_roll.reset()
            self.pid_pitch.reset()
            self.dive_start_t = time.time()
            self.dive_frames  = []

    def _publish_position_setpoint(self, x: float, y: float, z: float,
                                   yaw_rad: float = 0.0):
        now = time.time()
        if not hasattr(self, '_last_pos_sp'):
            self._last_pos_sp = None
            self._last_pos_sp_t = 0.0

        send_it = False
        if self._last_pos_sp is None:
            send_it = True
        else:
            lx, ly, lz = self._last_pos_sp
            dist = math.sqrt((x - lx)**2 + (y - ly)**2 + (z - lz)**2)
            if dist > 2.0 or (now - self._last_pos_sp_t) > 2.0:
                send_it = True

        if not send_it:
            return

        self._last_pos_sp = (x, y, z)
        self._last_pos_sp_t = now

        msg = PositionTarget()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
                         PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                         PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE)
                         
        msg.position.x = float(x)
        msg.position.y = float(y)
        msg.position.z = -float(z)
        
        self.pub_setpoint_pos.publish(msg)

    def _publish_attitude_setpoint(self, roll_deg: float, pitch_deg: float,
                                   yaw_deg: float, thrust: float = 0.0):
        msg = AttitudeTarget()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.type_mask = AttitudeTarget.IGNORE_ROLL_RATE  \
                      | AttitudeTarget.IGNORE_PITCH_RATE \
                      | AttitudeTarget.IGNORE_YAW_RATE

        roll  = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw   = math.radians(yaw_deg)

        cr = math.cos(roll  / 2); sr = math.sin(roll  / 2)
        cp = math.cos(pitch / 2); sp = math.sin(pitch / 2)
        cy = math.cos(yaw   / 2); sy = math.sin(yaw   / 2)

        msg.orientation.w = cr * cp * cy + sr * sp * sy
        msg.orientation.x = sr * cp * cy - cr * sp * sy
        # 🔥 Piste yan gitme problemini çözen kuaterniyon formülü (cy -> sy düzeltildi)
        msg.orientation.y = cr * sp * cy + sr * cp * sy
        msg.orientation.z = cr * cp * sy - sr * sp * cy
        msg.thrust = float(max(0.0, min(1.0, thrust)))

        self.pub_setpoint_att.publish(msg)

    def _publish_heartbeat_setpoint(self):
        if self.phase == MissionPhase.IDLE:
            self._publish_position_setpoint(
                self._x(), self._y(), max(self._alt(), 2.0))

    def _set_mode(self, mode: str):
        if not self.cli_mode.service_is_ready():
            return False
        req = SetMode.Request()
        req.custom_mode = mode
        self.cli_mode.call_async(req)
        return True

    def _arm(self, arm: bool = True):
        if not self.cli_arm.service_is_ready():
            return False
        req = CommandBool.Request()
        req.value = arm
        self.cli_arm.call_async(req)
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

        return delta_roll * half_fov_h, delta_pitch * half_fov_v

    def _control_loop(self):
        phase = self.phase

        if phase == MissionPhase.IDLE:
            return

        elif phase == MissionPhase.ARM_TAKEOFF:
            self._publish_status("📍 Faz: ARM_TAKEOFF")
            if self._phase_elapsed() < 2.0:
                self._publish_position_setpoint(self.virtual_wp_x, self.virtual_wp_y, SAFE_ALTITUDE_M, self._bearing_to_target())
                return

            if self.mode != "GUIDED":
                if time.time() - self.last_mode_req_t > 2.0:
                    self._set_mode("GUIDED")
                    self.last_mode_req_t = time.time()
                self._publish_position_setpoint(self.virtual_wp_x, self.virtual_wp_y, SAFE_ALTITUDE_M, 0.0)
                return

            if not self.armed:
                if time.time() - self.last_arm_req_t > 2.0:
                    self._arm(True)
                    self.last_arm_req_t = time.time()
                self._publish_position_setpoint(self.virtual_wp_x, self.virtual_wp_y, SAFE_ALTITUDE_M, 0.0)
                return

            if self.armed and self.mode == "GUIDED":
                self._set_phase(MissionPhase.CLIMB)

        elif phase == MissionPhase.CLIMB:
            target_alt = DIVE_ALTITUDE_M
            self._publish_status(f"📍 Faz: CLIMB | İrtifa: {self._alt():.1f} / {target_alt} m")

            self._publish_position_setpoint(self.virtual_wp_x, self.virtual_wp_y, target_alt, self._bearing_to_target())

            if abs(self._alt() - target_alt) <= 3.0:
                self.get_logger().info(f"✅ Hedef irtifa {target_alt} m'ye ulaşıldı.")
                self._set_phase(MissionPhase.POSITION)

        elif phase == MissionPhase.POSITION:
            dist_to_qr = self._dist_to_target_xy()  
            alt_err  = abs(self._alt() - DIVE_ALTITUDE_M)
            heading_err = abs(math.degrees(self._get_heading_error()))

            # 🔥 ÇOK YAKIN BAŞLAMA KORUMASI (Görsel / Otonom Fly-Away Manevrası):
            if self.moving_away:
                self._publish_status(f"📍 Faz: POSITION | Çok yakın başlandı, otonom uzaklaşılıyor... QR Mesafe: {dist_to_qr:.1f} m")
                
                # Hedef yönün tam aksine (geriye) dönmesi için gereken hayali kaçış açısı
                target_bearing = self._bearing_to_target()
                away_bearing = (target_bearing + math.pi + math.pi) % (2 * math.pi) - math.pi
                
                # Mevcut yaw açısını alıp arkaya olan sapmayı hesapla
                q = self.current_pose.pose.orientation
                siny_cosp = 2 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
                current_yaw = math.atan2(siny_cosp, cosy_cosp)
                
                away_err = (away_bearing - current_yaw + math.pi) % (2 * math.pi) - math.pi
                
                # Roll açısıyla uçağa L1 hatasına takılmadan temiz bir U dönüşü yaptırıyoruz
                roll_cmd = -math.degrees(away_err) * 1.5
                roll_cmd = max(-30.0, min(30.0, roll_cmd))
                
                # Güvenli irtifada arkaya doğru gaz açıp uzaklaş
                self._publish_attitude_setpoint(
                    roll_deg  = roll_cmd,
                    pitch_deg = 2.0, # Hafif irtifa koruma burnu
                    yaw_deg   = math.degrees(away_bearing),
                    thrust    = 0.8
                )
                
                # QR koddan 250 metre açıldığımız an uzaklaşma fazı biter ve ana düz şeride kilitlenir
                if dist_to_qr >= 250.0:
                    self.get_logger().info("✅ Güvenli kaçış mesafesi açıldı. Rota şeridine geri kilitleniliyor.")
                    self.moving_away = False
                return # Uzaklaşırken dalış şartlarını sorgulama, ara döngüyü kes.

            # Normal Düz Çizgi Takip Döngüsü (Uzaklaşma bittikten sonra burası çalışır)
            self._publish_status(f"📍 Faz: POSITION | Rota Sabitlendi | QR'a Mesafe: {dist_to_qr:.1f} m | Yön Sapması: {heading_err:.0f}°")
            self._publish_position_setpoint(self.virtual_wp_x, self.virtual_wp_y, DIVE_ALTITUDE_M, self._bearing_to_target())

            # Dalış yayı gecikmesini düşürerek 164 metre çizgisine girdiğinde dalışı başlat
            trigger_distance = DIVE_HORIZONTAL_M + 35.0

            if dist_to_qr <= trigger_distance and heading_err < 20.0 and alt_err < 15.0:
                self.get_logger().info(f"🚀 Uçak {dist_to_qr:.1f} m'de hatta oturdu! (Erken Tetikleme) → Dalış başlıyor!")
                self._set_phase(MissionPhase.DIVE_INIT)

        elif phase == MissionPhase.DIVE_INIT:
            elapsed = self._phase_elapsed()
            self._publish_status(f"📍 Faz: DIVE_INIT | Süre: {elapsed:.1f} s | İrtifa: {self._alt():.1f} m")

            bearing_deg = math.degrees(self._bearing_to_target())
            t_ratio = min(elapsed / 3.0, 1.0)
            pitch_target = DIVE_ANGLE_DEG * t_ratio

            if self.qr_detected:
                roll_corr, _ = self._camera_pid_corrections()
            else:
                heading_err = self._get_heading_error()
                roll_corr = max(-35.0, min(35.0, -math.degrees(heading_err) * 1.5))

            self._publish_attitude_setpoint(roll_deg=roll_corr, pitch_deg=pitch_target, yaw_deg=bearing_deg, thrust=0.8)
            if self._alt() <= DIVE_ALTITUDE_M - 8.0 or elapsed > 3.5:
                self._set_phase(MissionPhase.DIVE)

        elif phase == MissionPhase.DIVE:
            elapsed = self._phase_elapsed()
            alt     = self._alt()
            spd     = self._speed()
            bearing_deg = math.degrees(self._bearing_to_target())

            self._publish_status(f"📍 Faz: DIVE | İrtifa: {alt:.1f} m | Hız: {spd:.1f} m/s | QR: {'✅' if self.qr_detected else '❌'}")

            if self.qr_detected:
                roll_corr, pitch_corr = self._camera_pid_corrections()
            else:
                heading_err = self._get_heading_error()
                roll_corr = max(-35.0, min(35.0, -math.degrees(heading_err) * 1.5))
                pitch_corr = 0.0
                
            pitch_dive = DIVE_ANGLE_DEG + pitch_corr
            self._publish_attitude_setpoint(roll_deg=roll_corr, pitch_deg=pitch_dive, yaw_deg=bearing_deg, thrust=0.8)

            if alt <= RECOVERY_ALTITUDE_M:
                self.get_logger().info(f"⚠️  C noktasına ulaşıldı ({alt:.1f} m) → Toparlanma!")
                self._set_phase(MissionPhase.RECOVERY)
            elif self.qr_detected and elapsed > 4.0:
                self._set_phase(MissionPhase.RECOVERY)
            elif elapsed > 25.0:
                self._set_phase(MissionPhase.RECOVERY)

        elif phase == MissionPhase.RECOVERY:
            self._publish_status(f"📍 Faz: RECOVERY | İrtifa: {self._alt():.1f} m | Hız: {self._speed():.1f} m/s")

            bearing_deg = math.degrees(self._bearing_to_target())
            heading_err = self._get_heading_error()
            roll_corr = max(-35.0, min(35.0, -math.degrees(heading_err) * 1.5))
            
            self._publish_attitude_setpoint(roll_deg=roll_corr, pitch_deg=-15.0, yaw_deg=bearing_deg, thrust=0.9)

            if self._alt() >= SAFE_ALTITUDE_M - 10.0 and self._phase_elapsed() > 3.0:
                self.get_logger().info("✅ Güvenli irtifaya ulaşıldı!")
                self._set_mode("LOITER")
                self._set_phase(MissionPhase.DONE)

        elif phase == MissionPhase.DONE:
            if self._phase_elapsed() < 1.0:
                self._publish_status(f"🏁 Kamikaze görevi TAMAMLANDI | QR: {self.qr_content or 'OKUNAMADı'}")
            if SAVE_CLIP and self.dive_frames:
                threading.Thread(target=self._save_clip, args=(list(self.dive_frames),), daemon=True).start()
                self.dive_frames = []

        elif phase == MissionPhase.ABORT:
            if self._phase_elapsed() < 1.0:
                self.get_logger().warn("🛑 ABORT → LOITER moduna geçiliyor.")
                self._set_mode("LOITER")

    def _save_clip(self, frames: list):
        if not frames:
            return
        os.makedirs(SAVE_FOLDER, exist_ok=True)
        h, w = frames[0].shape[:2]
        ts  = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = f"{SAVE_FOLDER}/kamikaze_{ts}.mp4"
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(path, fourcc, 30, (w, h))
        for f in frames:
            writer.write(f)
        writer.release()
        self.get_logger().info(f"🎥 Dalış klibi kaydedildi: {path}")

def main(args=None):
    rclpy.init(args=args)
    node = KamikazeNode()

    def keyboard_loop():
        pub = node.create_publisher(String, "/kamikaze/command", 10)
        print("\n╔══════════════════════════════════╗")
        print("║   SEMA AVIATION - KAMİKAZE       ║")
        print("╠══════════════════════════════════╣")
        print("║  s → Görevi BAŞLAT               ║")
        print("║  a → ABORT                       ║")
        print("║  q → Programdan çık              ║")
        print("╚══════════════════════════════════╝\n")
        while rclpy.ok():
            try:
                cmd = input("Komut: ").strip().lower()
            except EOFError:
                break
            msg = String()
            if cmd == "s":
                msg.data = "START"
            elif cmd == "a":
                msg.data = "ABORT"
            elif cmd == "q":
                rclpy.shutdown()
                break
            else:
                continue
            pub.publish(msg)

    t = threading.Thread(target=keyboard_loop, daemon=True)
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
