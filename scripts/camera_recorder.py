#!/usr/bin/env python3
import os
import time
import threading
import collections
import subprocess
from datetime import datetime

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# --- YAPILANDIRMA ---
SERVER_RTMP_URL = "rtmp://127.0.0.1:1935/live/test"
SAVE_FOLDER = os.path.expanduser("~/Kayit")
FPS = 30
BUFFER_SECONDS = 5  # Daha uzun tampon
WIDTH, HEIGHT = 1280, 720

frame_buffer = collections.deque(maxlen=FPS * BUFFER_SECONDS)
is_running = True
stream_proc = None

def start_ffmpeg_stream(width: int, height: int):
    command = [
        "ffmpeg", "-y",
        "-loglevel", "error",
        "-f", "rawvideo", "-vcodec", "rawvideo",
        "-pix_fmt", "bgr24", "-s", f"{width}x{height}", "-r", str(FPS),
        "-i", "-",
        "-c:v", "h264_nvenc",
        "-pix_fmt", "yuv420p", # Tarayıcılar için en uyumlu format
        "-preset", "p1",
        "-tune", "ull",
        "-profile:v", "main",   # Firefox/Chrome için standart profil
        "-g", "30",            # Keyframe aralığını FPS ile eşitle (Önemli!)
        "-f", "flv",
        SERVER_RTMP_URL,
    ]
    return subprocess.Popen(command, stdin=subprocess.PIPE)

def save_event_clip(clip_frames, event_name):
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{event_name}_{timestamp_str}.mp4"
    local_path = os.path.join(SAVE_FOLDER, filename)

    print(f"\n[SİSTEM] {event_name} sıkıştırılarak kaydediliyor...")

    if not clip_frames:
        return

    height, width = clip_frames[0].shape[:2]
    
    # Kayıt için NVENC: Ultra Sıkıştırma (HEVC/H.265)
    # H.264'ten %50 daha fazla sıkıştırır!
    command = [
        "ffmpeg", "-y",
        "-f", "rawvideo", "-vcodec", "rawvideo",
        "-pix_fmt", "bgr24", "-s", f"{width}x{height}", "-r", str(FPS),
        "-i", "-",
        "-c:v", "hevc_nvenc", # RTX 4050 için en iyi sıkıştırma
        "-preset", "p6",      # Daha yavaş/kaliteli paketleme
        "-rc", "vbr",
        "-cq", "32",          # Daha yüksek sayı = Daha küçük dosya
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

        # BGR dönüşümü ve kare okuma
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
        if msg.encoding == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        if not self.streaming:
            WIDTH, HEIGHT = msg.width, msg.height
            stream_proc = start_ffmpeg_stream(WIDTH, HEIGHT)
            self.streaming = True
            print(f"[SİSTEM] Yayın başladı!")

        # Ekrana tarih yaz
        timestamp_text = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        cv2.putText(frame, timestamp_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        frame_buffer.append(frame.copy())

        try:
            if stream_proc and stream_proc.stdin:
                stream_proc.stdin.write(frame.tobytes())
                stream_proc.stdin.flush()
        except:
            pass

def main():
    if not os.path.exists(SAVE_FOLDER): os.makedirs(SAVE_FOLDER)
    
    rclpy.init()
    camera_node = GazeboCameraNode()
    
    # ROS 2'yi ayrı thread'de çalıştır
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
            elif cmd == "q": break
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()