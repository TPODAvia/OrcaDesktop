#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS 2 Humble node: HTTP MJPEG server for 6 synthetic videos + one white-noise audio stream.

- 6 x MJPEG endpoints: /cam0 .. /cam5
- 1 x WAV (PCM) endless white-noise endpoint: /audio.wav
- Simple index page at /

Params:
  cams (int, default=6)
  width (int, default=640)
  height (int, default=360)
  fps (int, default=15)
  http_port (int, default=8000)
  show_timestamp (bool, default=True)
  text_color (str, default="lime")  # CSS color or #RRGGBB
  audio_rate (int, default=48000)   # Hz
  audio_channels (int, default=1)   # 1=mono, 2=stereo
  audio_chunk_ms (int, default=100) # chunk size the server pushes

Dependencies:
  sudo apt-get install -y python3-flask python3-numpy python3-pil
  (or) pip install flask numpy pillow

Run:
  ros2 run <your_pkg> cam_noise_node.py
  # or set params:
  ros2 run <your_pkg> cam_noise_node.py --ros-args -p http_port:=8088 -p fps:=20 -p audio_channels:=2
"""

import io
import time
import threading
import datetime
import struct
from typing import Generator, List

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from flask import Flask, Response, send_file, abort

import rclpy
from rclpy.node import Node


# ------------------------
# Global Flask app
# ------------------------
app = Flask(__name__)


class SixCamsHttpNoise(Node):
    def __init__(self):
        super().__init__('six_cams_http_noise')

        # --- Parameters ---
        self.declare_parameter('cams', 6)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 360)
        self.declare_parameter('fps', 15)
        self.declare_parameter('http_port', 8000)
        self.declare_parameter('show_timestamp', True)
        self.declare_parameter('text_color', 'lime')
        self.declare_parameter('audio_rate', 48000)
        self.declare_parameter('audio_channels', 1)
        self.declare_parameter('audio_chunk_ms', 100)

        self.cams        = int(self.get_parameter('cams').value)
        self.W           = int(self.get_parameter('width').value)
        self.H           = int(self.get_parameter('height').value)
        self.FPS         = int(self.get_parameter('fps').value)
        self.HTTP_PORT   = int(self.get_parameter('http_port').value)
        self.SHOW_TS     = bool(self.get_parameter('show_timestamp').value)
        self.TEXT_COLOR  = str(self.get_parameter('text_color').value)
        self.A_RATE      = int(self.get_parameter('audio_rate').value)
        self.A_CH        = int(self.get_parameter('audio_channels').value)
        self.A_MS        = int(self.get_parameter('audio_chunk_ms').value)

        if self.FPS <= 0:
            self.FPS = 15
        if self.cams < 1:
            self.cams = 6
        if self.A_CH not in (1, 2):
            self.A_CH = 1
        if self.A_MS < 10:
            self.A_MS = 50

        self.period = 1.0 / float(self.FPS)

        # Font
        try:
            self.font = ImageFont.truetype(
                "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", size=22
            )
        except Exception:
            self.font = ImageFont.load_default()

        # Pre-create different RNG seeds per camera to get distinct noise
        self.cam_rng: List[np.random.Generator] = [
            np.random.default_rng(seed=12345 + i) for i in range(self.cams)
        ]

        # Bind Flask routes
        self._bind_routes()

        # Start HTTP server
        self._start_http_server()

        self.get_logger().info(
            f"HTTP MJPEG on 0.0.0.0:{self.HTTP_PORT} | cams={self.cams} "
            f"{self.W}x{self.H}@{self.FPS}fps | audio={self.A_RATE}Hz/{self.A_CH}ch"
        )
        self.get_logger().info(
            f"Open: http://127.0.0.1:{self.HTTP_PORT}/  "
            f"or single feeds: /cam0..cam{self.cams-1}, audio: /audio.wav"
        )

    # ------------------------
    # Video: MJPEG generators
    # ------------------------
    def _mjpeg_generator(self, cam_idx: int) -> Generator[bytes, None, None]:
        """
        MJPEG stream for a given camera index.
        """
        rng = self.cam_rng[cam_idx]
        boundary = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
        while rclpy.ok():
            t0 = time.time()
            # White-noise RGB (use rng for stable variability per camera)
            frame = rng.integers(0, 256, size=(self.H, self.W, 3), dtype=np.uint8)
            img = Image.fromarray(frame, mode="RGB")
            if self.SHOW_TS:
                draw = ImageDraw.Draw(img)
                ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                draw.text((8, 8), f"Cam {cam_idx} - {ts}", fill=self.TEXT_COLOR, font=self.font)
            # Encode JPEG
            buf = io.BytesIO()
            img.save(buf, format="JPEG", quality=80)
            jpeg = buf.getvalue()
            buf.close()
            yield boundary + jpeg + b"\r\n"

            # Pace
            dt = time.time() - t0
            sleep_left = self.period - dt
            if sleep_left > 0:
                time.sleep(sleep_left)

    # ------------------------
    # Audio: endless WAV (white noise)
    # ------------------------
    def _wav_header(self, sample_rate: int, channels: int, bits_per_sample: int = 16) -> bytes:
        """
        Builds a 'infinite' WAV header. We set sizes to 0xFFFFFFFF so players
        (VLC et al.) can accept endless chunked streaming.
        """
        byte_rate = sample_rate * channels * (bits_per_sample // 8)
        block_align = channels * (bits_per_sample // 8)
        data_size = 0xFFFFFFFF
        riff_size = 0xFFFFFFFF

        return b"".join([
            b"RIFF",
            struct.pack("<I", riff_size),
            b"WAVE",
            b"fmt ",
            struct.pack("<I", 16),                         # PCM fmt chunk size
            struct.pack("<H", 1),                          # AudioFormat = 1 (PCM)
            struct.pack("<H", channels),
            struct.pack("<I", sample_rate),
            struct.pack("<I", byte_rate),
            struct.pack("<H", block_align),
            struct.pack("<H", bits_per_sample),
            b"data",
            struct.pack("<I", data_size),
        ])

    def _audio_generator(self) -> Generator[bytes, None, None]:
        """
        Streams a WAV header once, then endless white-noise PCM chunks.
        """
        # Header first
        yield self._wav_header(self.A_RATE, self.A_CH, 16)

        # Chunk length in samples
        n = int(self.A_RATE * self.A_MS / 1000)
        if n <= 0:
            n = 1024

        # White noise PCM16
        while rclpy.ok():
            # float noise -> int16
            noise = (np.random.randn(n, self.A_CH) * 12000.0).clip(-32768, 32767).astype(np.int16)
            yield noise.tobytes(order="C")

    # ------------------------
    # Flask routes
    # ------------------------
    def _bind_routes(self):
        node = self

        @app.route("/")
        def index():
            # Simple page with all six streams and an audio player
            imgs = "\n".join(
                f"<div style='padding:6px;display:inline-block'><h4>Cam {i}</h4>"
                f"<img src='/cam{i}' style='max-width: 320px; height: auto; border:1px solid #ccc;' /></div>"
                for i in range(node.cams)
            )
            body = (
                "<html><head><meta charset='utf-8'><title>Noise Streams</title></head>"
                "<body style='font-family:sans-serif'>"
                "<h2>White-Noise Streams</h2>"
                f"{imgs}"
                "<div style='clear:both;padding-top:12px'>"
                "<h4>White-noise audio</h4>"
                "<audio controls autoplay loop src='/audio.wav'></audio>"
                "</div>"
                "</body></html>"
            )
            return Response(body, mimetype="text/html")

        # Create /cam0..N-1 endpoints
        for i in range(self.cams):
            route = f"/cam{i}"

            def make_cam(idx: int):
                def cam():
                    return Response(
                        node._mjpeg_generator(idx),
                        mimetype="multipart/x-mixed-replace; boundary=frame",
                    )
                cam.__name__ = f"cam_{idx}"  # Flask needs unique name
                return cam

            app.add_url_rule(route, view_func=make_cam(i), methods=["GET"])

        @app.route("/audio.wav")
        def audio_wav():
            return Response(
                node._audio_generator(),
                mimetype="audio/wav"
            )

    # ------------------------
    # HTTP server thread
    # ------------------------
    def _start_http_server(self):
        def _run():
            app.run(host="0.0.0.0", port=self.HTTP_PORT, threaded=True, debug=False)

        th = threading.Thread(target=_run, name="flask-http", daemon=True)
        th.start()

    # ------------------------
    # Cleanup
    # ------------------------
    def destroy_node(self):
        super().destroy_node()


def main():
    rclpy.init()
    node = SixCamsHttpNoise()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
