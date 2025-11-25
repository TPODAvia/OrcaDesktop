#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Always-on pseudo GPS -> InfluxDB v2 (RMF schema), aligned 5s cadence.

Schema (matches your RMF dashboards):
  measurement: rmf_robot_gps
  tag:         robot
  fields:      lat (float), lon (float), has_fix (int), speed (float, m/s), heading_deg (float)

Env vars:
  INFLUX_URL      (default: http://localhost:8086)
  INFLUX_TOKEN    (REQUIRED)
  INFLUX_ORG      (default: orca)
  INFLUX_BUCKET   (default: rmf)
  ROBOT_ID        (default: robot_1)  # falls back to VEHICLE_ID for compatibility
  PUBLISH_EVERY_SEC (default: 5)
  START_LAT       (default: 48.20833)
  START_LON       (default: 16.37306)
  GEN_SPEED_MPS   (default: 3.0)
  GEN_HEADING_DEG (default: 90)
  GEN_HEADING_JITTER (default: 10)
  UDP_ENABLE      (default: false)
  UDP_HOST        (default: 0.0.0.0)
  UDP_PORT        (default: 9001)

UDP JSON (optional):
  {"lat": ..., "lon": ..., "speed": 3.0, "heading_deg": 120.0, "has_fix": 1}
"""

import os, time, json, socket, threading, math, random
from datetime import datetime, timezone
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS

# ---------------- Influx config ----------------
INFLUX_URL    = os.getenv("INFLUX_URL", "http://localhost:8086")
INFLUX_TOKEN  = os.getenv("INFLUX_TOKEN", "")
INFLUX_ORG    = os.getenv("INFLUX_ORG", "orca")
INFLUX_BUCKET = os.getenv("INFLUX_BUCKET", "rmf")

# Match RMF tag naming; keep VEHICLE_ID as fallback for compatibility
ROBOT_ID      = os.getenv("ROBOT_ID") or os.getenv("VEHICLE_ID", "robot_1")

# ---------------- cadence / generator ----------------
PUBLISH_EVERY_SEC       = int(os.getenv("PUBLISH_EVERY_SEC", "5"))
START_LAT               = float(os.getenv("START_LAT", "48.20833"))
START_LON               = float(os.getenv("START_LON", "16.37306"))
GEN_SPEED_MPS           = float(os.getenv("GEN_SPEED_MPS", "3.0"))
GEN_HEADING_DEG         = float(os.getenv("GEN_HEADING_DEG", "90"))
GEN_HEADING_JITTER_DEG  = float(os.getenv("GEN_HEADING_JITTER", "10"))

# ---------------- optional UDP injector ----------------
UDP_ENABLE  = os.getenv("UDP_ENABLE", "false").lower() == "true"
UDP_HOST    = os.getenv("UDP_HOST", "0.0.0.0")
UDP_PORT    = int(os.getenv("UDP_PORT", "9001"))

# ---------------- safety ----------------
if not INFLUX_TOKEN:
    raise SystemExit("INFLUX_TOKEN is empty. Export a valid write token (DOCKER_INFLUXDB_INIT_ADMIN_TOKEN).")

# ---------------- client (sync writes => errors visible immediately) ----------------
client = InfluxDBClient(url=INFLUX_URL, token=INFLUX_TOKEN, org=INFLUX_ORG, timeout=10000)
write_api = client.write_api(write_options=SYNCHRONOUS)

def now_ns() -> int:
    return int(datetime.now(timezone.utc).timestamp() * 1e9)

def write_fix(lat: float, lon: float, speed: float = None, heading_deg: float = None, has_fix: int = 1):
    """
    Write one GPS point using RMF schema.
    """
    p = (
        Point("rmf_robot_gps")
        .tag("robot", ROBOT_ID)
        .field("lat", float(lat))
        .field("lon", float(lon))
        .field("has_fix", int(has_fix))
        .time(now_ns())
    )
    if speed is not None:
        p = p.field("speed", float(speed))          # m/s
    if heading_deg is not None:
        p = p.field("heading_deg", float(heading_deg))
    write_api.write(bucket=INFLUX_BUCKET, record=p)

# ---------------- shared state (so UDP can inject) ----------------
_lock = threading.Lock()
_current = dict(lat=START_LAT, lon=START_LON, speed=GEN_SPEED_MPS, heading_deg=GEN_HEADING_DEG, has_fix=1)

def set_point(lat, lon, speed=None, heading_deg=None, has_fix=1):
    with _lock:
        if speed is None:
            speed = _current["speed"]
        if heading_deg is None:
            heading_deg = _current["heading_deg"]
        _current.update(
            lat=float(lat), lon=float(lon),
            speed=float(speed), heading_deg=float(heading_deg),
            has_fix=int(has_fix)
        )

def get_point():
    with _lock:
        return dict(_current)

# ---------------- UDP injector (optional) ----------------
def run_udp():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_HOST, UDP_PORT))
    print(f"[GPS] UDP enabled at udp://{UDP_HOST}:{UDP_PORT} — send JSON {{lat,lon[,speed,heading_deg,has_fix]}}")
    while True:
        data, _ = sock.recvfrom(8192)
        try:
            o = json.loads(data.decode("utf-8"))
            set_point(
                o["lat"], o["lon"],
                o.get("speed"), o.get("heading_deg"),
                o.get("has_fix", 1)
            )
            print(f"[GPS][UDP] injected lat={o['lat']}, lon={o['lon']}")
        except Exception as e:
            print("[GPS][UDP] bad packet:", e)

# ---------------- generator + writer (aligned cadence) ----------------
def run_generator_writer():
    """
    Continuous random-walk that WRITES every PUBLISH_EVERY_SEC (aligned to cadence).
    """
    lat = START_LAT
    lon = START_LON
    heading_deg = GEN_HEADING_DEG

    M_PER_DEG_LAT = 111_320.0

    print(f"[GPS] Generator ON: start=({lat:.6f},{lon:.6f}), speed={GEN_SPEED_MPS} m/s, cadence={PUBLISH_EVERY_SEC}s")
    # align to next tick
    next_ts = time.time()
    next_ts = next_ts - (next_ts % PUBLISH_EVERY_SEC) + PUBLISH_EVERY_SEC

    while True:
        now = time.time()
        if now < next_ts:
            time.sleep(min(0.1, next_ts - now))
            continue

        # small heading jitter each step
        heading_deg += random.uniform(-GEN_HEADING_JITTER_DEG, GEN_HEADING_JITTER_DEG)

        # displacement over the interval
        d = GEN_SPEED_MPS * PUBLISH_EVERY_SEC  # meters
        # convert meters -> degrees
        dlat = (d * math.cos(math.radians(heading_deg))) / M_PER_DEG_LAT
        m_per_deg_lon = 111_320.0 * math.cos(math.radians(lat)) or 1e-6
        dlon = (d * math.sin(math.radians(heading_deg))) / m_per_deg_lon

        lat += dlat
        lon += dlon

        # keep in legal ranges (wrap lon, clamp lat)
        lat = max(-90.0, min(90.0, lat))
        lon = ((lon + 180.0) % 360.0) - 180.0

        # allow last-minute UDP injection to override before write
        cur = get_point()
        # If external injection differs a lot, snap to it
        if abs(cur["lat"] - lat) > 0.05 or abs(cur["lon"] - lon) > 0.05:
            lat, lon = cur["lat"], cur["lon"]
            heading_deg = cur["heading_deg"]
        else:
            set_point(lat, lon, GEN_SPEED_MPS, heading_deg, 1)

        try:
            write_fix(lat, lon, GEN_SPEED_MPS, heading_deg, 1)
            print(f"[Influx] wrote robot={ROBOT_ID} lat={lat:.6f} lon={lon:.6f} heading_deg={heading_deg:.1f}")
        except Exception as e:
            print("[Influx] write error:", e)

        next_ts += PUBLISH_EVERY_SEC

def main():
    print(f"[GPS] Influx={INFLUX_URL} org={INFLUX_ORG} bucket={INFLUX_BUCKET} robot={ROBOT_ID}")
    if UDP_ENABLE:
        threading.Thread(target=run_udp, daemon=True).start()
    threading.Thread(target=run_generator_writer, daemon=True).start()
    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        print("\n[GPS] stopping.")

if __name__ == "__main__":
    main()
