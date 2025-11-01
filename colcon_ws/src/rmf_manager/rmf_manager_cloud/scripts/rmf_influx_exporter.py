#!/usr/bin/env python3
"""
rmf_influx_exporter.py — ROS2 → InfluxDB (Flux/Grafana-friendly)

Measurements (stable field types!):
  • rmf_robot_metrics  (tags: ns,robot,map,floor)  fields: x,y,theta,battery_pct,battery_voltage,vehicle_speed,internal_temp,external_temp,barometer,path_len,reserved_len
  • rmf_robot_gps      (tags: ns,robot)           fields: lat,lon,alt,heading_deg,has_fix
  • rmf_robot_status   (tags: ns,robot,component) fields: code(int: 0 OK,1 WARN,2 ERROR), text(string: "OK"|"WARN"|"ERROR"|"UNKNOWN")
  • rmf_robot_info     (tags: ns,robot,map,floor) fields: current_task_id,current_vertex,next_vertex,path_vertices_csv,reserved_vertices_csv,mode(int),mode_text,task_type

Notes:
  - Status normalization accepts ints (0/1/2) AND strings ("OK"/"WARN"/"ERROR"), always writes numeric .code + string .text.
  - GPS falls back to MapMetadata WCS if native WGS84 is missing.
  - Uses header.stamp (ns) when available.
  - Auto-discovers */rmf/state and */rmf/map_metadata periodically.

Env:
  INFLUX_URL, INFLUX_TOKEN, INFLUX_ORG, INFLUX_BUCKET
  MEAS_METRICS, MEAS_GPS, MEAS_STATUS, MEAS_INFO
  DISCOVERY_SEC
"""

import os
import time
import math
import threading
from typing import Dict, Optional, List, Tuple
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from rmf_manager_msgs.msg import RobotState, MapMetadata

from influxdb_client import InfluxDBClient, Point, WriteOptions
from influxdb_client.client.write_api import ASYNCHRONOUS


# ========================== Env config ==========================
INFLUX_URL    = os.getenv("INFLUX_URL", "http://127.0.0.1:8086")
INFLUX_TOKEN  = os.getenv("INFLUX_TOKEN", "supersecrettoken")
INFLUX_ORG    = os.getenv("INFLUX_ORG", "orca")
INFLUX_BUCKET = os.getenv("INFLUX_BUCKET", "rmf")

MEAS_METRICS  = os.getenv("MEAS_METRICS", "rmf_robot_metrics")
MEAS_GPS      = os.getenv("MEAS_GPS", "rmf_robot_gps")
MEAS_STATUS   = os.getenv("MEAS_STATUS", "rmf_robot_status")
MEAS_INFO     = os.getenv("MEAS_INFO", "rmf_robot_info")

DISCOVERY_SEC = float(os.getenv("DISCOVERY_SEC", "2.0"))  # topic rescan period
# ================================================================


# =========================== Helpers ============================
def _flt(x) -> Optional[float]:
    try:
        v = float(x)
        return v if math.isfinite(v) else None
    except Exception:
        return None


def _int(x) -> Optional[int]:
    try:
        return int(x)
    except Exception:
        return None


def _csv(items: Optional[List[str]]) -> str:
    if not items:
        return ""
    try:
        return "|".join(str(s) for s in items)
    except Exception:
        return ""


def _yaw_from_quat(q) -> Optional[float]:
    """Return yaw (rad) from quaternion (geometry_msgs/Quaternion-like)."""
    try:
        s = 2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y))
        c = 1.0 - 2.0 * (float(q.y) * float(q.y) + float(q.z) * float(q.z))
        return math.atan2(s, c)
    except Exception:
        return None


def _valid_latlon(lat: Optional[float], lon: Optional[float]) -> bool:
    if lat is None or lon is None:
        return False
    try:
        return (
            math.isfinite(lat) and math.isfinite(lon) and
            -90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0
        )
    except Exception:
        return False


def _wrap_deg_0_360(deg: Optional[float]) -> Optional[float]:
    if deg is None or not math.isfinite(deg):
        return None
    d = deg % 360.0
    if d < 0:
        d += 360.0
    return d


# ---- ENU→WGS84 helpers ----
_EARTH_R = 6378137.0  # WGS84 equatorial radius (m)


def _enu_to_latlon(lat0_deg: float, lon0_deg: float, east_m: float, north_m: float) -> Tuple[float, float]:
    lat0 = math.radians(lat0_deg)
    dlat = north_m / _EARTH_R
    dlon = east_m / (_EARTH_R * math.cos(lat0))
    return math.degrees(lat0 + dlat), lon0_deg + math.degrees(dlon)


# =========================== Models ============================
@dataclass
class GeoRef:
    available: bool = False
    lat0: float = float("nan")
    lon0: float = float("nan")
    yaw_map_to_enu: float = 0.0
    # Optional map affine (kept for forward compatibility; not used by default)
    scale: float = 1.0
    rotation: float = 0.0
    tx: float = 0.0
    ty: float = 0.0

    @staticmethod
    def from_meta(m: MapMetadata) -> "GeoRef":
        def f(name, d=0.0):
            return float(getattr(m, name, d))
        return GeoRef(
            available=bool(getattr(m, "wcs_available", False)),
            lat0=float(getattr(m, "wcs_origin_lat", float("nan"))),
            lon0=float(getattr(m, "wcs_origin_lon", float("nan"))),
            yaw_map_to_enu=float(getattr(m, "wcs_yaw", 0.0)),
            scale=f("scale", 1.0),
            rotation=f("rotation", 0.0),
            tx=f("translation_x", 0.0),
            ty=f("translation_y", 0.0),
        )


@dataclass
class SubEntry:
    ns: str
    sub: any
# ===============================================================


# ========================== Main node ==========================
class RMFInfluxExporter(Node):
    def __init__(self):
        super().__init__("rmf_influx_exporter")

        # InfluxDB client
        self._client = InfluxDBClient(
            url=INFLUX_URL, token=INFLUX_TOKEN, org=INFLUX_ORG, timeout=30_000
        )
        self._write = self._client.write_api(
            write_options=WriteOptions(
                batch_size=100,
                flush_interval=1000,
                jitter_interval=250,
                retry_interval=2000,
                write_type=ASYNCHRONOUS,
            )
        )

        # QoS
        self._qos_state = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._qos_meta = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._subs_state: Dict[str, SubEntry] = {}
        self._subs_meta: Dict[str, SubEntry] = {}
        self._georef: Dict[str, GeoRef] = {}  # ns -> GeoRef
        self._lock = threading.Lock()

        self.create_timer(DISCOVERY_SEC, self._discover)

        self.get_logger().info(f"Influx -> {INFLUX_URL} org={INFLUX_ORG} bucket={INFLUX_BUCKET}")
        self.get_logger().info("Scanning for */rmf/state and */rmf/map_metadata …")

    # -------------------- Discovery --------------------
    def _discover(self):
        try:
            topics = self.get_topic_names_and_types()
        except Exception as e:
            self.get_logger().warn(f"topic discovery failed: {e}")
            return

        want_ns = set()
        want_meta = set()
        for name, _types in topics:
            if name.endswith("/rmf/state"):
                ns = name[:-len("/rmf/state")] or ""
                want_ns.add(ns)
            if name.endswith("/rmf/map_metadata"):
                ns = name[:-len("/rmf/map_metadata")] or ""
                want_meta.add(ns)

        with self._lock:
            # state: add
            for ns in want_ns:
                if ns not in self._subs_state:
                    topic = f"{ns}/rmf/state" if ns else "/rmf/state"
                    self.get_logger().info(f"Subscribe: {topic}")
                    sub = self.create_subscription(
                        RobotState, topic, lambda m, ns=ns: self._on_state(ns, m), self._qos_state
                    )
                    self._subs_state[ns] = SubEntry(ns=ns, sub=sub)
            # state: remove
            for ns in [k for k in list(self._subs_state.keys()) if k not in want_ns]:
                try:
                    self.destroy_subscription(self._subs_state[ns].sub)
                except Exception:
                    pass
                del self._subs_state[ns]
                self.get_logger().warn(f"Unsubscribed (state gone): {ns or '/'} /rmf/state")

            # meta: add
            for ns in want_meta:
                if ns not in self._subs_meta:
                    topic = f"{ns}/rmf/map_metadata" if ns else "/rmf/map_metadata"
                    self.get_logger().info(f"Subscribe: {topic}")
                    subm = self.create_subscription(
                        MapMetadata, topic, lambda m, ns=ns: self._on_meta(ns, m), self._qos_meta
                    )
                    self._subs_meta[ns] = SubEntry(ns=ns, sub=subm)
            # meta: remove
            for ns in [k for k in list(self._subs_meta.keys()) if k not in want_meta]:
                try:
                    self.destroy_subscription(self._subs_meta[ns].sub)
                except Exception:
                    pass
                del self._subs_meta[ns]
                self.get_logger().warn(f"Unsubscribed (meta gone): {ns or '/'} /rmf/map_metadata")

    def _on_meta(self, ns: str, m: MapMetadata):
        try:
            self._georef[ns] = GeoRef.from_meta(m)
            self.get_logger().info(f"[meta] ns='{ns or '/'}' georef available={self._georef[ns].available}")
        except Exception as e:
            self.get_logger().warn(f"[meta] parse failed: {e}")

    # -------------------- State handler --------------------
    def _on_state(self, ns: str, msg: RobotState):
        robot = msg.robot_name or ""
        mapname = msg.map_name or ""
        floor = msg.floor_name or ""
        ts_ns = self._stamp_ns(msg)

        # Pose & metrics
        x = _flt(getattr(msg.pose, "x", None))
        y = _flt(getattr(msg.pose, "y", None))
        theta = _flt(getattr(msg.pose, "theta", None))

        batt_pct = _flt(getattr(msg, "battery_pct", None))     # keep 0..1 (Grafana scales to %)
        batt_voltage = _flt(getattr(msg, "battery_voltage", None))
        vehicle_speed = _flt(getattr(msg, "vehicle_speed", None))
        internal_temp = _flt(getattr(msg, "internal_temp", None))
        external_temp = _flt(getattr(msg, "external_temp", None))
        barometer = _flt(getattr(msg, "barometer", None))

        path_len = len(msg.path_vertices) if msg.path_vertices is not None else 0
        reserved_len = len(msg.reserved_vertices) if msg.reserved_vertices is not None else 0

        # GPS native
        has_fix = 1 if bool(getattr(msg, "wcs_has_fix", False)) else 0
        lat = _flt(getattr(msg, "wcs_lat", None))
        lon = _flt(getattr(msg, "wcs_lon", None))
        alt = _flt(getattr(msg, "wcs_alt", None))
        heading_deg = None

        # Heading from ENU quaternion (if available)
        try:
            q = getattr(msg, "wcs_orientation_enu", None)
            if q:
                yaw = _yaw_from_quat(q)
                if yaw is not None and math.isfinite(yaw):
                    # yaw is CCW from +X; compass heading is 0=N, 90=E
                    heading_deg = 90.0 - math.degrees(yaw)
        except Exception:
            pass

        # GPS fallback via georef + local pose
        if (not _valid_latlon(lat, lon)) or has_fix == 0:
            g = self._georef.get(ns)
            if g and g.available and math.isfinite(g.lat0) and math.isfinite(g.lon0):
                x_m = x or 0.0
                y_m = y or 0.0
                # Rotate map (x,y) into ENU using georef yaw
                c, s = math.cos(g.yaw_map_to_enu), math.sin(g.yaw_map_to_enu)
                east = c * x_m - s * y_m
                north = s * x_m + c * y_m
                lat_f, lon_f = _enu_to_latlon(g.lat0, g.lon0, east, north)
                if _valid_latlon(lat_f, lon_f):
                    lat, lon = lat_f, lon_f
                    has_fix = 1
                    if heading_deg is None:
                        th = theta or 0.0
                        heading_deg = 90.0 - math.degrees(g.yaw_map_to_enu + th)

        # Final heading normalization
        heading_deg = _wrap_deg_0_360(heading_deg)

        # ---------------- Build Influx points ----------------
        recs: List[Point] = []

        # metrics
        p_metrics = Point(MEAS_METRICS).tag("ns", ns or "/").tag("robot", robot).tag("map", mapname).tag("floor", floor)
        _field = p_metrics.field  # shorthand; only add when value is not None
        if x is not None: _field("x", x)
        if y is not None: _field("y", y)
        if theta is not None: _field("theta", theta)
        if batt_pct is not None: _field("battery_pct", batt_pct)
        if batt_voltage is not None: _field("battery_voltage", batt_voltage)
        if vehicle_speed is not None: _field("vehicle_speed", vehicle_speed)
        if internal_temp is not None: _field("internal_temp", internal_temp)
        if external_temp is not None: _field("external_temp", external_temp)
        if barometer is not None: _field("barometer", barometer)
        _field("path_len", int(path_len))
        _field("reserved_len", int(reserved_len))
        p_metrics.time(ts_ns)
        recs.append(p_metrics)

        # gps — write ONLY when lat/lon are valid (Geomap requires both in same row)
        if _valid_latlon(lat, lon):
            p_gps = Point(MEAS_GPS).tag("ns", ns or "/").tag("robot", robot)
            p_gps.field("lat", float(lat))
            p_gps.field("lon", float(lon))
            if alt is not None and math.isfinite(alt):
                p_gps.field("alt", alt)
            if heading_deg is not None:
                p_gps.field("heading_deg", heading_deg)
            p_gps.field("has_fix", 1 if has_fix else 0)
            p_gps.time(ts_ns)
            recs.append(p_gps)

        # info (strings + mode int)
        p_info = Point(MEAS_INFO).tag("ns", ns or "/").tag("robot", robot).tag("map", mapname).tag("floor", floor)
        mode_i = _int(getattr(msg, "mode", None))
        p_info.field("current_task_id", msg.current_task_id or "")
        p_info.field("current_vertex", msg.current_vertex or "")
        p_info.field("next_vertex", msg.next_vertex or "")
        p_info.field("path_vertices_csv", _csv(msg.path_vertices))
        p_info.field("reserved_vertices_csv", _csv(msg.reserved_vertices))
        if mode_i is not None:
            p_info.field("mode", mode_i)
        p_info.field("mode_text", msg.mode_text or "")
        p_info.field("task_type", msg.current_task_type or "")
        p_info.time(ts_ns)
        recs.append(p_info)

        # statuses — normalized code + text
        component_fields = {
            "motor": "motor_status",
            "lidar": "lidar_status",
            "imu": "imu_status",
            "mag": "mag_status",
            "gps": "gps_status",
            "screen": "screen_status",
            "camera": "camera_status",
            "cpu": "cpu_status",
            "gpu": "gpu_status",
            "network": "network_status",
            # project-optional:
            "led": "led_status",
            "odom": "odom_status",
            "map": "map_status",
            "nav": "nav_status",
            "rmf": "rmf_status",
            "diagnostic": "diagnostic_status",
        }
        for comp, attr in component_fields.items():
            if not hasattr(msg, attr):
                continue
            code = _status_code(getattr(msg, attr))
            if code is None:
                continue
            text = _status_text_from_code(code)
            p_stat = (
                Point(MEAS_STATUS)
                .tag("ns", ns or "/")
                .tag("robot", robot)
                .tag("component", comp)
                .field("code", int(code))
                .field("text", text)
                .time(ts_ns)
            )
            recs.append(p_stat)

        # write batch
        try:
            if recs:
                self._write.write(bucket=INFLUX_BUCKET, org=INFLUX_ORG, record=recs)
        except Exception as e:
            self.get_logger().warn(f"influx write failed: {e}")

    # -------------------- Timestamp helper --------------------
    def _stamp_ns(self, msg: RobotState) -> int:
        try:
            st = msg.header.stamp
            return int(st.sec) * 1_000_000_000 + int(st.nanosec)
        except Exception:
            return int(time.time() * 1e9)
# ===============================================================


def _status_code(val) -> Optional[int]:
    """
    Normalize various incoming status representations to numeric code:
      0 = OK, 1 = WARN, 2 = ERROR
    Accepts ints, floats, strings like "OK"/"WARN"/"ERROR"/"0"/"1"/"2".
    """
    if val is None:
        return None

    # numeric path
    if isinstance(val, (int, float)):
        v = int(val)
        if v <= 0:
            return 0
        if v == 1:
            return 1
        if v >= 2:
            return 2
        return None

    # string path
    try:
        s = str(val).strip().lower()
    except Exception:
        return None

    if s in ("ok", "okay", "good", "pass", "green", "0", "true", "up"):
        return 0
    if s in ("warn", "warning", "yellow", "1"):
        return 1
    if s in ("error", "err", "fail", "failed", "critical", "crit", "red", "down", "2"):
        return 2

    # last resort: numeric cast
    try:
        v = int(float(s))
        return _status_code(v)
    except Exception:
        return None


def _status_text_from_code(code: Optional[int]) -> Optional[str]:
    if code is None:
        return None
    return "OK" if code == 0 else ("WARN" if code == 1 else ("ERROR" if code == 2 else "UNKNOWN"))


# ============================= Main ============================
def main():
    import sys

    # Optional convenience: allow "--ros-domain-id <N>"
    if "--ros-domain-id" in sys.argv:
        try:
            i = sys.argv.index("--ros-domain-id")
            if i + 1 < len(sys.argv):
                os.environ["ROS_DOMAIN_ID"] = str(int(sys.argv[i + 1]))
            del sys.argv[i : i + 2]
        except Exception:
            sys.argv = [a for a in sys.argv if a != "--ros-domain-id"]

    rclpy.init(args=sys.argv)
    node = RMFInfluxExporter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
