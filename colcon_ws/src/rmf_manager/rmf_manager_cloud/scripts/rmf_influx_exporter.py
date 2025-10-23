#!/usr/bin/env python3
# rmf_influx_exporter.py
# Ingests ALL fields from rmf_manager_msgs/RobotState and writes to InfluxDB 2.x
# - Auto-discovers every <ns>/rmf/state
# - Robust to publisher restarts (periodic re-scan)
# - Uses header.stamp when present; falls back to wall clock
# - Tags kept low-cardinality; everything else as fields (incl. strings)

import os, time, math, threading
from typing import Dict, Optional, List
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from rmf_manager_msgs.msg import RobotState

from influxdb_client import InfluxDBClient, Point, WriteOptions
from influxdb_client.client.write_api import ASYNCHRONOUS

# ---------------- Env config ----------------
INFLUX_URL    = os.getenv("INFLUX_URL", "http://127.0.0.1:8086")
INFLUX_TOKEN  = os.getenv("INFLUX_TOKEN", "supersecrettoken")
INFLUX_ORG    = os.getenv("INFLUX_ORG", "orca")
INFLUX_BUCKET = os.getenv("INFLUX_BUCKET", "rmf")
MEASUREMENT   = os.getenv("MEASUREMENT", "rmf_robot_state")

DISCOVERY_SEC = float(os.getenv("DISCOVERY_SEC", "2.0"))  # topic rescan period
# --------------------------------------------

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

def _csv(items: Optional[List[str]]) -> Optional[str]:
    if not items:
        return ""
    # join with '|' to avoid commas in names breaking CSV reads
    try:
        return "|".join(str(s) for s in items)
    except Exception:
        return ""

def _yaw_from_quat(q) -> Optional[float]:
    try:
        # ENU yaw from quaternion (x,y,z,w)
        s = 2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y))
        c = 1.0 - 2.0 * (float(q.y) * float(q.y) + float(q.z) * float(q.z))
        return math.atan2(s, c)
    except Exception:
        return None

@dataclass
class SubEntry:
    ns: str
    sub: any

class RMFInfluxExporter(Node):
    def __init__(self):
        super().__init__("rmf_influx_exporter")

        # InfluxDB client
        self._client = InfluxDBClient(
            url=INFLUX_URL, token=INFLUX_TOKEN, org=INFLUX_ORG, timeout=30_000
        )
        self._write = self._client.write_api(write_options=WriteOptions(
            batch_size=20, flush_interval=1000, jitter_interval=250,
            retry_interval=2000, write_type=ASYNCHRONOUS
        ))

        # QoS: rmf/state generally VOLATILE + BEST_EFFORT
        self._qos_state = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._subs: Dict[str, SubEntry] = {}
        self._lock = threading.Lock()

        self.create_timer(DISCOVERY_SEC, self._discover)

        self.get_logger().info(
            f"Influx -> {INFLUX_URL} org={INFLUX_ORG} bucket={INFLUX_BUCKET} measurement={MEASUREMENT}"
        )
        self.get_logger().info("Scanning for */rmf/state…")

    # --------------- discovery ---------------
    def _discover(self):
        try:
            topics = self.get_topic_names_and_types()
        except Exception as e:
            self.get_logger().warn(f"topic discovery failed: {e}")
            return

        want = set()
        for name, _types in topics:
            if name.endswith("/rmf/state"):
                ns = name[:-len("/rmf/state")] or ""
                want.add(ns)

        with self._lock:
            # add new
            for ns in want:
                if ns in self._subs:
                    continue
                topic = f"{ns}/rmf/state" if ns else "/rmf/state"
                self.get_logger().info(f"Subscribe: {topic}")
                sub = self.create_subscription(
                    RobotState, topic, lambda m, ns=ns: self._on_state(ns, m), self._qos_state
                )
                self._subs[ns] = SubEntry(ns=ns, sub=sub)

            # remove disappeared
            for ns in [k for k in self._subs.keys() if k not in want]:
                try:
                    self.destroy_subscription(self._subs[ns].sub)
                except Exception:
                    pass
                del self._subs[ns]
                self.get_logger().warn(f"Unsubscribed (topic gone): {ns or '/'} /rmf/state")

    # --------------- handle RobotState ---------------
    def _on_state(self, ns: str, msg: RobotState):
        # Tags (keep cardinality low)
        robot   = msg.robot_name or ""
        mapname = msg.map_name or ""
        floor   = msg.floor_name or ""
        mode_t  = msg.mode_text or ""
        task_ty = msg.current_task_type or ""

        # Time
        ts_ns = self._stamp_ns(msg)

        # Numeric core
        x = _flt(getattr(msg.pose, "x", None))
        y = _flt(getattr(msg.pose, "y", None))
        theta = _flt(getattr(msg.pose, "theta", None))

        batt_pct      = _flt(msg.battery_pct)
        batt_voltage  = _flt(msg.battery_voltage)
        vehicle_speed = _flt(msg.vehicle_speed)
        internal_temp = _flt(msg.internal_temp)
        external_temp = _flt(msg.external_temp)
        barometer     = _flt(msg.barometer)

        # statuses (uint8 -> int fields)
        status_fields = {
            "motor_status":         _int(msg.motor_status),
            "lidar_status":         _int(msg.lidar_status),
            "imu_status":           _int(msg.imu_status),
            "mag_status":           _int(msg.mag_status),
            "gps_status":           _int(msg.gps_status),
            "screen_status":        _int(msg.screen_status),
            "camera_status":        _int(msg.camera_status),
            "cpu_status":           _int(msg.cpu_status),
            "gpu_status":           _int(msg.gpu_status),
            "network_status":       _int(msg.network_status),
            "led_status":           _int(msg.led_status),
            "odom_status":          _int(msg.odom_status),
            "map_status":           _int(msg.map_status),
            "nav_status":           _int(msg.nav_status),
            "rmf_status":           _int(msg.rmf_status),
            "diagnostic_status":    _int(msg.diagnostic_status),
        }

        # WCS (GPS/IMU)
        has_fix = 1 if bool(getattr(msg, "wcs_has_fix", False)) else 0
        lat     = _flt(getattr(msg, "wcs_lat", None))
        lon     = _flt(getattr(msg, "wcs_lon", None))
        alt     = _flt(getattr(msg, "wcs_alt", None))
        yaw_enu = None
        heading_deg = None
        try:
            q = msg.wcs_orientation_enu
            yaw_enu = _yaw_from_quat(q)
            if yaw_enu is not None and math.isfinite(yaw_enu):
                heading_deg = 90.0 - math.degrees(yaw_enu)  # 0=north, CW+
        except Exception:
            pass

        # Strings as fields (Influx supports string fields)
        # Keep current_task_id as field to avoid tag cardinality explosion
        path_csv     = _csv(msg.path_vertices)
        reserved_csv = _csv(msg.reserved_vertices)

        p = Point(MEASUREMENT)\
            .tag("ns", ns or "/")\
            .tag("robot", robot)\
            .tag("map", mapname)\
            .tag("floor", floor)\
            .tag("mode_text", mode_t)\
            .tag("task_type", task_ty)\
            .field("mode", _int(msg.mode))\
            .field("current_task_id", msg.current_task_id or "")\
            .field("current_vertex", msg.current_vertex or "")\
            .field("next_vertex", msg.next_vertex or "")\
            .field("path_vertices_csv", path_csv)\
            .field("reserved_vertices_csv", reserved_csv)\
            .field("path_len", len(msg.path_vertices) if msg.path_vertices is not None else 0)\
            .field("reserved_len", len(msg.reserved_vertices) if msg.reserved_vertices is not None else 0)\
            .field("x", x).field("y", y).field("theta", theta)\
            .field("battery_pct", batt_pct)\
            .field("battery_voltage", batt_voltage)\
            .field("vehicle_speed", vehicle_speed)\
            .field("internal_temp", internal_temp)\
            .field("external_temp", external_temp)\
            .field("barometer", barometer)\
            .field("has_fix", has_fix)\
            .field("lat", lat).field("lon", lon).field("alt", alt)\
            .field("yaw_enu", yaw_enu if (yaw_enu is not None and math.isfinite(yaw_enu)) else None)\
            .field("heading_deg", heading_deg if (heading_deg is not None and math.isfinite(heading_deg)) else None)\
            .time(ts_ns)

        # add all status fields
        for k, v in status_fields.items():
            if v is not None:
                p.field(k, v)

        try:
            self._write.write(bucket=INFLUX_BUCKET, org=INFLUX_ORG, record=p)
        except Exception as e:
            self.get_logger().warn(f"influx write failed: {e}")

    def _stamp_ns(self, msg: RobotState) -> int:
        # Prefer msg.header.stamp; fall back to wall clock
        try:
            st = msg.header.stamp
            return int(st.sec) * 1_000_000_000 + int(st.nanosec)
        except Exception:
            return int(time.time() * 1e9)

def main():
    import sys
    # optional --ros-domain-id passthrough
    if "--ros-domain-id" in sys.argv:
        try:
            i = sys.argv.index("--ros-domain-id")
            if i + 1 < len(sys.argv):
                os.environ["ROS_DOMAIN_ID"] = str(int(sys.argv[i + 1]))
            del sys.argv[i:i+2]
        except Exception:
            sys.argv = [a for a in sys.argv if a != "--ros-domain-id"]

    rclpy.init(args=sys.argv)
    node = RMFInfluxExporter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try: node.destroy_node()
        except Exception: pass
        try:
            if rclpy.ok(): rclpy.shutdown()
        except Exception: pass

if __name__ == "__main__":
    main()
