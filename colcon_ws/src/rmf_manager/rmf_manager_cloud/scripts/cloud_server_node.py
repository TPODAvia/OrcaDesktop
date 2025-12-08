#!/usr/bin/env python3
# Multi-robot OSM view with vertices/lanes + optional raster overlay
# - Discovers namespaces by /<ns>/rmf/state
# - Eagerly creates service clients so they appear in `ros2 node info`
# - Full task-control REST: list, patrol, goto, delivery, cancel(s), resume_after_charge, trigger
# - Return-home via /rmf/command publisher
# - Path endpoint returns current planned path in WGS84 for each ns

import os, sys, math, time, json, threading
import yaml
from dataclasses import dataclass
from typing import Dict, Tuple, Optional, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.qos_event import SubscriptionEventCallbacks

from flask import Flask, jsonify, render_template, make_response, request
from waitress import serve

from rmf_manager_msgs.msg import RobotState, MapMetadata
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Quaternion, Twist

# Optional cv2 encoder
try:
    from cv_bridge import CvBridge
    import cv2
    import numpy as np
    _HAS_CV = True
except Exception:
    _HAS_CV = False

# Services + command publisher (best-effort import)
try:
    from rmf_manager_msgs.srv import (
        SchedulePatrol,
        ScheduleDelivery,
        ScheduleGoTo,
        CancelTask,
        CancelAll,
        ListTasks,
        SetResumeAfterCharge,
        TriggerDelivery,
    )
    _HAS_SRVS = True
except Exception:
    _HAS_SRVS = False

try:
    from rmf_manager_msgs.msg import RobotCommand
    _HAS_CMD = True
except Exception:
    _HAS_CMD = False


# ---------------- models ----------------

@dataclass(frozen=True)
class MapKey:
    ns: str
    map_name: str
    floor_name: str

@dataclass
class GeoRef:
    available: bool = False
    lat0: float = float("nan")
    lon0: float = float("nan")
    alt0: float = float("nan")
    yaw_map_to_enu: float = 0.0
    epsg: str = ""

    @classmethod
    def from_meta(cls, m: MapMetadata) -> "GeoRef":
        def _f(x, d=0.0):
            try: return float(x)
            except Exception: return d
        return cls(
            available=bool(getattr(m, "wcs_available", False)),
            lat0=_f(getattr(m, "wcs_origin_lat", float("nan")), float("nan")),
            lon0=_f(getattr(m, "wcs_origin_lon", float("nan")), float("nan")),
            alt0=_f(getattr(m, "wcs_origin_alt", float("nan")), float("nan")),
            yaw_map_to_enu=_f(getattr(m, "wcs_yaw", 0.0), 0.0),
            epsg=str(getattr(m, "wcs_epsg", "")),
        )

@dataclass
class RobotEntry:
    ns: str
    last_state: Optional[RobotState] = None
    last_seen: float = 0.0
    lat: float = float("nan")
    lon: float = float("nan")
    alt: float = float("nan")
    heading_deg: float = float("nan")
    map_key: Optional[MapKey] = None
    has_fix: bool = False


# ---------------- helpers ----------------

def quat_to_yaw(q: Quaternion) -> float:
    x, y, z, w = q.x, q.y, q.z, q.w
    s = 2.0 * (w * z + x * y)
    c = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(s, c)

def wrap_deg(d: float) -> float:
    while d <= -180.0: d += 360.0
    while d >   180.0: d -= 360.0
    return d

_EARTH_R = 6378137.0
def enu_to_latlon(lat0_deg: float, lon0_deg: float, east_m: float, north_m: float) -> Tuple[float, float]:
    lat0 = math.radians(lat0_deg)
    dlat = north_m / _EARTH_R
    dlon = east_m / (_EARTH_R * math.cos(lat0))
    return math.degrees(lat0 + dlat), lon0_deg + math.degrees(dlon)


# ---------------- main ----------------

class MultiRobotDashboard(Node):
    def __init__(self):
        super().__init__("rmf_dashboard_multi")

        # Params
        self.ui_host = self.declare_parameter("ui_host", "0.0.0.0").get_parameter_value().string_value
        self.ui_port = int(self.declare_parameter("ui_port", 5080).get_parameter_value().integer_value or 5080)
        self.viz_update_hz = float(self.declare_parameter("viz_update_hz", 1.0).get_parameter_value().double_value or 1.0)
        self.discovery_hz = float(self.declare_parameter("discovery_hz", 0.5).get_parameter_value().double_value or 0.5)
        self.debug = bool(self.declare_parameter("debug", True).get_parameter_value().bool_value)
        self.image_overlay_enable = bool(self.declare_parameter("image_overlay_enable", True).get_parameter_value().bool_value)
        self.image_overlay_force_bbox = bool(self.declare_parameter("image_overlay_force_bbox", False).get_parameter_value().bool_value)

        # Same value as map_image_downscale_factor in rmf_manager_bridge
        # 0 < factor <= 1.0, 1.0 means no downscale
        self.image_downscale_factor = float(
            self.declare_parameter("map_image_downscale_factor", 1.0)
                .get_parameter_value().double_value or 1.0
        )

        # go2rtc integration (RTSP → WebRTC/HLS) for camera visualization
        # Path to go2rtc.yaml & optional explicit base URL (if empty, UI will assume same host:1984)
        self.go2rtc_config_path = self.declare_parameter(
            "go2rtc_config_path", "/config/go2rtc.yaml"
        ).get_parameter_value().string_value
        self.go2rtc_base_url = self.declare_parameter(
            "go2rtc_base_url", ""
        ).get_parameter_value().string_value

        # ns -> list of {id, stream, label}
        self._camera_map: Dict[str, List[Dict[str, str]]] = {}
        self._load_go2rtc_config()

        # QoS (tolerant defaults; meta/img are best_effort to avoid stalls)
        self._qos_state = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                                     durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
        self._qos_meta  = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
        self._qos_img   = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)

        # Stores
        self._robots: Dict[str, RobotEntry] = {}
        self._known_ns: Dict[str, Dict[str, object]] = {}
        self._georef: Dict[str, GeoRef] = {}
        self._meta: Dict[str, MapMetadata] = {}
        self._img_png: Dict[str, bytes] = {}
        self._img_wh: Dict[str, Tuple[int,int]] = {}
        self._img_rot: Dict[str, Dict[str, float]] = {}
        self._bridge = CvBridge() if _HAS_CV else None

        # Path change tracking
        self._last_path_names: Dict[str, List[str]] = {}
        self._path_seq: Dict[str, int] = {}

        # Optional command publisher
        self._cmd_pubs = {}  # ns -> Publisher
        if _HAS_CMD:
            self._qos_cmd = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST
            )
        else:
            self._qos_cmd = None

        # Keyboard teleopt parameters
        self._vel_pubs: Dict[str, object] = {}
        self._vel_state: Dict[str, Dict[str, float]] = {}

        self.teleop_timeout = float(self.declare_parameter("teleop_timeout_s", 0.2).get_parameter_value().double_value or 0.5)
        self.teleop_update_hz = float(self.declare_parameter("teleop_update_hz", 10.0).get_parameter_value().double_value or 0.5)
        self._teleop_enabled: bool = False

        self._qos_vel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        # Service clients cache (per-ns, eager) — avoid shadowing Node._clients
        self._svc_clients: Dict[Tuple[str,str], object] = {}

        # Web + timers
        self._start_web()
        self.create_timer(1.0/max(0.1,self.discovery_hz), self._discover_ns)
        self.create_timer(1.0/max(0.1,self.viz_update_hz), self._tick_projection)
        self.create_timer(1.0 / max(1e-3, self.teleop_update_hz), self._tick_teleop)

        self.get_logger().info(f"UI http://{self.ui_host}:{self.ui_port}  overlay={self.image_overlay_enable}")

    # ---------- discovery ----------
    def _discover_ns(self):
        try:
            topics = self.get_topic_names_and_types()
        except Exception as e:
            self.get_logger().warn(f"[discover] failed: {e}")
            return
        ns_set = set()
        for name, _ in topics:
            if name.endswith("/rmf/state"):
                ns_set.add(name[:-len("/rmf/state")] or "")
        for ns in ns_set:
            if ns in self._known_ns:
                continue
            self._attach_ns(ns)

    def _attach_ns(self, ns: str):
        s_topic = f"{ns}/rmf/state" if ns else "/rmf/state"
        m_topic = f"{ns}/rmf/map_metadata" if ns else "/rmf/map_metadata"
        i_topic = f"{ns}/rmf/map_image/compressed" if ns else "/rmf/map_image/compressed"

        self.get_logger().info(f"[discover] attach ns='{ns or '/'}'  state={s_topic} meta={m_topic} image={i_topic}")


        cb_ev = SubscriptionEventCallbacks(
            incompatible_qos=lambda info: self.get_logger().warn(
                f"[QoS] incompatible on ns='{ns or '/'}' total={info.total_count} last={getattr(info,'last_policy_kind',None)}")
        )

        subs = {}
        subs["state"] = self.create_subscription(RobotState, s_topic, lambda m,ns=ns:self._on_state(ns,m), self._qos_state, event_callbacks=cb_ev)
        subs["meta"]  = self.create_subscription(MapMetadata, m_topic, lambda m,ns=ns:self._on_meta(ns,m),  self._qos_meta,  event_callbacks=cb_ev)
        subs["image"] = self.create_subscription(CompressedImage, i_topic, lambda m,ns=ns:self._on_image(ns,m), self._qos_img,   event_callbacks=cb_ev)

        if ns not in self._robots:
            self._robots[ns] = RobotEntry(ns=ns)
        self._known_ns[ns] = subs

        # Eagerly create service clients so they appear immediately in `ros2 node info`
        self._prime_service_clients(ns)

        if _HAS_CMD and self._qos_cmd is not None:
            cmd_topic = f"{ns}/rmf/command" if ns else "/rmf/command"
            self._cmd_pubs[ns] = self.create_publisher(RobotCommand, cmd_topic, self._qos_cmd)
            self.get_logger().info(f"[discover] cmd publisher for ns='{ns or '/'}' -> {cmd_topic}")

        # cmd_vel publisher for keyboard teleop
        vel_topic = f"{ns}/cmd_vel" if ns else "/cmd_vel"
        self._vel_pubs[ns] = self.create_publisher(Twist, vel_topic, self._qos_vel)
        self.get_logger().info(f"[discover] cmd_vel publisher for ns='{ns or '/'}' -> {vel_topic}")

    # ---------- service client management ----------
    def _prime_service_clients(self, ns: str):
        if not _HAS_SRVS:
            return
        services = [
            ("schedule_patrol",          SchedulePatrol),
            ("schedule_delivery",        ScheduleDelivery),
            ("schedule_goto",            ScheduleGoTo),
            ("cancel_task",              CancelTask),
            ("cancel_all",               CancelAll),
            ("list_tasks",               ListTasks),
            ("set_resume_after_charge",  SetResumeAfterCharge),
            ("trigger_delivery",         TriggerDelivery),
        ]
        for name, typ in services:
            key = (ns, name)
            if key in self._svc_clients:
                continue
            full = f"{ns}/{name}" if ns else f"/{name}"
            self._svc_clients[key] = self.create_client(typ, full)

    def _client(self, ns: str, srv_name: str, srv_type):
        key = (ns, srv_name)
        cli = self._svc_clients.get(key)
        if cli is None:
            full = f"{ns}/{srv_name}" if ns else f"/{srv_name}"
            cli = self.create_client(srv_type, full)
            self._svc_clients[key] = cli
        return cli

    def _call(self, cli, req, timeout=8.0):
        if not cli.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("service unavailable")
        fut = cli.call_async(req)
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < timeout:
            if fut.done():
                return fut.result()
            time.sleep(0.01)
        raise RuntimeError("service timeout")

    # ---------- ROS callbacks ----------
    def _on_state(self, ns: str, msg: RobotState):
        ent = self._robots.get(ns) or RobotEntry(ns=ns)
        ent.last_state = msg
        ent.last_seen = time.time()
        ent.map_key = MapKey(ns, msg.map_name or "", msg.floor_name or "")
        self._robots[ns] = ent

    def _on_meta(self, ns: str, msg: MapMetadata):
        self._meta[ns] = msg
        self._georef[ns] = GeoRef.from_meta(msg)
        self._last_path_names[ns] = []
        self._path_seq[ns] = 0

        # 🔵 ЛОГИРУЕМ, ЧТО ПОЛУЧИЛИ МЕТАДАННЫЕ КАРТЫ
        try:
            v_count = len(getattr(msg, "vertices", []))
            l_count = len(getattr(msg, "lanes", []))
        except Exception:
            v_count = l_count = -1
       
        self.get_logger().info(
            f"[meta_cb] ns='{ns or '/'}' "
            f"map={msg.map_name}/{msg.floor_name} "
            f"vertices={v_count} lanes={l_count} "
            f"wcs_available={self._georef[ns].available}"
        )

    def _on_image(self, ns: str, img: CompressedImage):
        # If OpenCV is unavailable, we can’t decode to get size → skip
        if not _HAS_CV or self._bridge is None:
            self.get_logger().warn(
                f"[image_cb] ns='{ns or '/'}' received image but OpenCV/CvBridge is unavailable"
            )
            return
        try:
            # img.data is JPEG (format usually "jpeg" or "jpg")
            np_arr = np.frombuffer(img.data, dtype=np.uint8)
            cv_img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
            if cv_img is None:
                raise RuntimeError("cv2.imdecode returned None")

            # --- prepare for transparent/white border ---
            # if 3-channel BGR → convert to BGRA so outer area can be transparent
            if len(cv_img.shape) == 3 and cv_img.shape[2] == 3:
                cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2BGRA)

            # --- rotate image into bird-eye (ENU) axis using wcs_yaw, if available ---
            angle_deg = 0.0
            gr = self._georef.get(ns)
            if gr and gr.available and math.isfinite(gr.yaw_map_to_enu):
                # rotation direction you liked (map → ENU)
                angle_deg = math.degrees(gr.yaw_map_to_enu)

            h0, w0 = cv_img.shape[:2]

            if abs(angle_deg) > 1e-3:
                center0 = (w0 / 2.0, h0 / 2.0)
                M = cv2.getRotationMatrix2D(center0, angle_deg, 1.0)

                # compute new canvas size so nothing is cropped
                cosA = abs(M[0, 0])
                sinA = abs(M[0, 1])
                new_w = int(h0 * sinA + w0 * cosA)
                new_h = int(h0 * cosA + w0 * sinA)

                # shift so rotated content is centered in the new canvas
                M[0, 2] += (new_w / 2.0) - center0[0]
                M[1, 2] += (new_h / 2.0) - center0[1]

                # background outside original map:
                #  - grayscale → white
                #  - BGRA      → transparent border with white RGB
                if len(cv_img.shape) == 2:
                    border = 255  # white
                else:
                    if cv_img.shape[2] == 4:
                        border = (255, 255, 255, 0)  # white, fully transparent
                    else:
                        border = (255, 255, 255)      # white (fallback)

                cv_img = cv2.warpAffine(
                    cv_img, M, (new_w, new_h),
                    flags=cv2.INTER_LINEAR,
                    borderValue=border,
                )

                h, w = cv_img.shape[:2]

                # remember rotation to fix georeferencing later
                angle_rad = math.radians(angle_deg)
                self._img_rot[ns] = {
                    "angle_rad": angle_rad,
                    "w0": float(w0),
                    "h0": float(h0),
                    "w": float(w),
                    "h": float(h),
                }
            else:
                # no rotation → clear any stale info
                h, w = h0, w0
                self._img_rot.pop(ns, None)

            yaw_val = getattr(gr, "yaw_map_to_enu", float("nan")) if gr else float("nan")
            self.get_logger().info(
                f"[image_cb] ns='{ns or '/'}' got compressed image "
                f"format={getattr(img, 'format', '')} size={w}x{h} bytes={len(img.data)} "
                f"wcs_yaw={yaw_val:.4f} rad rot_deg={angle_deg:.2f}"
            )

            # Re-encode as PNG for the web endpoint
            ok, buf = cv2.imencode(".png", cv_img)
            if ok:
                self._img_png[ns] = buf.tobytes()
                self._img_wh[ns] = (w, h)

                # ✅ Диагностика наложения overlay сразу после ПЕРВОГО кадра
                ok_overlay, reason, bbox = self._image_bounds_latlon(ns)
                self.get_logger().info(
                    f"[image_cb] overlay ns='{ns or '/'}' ok={ok_overlay} "
                    f"reason={reason} bbox={bbox}"
                )
        except Exception as e:
            self.get_logger().warn(
                f"[map_image/compressed] ns='{ns or '/'}' decode failed: {e}"
            )

    # ---------- projection ----------
    def _tick_projection(self):
        for ns, ent in self._robots.items():
            s = ent.last_state
            if not s:
                continue
            gr = self._georef.get(ns)
            lat = lon = alt = float("nan")
            heading_deg = float("nan")
            has_fix = False
            if gr and gr.available and math.isfinite(gr.lat0) and math.isfinite(gr.lon0):
                x = float(getattr(s.pose, "x", 0.0))
                y = float(getattr(s.pose, "y", 0.0))
                c, sn = math.cos(gr.yaw_map_to_enu), math.sin(gr.yaw_map_to_enu)
                east =  c * x - sn * y
                north = sn * x +  c * y
                lat, lon = enu_to_latlon(gr.lat0, gr.lon0, east, north)
                yaw_world = gr.yaw_map_to_enu + float(getattr(s.pose, "theta", 0.0))
                heading_deg = wrap_deg(90.0 - math.degrees(yaw_world))
                has_fix = True
            elif bool(getattr(s, "wcs_has_fix", False)):
                lat = float(getattr(s, "wcs_lat", float("nan")))
                lon = float(getattr(s, "wcs_lon", float("nan")))
                alt = float(getattr(s, "wcs_alt", float("nan")))
                q = getattr(s, "wcs_orientation_enuteleop_timeout_s", Quaternion())
                heading_deg = wrap_deg(90.0 - math.degrees(quat_to_yaw(q)))
                has_fix = True
            ent.lat, ent.lon, ent.alt, ent.heading_deg, ent.has_fix = lat, lon, alt, heading_deg, has_fix

    def _tick_teleop(self):
        if not self._vel_pubs:
            return

        # teleop gate – do not publish anything if teleop is OFF
        if not getattr(self, "_teleop_enabled", False):
            return

        # 0 < ALPHA <= 1: 1.0 = без фильтра, 0.1 = сильно сглажено
        LPF_ALPHA = 0.25
        VEL_EPS   = 1e-3

        now = time.time()

        for ns, pub in self._vel_pubs.items():
            st = self._vel_state.get(ns)
            if not st:
                continue

            vx_cmd = float(st.get("vx_cmd", 0.0))
            wz_cmd = float(st.get("wz_cmd", 0.0))
            vx_lp  = float(st.get("vx_lp", 0.0))
            wz_lp  = float(st.get("wz_lp", 0.0))
            last   = float(st.get("last_update", 0.0))
            dt_cmd = now - last

            # -------- Dead-man: команда устарела --------
            # if dt_cmd > self.teleop_timeout:
            #     # Если уже почти стоим — вообще НИЧЕГО не шлём.
            #     if abs(vx_lp) < VEL_EPS and abs(wz_lp) < VEL_EPS:
            #         st["vx_lp"] = 0.0
            #         st["wz_lp"] = 0.0
            #         st["vx_cmd"] = 0.0
            #         st["wz_cmd"] = 0.0
            #         continue

            #     # Иначе: последний раз шли НЕ нулевые скорости → шлём ОДИН stop
            #     msg = Twist()  # все поля по нулям
            #     pub.publish(msg)

            #     st["vx_lp"] = 0.0
            #     st["wz_lp"] = 0.0
            #     st["vx_cmd"] = 0.0
            #     st["wz_cmd"] = 0.0
            #     # На следующем тике попадём в ветку выше и уже ничего не отправим
            #     continue

            # -------- Нормальный режим: есть свежие команды --------
            vx_lp = vx_lp + LPF_ALPHA * (vx_cmd - vx_lp)
            wz_lp = wz_lp + LPF_ALPHA * (wz_cmd - wz_lp)

            st["vx_lp"] = vx_lp
            st["wz_lp"] = wz_lp

            msg = Twist()
            msg.linear.x = vx_lp
            msg.angular.z = wz_lp
            pub.publish(msg)


    # ---------- transforms ----------
    @staticmethod
    def _px_to_m(meta: MapMetadata, xpx: float, ypx: float) -> Tuple[float, float]:
        xr = xpx * float(meta.scale)
        yr = ypx * float(meta.scale)
        c, s = math.cos(float(meta.rotation)), math.sin(float(meta.rotation))
        xo = xr * c - yr * s + float(meta.translation_x)
        yo = -(xr * s + yr * c) + float(meta.translation_y)
        return xo, yo

    @staticmethod
    def _m_to_enu(gr: GeoRef, xm: float, ym: float) -> Tuple[float, float]:
        c, s = math.cos(gr.yaw_map_to_enu), math.sin(gr.yaw_map_to_enu)
        east =  c * xm - s * ym
        north = s * xm +  c * ym
        return east, north

    def _vertex_latlon(self, ns: str, v) -> Optional[Tuple[float,float]]:
        meta = self._meta.get(ns); gr = self._georef.get(ns)
        if not (meta and gr and gr.available and math.isfinite(gr.lat0) and math.isfinite(gr.lon0)):
            return None
        xm = float(getattr(v, "xm", float("nan"))); ym = float(getattr(v, "ym", float("nan")))
        if not (math.isfinite(xm) and math.isfinite(ym)):
            xp = float(getattr(v, "xp", 0.0)); yp = float(getattr(v, "yp", 0.0))
            xm, ym = self._px_to_m(meta, xp, yp)
        east, north = self._m_to_enu(gr, xm, ym)
        return enu_to_latlon(gr.lat0, gr.lon0, east, north)

    def _image_bounds_latlon(self, ns: str):
        if not self.image_overlay_enable:
            return (False, "overlay_disabled", None)
        meta = self._meta.get(ns); gr = self._georef.get(ns); wh = self._img_wh.get(ns)
        if not meta: return (False, "no_metadata", None)
        if not (gr and gr.available and math.isfinite(gr.lat0) and math.isfinite(gr.lon0)):
            return (False, "no_georef", None)
        if not wh: return (False, "no_image", None)
        w, h = wh
        corners_px = [(0,0), (w,0), (w,h), (0,h)]
        pts_ll = []
        # if we rotated the raster, convert new pixel coords back to original
        rot = self._img_rot.get(ns)
        has_rot = bool(rot) and abs(rot.get("angle_rad", 0.0)) > 1e-6
        if has_rot:
            angle = rot["angle_rad"]
            cosA = math.cos(angle)
            sinA = math.sin(angle)
            w0 = rot["w0"]; h0 = rot["h0"]
            # original & new centers in pixel coordinates
            cx0 = w0 / 2.0
            cy0 = h0 / 2.0
            cx1 = w  / 2.0
            cy1 = h  / 2.0

        for (px_new, py_new) in corners_px:
            # map corner from rotated canvas back into original pixel grid
            if has_rot:
                dx1 = float(px_new) - cx1
                dy1 = float(py_new) - cy1
                # inverse rotation: R(-angle) * [dx1; dy1]
                dx0 =  cosA * dx1 + sinA * dy1
                dy0 = -sinA * dx1 + cosA * dy1
                px_orig = dx0 + cx0
                py_orig = dy0 + cy0
            else:
                px_orig = float(px_new)
                py_orig = float(py_new)

            xm, ym = self._px_to_m(meta, px_orig, py_orig)
            east, north = self._m_to_enu(gr, xm, ym)
            lat, lon = enu_to_latlon(gr.lat0, gr.lon0, east, north)
            pts_ll.append((lat, lon))
        rotated = (abs(float(meta.rotation)) > 1e-3)
        if rotated and not self.image_overlay_force_bbox:
            return (False, "rotated_image_overlay_requires_bbox_or_plugin", None)
        lats = [p[0] for p in pts_ll]
        lons = [p[1] for p in pts_ll]
        return (True, "ok", [[min(lats), min(lons)], [max(lats), max(lons)]])

    # ---------- go2rtc camera config ----------
    def _load_go2rtc_config(self):
        """
        Build mapping ns -> list of camera descriptors from go2rtc.yaml.

        Supported formats:

        1) Explicit mapping (recommended):

           cameras:
             /yhs_yhsros2:
               - id: front
                 stream: yhs_yhsros2_cam_front
                 label: Front
               - id: rear
                 stream: yhs_yhsros2_cam_rear
                 label: Rear
               - id: left
                 stream: yhs_yhsros2_cam_left
                 label: Left
               - id: right
                 stream: yhs_yhsros2_cam_right
                 label: Right

        2) Fallback: infer from stream names, e.g.:

           streams:
             yhs_yhsros2_cam1: rtsp://...
             yhs_yhsros2_cam2: rtsp://...

           → ns "/yhs_yhsros2" with cameras id=cam1,cam2
        """
        path = self.go2rtc_config_path
        mapping: Dict[str, List[Dict[str, str]]] = {}
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}

            # 1) Explicit "cameras" section (preferred)
            cams_cfg = data.get("cameras")
            if isinstance(cams_cfg, dict):
                for ns_raw, lst in cams_cfg.items():
                    ns = "/" + (ns_raw or "").strip("/")
                    entries: List[Dict[str, str]] = []
                    if isinstance(lst, dict):
                        # allow flat dict {id: stream_name}
                        for cid, stream_name in lst.items():
                            entries.append({
                                "id": str(cid),
                                "stream": str(stream_name),
                                "label": str(cid),
                            })
                    elif isinstance(lst, list):
                        for item in lst:
                            if not isinstance(item, dict):
                                continue
                            cid = str(item.get("id", item.get("name", "")) or "")
                            stream_name = str(item.get("stream", "") or "")
                            if not stream_name:
                                continue
                            label = str(item.get("label", cid or stream_name))
                            entries.append({
                                "id": cid or stream_name,
                                "stream": stream_name,
                                "label": label,
                            })
                    if entries:
                        mapping[ns] = entries

            # 2) Fallback: infer from stream names
            if not mapping:
                streams = data.get("streams", {}) or {}
                for raw_name in streams.keys():
                    name = str(raw_name)
                    # split on last '_' → prefix = ns-ish, suffix = cam id
                    if "_" not in name:
                        continue
                    prefix, cam_id = name.rsplit("_", 1)
                    ns = "/" + prefix.strip("/")
                    label = cam_id
                    mapping.setdefault(ns, []).append({
                        "id": cam_id,
                        "stream": name,
                        "label": label,
                    })

            self._camera_map = mapping
            self.get_logger().info(
                f"[go2rtc] loaded camera map from '{path}' for {len(mapping)} namespaces"
            )
        except Exception as e:
            self._camera_map = {}
            self.get_logger().warn(f"[go2rtc] failed to load config from '{path}': {e}")

    # ---------- web ----------
    def _start_web(self):
        app = Flask(
            __name__,
            template_folder=os.path.join(self._web_dir(), "templates"),
            static_folder=os.path.join(self._web_dir(), "static"),
            static_url_path="/static",
        )

        @app.after_request
        def _nocache(resp):
            resp.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
            resp.headers["Pragma"] = "no-cache"
            resp.headers["Expires"] = "0"
            return resp

        @app.get("/")
        def index():
            return render_template("viz.html", debug="true" if self.debug else "false")

        @app.get("/api/robots")
        def api_robots():
            rows = []
            for ns, ent in self._robots.items():
                s = ent.last_state
                rows.append({
                    "ns": ns or "/",
                    "robot_name": getattr(s, "robot_name", ns) if s else ns,
                    "map_name": getattr(s, "map_name", "") if s else "",
                    "floor_name": getattr(s, "floor_name", "") if s else "",
                    "mode_text": getattr(s, "mode_text", "") if s else "",
                    "mode": int(getattr(s, "mode", 0)) if s else 0,
                    "battery_pct": (float(getattr(s, "battery_pct", float("nan"))) if s else float("nan")),
                    "battery_voltage": (float(getattr(s, "battery_voltage", float("nan"))) if s else float("nan")),
                    "vehicle_speed": (float(getattr(s, "vehicle_speed", float("nan"))) if s else float("nan")),
                    "has_fix": bool(ent.has_fix),
                    "lat": float(ent.lat) if math.isfinite(ent.lat) else None,
                    "lon": float(ent.lon) if math.isfinite(ent.lon) else None,
                    "alt": float(ent.alt) if math.isfinite(ent.alt) else None,
                    "heading_deg": float(ent.heading_deg) if math.isfinite(ent.heading_deg) else None,
                    "last_seen": ent.last_seen or 0.0,
                })
            return jsonify({"robots": rows, "t": time.time()})

        @app.get("/api/maps")
        def api_maps():
            out = []
            for ns, meta in self._meta.items():
                gr = self._georef.get(ns)
                info = {
                    "ns": ns or "/",
                    "map_name": getattr(meta, "map_name", ""),
                    "floor_name": getattr(meta, "floor_name", ""),
                    "wcs_available": bool(gr.available) if gr else False,
                    "overlay": None,
                    "overlay_reason": None,
                    "image_available": bool(self._img_png.get(ns) is not None),
                    "image_url": f"/map_image/{ns.strip('/') or '_root'}" if self._img_png.get(ns) else "",
                    "vertices": [],
                    "edges": [],
                }
                # vertices → WGS84
                try:
                    v_ll = []
                    for v in getattr(meta, "vertices", []):
                        ll = self._vertex_latlon(ns, v)
                        if ll:
                            v_ll.append({"name": v.name, "lat": ll[0], "lon": ll[1], "id": int(getattr(v,"id",-1))})
                    info["vertices"] = v_ll
                except Exception as e:
                    info["vertices_error"] = str(e)

                # edges → lines in WGS84
                try:
                    id_to_ll = {int(v["id"]): (v["lat"], v["lon"]) for v in info["vertices"] if "id" in v}
                    edges = []
                    for e in getattr(meta, "lanes", []):
                        a = id_to_ll.get(int(getattr(e,"src_id", -1)))
                        b = id_to_ll.get(int(getattr(e,"dst_id", -1)))
                        if a and b:
                            edges.append({"a": {"lat":a[0],"lon":a[1]}, "b": {"lat":b[0],"lon":b[1]}, "bidir": bool(getattr(e,"bidirectional",False))})
                    info["edges"] = edges
                except Exception as e:
                    info["edges_error"] = str(e)

                ok, reason, bbox = self._image_bounds_latlon(ns)
                if ok and bbox: info["overlay"] = {"bounds": bbox, "note": "axis-aligned bbox"}
                else:           info["overlay_reason"] = reason

                out.append(info)
            return jsonify({"maps": out, "t": time.time()})

        @app.get("/map_image/<path:ns>")
        def map_image(ns):
            ns_key = "/" + ns.strip("/")
            data = self._img_png.get(ns_key)
            if not data: return ("", 404)
            resp = make_response(data)
            resp.headers["Content-Type"] = "image/png"
            resp.headers["Cache-Control"] = "no-store"
            return resp

        # --------- helpers ----------
        def _nskey(ns_in: str) -> str:
            return "/" + (ns_in or "").strip("/")

        # --------- REST: keyboard teleop cmd_vel ----------
        @app.post("/api/<path:ns>/teleop_cmd")
        def api_teleop_cmd(ns):
            """
            Accepts JSON with {linear_x, angular_z} and updates the current
            teleop command for the given namespace.

            Actual publishing to /cmd_vel is done from the ROS2 timer
            (_tick_teleop) with a low-pass filter + dead-man.
            """
            body = request.get_json(silent=True) or {}
            try:
                lin_x = float(body.get("linear_x", 0.0) or 0.0)
                ang_z = float(body.get("angular_z", 0.0) or 0.0)
            except Exception:
                return jsonify({"ok": False, "error": "invalid linear_x/angular_z"}), 400

            ns_key = _nskey(ns)           # "/yhs_yhsros2" or "/"
            ros_ns = "" if ns_key == "/" else ns_key

            pub = self._vel_pubs.get(ros_ns)
            if not pub:
                return jsonify({
                    "ok": False,
                    "error": f"cmd_vel publisher unavailable for ns '{ros_ns or '/'}'"
                }), 501

            now = time.time()

            st = self._vel_state.get(ros_ns)
            if st is None:
                # first command for this ns
                st = {
                    "vx_cmd": lin_x,   # desired (unfiltered) velocity
                    "wz_cmd": ang_z,
                    "vx_lp": 0.0,      # filtered velocity state
                    "wz_lp": 0.0,
                    "last_update": now,
                }
                self._vel_state[ros_ns] = st
            else:
                st["vx_cmd"] = lin_x
                st["wz_cmd"] = ang_z
                st["last_update"] = now

            # ⛔️ Ничего не публикуем отсюда — только обновляем состояние.
            # Публикация идёт ТОЛЬКО из _tick_teleop() с фильтром.
            return jsonify({"ok": True})

        @app.post("/api/teleop_state")
        def api_teleop_state():
            """
            Global teleop ON/OFF flag coming from the bottom-bar button.
            When turning OFF, publish a single zero Twist for all namespaces
            so the robots stop, and then stop publishing from _tick_teleop().
            """
            body = request.get_json(silent=True) or {}
            enabled = bool(body.get("enabled", True))

            self._teleop_enabled = enabled
            self.get_logger().info(f"[teleop] global enabled={enabled}")

            if not enabled:
                # Immediately send one stop Twist on all cmd_vel topics
                for ns, pub in self._vel_pubs.items():
                    try:
                        msg = Twist()  # all zeros
                        pub.publish(msg)
                    except Exception as e:
                        self.get_logger().warn(
                            f"[teleop] stop publish failed for ns='{ns or '/'}': {e}"
                        )

                # Reset internal state so dead-man timer is "cold"
                for st in self._vel_state.values():
                    st["vx_cmd"] = 0.0
                    st["wz_cmd"] = 0.0
                    st["vx_lp"]  = 0.0
                    st["wz_lp"]  = 0.0
                    st["last_update"] = 0.0

            return jsonify({"ok": True, "enabled": enabled})



        # --------- REST: cameras (go2rtc) ----------
        @app.get("/api/cameras")
        def api_cameras():
            """
            Return all cameras for all namespaces, derived from go2rtc.yaml.
            {
              "ok": true,
              "base_url": "http://host:1984"  # or ""
              "cameras": {
                 "/yhs_yhsros2": [
                   {"id":"front","stream":"yhs_yhsros2_cam_front","label":"Front"},
                   ...
                 ],
                 ...
              }
            }
            """
            return jsonify({
                "ok": True,
                "base_url": self.go2rtc_base_url or "",
                "cameras": self._camera_map,
                "t": time.time(),
            })

        @app.get("/api/<path:ns>/cameras")
        def api_cameras_ns(ns):
            """
            Cameras only for the given namespace.
            """
            ns_key = _nskey(ns)
            cams = self._camera_map.get(ns_key, [])
            return jsonify({
                "ok": True,
                "ns": ns_key,
                "base_url": self.go2rtc_base_url or "",
                "cameras": cams,
                "t": time.time(),
            })

        # --------- REST: tasks/list ----------
        @app.get("/api/<path:ns>/tasks")
        def api_tasks(ns):
            if not _HAS_SRVS:
                return jsonify({"ok": False, "error": "services unavailable"}), 501
            try:
                cli = self._client(_nskey(ns), "list_tasks", ListTasks)
                res = self._call(cli, ListTasks.Request(), timeout=6.0)
                payload = {}
                try:
                    payload = json.loads(getattr(res, "tasks_json", "{}") or "{}")
                except Exception as e:
                    payload = {"parse_error": str(e), "raw": getattr(res, "tasks_json", "")}
                return jsonify({"ok": True, "tasks": payload})
            except Exception as e:
                return jsonify({"ok": False, "error": str(e)}), 500

        # --------- REST: schedule patrol ----------
        @app.post("/api/<path:ns>/patrol")
        def api_patrol(ns):
            if not _HAS_SRVS:
                return jsonify({"ok": False, "error": "services unavailable"}), 501

            body = request.get_json(silent=True) or {}
            seq = body.get("sequence")
            if not (isinstance(seq, list) and seq):
                return jsonify({"ok": False, "error": "missing 'sequence' list"}), 400

            # --- build request as before ---
            try:
                req = SchedulePatrol.Request()

                field_name = None
                if hasattr(req, "waypoints"):
                    req.waypoints = [str(x) for x in seq]
                    field_name = "waypoints"
                elif hasattr(req, "vertices"):
                    req.vertices = [str(x) for x in seq]
                    field_name = "vertices"
                else:
                    return jsonify({"ok": False, "error": "SchedulePatrol has no waypoints/vertices field"}), 501

                loops = int(body.get("loops", 1) or 1)
                req.loops = loops

                start_now_val = False
                if hasattr(req, "start_now"):
                    start_now_val = bool(body.get("start_now", False))
                    req.start_now = start_now_val

                # --- LOG: show equivalent terminal command ---
                ns_key = _nskey(ns)      # e.g. "/yhs_yhsros2"
                srv_name = f"{ns_key}/schedule_patrol"

                seq_list_str = ", ".join(f"'{str(x)}'" for x in seq)

                # yaml-ish payload
                payload_parts = [f"{field_name}: [{seq_list_str}]", f"loops: {loops}"]
                if hasattr(req, "start_now"):
                    payload_parts.append(f"start_now: {str(start_now_val).lower()}")

                payload_str = ", ".join(payload_parts)

                self.get_logger().info(
                    "\n[REST][patrol] scheduling patrol\n"
                    f"  ns: {ns_key}\n"
                    f"  sequence: {seq}\n"
                    f"  loops: {loops}\n"
                    f"  start_now: {start_now_val}\n"
                    "  # Equivalent CLI:\n"
                    f"  ros2 service call {srv_name} "
                    "rmf_manager_msgs/srv/SchedulePatrol "
                    f"\"{{{payload_str}}}\"\n"
                )

                # --- call service as before ---
                cli = self._client(ns_key, "schedule_patrol", SchedulePatrol)
                res = self._call(cli, req, timeout=8.0)
                return jsonify({"ok": True, "task_id": getattr(res, "task_id", "")})
            except Exception as e:
                return jsonify({"ok": False, "error": str(e)}), 500

        # --------- REST: schedule goto ----------
        @app.post("/api/<path:ns>/goto")
        def api_goto(ns):
            if not _HAS_SRVS:
                return jsonify({"ok": False, "error": "services unavailable"}), 501
            body = request.get_json(silent=True) or {}
            goal = str(body.get("goal", "")).strip()
            if not goal:
                return jsonify({"ok": False, "error": "missing 'goal'"}), 400
            try:
                req = ScheduleGoTo.Request()
                if hasattr(req, "goal"): req.goal = goal
                if hasattr(req, "start_now"): req.start_now = bool(body.get("start_now", False))
                cli = self._client(_nskey(ns), "schedule_goto", ScheduleGoTo)
                res = self._call(cli, req, timeout=8.0)
                return jsonify({"ok": True, "task_id": getattr(res, "task_id", "")})
            except Exception as e:
                return jsonify({"ok": False, "error": str(e)}), 500

        # --------- REST: schedule delivery ----------
        @app.post("/api/<path:ns>/delivery")
        def api_delivery(ns):
            if not _HAS_SRVS:
                return jsonify({"ok": False, "error": "services unavailable"}), 501
            body = request.get_json(silent=True) or {}
            pickup  = str(body.get("pickup", "")).strip()
            dropoff = str(body.get("dropoff", "")).strip()
            if not (pickup and dropoff):
                return jsonify({"ok": False, "error": "missing 'pickup' or 'dropoff'"}), 400
            try:
                req = ScheduleDelivery.Request()
                if hasattr(req, "pickup"):  req.pickup  = pickup
                if hasattr(req, "dropoff"): req.dropoff = dropoff
                if hasattr(req, "wait_for_trigger"):
                    req.wait_for_trigger = bool(body.get("wait_for_trigger", False))
                cli = self._client(_nskey(ns), "schedule_delivery", ScheduleDelivery)
                res = self._call(cli, req, timeout=8.0)
                return jsonify({"ok": True, "task_id": getattr(res, "task_id", "")})
            except Exception as e:
                return jsonify({"ok": False, "error": str(e)}), 500

        # --------- REST: cancel all ----------
        @app.post("/api/<path:ns>/cancel_all")
        def api_cancel_all(ns):
            if not _HAS_SRVS:
                return jsonify({"ok": False, "error": "services unavailable"}), 501
            try:
                cli = self._client(_nskey(ns), "cancel_all", CancelAll)
                self._call(cli, CancelAll.Request(), timeout=6.0)
                return jsonify({"ok": True})
            except Exception as e:
                return jsonify({"ok": False, "error": str(e)}), 500

        # --------- REST: cancel specific task ----------
        @app.post("/api/<path:ns>/cancel/<task_id>")
        def api_cancel(ns, task_id):
            if not _HAS_SRVS:
                return jsonify({"ok": False, "error": "services unavailable"}), 501
            try:
                req = CancelTask.Request()
                if hasattr(req, "task_id"): req.task_id = str(task_id)
                cli = self._client(_nskey(ns), "cancel_task", CancelTask)
                self._call(cli, req, timeout=6.0)
                return jsonify({"ok": True})
            except Exception as e:
                return jsonify({"ok": False, "error": str(e)}), 500

        # --------- REST: resume after charge toggle ----------
        @app.post("/api/<path:ns>/resume_after_charge")
        def api_resume_after_charge(ns):
            if not _HAS_SRVS:
                return jsonify({"ok": False, "error": "services unavailable"}), 501
            body = request.get_json(silent=True) or {}
            enabled = bool(body.get("enabled", True))
            try:
                req = SetResumeAfterCharge.Request()
                if hasattr(req, "enabled"): req.enabled = enabled
                cli = self._client(_nskey(ns), "set_resume_after_charge", SetResumeAfterCharge)
                self._call(cli, req, timeout=6.0)
                return jsonify({"ok": True})
            except Exception as e:
                return jsonify({"ok": False, "error": str(e)}), 500

        # --------- REST: trigger delivery stage ----------
        @app.post("/api/<path:ns>/trigger/<task_id>/<stage>")
        def api_trigger(ns, task_id, stage):
            if not _HAS_SRVS:
                return jsonify({"ok": False, "error": "services unavailable"}), 501
            try:
                req = TriggerDelivery.Request()
                if hasattr(req, "task_id"): req.task_id = str(task_id)
                if hasattr(req, "stage"):   req.stage   = str(stage)
                cli = self._client(_nskey(ns), "trigger_delivery", TriggerDelivery)
                self._call(cli, req, timeout=6.0)
                return jsonify({"ok": True})
            except Exception as e:
                return jsonify({"ok": False, "error": str(e)}), 500

        # --------- REST: return home (RobotCommand) ----------
        @app.post("/api/<path:ns>/return_home")
        def api_return_home(ns):
            # Normalize ns from the URL into the same form used in _attach_ns
            ns_key = _nskey(ns)          # "/yhs_yhsros2" or "/"
            ros_ns = "" if ns_key == "/" else ns_key

            pub = self._cmd_pubs.get(ros_ns)
            if not pub:
                return jsonify({
                    "ok": False,
                    "error": f"RobotCommand publisher unavailable for ns '{ros_ns or '/'}'"
                }), 501

            try:
                msg = RobotCommand()
                if hasattr(msg, "type"):
                    msg.type = 5            # your "return home" command code
                if hasattr(msg, "robot_name"):
                    msg.robot_name = "*"    # or a specific robot name if you want

                pub.publish(msg)
                return jsonify({"ok": True})
            except Exception as e:
                self.get_logger().warn(f"[return_home] failed for ns='{ros_ns or '/'}': {e}")
                return jsonify({"ok": False, "error": str(e)}), 500

        # --------- REST: path (per-ns) ----------
        @app.get("/api/<path:ns>/path")
        def api_path(ns):
            ns = _nskey(ns)
            ent = self._robots.get(ns)
            st = ent.last_state if ent else None
            meta = self._meta.get(ns)
            if not (st and meta):
                return jsonify({"ok": False, "seq": self._path_seq.get(ns, 0), "points": []})
            names = list(getattr(st, "path_vertices", []))
            if names != self._last_path_names.get(ns, []):
                self._path_seq[ns] = int(self._path_seq.get(ns, 0)) + 1
                self._last_path_names[ns] = names
            # name -> lat/lon lookup
            vmap = {}
            for v in meta.vertices:
                latlon = self._vertex_latlon(ns, v)
                if latlon: vmap[str(v.name)] = {"lat": latlon[0], "lon": latlon[1]}
            pts = [vmap[n] for n in names if n in vmap]
            return jsonify({"ok": True, "seq": self._path_seq.get(ns, 0), "points": pts, "t": time.time()})

        # Debug snapshot
        @app.get("/__/snapshot")
        def snapshot():
            sn = {}
            for ns, m in self._meta.items():
                gr = self._georef.get(ns)
                ok, reason, _ = self._image_bounds_latlon(ns)
                sn[ns or "/"] = {
                    "map": f"{getattr(m,'map_name','')}/{getattr(m,'floor_name','')}",
                    "wcs_available": bool(gr.available) if gr else False,
                    "image": bool(self._img_png.get(ns)),
                    "img_wh": self._img_wh.get(ns, None),
                    "overlay_ok": ok, "overlay_reason": reason
                }
            return jsonify(sn)

        def _serve():
            self.get_logger().info(f"UI: http://{self.ui_host}:{self.ui_port}")
            serve(app, host=self.ui_host, port=self.ui_port, threads=8)

        threading.Thread(target=_serve, daemon=True).start()

    def _web_dir(self) -> str:
        try:
            from ament_index_python.packages import get_package_share_directory
            return os.path.join(get_package_share_directory("rmf_manager_cloud"), "web")
        except Exception:
            return os.path.join(os.path.dirname(__file__), "..", "web")


# ----- main -----

def main():
    if "--ros-domain-id" in sys.argv:
        try:
            i = sys.argv.index("--ros-domain-id")
            if i + 1 < len(sys.argv):
                os.environ["ROS_DOMAIN_ID"] = str(int(sys.argv[i + 1]))
            del sys.argv[i:i+2]
        except Exception:
            sys.argv = [a for a in sys.argv if a != "--ros-domain-id"]

    rclpy.init(args=sys.argv)
    node = MultiRobotDashboard()
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
