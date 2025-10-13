#!/usr/bin/env python3
import os, threading, time, math, json, signal
from dataclasses import dataclass
from typing import Dict, Optional, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.task import Future
from rclpy._rclpy_pybind11 import RCLError
from rclpy.serialization import deserialize_message, serialize_message

from ament_index_python.packages import get_package_share_directory

from flask import Flask, Response, render_template, request, redirect, url_for, send_file, jsonify, make_response
from waitress import serve

from rmf_manager_msgs.srv import (
    SchedulePatrol, ScheduleDelivery, ScheduleGoTo,
    CancelTask, CancelAll, ListTasks, SetResumeAfterCharge,
    TriggerDelivery, MoveTask
)
from rmf_manager_msgs.msg import RobotState, RobotCommand

import yaml
import sys
import zenoh


# --------- small graph helper (pixels + meters) ----------
@dataclass
class V:
    name: str; xm: float; ym: float; xp: float; yp: float

class Graph:
    def __init__(self):
        self.V: Dict[str, V] = {}
        self.E: List[dict] = []
        self.image_path = ''
        self.rot = 0.0
        self.scale = 1.0
        self.tx = 0.0
        self.ty = 0.0

    def load_params(self, ppath: str):
        if not ppath or not os.path.exists(ppath):
            return
        with open(ppath, 'r') as f:
            d = yaml.safe_load(f) or {}
        params = d.get('/**', {}).get('ros__parameters', {})
        self.rot   = float(params.get('rotation', 0.0))
        self.scale = float(params.get('scale', 1.0))
        self.tx    = float(params.get('translation_x', 0.0))
        self.ty    = float(params.get('translation_y', 0.0))

    def apply_tr(self, xpx, ypx):
        xr = xpx * self.scale
        yr = ypx * self.scale
        c = math.cos(self.rot); s = math.sin(self.rot)
        xo = xr*c - yr*s + self.tx
        yo = -(xr*s + yr*c) + self.ty
        return xo, yo

    def invert_tr(self, xm, ym):
        x = xm - self.tx
        y = self.ty - ym
        c = math.cos(self.rot); s = math.sin(self.rot)
        xr = x*c + y*s
        yr = -x*s + y*c
        if self.scale == 0:
            return 0.0, 0.0
        return xr/self.scale, yr/self.scale

    def load_building(self, bpath: str):
        if not bpath or not os.path.exists(bpath):
            return
        with open(bpath, 'r') as f:
            b = yaml.safe_load(f) or {}
        levels = b.get('levels', {})
        lvl = list(levels.keys())[0] if levels else None
        L = levels.get(lvl, {})

        drawing = L.get('drawing', {})
        if isinstance(drawing, dict) and 'filename' in drawing:
            self.image_path = os.path.abspath(
                os.path.join(os.path.dirname(bpath), drawing['filename'])
            )

        verts = L.get('vertices', [])
        id2name: Dict[int, str] = {}
        self.V.clear()
        for i, v in enumerate(verts):
            if isinstance(v, dict):
                xpx = float(v.get('x', 0)); ypx = float(v.get('y', 0))
                name = str(v.get('name', f'v{i}')) or f'v{i}'
            else:
                xpx = float(v[0]); ypx = float(v[1])
                name = str(v[3] if len(v) > 3 else (v[2] if len(v) > 2 else f'v{i}')) or f'v{i}'
            xm, ym = self.apply_tr(xpx, ypx)
            self.V[name] = V(name, xm, ym, xpx, ypx)
            id2name[i] = name

        self.E = []
        for ln in L.get('lanes', []):
            if isinstance(ln, list) and len(ln) >= 2 and ln[0] in id2name and ln[1] in id2name:
                a = id2name[ln[0]]; b = id2name[ln[1]]
                ax, ay = self.V[a].xp, self.V[a].yp
                bx, by = self.V[b].xp, self.V[b].yp
                self.E.append({'a': a, 'b': b, 'ax': ax, 'ay': ay, 'bx': bx, 'by': by})


# ---------------- Zenoh helpers ----------------
def _norm_ns(ns: str) -> str:
    parts = [p for p in str(ns).split('/') if p]
    return '/'.join(parts)

def _ns_key(ns: str, topic: str) -> str:
    ns_clean = _norm_ns(ns)
    top_clean = _norm_ns(topic)
    return f'{ns_clean}/{top_clean}' if ns_clean else top_clean


def _guess_image_mime(b: bytes) -> Optional[str]:
    if not b or len(b) < 4:
        return None
    if b[:8] == b'\x89PNG\r\n\x1a\n':
        return 'image/png'
    if b[:3] == b'\xff\xd8\xff':
        return 'image/jpeg'
    return None


# --------- main node ----------
class DashboardNode(Node):
    def __init__(self):
        super().__init__('rmf_dashboard')

        # ----------------- params -----------------
        self.ui_host = self.declare_parameter('ui_host', '0.0.0.0').value
        self.ui_port = int(self.declare_parameter('ui_port', 5005).value)
        self.ui_viz_host = self.declare_parameter('ui_viz_host', '0.0.0.0').value
        self.ui_viz_port = int(self.declare_parameter('ui_viz_port', 8080).value)
        self.viz_update_hz = float(self.declare_parameter('viz_update_hz', 1.0).value)
        self.param_yaml_path = self.declare_parameter('param_yaml_path', '').value
        self.building_yaml_path = self.declare_parameter('building_yaml_path', '').value

        # OSM default center (used by viz; page still works with no robots)
        self.osm_default_lat = float(self.declare_parameter('osm_default_lat', 55.751244).value)
        self.osm_default_lon = float(self.declare_parameter('osm_default_lon', 37.618423).value)
        self.osm_default_zoom = int(self.declare_parameter('osm_default_zoom', 17).value)

        # multi-robot via Zenoh (comma-separated)
        ns_csv = self.declare_parameter('robot_namespaces', '/vboxuser_Ubuntu22').value
        self.namespaces: List[str] = [_norm_ns(n) for n in str(ns_csv).split(',') if n.strip()]
        if not self.namespaces:
            self.namespaces = ['vboxuser_Ubuntu22']
        self.primary_ns = self.namespaces[0]

        # Zenoh config (match your router)
        self.zenoh_mode   = self.declare_parameter('zenoh_mode', 'peer').value        # peer|client|router
        self.zenoh_connect= self.declare_parameter('zenoh_connect', 'tcp/192.168.196.48:7447').value
        self.zenoh_listen = self.declare_parameter('zenoh_listen', 'tcp/0.0.0.0:7448').value

        # graph
        self.g = Graph()
        self.g.load_params(self.param_yaml_path)
        self.g.load_building(self.building_yaml_path)

        # --------------- ROS I/O ---------------
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.state_sub = self.create_subscription(RobotState, '/rmf/state', self._on_state_ros, qos)
        self.cmd_pub   = self.create_publisher(RobotCommand, '/rmf/command', qos)

        # services clients
        self.cli_list   = self.create_client(ListTasks, 'list_tasks')
        self.cli_patrol = self.create_client(SchedulePatrol, 'schedule_patrol')
        self.cli_deliv  = self.create_client(ScheduleDelivery, 'schedule_delivery')
        self.cli_goto   = self.create_client(ScheduleGoTo, 'schedule_goto')
        self.cli_cancel = self.create_client(CancelTask, 'cancel_task')
        self.cli_cancel_all = self.create_client(CancelAll, 'cancel_all')
        self.cli_resume = self.create_client(SetResumeAfterCharge, 'set_resume_after_charge')
        self.cli_trigger= self.create_client(TriggerDelivery, 'trigger_delivery')
        self.cli_move   = self.create_client(MoveTask, 'move_task')

        # --------------- caches ---------------
        self._last_state: Optional[RobotState] = None
        self._z_states: Dict[str, Optional[RobotState]] = {ns: None for ns in self.namespaces}
        self._z_events: Dict[str, Optional[bytes]] = {ns: None for ns in self.namespaces}
        self._z_last_heard: Dict[str, float] = {ns: 0.0 for ns in self.namespaces}
        self._z_diag_text: Dict[str, Optional[bytes]] = {ns: None for ns in self.namespaces}
        self._z_image: Dict[str, Optional[bytes]] = {ns: None for ns in self.namespaces}

        self._tasks_lock = threading.Lock()
        self._tasks_cache = {
            'rev': 0,
            '_sig': '',
            'ts': 0.0,
            'queue_len': 0,
            'scheduled': [],
            'current_task': None,
            'returning_home': False,
            'docked': False,
            'resume_after_charge': False,
        }
        self._tasks_ttl = 1.0  # seconds
        self._FINAL_STATUSES = {
            'done','finished','completed','complete','success','succeeded',
            'canceled','cancelled','failed','aborted','timeout','timed out'
        }

        # --------------- Zenoh ---------------
        self._open_zenoh()
        self._declare_zenoh_subscriptions()

        # --------------- web servers ---------------
        self._start_dashboard_server()  # :5005 Task Manager + Robots
        self._start_viz_server()        # :8080 OSM Visualizer

    # ---------------- Zenoh ----------------
    def _open_zenoh(self):
        cfg = zenoh.Config()
        cfg.insert_json5('mode', json.dumps(self.zenoh_mode))
        if self.zenoh_connect:
            eps = [e.strip() for e in str(self.zenoh_connect).split(',') if e.strip()]
            if eps:
                cfg.insert_json5('connect/endpoints', json.dumps(eps))
        if self.zenoh_listen:
            eps = [e.strip() for e in str(self.zenoh_listen).split(',') if e.strip()]
            if eps:
                cfg.insert_json5('listen/endpoints', json.dumps(eps))
        self.get_logger().info(f'[zenoh] opening session: mode={self.zenoh_mode}')
        self._z = zenoh.open(cfg)

        self._z_cmd_writers: Dict[str, zenoh.Publisher] = {}
        for ns in self.namespaces:
            key = _ns_key(ns, '/rmf/command')
            self._z_cmd_writers[ns] = self._z.declare_publisher(key)

    def _declare_zenoh_subscriptions(self):
        def mk_state_cb(ns: str):
            def _cb(sample: zenoh.Sample):
                try:
                    msg = deserialize_message(sample.payload.to_bytes(), RobotState)
                    self._z_states[ns] = msg
                    self._z_last_heard[ns] = time.time()
                except Exception as e:
                    self.get_logger().debug(f'[{ns}] /rmf/state deserialization failed: {e}')
            return _cb

        for ns in self.namespaces:
            self._z.declare_subscriber(_ns_key(ns, '/rmf/state'), mk_state_cb(ns))
            self._z.declare_subscriber(_ns_key(ns, '/rmf/event'),
                                       lambda s, ns=ns: self._cache_bytes(ns, '_z_events', s.payload.to_bytes()))
            self._z.declare_subscriber(_ns_key(ns, '/diagnostics'),
                                       lambda s, ns=ns: self._cache_bytes(ns, '_z_diag_text', s.payload.to_bytes()))
            self._z.declare_subscriber(_ns_key(ns, '/depth_camera/image'),
                                       lambda s, ns=ns: self._cache_bytes(ns, '_z_image', s.payload.to_bytes()))
            for t in ('/chatter','/gps/data','/imu/data','/map/2d_map','/tf'):
                self._z.declare_subscriber(_ns_key(ns, t), lambda s, ns=ns: self._touch_heard(ns))

        self.get_logger().info(f'[zenoh] subscriptions ready for {len(self.namespaces)} namespaces')

    def _cache_bytes(self, ns: str, attr: str, payload: bytes):
        getattr(self, attr)[ns] = payload
        self._z_last_heard[ns] = time.time()

    def _touch_heard(self, ns: str):
        self._z_last_heard[ns] = time.time()

    def _publish_command_zenoh(self, msg: RobotCommand, ns: Optional[str] = None):
        data = serialize_message(msg)
        targets = [ns] if ns else list(self.namespaces)
        for tns in targets:
            pub = self._z_cmd_writers.get(tns)
            if pub:
                pub.put(data)

    # ---------------- ROS callbacks ----------------
    def _on_state_ros(self, msg: RobotState):
        self._last_state = msg

    # ---- util to call services sync-ish ----
    def _call(self, client, req, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"service {client.srv_name} unavailable")
        fut: Future = client.call_async(req)
        deadline = time.time() + timeout
        while rclpy.ok() and not fut.done() and time.time() < deadline:
            time.sleep(0.01)
        if not fut.done():
            raise RuntimeError(f"{client.srv_name} timeout")
        return fut.result()

    def _is_final(self, t: dict) -> bool:
        st = (t.get('status') or '').strip().lower()
        if t.get('finished') or t.get('is_finished') or t.get('done'):
            return True
        return any(word in st for word in self._FINAL_STATUSES)

    def _filter_and_order_scheduled(self, items: list) -> list:
        dedup = {}
        for idx, t in enumerate(items or []):
            if not isinstance(t, dict):
                continue
            if self._is_final(t):
                continue
            tid = str(t.get('id', ''))
            if 'position' not in t:
                t['position'] = idx
            dedup[tid] = t
        active = list(dedup.values())
        active.sort(key=lambda x: int(x.get('position', 0)))
        for i, t in enumerate(active):
            t.setdefault('type', '')
            t.setdefault('status', '')
            t.setdefault('waypoints', [])
            t.setdefault('loops', 1)
            t.setdefault('pickup', '')
            t.setdefault('dropoff', '')
            t.setdefault('wait_for_trigger', False)
            t['position'] = i
        return active

    def _normalize_tasks_payload(self, payload: dict) -> dict:
        scheduled = self._filter_and_order_scheduled(payload.get('scheduled') or [])
        cur = payload.get('current_task') or None
        if isinstance(cur, dict) and self._is_final(cur):
            cur = None
        scheduled_with_current = list(scheduled)
        if isinstance(cur, dict):
            cur = dict(cur)
            cur.setdefault('type', '')
            cur.setdefault('status', '')
            cur.setdefault('waypoints', [])
            cur.setdefault('loops', 1)
            cur.setdefault('pickup', '')
            cur.setdefault('dropoff', '')
            cur.setdefault('wait_for_trigger', False)
            cur['position'] = -1
            cur['is_current'] = True
            scheduled_with_current = [cur] + scheduled
        return {
            'scheduled': scheduled,
            'scheduled_with_current': scheduled_with_current,
            'queue_len': len(scheduled),
            'current_task': cur,
            'returning_home': bool(payload.get('returning_home', False)),
            'docked': bool(payload.get('docked', False)),
            'resume_after_charge': bool(payload.get('resume_after_charge', False)),
        }

    def _fetch_tasks_now(self) -> dict:
        res = self._call(self.cli_list, ListTasks.Request(), timeout=2.0)
        raw = {}
        try:
            raw = json.loads(res.tasks_json or '{}')
        except Exception as e:
            self.get_logger().warn(f"list_tasks JSON parse error: {e}")
            raw = {}
        return self._normalize_tasks_payload(raw)

    def _get_tasks_snapshot(self, force=False) -> dict:
        now = time.time()
        with self._tasks_lock:
            stale = (now - self._tasks_cache['ts']) > self._tasks_ttl
            if force or stale:
                try:
                    fresh = self._fetch_tasks_now()
                    sig = json.dumps(fresh, sort_keys=True, separators=(',',':'))
                    if sig != self._tasks_cache['_sig']:
                        self._tasks_cache.update(fresh)
                        self._tasks_cache['rev'] = int(self._tasks_cache['rev']) + 1
                        self._tasks_cache['_sig'] = sig
                    self._tasks_cache['ts'] = now
                    self._tasks_cache.pop('error', None)
                except Exception as e:
                    self._tasks_cache['error'] = str(e)
                    self._tasks_cache['ts'] = now
            return {k:v for k,v in self._tasks_cache.items() if k not in ('_sig',)}

    # ---------------- DASHBOARD SERVER (Task Manager + Robots UI on :5005) ----------------
    def _start_dashboard_server(self):
        # IMPORTANT: this package name must match where your web assets are installed
        pkg_share = get_package_share_directory('rmf_manager_cloud')
        base = os.path.join(pkg_share, 'web')
        self.get_logger().info(f"Web assets (dashboard): {base}")

        app = Flask(
            __name__,
            template_folder=os.path.join(base, 'templates'),
            static_folder=os.path.join(base, 'static'),
            static_url_path='/static'
        )

        @app.after_request
        def _nocache(resp):
            resp.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate, max-age=0'
            resp.headers['Pragma'] = 'no-cache'
            resp.headers['Expires'] = '0'
            return resp

        @app.get('/')
        def index():
            data = self._get_tasks_snapshot(force=True)
            vertices = sorted(list(self.g.V.keys()))
            home_id = 'station'
            return render_template('index.html', data=data, vertices=vertices, home_id=home_id)

        @app.get('/api/tasks')
        def api_tasks():
            snap = self._get_tasks_snapshot(force=False)
            r = make_response(jsonify(snap))
            r.headers['Cache-Control'] = 'no-store'
            return r

        @app.get('/api/robots')
        def api_robots():
            out = []
            now = time.time()
            for ns in self.namespaces:
                s = self._z_states.get(ns)
                pose = None; path = []
                if s:
                    pose = {'x': float(s.pose.x), 'y': float(s.pose.y), 'yaw': float(s.pose.theta)}
                    path = [str(v) for v in s.path_vertices]
                out.append({
                    'ns': ns,
                    'age': (now - self._z_last_heard[ns]) if self._z_last_heard[ns] else None,
                    'pose': pose,
                    'path': path,
                })
            return jsonify(out)

        @app.get('/robots')
        def robots_page():
            return render_template('robots.html')

        @app.get('/api/status')
        def api_status_for_dashboard():
            ns = request.args.get('ns')
            s = None
            if ns and _norm_ns(ns) in self._z_states and self._z_states[_norm_ns(ns)] is not None:
                s = self._z_states[_norm_ns(ns)]
            else:
                s = self._last_state
            if not s:
                return jsonify({'ok': False})
            xpx, ypx = self.g.invert_tr(s.pose.x, s.pose.y)
            return jsonify({'ok': True, 'x': xpx, 'y': ypx, 'yaw': s.pose.theta, 't': time.time()})

        @app.get('/api/robot/<path:ns>/diagnostics')
        def api_robot_diag(ns):
            ns = _norm_ns(ns)
            b = self._z_diag_text.get(ns)
            if not b:
                return Response('N/A', mimetype='text/plain')
            try:
                txt = b.decode('utf-8', errors='replace')
            except Exception:
                txt = b[:2048].hex()
            return Response(txt, mimetype='text/plain')

        @app.get('/api/robot/<path:ns>/image')
        def api_robot_image(ns):
            ns = _norm_ns(ns)
            b = self._z_image.get(ns)
            if not b:
                return ('', 404)
            mime = _guess_image_mime(b)
            if not mime:
                return ('', 404)
            return Response(b, mimetype=mime)

        # ----- Original forms/actions -----
        @app.post('/resume_after_charge')
        def resume_after_charge():
            enabled = 'enabled' in request.form
            try:
                req = SetResumeAfterCharge.Request(); req.enabled = bool(enabled)
                self._call(self.cli_resume, req)
            except Exception as e:
                self.get_logger().warn(f"resume_after_charge: {e}")
            return redirect(url_for('index'))

        @app.post('/return_home')
        def return_home():
            msg = RobotCommand(); msg.type = 5; msg.robot_name = '*'
            self.cmd_pub.publish(msg)
            self._publish_command_zenoh(msg, ns=None)
            return redirect(url_for('index'))

        @app.post('/schedule/patrol')
        def schedule_patrol():
            wps = request.form.getlist('waypoints')
            loops = int(request.form.get('loops','1') or 1)
            position = request.form.get('position','bottom')
            try:
                req = SchedulePatrol.Request(); req.waypoints = wps; req.loops = loops; req.start_now = False
                r = self._call(self.cli_patrol, req)
                if position == 'top' and r.task_id:
                    m = MoveTask.Request(); m.task_id = r.task_id; m.direction = 'top'; m.new_index = -1
                    self._call(self.cli_move, m)
            except Exception as e:
                self.get_logger().warn(f"patrol: {e}")
            return redirect(url_for('index'))

        @app.post('/schedule/delivery')
        def schedule_delivery():
            pickup = request.form.get('pickup',''); dropoff = request.form.get('dropoff','')
            wait_for_trigger = bool(request.form.get('wait_for_trigger'))
            position = request.form.get('position','bottom')
            try:
                req = ScheduleDelivery.Request()
                req.pickup = pickup; req.dropoff = dropoff; req.wait_for_trigger = wait_for_trigger
                r = self._call(self.cli_deliv, req)
                if position == 'top' and r.task_id:
                    m = MoveTask.Request(); m.task_id = r.task_id; m.direction='top'; m.new_index=-1
                    self._call(self.cli_move, m)
            except Exception as e:
                self.get_logger().warn(f"delivery: {e}")
            return redirect(url_for('index'))

        @app.post('/schedule/goto')
        def schedule_goto():
            goal = request.form.get('goal','')
            position = request.form.get('position','bottom')
            try:
                req = ScheduleGoTo.Request(); req.goal = goal; req.start_now=False
                r = self._call(self.cli_goto, req)
                if position == 'top' and r.task_id:
                    m = MoveTask.Request(); m.task_id = r.task_id; m.direction='top'; m.new_index=-1
                    self._call(self.cli_move, m)
            except Exception as e:
                self.get_logger().warn(f"goto: {e}")
            return redirect(url_for('index'))

        @app.post('/cancel/<task_id>')
        def cancel(task_id):
            try:
                req = CancelTask.Request(); req.task_id = task_id
                self._call(self.cli_cancel, req)
            except Exception as e:
                self.get_logger().warn(f"cancel: {e}")
            return redirect(url_for('index'))

        @app.post('/cancel_all')
        def cancel_all():
            try:
                self._call(self.cli_cancel_all, CancelAll.Request())
            except Exception as e:
                self.get_logger().warn(f"cancel_all: {e}")
            return redirect(url_for('index'))

        @app.post('/move/<task_id>/<direction>')
        def move_dir(task_id, direction):
            try:
                req = MoveTask.Request(); req.task_id = task_id; req.direction = direction; req.new_index = -1
                self._call(self.cli_move, req)
            except Exception as e:
                self.get_logger().warn(f"move_dir: {e}")
            return redirect(url_for('index'))

        @app.post('/reorder/<task_id>')
        def reorder(task_id):
            try:
                idx = int(request.form.get('index','0') or 0)
                req = MoveTask.Request(); req.task_id = task_id; req.direction = ''; req.new_index = idx
                self._call(self.cli_move, req)
            except Exception as e:
                self.get_logger().warn(f"reorder: {e}")
            return redirect(url_for('index'))

        @app.post('/trigger/<task_id>/<stage>')
        def trigger(task_id, stage):
            try:
                req = TriggerDelivery.Request(); req.task_id = task_id; req.stage = stage
                self._call(self.cli_trigger, req)
            except Exception as e:
                self.get_logger().warn(f"trigger: {e}")
            return redirect(url_for('index'))

        def _serve():
            self.get_logger().info(f"Dashboard (Task Manager + Robots): http://{self.ui_host}:{self.ui_port}")
            serve(app, host=self.ui_host, port=self.ui_port, threads=8)
        threading.Thread(target=_serve, daemon=True).start()

    # ---------------- VISUALIZER SERVER (OSM on :8080) ----------------
    def _start_viz_server(self):
        pkg_share = get_package_share_directory('rmf_manager_cloud')
        base = os.path.join(pkg_share, 'web')
        tpl = os.path.join(base, 'templates', 'viz.html')
        self.get_logger().info(f"Web assets (visualizer): {base}")
        self.get_logger().info(f"Visualizer template exists={os.path.exists(tpl)} path={tpl}")

        app = Flask(
            __name__,
            template_folder=os.path.join(base, 'templates'),
            static_folder=os.path.join(base, 'static'),
            static_url_path='/static'
        )

        @app.after_request
        def _nocache(resp):
            resp.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate, max-age=0'
            resp.headers['Pragma'] = 'no-cache'
            resp.headers['Expires'] = '0'
            return resp

        @app.get('/healthz')
        def healthz():
            return jsonify({'ok': True, 'ts': time.time()})

        @app.get('/api/osm_defaults')
        def api_osm_defaults():
            return jsonify({
                'lat': self.osm_default_lat,
                'lon': self.osm_default_lon,
                'zoom': self.osm_default_zoom
            })

        @app.get('/')
        def viz_index():
            return render_template('viz.html')

        @app.get('/viz/image')
        def viz_image():
            if not self.g.image_path or not os.path.exists(self.g.image_path):
                return ('', 404)
            return send_file(self.g.image_path)

        @app.get('/api/graph')
        def api_graph():
            verts = [{'name': n, 'x': v.xp, 'y': v.yp} for n, v in self.g.V.items()]
            return jsonify({'vertices': verts, 'edges': self.E_as_px(), 'image_available': bool(self.g.image_path)})

        # same edges helper (px space)
        def _edge_dict(e):
            return {'a': e['a'], 'b': e['b'], 'ax': e['ax'], 'ay': e['ay'], 'bx': e['bx'], 'by': e['by']}
        def _edges_px():
            return [_edge_dict(e) for e in self.g.E]
        self.E_as_px = _edges_px

        @app.get('/api/robots')
        def api_robots():
            out = []
            now = time.time()
            for ns in self.namespaces:
                s = self._z_states.get(ns)
                pose = None; path = []
                if s:
                    pose = {'x': float(s.pose.x), 'y': float(s.pose.y), 'yaw': float(s.pose.theta)}
                    path = [str(v) for v in s.path_vertices]
                out.append({
                    'ns': ns,
                    'age': (now - self._z_last_heard[ns]) if self._z_last_heard[ns] else None,
                    'pose': pose,
                    'path': path,
                })
            return jsonify(out)

        @app.get('/api/status')
        def api_status():
            ns = request.args.get('ns')
            s = None
            if ns and _norm_ns(ns) in self._z_states and self._z_states[_norm_ns(ns)] is not None:
                s = self._z_states[_norm_ns(ns)]
            else:
                s = self._last_state

            if not s:
                return jsonify({'ok': False})
            xpx, ypx = self.g.invert_tr(s.pose.x, s.pose.y)
            return jsonify({'ok': True, 'x': xpx, 'y': ypx, 'yaw': s.pose.theta, 't': time.time()})

        @app.get('/api/path')
        def api_path():
            ns = request.args.get('ns')
            s = None
            if ns and _norm_ns(ns) in self._z_states and self._z_states[_norm_ns(ns)] is not None:
                s = self._z_states[_norm_ns(ns)]
            else:
                s = self._last_state

            if not s:
                return jsonify({'path': [], 'seq': 0})
            pts = []
            for n in list(s.path_vertices):
                if n in self.g.V:
                    v = self.g.V[n]
                    pts.append({'x': v.xp, 'y': v.yp, 'name': n})
            return jsonify({'path': pts, 'seq': int(time.time()*1000), 't': time.time()})

        # allow patrol scheduling from the viz origin (same-port POST)
        @app.post('/api/schedule_patrol')
        def viz_schedule_patrol():
            try:
                data = request.get_json(force=True, silent=True) or {}
                wps = data.get('waypoints') or []
                loops = int(data.get('loops') or 1)
                position = str(data.get('position') or 'bottom')
                if not isinstance(wps, list) or not wps:
                    return jsonify({'ok': False, 'error': 'no waypoints'}), 400

                req = SchedulePatrol.Request()
                req.waypoints = [str(w) for w in wps]
                req.loops = loops
                req.start_now = False

                r = self._call(self.cli_patrol, req)
                task_id = getattr(r, 'task_id', '')

                moved = False
                warning = ""
                if position == 'top' and task_id:
                    try:
                        if self.cli_move.wait_for_service(timeout_sec=0.25):
                            m = MoveTask.Request()
                            m.task_id = task_id
                            m.direction = 'top'
                            m.new_index = -1
                            self._call(self.cli_move, m)
                            moved = True
                        else:
                            warning = "move_task unavailable"
                    except Exception as e:
                        self.get_logger().warn(f"viz move_task: {e}")
                        warning = str(e) or "move_task failed"

                payload = {'ok': True, 'task_id': task_id, 'moved': moved}
                if warning:
                    payload['warning'] = warning
                resp = make_response(jsonify(payload), 200)
                resp.headers['Cache-Control'] = 'no-store'
                return resp
            except Exception as e:
                self.get_logger().warn(f"viz patrol: {e}")
                return jsonify({'ok': False, 'error': str(e)}), 500

        def _serve():
            self.get_logger().info(f"Visualizer (OSM): http://{self.ui_viz_host}:{self.ui_viz_port}")
            serve(app, host=self.ui_viz_host, port=self.ui_viz_port, threads=8)
        threading.Thread(target=_serve, daemon=True).start()


def main():
    if '--ros-domain-id' in sys.argv:
        try:
            i = sys.argv.index('--ros-domain-id')
            if i + 1 < len(sys.argv):
                os.environ['ROS_DOMAIN_ID'] = str(int(sys.argv[i + 1]))
            del sys.argv[i:i+2]
        except Exception:
            sys.argv = [a for a in sys.argv if a != '--ros-domain-id']

    rclpy.init(args=sys.argv)
    node = DashboardNode()

    def _on_sigint(signum, frame):
        try:
            node.get_logger().info('SIGINT received, shutting down…')
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except RCLError:
            pass
        except Exception:
            pass

    signal.signal(signal.SIGINT, _on_sigint)

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
        except RCLError:
            pass
        except Exception:
            pass


if __name__ == '__main__':
    main()
