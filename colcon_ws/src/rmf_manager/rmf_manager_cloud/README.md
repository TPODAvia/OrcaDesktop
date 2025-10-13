
---

# 3) Build & run

```bash
# In your colcon workspace src/
cd ~/colcon_ws/src
git clone <this-two-folders>   # or copy them in

cd ..
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install

# Robot (on each robot host)
. install/setup.bash
ros2 launch decent_fleet_client client.launch.py

# Server (run as many servers as you like)
. install/setup.bash
ros2 launch decent_fleet_server server.launch.py
# UI at http://<server_ip>:5007 ; Global map at http://<server_ip>:8090
```

---

# 4) CockroachDB schema (manual once)

```sql
-- connect: cockroach sql --insecure --host <node>
CREATE DATABASE IF NOT EXISTS fleet;
SET DATABASE = fleet;

CREATE TABLE IF NOT EXISTS telemetry (
  robot_id STRING NOT NULL,
  ts TIMESTAMPTZ NOT NULL DEFAULT now(),
  map_id STRING, floor_name STRING,
  x_local FLOAT8, y_local FLOAT8, yaw_local FLOAT8,
  x_wcs FLOAT8, y_wcs FLOAT8, yaw_wcs FLOAT8,
  battery FLOAT8, mode STRING,
  PRIMARY KEY (robot_id, ts DESC)
) WITH (ttl_expiration_expression = now() + interval '8760 hours', ttl_job_cron='@hourly');

CREATE TABLE IF NOT EXISTS map_registry (
  map_id STRING PRIMARY KEY,
  floor_name STRING,
  rot FLOAT8, scale FLOAT8, tx FLOAT8, ty FLOAT8,
  updated_at TIMESTAMPTZ NOT NULL DEFAULT now()
);

CREATE TABLE IF NOT EXISTS rag_embeddings (
  key STRING PRIMARY KEY,
  created_at TIMESTAMPTZ NOT NULL DEFAULT now(),
  text STRING,
  embedding VECTOR(384),
  metadata JSONB
) WITH (ttl_expiration_expression = now() + interval '8760 hours', ttl_job_cron='@hourly');

CREATE INDEX IF NOT EXISTS rag_embeddings_vec_idx ON rag_embeddings (embedding) USING ivfflat WITH (lists = 100);
```

> If your Cockroach version prefers `ALTER TABLE ... SET (ttl_expiration_expression = ...)`, run the same settings via `ALTER TABLE`.

---

# 5) How it works (quick)

* **Clients** publish:

  * `map_meta.<map_id>` once (WCS transform)
  * `telemetry.<robot_id>` at `telemetry_hz` with local + WCS pose
  * **Commands**: they listen to `cmd.<robot_id>` and translate JSON → local RMF services

* **Servers** (many):

  * Subscribe to all telemetry + map_meta
  * Store **1-year history** in Cockroach (`telemetry`)
  * Offer **UI** to multi-select robots and send tasks (publishes to `cmd.<robot_id>`)
  * Show **global WCS** view of all robots across heterogeneous maps
  * Expose **RAG** endpoints with `VECTOR` for embeddings

* **Decentralized**: any number of servers attaches to the same **NATS** + **Cockroach** and sees/controls the fleet.

---

If you want, I can also add:

* A **TLS/mTLS** example for NATS
* A **WireGuard** sample for site-to-site WAN
* ROS2-native fallback (DDS Router / zenoh-bridge-ros2dds) if you prefer no NATS bus.




Short answer: almost out-of-the-box inside your VPN — you just need to bring up **one NATS endpoint** and a **CockroachDB cluster**, give each robot a **unique ID + WCS transform**, and point both packages at those two endpoints. Everything else is already wired.

Here’s the minimal checklist so nothing bites you.

# What you must do once (shared infra)

1. **Start NATS in the VPN**

* Any single IP that all robots/servers can reach:

  ```bash
  docker run -d --name nats -p 4222:4222 -p 8222:8222 nats:2 -js -sd /data
  ```
* Keep the IP/hostname (e.g. `10.0.0.10:4222`) for configs.

2. **Start CockroachDB (vector support built-in)**

* Dev single-node (quick try):

  ```bash
  docker run -d --name crdb -p 26257:26257 -p 8080:8080 \
    cockroachdb/cockroach:latest start-single-node --insecure
  ```
* For prod, run 3+ nodes and `--join` them.
* The server node will auto-create the `fleet` DB and tables on first run, but the DB must be reachable.

3. **Open VPN/firewall ports**

* NATS: `4222` (+ `8222` optional monitoring)
* Cockroach: `26257`
* Server UI: `5007` (tasks) and `8090` (global map) per server instance

4. **Time sync**

* TTL and history rely on good clocks. On all hosts:

  ```bash
  sudo apt-get install -y chrony
  sudo systemctl enable --now chrony
  ```

# Per robot (client package)

* File: `decent_fleet_client/params/client.yaml`

  * `robot_id`: **unique** (e.g. `yhs_01`, `yhs_02`, …)
  * `map_id`, `floor_name`: set to the local map and floor this robot uses
  * `wcs_rotation_rad`, `wcs_scale`, `wcs_tx`, `wcs_ty`: **your WCS calibration** for that map
  * `nats_url`: `nats://<NATS_IP>:4222`
  * `state_topic` / service names: keep defaults if your RMF core matches, otherwise adjust
* Run:

  ```bash
  . ~/colcon_ws/install/setup.bash
  ros2 launch decent_fleet_client client.launch.py
  ```
* That’s it. The client:

  * publishes `telemetry.<robot_id>` (local + WCS pose)
  * publishes `map_meta.<map_id>` (WCS transform)
  * listens on `cmd.<robot_id>` and translates JSON commands to your existing RMF services

# Per server (server package) — you can run many

* File: `decent_fleet_server/params/server.yaml`

  * `nats_url`: `nats://<NATS_IP>:4222`
  * `db_dsn`: e.g. `host=<CRDB_IP> port=26257 dbname=fleet user=root sslmode=disable`
  * (optional) `rag_model`: leave default, or set your SentenceTransformer name
  * If you run multiple servers on the **same host**, give each a different `ui_port`/`ui_viz_port`.
* Run:

  ```bash
  . ~/colcon_ws/install/setup.bash
  ros2 launch decent_fleet_server server.launch.py
  ```
* First server up will **ensure schema** (creates tables with 1-year TTL and VECTOR index).
* Any additional server just connects and shows the same fleet.

# What works immediately (no extra wiring)

* **Multi-server:** every server sees the same telemetry and can command any subset of robots.
* **Multi-robot tasking:** pick 1..N robots in the UI and schedule patrol/goto/delivery; it fan-outs over NATS.
* **Global map view (WCS):** all robots are shown together even if they run different maps — provided each robot’s `client.yaml` has the correct WCS transform (that’s the only “calibration” you must provide).
* **1-year history:** telemetry is written to Cockroach with a row-TTL of ~8760h.
* **RAG:** `/api/rag/insert` and `/api/rag/search` work as soon as you install `sentence-transformers` on the server host (or you can POST your own embeddings).

# Nice-to-have (optional)

* **JetStream persistence on the bus:** the code uses plain pub/sub; Cockroach keeps history. If you also want the bus to persist while servers are down, define streams/consumers in NATS JetStream (not required).
* **TLS/mTLS:** NATS and Cockroach can both run with TLS; just change `nats_url`/`db_dsn` accordingly.
* **Systemd services:** wrap the two launch commands for auto-start.

# Quick sanity check (5 minutes)

1. Bring up NATS + Cockroach in the VPN.
2. Start one server — open `http://<server_ip>:5007` (Tasks) and `:8090` (Global).
3. Start two robots with different `robot_id` and WCS params.
4. See both robots appear in **Global**, multi-select them, queue a patrol — each robot should receive a `cmd.<robot_id>` and your RMF services will execute it.

If you keep those four moving parts consistent (NATS IP, Cockroach IP, unique `robot_id`, correct WCS), it’ll “just work.”
