> ⚠️ Обрати внимание на отступы: два пробела перед `zoneminder`, как у `grafana`.

We keep ZoneMinder on **bridge network with port mapping**, not `network_mode: host`, to avoid port 80 conflicts with anything on the host.

---

## 3️⃣ Build and run

From the compose directory:

```bash
cd /home/rover2/OrcaDesktops/docker

# Build only ZM image
docker compose build zoneminder

# Start it
docker compose up -d zoneminder

# Check logs
docker compose logs -f zoneminder
```

---

## 4️⃣ Open ZoneMinder UI

On `rover2`:

```text
http://localhost:8080/zm
```

From another machine in LAN:

```text
http://<IP_of_rover2>:8080/zm
```

First start usually has no auth; you can enable it later in **Options → System**.

---

If you want next step, I can:

* add a **generic RTSP camera** example (Hikvision/Dahua/Reolink or robot camera),
* or convert this into a more production-style setup with **separate DB container** and persistent DB volume.
