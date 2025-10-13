#!/usr/bin/env bash
set -euo pipefail

APP_NAME="gundb-p2p-pubsub-python"
NET="${APP_NAME}-net"
GUN_IMG="${APP_NAME}:server"
R1_CTN="${APP_NAME}-relay1"
R2_CTN="${APP_NAME}-relay2"
SND_CTN="${APP_NAME}-sender"
RCV_CTN="${APP_NAME}-consumer"

# container-internal ports (relays listen here)
REL_PORT=8765
# optional host ports just for manual testing:
HOST_R1=18765
HOST_R2=18766

CHANNEL="demo"
COUNT=10

# volumes
R1_DATA="$(pwd)/gun-data-r1"
R2_DATA="$(pwd)/gun-data-r2"
RCV_DB_DIR="$(pwd)/consumer-db"
RCV_DB_PATH="${RCV_DB_DIR}/messages.db"

say(){ echo -e "[$(date +%H:%M:%S)] $*"; }

# --- clean leftovers that might still hold host ports 8765/8766
docker rm -f "$R1_CTN" "$R2_CTN" >/dev/null 2>&1 || true

say "Creating isolated docker network: $NET"
docker network create "$NET" >/dev/null 2>&1 || true

say "Building GunDB relay image (${GUN_IMG})..."
docker build -t "$GUN_IMG" - <<'DOCKERFILE'
FROM node:20-alpine
WORKDIR /app
RUN apk add --no-cache tini curl
RUN npm init -y >/dev/null 2>&1 && npm i gun@^0.2020.1237 >/dev/null 2>&1
ADD <<'JS' /app/server.js
const http = require('http');
const { URL } = require('url');
const Gun = require('gun');
require('gun/sea');
require('gun/lib/open');
require('gun/lib/load');

const PORT  = Number(process.env.PORT || 8765);
const HOST  = process.env.HOST || '0.0.0.0';
const PEERS = (process.env.PEERS || '').split(',').filter(Boolean);
const DATA  = process.env.DATA_DIR || '/data';
const ROOT  = process.env.ROOT || 'demo';

const srv = http.createServer((req, res) => {
  const u = new URL(req.url, `http://${req.headers.host}`);
  const send = (code, obj) => { res.writeHead(code, {'content-type':'application/json'}); res.end(JSON.stringify(obj)); };

  if (u.pathname === '/healthz') return send(200, {ok:true});

  if (u.pathname === '/put') {
    const root = u.searchParams.get('root') || ROOT;
    const key  = u.searchParams.get('key');
    const val  = u.searchParams.get('val');
    if (!key || val === null) return send(400, {ok:false, error:'missing key/val'});
    gun.get(root).get(key).put({ v: val, ts: Date.now() }, () => send(200, {ok:true}));
    return;
  }

  if (u.pathname === '/get') {
    const root = u.searchParams.get('root') || ROOT;
    const key  = u.searchParams.get('key');
    if (!key) return send(400, {ok:false, error:'missing key'});
    gun.get(root).get(key).once((d) => send(200, {ok:true, value: d && d.v, raw: d || null}));
    return;
  }

  // Publish a message to a channel
  if (u.pathname === '/pub') {
    const chan = u.searchParams.get('chan') || 'demo';
    const msg  = u.searchParams.get('msg');
    const from = u.searchParams.get('from') || 'sender';
    if (msg == null) return send(400, {ok:false, error:'missing msg'});
    const id = `${Date.now()}-${Math.random().toString(16).slice(2)}`;
    const obj = { id, ts: Date.now(), msg, from, chan };
    gun.get(ROOT).get('channels').get(chan).get('msgs').get(id).put(obj, () => send(200, {ok:true, id}));
    return;
  }

  // Poll messages newer than ?since=epoch_ms
  if (u.pathname === '/poll') {
    const chan  = u.searchParams.get('chan') || 'demo';
    const since = Number(u.searchParams.get('since') || 0);
    const results = [];
    const seen = {};
    const node = gun.get(ROOT).get('channels').get(chan).get('msgs');
    const timer = setTimeout(() => {
      results.sort((a,b)=>a.ts-b.ts);
      send(200, {ok:true, items: results});
    }, 250);
    node.map().once((d, key) => {
      if (!d || !d.ts || !key || seen[key]) return;
      if (d.ts > since) { results.push(d); seen[key] = true; }
    });
    return;
  }

  res.writeHead(404); res.end('gun');
});

const gun = Gun({ web: srv, file: DATA, peers: PEERS });
srv.listen(PORT, HOST, () => {
  console.log(`[gun] listening on http://${HOST}:${PORT}/gun`);
  console.log(`[gun] peers:`, PEERS);
  console.log(`[bridge] REST: /healthz /put /get /pub /poll`);
});
JS
EXPOSE 8765
ENTRYPOINT ["/sbin/tini","--","node","/app/server.js"]
DOCKERFILE

say "Preparing data dirs..."
mkdir -p "$R1_DATA" "$R2_DATA" "$RCV_DB_DIR"

say "Starting Relay-1 (name=${R1_CTN}) on network ${NET}..."
docker run -d --name "$R1_CTN" --network "$NET" \
  -e HOST="0.0.0.0" -e PORT="$REL_PORT" \
  -e DATA_DIR="/data" -e ROOT="$CHANNEL" \
  -e PEERS="http://${R2_CTN}:${REL_PORT}/gun" \
  -v "$R1_DATA:/data" \
  -p "${HOST_R1}:${REL_PORT}" \
  "$GUN_IMG" >/dev/null

say "Starting Relay-2 (name=${R2_CTN}) on network ${NET}..."
docker run -d --name "$R2_CTN" --network "$NET" \
  -e HOST="0.0.0.0" -e PORT="$REL_PORT" \
  -e DATA_DIR="/data" -e ROOT="$CHANNEL" \
  -e PEERS="http://${R1_CTN}:${REL_PORT}/gun" \
  -v "$R2_DATA:/data" \
  -p "${HOST_R2}:${REL_PORT}" \
  "$GUN_IMG" >/dev/null

# health check via host ports (optional) AND intra-net by container name
say "Waiting for relays..."
for i in $(seq 1 60); do
  if curl -fsS "http://127.0.0.1:${HOST_R1}/healthz" >/dev/null 2>&1 &&
     curl -fsS "http://127.0.0.1:${HOST_R2}/healthz" >/dev/null 2>&1; then
    say "Both relays up."
    break
  fi
  sleep 1
done

# SENDER on relay-1 (publish COUNT messages)
say "Python SENDER -> ${R1_CTN} (${COUNT} msgs on channel=${CHANNEL})..."
docker rm -f "$SND_CTN" >/dev/null 2>&1 || true
docker run --rm --name "$SND_CTN" --network "$NET" \
  -e RELAY_URL="http://${R1_CTN}:${REL_PORT}" \
  -e CHANNEL="$CHANNEL" -e COUNT="$COUNT" \
  python:3.11-slim bash -lc '
  pip -q install requests >/dev/null
  python - <<PY
import os, time, requests
relay = os.environ["RELAY_URL"]
chan  = os.environ.get("CHANNEL","demo")
count = int(os.environ.get("COUNT","10"))
print("[sender] relay =", relay, "channel =", chan)
print("[sender] health =", requests.get(relay+"/healthz", timeout=5).ok)
for i in range(count):
    msg = f"msg-{i}"
    r = requests.get(relay+"/pub", params={"chan":chan,"msg":msg,"from":"sender1"}, timeout=5)
    print("[sender] pub", msg, "->", r.status_code, r.text.strip())
    time.sleep(0.05)
print("[sender] done")
PY
'

# CONSUMER on relay-2: poll & store to SQLite
say "Python CONSUMER <- ${R2_CTN} (store to SQLite)..."
docker rm -f "$RCV_CTN" >/dev/null 2>&1 || true
docker run --rm --name "$RCV_CTN" --network "$NET" \
  -e RELAY_URL="http://${R2_CTN}:${REL_PORT}" \
  -e CHANNEL="$CHANNEL" -e EXPECT="$COUNT" \
  -v "$RCV_DB_DIR:/db" \
  python:3.11-slim bash -lc '
  pip -q install requests >/dev/null
  python - <<PY
import os, time, sqlite3, requests
relay = os.environ["RELAY_URL"]
chan  = os.environ.get("CHANNEL","demo")
expect = int(os.environ.get("EXPECT","10"))
dbp = "/db/messages.db"

print("[consumer] relay =", relay, "channel =", chan)
print("[consumer] health =", requests.get(relay+"/healthz", timeout=5).ok)

con = sqlite3.connect(dbp); cur = con.cursor()
cur.execute("""CREATE TABLE IF NOT EXISTS messages(
  id TEXT PRIMARY KEY, ts INTEGER, chan TEXT, sender TEXT, msg TEXT
)"""); con.commit()

since = 0; seen = set(); received = 0; deadline = time.time() + 60.0
while time.time() < deadline and received < expect:
    r = requests.get(relay+"/poll", params={"chan":chan,"since":since}, timeout=10)
    js = {}
    try: js = r.json()
    except: time.sleep(0.5); continue
    for it in js.get("items", []):
        mid = it.get("id")
        if not mid or mid in seen: continue
        seen.add(mid)
        ts  = int(it.get("ts", 0))
        msg = it.get("msg")
        sender = it.get("from","unknown")
        cur.execute("INSERT OR IGNORE INTO messages(id,ts,chan,sender,msg) VALUES(?,?,?,?,?)",
                    (mid, ts, chan, sender, msg))
        received += 1
        since = max(since, ts)
        print(f"[consumer] got {mid} -> {msg}")
    con.commit()
    time.sleep(0.3)

print(f"[consumer] total received: {received}/{expect}")
cur.execute("SELECT COUNT(*), MIN(ts), MAX(ts) FROM messages")
print("[consumer] db stats:", cur.fetchone())
con.close()
PY
'

say "---- RELAY-1 LOGS (tail 15) ----"; docker logs --tail=15 "$R1_CTN" || true
say "---- RELAY-2 LOGS (tail 15) ----"; docker logs --tail=15 "$R2_CTN" || true

say "Reading stored messages from host via Python..."
docker run --rm -v "$RCV_DB_DIR:/db" python:3.11-slim python - <<'PY'
import sqlite3
con=sqlite3.connect("/db/messages.db"); cur=con.cursor()
cur.execute("SELECT COUNT(*) FROM messages"); print("[host] messages:", cur.fetchone()[0])
cur.execute("SELECT id, ts, sender, msg FROM messages ORDER BY ts LIMIT 5")
for row in cur.fetchall(): print("[host]", row)
con.close()
PY

cat <<EOF

==> DONE

Two isolated relays on a private network:
  • Relay-1: http://localhost:${HOST_R1}/gun  (REST: /pub /poll /put /get /healthz)
  • Relay-2: http://localhost:${HOST_R2}/gun

Sender talks to Relay-1 (container name):
  http://${R1_CTN}:${REL_PORT}/pub

Consumer polls Relay-2 and stores to SQLite:
  ${RCV_DB_PATH}

Manual tests (from host):
  curl 'http://localhost:${HOST_R1}/pub?chan=${CHANNEL}&msg=hello'
  curl 'http://localhost:${HOST_R2}/poll?chan=${CHANNEL}&since=0'

Clean up:
  docker rm -f ${R1_CTN} ${R2_CTN}
  rm -rf "$R1_DATA" "$R2_DATA" "$RCV_DB_DIR"
EOF
