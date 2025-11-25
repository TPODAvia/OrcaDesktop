#!/usr/bin/env python3
"""
Minimal camera test UI.

- Serves http://<host>:5090/
- Shows a single iframe that loads your go2rtc stream.html page.
"""

from flask import Flask, render_template_string

app = Flask(__name__)

# >>> CHANGE THIS to your actual go2rtc URL <<<
# If you open the UI from your laptop, DO NOT use 127.0.0.1 here.
# Use the robot's IP or hostname, e.g. "http://192.168.1.50:1984/..."
CAMERA_URL = "http://localhost:1984/stream.html?src=yhs_yhsros2_cam1"

HTML = r"""
<!doctype html>
<html>
  <head>
    <meta charset="utf-8" />
    <title>Camera test</title>
    <meta name="viewport" content="width=device-width,initial-scale=1" />
    <style>
      body {
        margin: 0;
        background: #0f1115;
        color: #e5e7eb;
        font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
        display: flex;
        flex-direction: column;
        height: 100vh;
      }
      header {
        padding: 8px 12px;
        background: #111827;
        border-bottom: 1px solid #1f2933;
        font-size: 14px;
      }
      main {
        flex: 1;
        display: flex;
        align-items: stretch;
        justify-content: center;
        padding: 8px;
      }
      .card {
        flex: 1;
        max-width: 960px;
        border-radius: 12px;
        border: 1px solid #1f2933;
        overflow: hidden;
        background: #020617;
        display: flex;
        flex-direction: column;
      }
      .card-header {
        padding: 8px 12px;
        font-size: 13px;
        border-bottom: 1px solid #1f2933;
        display: flex;
        justify-content: space-between;
        align-items: center;
      }
      .card-url {
        font-size: 11px;
        color: #9ca3af;
        overflow: hidden;
        text-overflow: ellipsis;
        white-space: nowrap;
        max-width: 70%;
      }
      .card-body {
        flex: 1;
        background: black;
      }
      iframe {
        width: 100%;
        height: 100%;
        border: none;
        display: block;
        background: black;
      }
    </style>
  </head>
  <body>
    <header>
      <strong>Camera test UI</strong>
      <span style="font-size:12px;margin-left:8px;">If you see video below, go2rtc + networking are OK.</span>
    </header>
    <main>
      <div class="card">
        <div class="card-header">
          <span>Hardcoded camera</span>
          <span class="card-url">{{ camera_url }}</span>
        </div>
        <div class="card-body">
          <iframe src="{{ camera_url }}" allowfullscreen></iframe>
        </div>
      </div>
    </main>
  </body>
</html>
"""


@app.route("/")
def index():
    return render_template_string(HTML, camera_url=CAMERA_URL)


if __name__ == "__main__":
    # Run on all interfaces so you can open it from your laptop
    app.run(host="0.0.0.0", port=5090, debug=True)
