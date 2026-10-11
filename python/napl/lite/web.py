"""Operator web UI (stdlib http.server, no dependency): single page, MJPEG feed, SSE state, POST commands.

    GET  /?token=T              ui.html
    GET  /video.mjpg?token=T    multipart/x-mixed-replace JPEG (15 fps, from `jpeg_source.jpeg()`); boxes are drawn client-side from /events
    GET  /events?token=T        SSE, 10 Hz: App.snapshot() as JSON
    POST /api/{arm,disarm,estop_ui,home,authorize,auth_mode,sector,range,deadman}?token=T    JSON body

Every request needs the token (random at start, printed on the console). Binds to localhost unless the runner passes --lan. The UI "E-stop" button only
disarms: it is not the safety E-stop (the hardware mushroom + key are). Commands only enqueue into the App (single-writer rule); `deadman` stamps a
time. The page posts /api/deadman every 200 ms while the HOLD control is pressed."""

import json
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

PAGE = Path(__file__).parent / "static" / "ui.html"
COMMANDS = {
    "arm",
    "disarm",
    "estop_ui",
    "home",
    "authorize",
    "auth_mode",
    "sector",
    "range",
    "deadman",
}
BOUNDARY = "cuasframe"


def serve(app, host, port, token, jpeg_source=None):
    class H(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass

        def _auth(self):
            q = parse_qs(urlparse(self.path).query)
            if q.get("token", [""])[0] != token:
                self.send_error(403, "token required")
                return False
            return True

        def _send(self, code, body, ctype="application/json"):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if not self._auth():
                return
            path = urlparse(self.path).path
            if path == "/":
                return self._send(200, PAGE.read_bytes(), "text/html; charset=utf-8")
            if path == "/events":
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                try:
                    while True:
                        self.wfile.write(
                            b"data: "
                            + json.dumps(app.snapshot(), default=float).encode()
                            + b"\n\n"
                        )
                        self.wfile.flush()
                        time.sleep(0.1)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    return
            if (
                path == "/video.mjpg"
                and jpeg_source is not None
                and hasattr(jpeg_source, "jpeg")
            ):
                self.send_response(200)
                self.send_header(
                    "Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}"
                )
                self.end_headers()
                try:
                    while True:
                        jpg = jpeg_source.jpeg()
                        if jpg:
                            self.wfile.write(
                                f"--{BOUNDARY}\r\nContent-Type: image/jpeg\r\nContent-Length: {len(jpg)}\r\n\r\n".encode()
                                + jpg
                                + b"\r\n"
                            )
                            self.wfile.flush()
                        time.sleep(1 / 15)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    return
            self.send_error(404)

        def do_POST(self):
            if not self._auth():
                return
            cmd = urlparse(self.path).path.rpartition("/")[2]
            if not self.path.startswith("/api/") or cmd not in COMMANDS:
                return self._send(404, b'{"ok":false}')
            n = int(self.headers.get("Content-Length") or 0)
            try:
                kw = json.loads(self.rfile.read(n) or b"{}")
                if not isinstance(kw, dict):
                    raise ValueError
            except ValueError:
                return self._send(400, b'{"ok":false,"error":"bad json"}')
            if cmd == "deadman":
                app.deadman()
            else:
                app.post(cmd, **kw)
            self._send(200, b'{"ok":true}')

    srv = ThreadingHTTPServer((host, port), H)
    srv.daemon_threads = True
    return srv
