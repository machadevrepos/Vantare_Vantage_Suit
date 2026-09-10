#!/usr/bin/env python3
"""Serve the live tool with caching disabled.

`python -m http.server` lets the browser cache the ES modules, so index.html can
reload with a new control on it while js/*.js is still served from cache. The
symptom is a button that does nothing and no error anywhere — which cost a
recording session on 2026-09-10 (the log showed the old 14-column motion stream
and no hinge events at all).

    python host/live_tool/serve.py [port]

Then open http://localhost:8080. Web Bluetooth needs localhost or https; opening
index.html as a file will not work.
"""

from __future__ import annotations

import functools
import http.server
import os
import socketserver
import sys

DEFAULT_PORT = 8080
ROOT = os.path.dirname(os.path.abspath(__file__))


class NoCacheHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        super().end_headers()

    def send_header(self, keyword, value):
        # SimpleHTTPRequestHandler emits Last-Modified, which is enough for a
        # browser to revalidate into a 304 and keep using the cached module.
        if keyword == "Last-Modified":
            return
        super().send_header(keyword, value)

    def log_message(self, fmt, *args):
        sys.stderr.write("%s - %s\n" % (self.address_string(), fmt % args))


def main() -> int:
    port = int(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_PORT
    handler = functools.partial(NoCacheHandler, directory=ROOT)
    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(("127.0.0.1", port), handler) as httpd:
        print(f"Serving {ROOT} with caching disabled")
        print(f"  http://localhost:{port}")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nstopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
