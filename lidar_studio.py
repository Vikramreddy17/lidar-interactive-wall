import tkinter as tk
import threading
import serial
import socket
import json
import math
import os
import time

# ==========================================
# CONFIGURATION - SAME MAP AS THE GAME
# ==========================================
#
#   Wall      : 149 in wide x 80 in tall
#   Dead zone : 4 in on the left, 4 in on the right (8 in waste)
#   Active    : 141 in
#   LiDAR     : left edge, 38 in from the top (42 in to the bottom)
#
#   Window    : 1490 x 800 px  ->  10 px per inch (same as game.py)
#   The map fills the WHOLE window. No sidebar, no headings, no margins.

FULL_W_IN = 149.0
HEIGHT_IN = 80.0
DEAD_LEFT_IN = 4.0
DEAD_RIGHT_IN = 4.0
SENSOR_Y_DEFAULT = 38.0

# Changes whenever the geometry changes, so old saved settings are ignored
LAYOUT = "149-4-4-38"

MM = 25.4
FULL_W = FULL_W_IN * MM
HEIGHT = HEIGHT_IN * MM
X0 = DEAD_LEFT_IN * MM
X1 = (FULL_W_IN - DEAD_RIGHT_IN) * MM
ACTIVE_W = X1 - X0

PPI = 10
WIN_W = int(FULL_W_IN * PPI)   # 1490
WIN_H = int(HEIGHT_IN * PPI)   # 800

PORT = "COM7"
BAUD = 230400

UDP = ("127.0.0.1", 5005)

SETTINGS_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "lidar_settings.json"
)

BIN_COUNT = 720

MIN_RANGE = 80
MAX_RANGE = 12000
MIN_INTENSITY = 10

STALE = 0.25

# Background subtraction
DIFF = 75
BG_NEIGHBORS = 3
CAPTURE_SECONDS = 1.5
AUTO_CAPTURE_DELAY = 2.0

# Object detection and tracking
GROUP_GAP = 115
TRACK_GATE = 320
TRACK_TTL = 0.27
HEARTBEAT = 0.08


# ==========================================
# CRC8 - LD19 PACKETS
# ==========================================

def crc8(data):
    crc = 0

    for byte in data:
        crc ^= byte

        for _ in range(8):
            crc = (
                (crc << 1) ^
                (0x4D if crc & 0x80 else 0)
            ) & 255

    return crc


# ==========================================
# SERIAL READER
# ==========================================

class Reader:

    def __init__(self):
        self.lock = threading.Lock()
        self.bins = {}
        self.ok = 0
        self.bad = 0
        self.rpm = 0
        self.last = 0.0
        self.error = ""
        self.stop = threading.Event()
        self.thread = None

    def start(self, port):
        self.thread = threading.Thread(
            target=self.run,
            args=(port,),
            daemon=True
        )

        self.thread.start()

    def run(self, port):
        try:
            with serial.Serial(
                port,
                BAUD,
                timeout=0.02
            ) as device:

                buf = bytearray()

                while not self.stop.is_set():
                    buf.extend(device.read(512))

                    while True:
                        k = buf.find(b"\x54\x2c")

                        if k < 0:
                            if len(buf) > 1:
                                del buf[:-1]
                            break

                        if k:
                            del buf[:k]

                        if len(buf) < 47:
                            break

                        pkt = bytes(buf[:47])

                        if crc8(pkt[:46]) != pkt[46]:
                            with self.lock:
                                self.bad += 1

                            del buf[0]
                            continue

                        del buf[:47]

                        speed = int.from_bytes(pkt[2:4], "little")
                        start = int.from_bytes(pkt[4:6], "little")
                        end = int.from_bytes(pkt[42:44], "little")

                        step = ((end - start) % 36000) / 11.0

                        now = time.monotonic()

                        with self.lock:
                            for i in range(12):
                                off = 6 + i * 3

                                distance = int.from_bytes(
                                    pkt[off:off + 2],
                                    "little"
                                )

                                intensity = pkt[off + 2]

                                angle = (
                                    (start + i * step) / 100
                                ) % 360

                                j = int(angle * 2) % BIN_COUNT

                                self.bins[j] = (
                                    distance,
                                    intensity,
                                    now
                                )

                            self.rpm = speed / 6.0
                            self.ok += 1
                            self.last = now

        except (OSError, serial.SerialException) as exc:
            with self.lock:
                self.error = str(exc)

    def snapshot(self):
        t = time.monotonic()

        with self.lock:
            scan = [
                (i, i * 0.5, d, intensity)
                for i, (d, intensity, ts)
                in self.bins.items()
                if t - ts < STALE
            ]

            stats = (
                self.ok,
                self.bad,
                self.rpm,
                t - self.last if self.last else 999,
                self.error
            )

        return sorted(scan), stats

    def close(self):
        self.stop.set()

        if self.thread:
            self.thread.join(timeout=0.35)


# ==========================================
# LIDAR MAP (FULL WINDOW)
# ==========================================

class Studio:

    def __init__(self, root):
        self.root = root

        root.title("LD19 Map - 149 x 80 in")
        root.geometry(f"{WIN_W}x{WIN_H}")
        root.configure(bg="#0E1F39")

        self.canvas = tk.Canvas(
            root,
            bg="#0E1F39",
            highlightthickness=0,
            bd=0
        )

        self.canvas.pack(fill="both", expand=True)

        # Settings (saved between runs)
        self.sensor_y = SENSOR_Y_DEFAULT
        self.forward = 0.0
        self.mirror = False
        self.load_settings()

        self.port = PORT
        self.demo = False
        self.show_grid = False
        self.show_info = False
        self.fullscreen = False
        self.auto_capture = True

        self.background_enabled = False
        self.status = "Disconnected"

        self.reader = None
        self.baseline = None

        self.cap_end = None
        self.cap_samples = {}
        self.auto_done = False
        self.first_data = None

        # ID -> [x, y, last_seen, sent_x, sent_y, last_emit]
        self.tracks = {}
        self.next_id = 1

        self.demo_down = False
        self.was_demo = False

        self.stats_text = ""

        self.sender = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM
        )

        self.canvas.bind("<ButtonPress-1>", self.on_down)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_up)
        root.bind("<Key>", self.on_key)

        root.protocol("WM_DELETE_WINDOW", self.close)

        self.connect()
        self.tick()

    # ======================================
    # SETTINGS
    # ======================================

    def load_settings(self):
        try:
            with open(SETTINGS_FILE) as f:
                data = json.load(f)

            if data.get("layout") != LAYOUT:
                return

            self.sensor_y = float(data.get("sensor_y", self.sensor_y))
            self.forward = float(data.get("forward", self.forward))
            self.mirror = bool(data.get("mirror", self.mirror))

        except (OSError, ValueError, TypeError):
            pass

    def save_settings(self):
        try:
            with open(SETTINGS_FILE, "w") as f:
                json.dump(
                    {
                        "layout": LAYOUT,
                        "sensor_y": self.sensor_y,
                        "forward": self.forward,
                        "mirror": self.mirror
                    },
                    f
                )
        except OSError:
            pass

    # ======================================
    # KEYBOARD (no sidebar, no buttons)
    # ======================================
    #
    #   H        info overlay
    #   G        10 in grid
    #   C        capture EMPTY wall again
    #   M        mirror Y
    #   Up/Down  sensor position from top (0.5 in)
    #   Left/Right  forward angle (1 degree)
    #   D        mouse demo
    #   R        reconnect sensor
    #   F11      fullscreen

    def on_key(self, event):
        key = event.keysym.lower()

        if key == "h":
            self.show_info = not self.show_info

        elif key == "g":
            self.show_grid = not self.show_grid

        elif key == "c":
            self.start_capture()

        elif key == "m":
            self.mirror = not self.mirror
            self.save_settings()

        elif key == "up":
            self.sensor_y = max(0.0, self.sensor_y - 0.5)
            self.save_settings()

        elif key == "down":
            self.sensor_y = min(HEIGHT_IN, self.sensor_y + 0.5)
            self.save_settings()

        elif key == "left":
            self.forward = (self.forward - 1) % 360
            self.save_settings()

        elif key == "right":
            self.forward = (self.forward + 1) % 360
            self.save_settings()

        elif key == "d":
            self.demo = not self.demo

        elif key == "r":
            self.connect()

        elif key == "f11":
            self.fullscreen = not self.fullscreen
            self.root.attributes("-fullscreen", self.fullscreen)

        elif key == "escape":
            if self.fullscreen:
                self.fullscreen = False
                self.root.attributes("-fullscreen", False)

    # ======================================
    # SEND COORDINATES
    # ======================================

    def in_active(self, x, y):
        return X0 <= x <= X1 and 0 <= y <= HEIGHT

    def emit(self, kind, tid, x, y):
        # Nothing is sent from the 4 in dead zones
        if kind != "touch_up" and not self.in_active(x, y):
            return

        payload = {
            "type": kind,
            "id": tid,

            "x_mm": round(x, 1),
            "y_mm": round(y, 1),

            # Same inch coordinates the game draws at 10 px per inch
            "x_inches": round(x / MM, 2),
            "y_inches": round(y / MM, 2),

            "x_norm": round(
                min(1.0, max(0.0, (x - X0) / ACTIVE_W)), 6
            ),

            "y_norm": round(
                min(1.0, max(0.0, y / HEIGHT)), 6
            ),

            "timestamp": time.time()
        }

        try:
            self.sender.sendto(
                json.dumps(payload).encode(),
                UDP
            )

        except OSError as exc:
            self.status = str(exc)

        if kind != "touch_move":
            print(json.dumps(payload), flush=True)

    def emit_move(self, tid, tr):
        self.emit("touch_move", tid, tr[0], tr[1])
        tr[3] = tr[0]
        tr[4] = tr[1]
        tr[5] = time.monotonic()

    def new_track(self, x, y):
        now = time.monotonic()
        return [x, y, now, x, y, now]

    def clear_tracks(self):
        for tid, tr in list(self.tracks.items()):
            self.emit("touch_up", tid, tr[0], tr[1])

        self.tracks.clear()

    # ======================================
    # CONNECT SENSOR
    # ======================================

    def connect(self):
        if self.reader:
            self.reader.close()

            if (
                self.reader.thread
                and self.reader.thread.is_alive()
            ):
                self.status = "Previous serial reader closing"
                return

        self.clear_tracks()

        self.baseline = None
        self.cap_end = None
        self.cap_samples = {}
        self.auto_done = False
        self.first_data = None
        self.background_enabled = False

        self.reader = Reader()
        self.reader.start(self.port)

        self.status = f"Opening {self.port} @ {BAUD}"

    # ======================================
    # POLAR TO CARTESIAN
    # ======================================

    def to_xy(self, angle, distance):
        theta = math.radians(angle - self.forward)

        sign = -1 if self.mirror else 1

        x = distance * math.cos(theta)

        y = (
            self.sensor_y * MM
            + sign * distance * math.sin(theta)
        )

        return x, y

    # ======================================
    # FILTER DETECTION AREA
    # ======================================

    def is_background(self, j, d):
        for k in range(-BG_NEIGHBORS, BG_NEIGHBORS + 1):
            old = self.baseline.get((j + k) % BIN_COUNT)

            if old is not None and abs(old - d) < DIFF:
                return True

        return False

    def filtered(self, scan):
        out = []

        for j, a, d, intensity in scan:
            if not (
                MIN_RANGE <= d <= MAX_RANGE
                and intensity >= MIN_INTENSITY
            ):
                continue

            x, y = self.to_xy(a, d)

            if not self.in_active(x, y):
                continue

            if self.background_enabled:
                if self.baseline is None:
                    continue

                if self.is_background(j, d):
                    continue

            out.append((j, x, y))

        return sorted(out)

    # ======================================
    # OBJECT CLUSTERING
    # ======================================

    def clusters(self, pts):
        groups = []
        current = []

        for point in pts:
            if current:
                index_gap = point[0] - current[-1][0]

                spatial_gap = math.hypot(
                    point[1] - current[-1][1],
                    point[2] - current[-1][2]
                )

                if index_gap > 3 or spatial_gap > GROUP_GAP:
                    groups.append(current)
                    current = []

            current.append(point)

        if current:
            groups.append(current)

        detections = []

        for group in groups:
            if 2 <= len(group) <= 28:
                x = sum(p[1] for p in group) / len(group)
                y = sum(p[2] for p in group) / len(group)

                detections.append((x, y))

        return detections

    # ======================================
    # TRACKING AND TOUCH EVENTS
    # ======================================

    def track(self, detected):
        now = time.monotonic()
        matched = set()

        for x, y in detected:
            match = None
            closest = TRACK_GATE

            for tid, track in self.tracks.items():
                if tid in matched:
                    continue

                gap = math.hypot(track[0] - x, track[1] - y)

                if gap < closest:
                    match = tid
                    closest = gap

            if match is None:
                match = self.next_id
                self.next_id += 1

                self.tracks[match] = self.new_track(x, y)

                self.emit("touch_down", match, x, y)

            else:
                tr = self.tracks[match]

                tr[0] = 0.85 * x + 0.15 * tr[0]
                tr[1] = 0.85 * y + 0.15 * tr[1]
                tr[2] = now

                movement = math.hypot(
                    tr[0] - tr[3],
                    tr[1] - tr[4]
                )

                if movement >= 6:
                    self.emit_move(match, tr)

            matched.add(match)

        for tid, tr in list(self.tracks.items()):
            if tid not in matched:
                if now - tr[2] > TRACK_TTL:
                    self.emit("touch_up", tid, tr[0], tr[1])
                    del self.tracks[tid]

    def heartbeat(self):
        now = time.monotonic()

        for tid, tr in self.tracks.items():
            if now - tr[5] >= HEARTBEAT:
                self.emit_move(tid, tr)

    # ======================================
    # BACKGROUND CAPTURE
    # ======================================

    def start_capture(self):
        if self.reader is None:
            return

        _, stats = self.reader.snapshot()

        if stats[3] > 0.5:
            self.status = "Sensor not streaming"
            return

        self.clear_tracks()

        self.background_enabled = False
        self.baseline = None
        self.cap_samples = {}
        self.cap_end = time.monotonic() + CAPTURE_SECONDS

        self.status = "Capturing EMPTY wall - stay clear"

    def collect_capture(self, scan):
        for j, a, d, intensity in scan:
            if (
                MIN_RANGE <= d <= MAX_RANGE
                and intensity >= MIN_INTENSITY
            ):
                self.cap_samples.setdefault(j, []).append(d)

        if time.monotonic() >= self.cap_end:
            self.baseline = {
                j: sorted(v)[len(v) // 2]
                for j, v in self.cap_samples.items()
            }

            self.cap_end = None
            self.cap_samples = {}
            self.background_enabled = True

            self.status = (
                f"Baseline captured ({len(self.baseline)} bins)"
            )

    # ======================================
    # MOUSE DEMO (map fills the window)
    # ======================================

    def size(self):
        return (
            max(2, self.canvas.winfo_width()),
            max(2, self.canvas.winfo_height())
        )

    def mouse_pos(self, event):
        w, h = self.size()

        x = event.x / w * FULL_W
        y = event.y / h * HEIGHT

        return (
            max(0, min(FULL_W, x)),
            max(0, min(HEIGHT, y))
        )

    def on_down(self, event):
        if not self.demo:
            return

        x, y = self.mouse_pos(event)

        if not self.in_active(x, y):
            return

        self.demo_down = True
        self.tracks[0] = self.new_track(x, y)
        self.emit("touch_down", 0, x, y)

    def on_drag(self, event):
        if self.demo and self.demo_down:
            x, y = self.mouse_pos(event)

            if self.in_active(x, y) and 0 in self.tracks:
                tr = self.tracks[0]
                tr[0] = x
                tr[1] = y
                tr[2] = time.monotonic()

                self.emit_move(0, tr)

    def on_up(self, event):
        if self.demo_down:
            tr = self.tracks.pop(0, None)

            if tr:
                self.emit("touch_up", 0, tr[0], tr[1])

            self.demo_down = False

    # ======================================
    # DRAW MAP - EDGE TO EDGE
    # ======================================

    def draw(self, pts):
        c = self.canvas
        c.delete("all")

        w, h = self.size()

        # mm -> pixels (map fills the whole window)
        sx = w / FULL_W
        sy = h / HEIGHT

        # Dead zones: 4 in left and 4 in right
        c.create_rectangle(
            0, 0, X0 * sx, h,
            fill="#503143", outline=""
        )

        c.create_rectangle(
            X1 * sx, 0, w, h,
            fill="#503143", outline=""
        )

        # Optional 10 in grid (same as the game)
        if self.show_grid:
            for inch in range(0, int(FULL_W_IN) + 1, 10):
                x = inch * MM * sx

                c.create_line(x, 0, x, h, fill="#28506A")

                c.create_text(
                    x + 3, h - 12,
                    text=str(inch),
                    anchor="w",
                    fill="#78AABE",
                    font=("Segoe UI", 9)
                )

            for inch in range(0, int(HEIGHT_IN) + 1, 10):
                y = inch * MM * sy

                c.create_line(0, y, w, y, fill="#28506A")

                c.create_text(
                    X0 * sx + 4, y + 8,
                    text=str(inch),
                    anchor="w",
                    fill="#78AABE",
                    font=("Segoe UI", 9)
                )

        # Active area boundaries
        for bx in (X0, X1):
            c.create_line(
                bx * sx, 0, bx * sx, h,
                fill="#FF8D9E", width=2
            )

        # LiDAR sensor marker on the left edge
        sy_px = self.sensor_y * MM * sy

        c.create_polygon(
            0, sy_px - 10,
            18, sy_px,
            0, sy_px + 10,
            fill="#6AE6D7", outline=""
        )

        # Scan point cloud
        for _, x, y in pts:
            px = x * sx
            py = y * sy

            c.create_oval(
                px - 2, py - 2, px + 2, py + 2,
                fill="#61AFFF", outline=""
            )

        # Tracked hands (same look as the game cursor)
        for tid, tr in self.tracks.items():
            px = tr[0] * sx
            py = tr[1] * sy

            c.create_oval(
                px - 26, py - 26, px + 26, py + 26,
                outline="#4DF4E3", width=3
            )

            c.create_oval(
                px - 7, py - 7, px + 7, py + 7,
                fill="#FFDD86", outline="white", width=2
            )

            c.create_text(
                px + 44, py - 30,
                text=f"{tr[0] / MM:.1f}, {tr[1] / MM:.1f} in",
                anchor="w",
                fill="#FFDD86",
                font=("Segoe UI", 11)
            )

        # Info overlay only when H is pressed
        if self.show_info:
            lines = [
                self.status,
                self.stats_text,
                f"Sensor {self.sensor_y:.1f} in from top  "
                f"Forward {self.forward:.0f} deg  "
                f"Mirror {'ON' if self.mirror else 'OFF'}",
                "H info | G grid | C capture | M mirror | "
                "Up/Down sensor | Left/Right angle | D mouse | "
                "R reconnect | F11 fullscreen"
            ]

            c.create_rectangle(
                50, 12, 1070, 12 + 22 * len(lines) + 8,
                fill="#081625", outline="#28506A"
            )

            for i, line in enumerate(lines):
                c.create_text(
                    60, 24 + i * 22,
                    text=line,
                    anchor="w",
                    fill="#E7F2FA",
                    font=("Consolas", 10)
                )

    # ======================================
    # FAST UPDATE LOOP
    # ======================================

    def tick(self):
        if self.demo != self.was_demo:
            self.clear_tracks()
            self.demo_down = False
            self.was_demo = self.demo

        if self.demo:
            self.heartbeat()
            self.draw([])

        else:
            scan, stats = self.reader.snapshot()

            ok, bad, rpm, age, error = stats
            now = time.monotonic()

            if self.cap_end is not None:
                self.collect_capture(scan)

            elif (
                self.auto_capture
                and not self.auto_done
                and ok >= 50
                and age < 0.5
            ):
                if self.first_data is None:
                    self.first_data = now
                    self.status = "Sensor live - keep wall clear"

                elif now - self.first_data >= AUTO_CAPTURE_DELAY:
                    self.auto_done = True
                    self.start_capture()

            pts = self.filtered(scan)

            if (
                self.cap_end is None
                and self.background_enabled
                and self.baseline is not None
                and age < 0.5
            ):
                self.track(self.clusters(pts))
                self.heartbeat()
            else:
                self.clear_tracks()

            self.draw(pts)

            self.stats_text = (
                f"Packets {ok}  CRC fails {bad}  RPM {rpm:.1f}  "
                f"Points {len(pts)}  Age {age * 1000:.0f} ms"
            )

            if error:
                self.status = "Error: " + error

            elif self.cap_end is not None:
                pass

            elif age < 0.5:
                if self.baseline is not None:
                    self.status = "LIVE " + self.port

            else:
                self.status = "Waiting for sensor"

        self.root.after(20, self.tick)

    # ======================================
    # CLEAN SHUTDOWN
    # ======================================

    def close(self):
        self.clear_tracks()

        if self.reader:
            self.reader.close()

        self.sender.close()
        self.root.destroy()


# ==========================================
# START
# ==========================================

if __name__ == "__main__":
    root = tk.Tk()
    Studio(root)
    root.mainloop()
