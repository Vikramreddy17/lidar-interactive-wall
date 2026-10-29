import pygame
import socket
import json
import random
import math
import time

# ==========================================
# CONFIGURATION - 1:1 REPLICA OF THE LIDAR MAP
# ==========================================
#
# The game world uses the SAME coordinates as the LiDAR studio map:
#
#   x_inches : 0 .. 149   (0 = left edge, where the LiDAR sits)
#   y_inches : 0 .. 80    (0 = top edge, LiDAR at 38 in from top)
#
#   4 in dead zone on the left  (x   0 ..   4)
#   4 in dead zone on the right (x 145 .. 149)
#   141 in active/playable      (x   4 .. 145)
#
# 1 inch = PPI pixels, so a point at (x_in, y_in) is drawn at
# (x_in * PPI, y_in * PPI). No normalizing, no calibration.

PPI = 10

FULL_W_IN = 149
HEIGHT_IN = 80
DEAD_LEFT_IN = 4
DEAD_RIGHT_IN = 4
SENSOR_Y_IN = 38

W = FULL_W_IN * PPI          # 1490
H = HEIGHT_IN * PPI          # 800

X0 = DEAD_LEFT_IN * PPI                       # active area starts
X1 = (FULL_W_IN - DEAD_RIGHT_IN) * PPI        # active area ends

FPS = 60

UDP = ("127.0.0.1", 5005)

SPAWN_SECONDS = 0.42
MAX_BALLOONS = 22

TAP_RADIUS_BONUS = 14

MIN_SLICE_PIXELS = 6
MAX_SEGMENT_PIXELS = 380

TOUCH_TIMEOUT = 0.6       # drop a hand if no update arrives
SWIPE_WINDOW = 0.35       # max gap between two moves to count as a swipe
CURSOR_TRAIL_SECONDS = 0.35

COLORS = [
    (255, 103, 147),
    (255, 185, 94),
    (84, 223, 193),
    (110, 164, 255),
    (195, 125, 251),
    (255, 225, 116)
]


# ==========================================
# COLOR HELPER
# ==========================================

def col(color, factor):
    return tuple(
        max(0, min(255, int(v * factor)))
        for v in color
    )


# ==========================================
# BALLOON CLASS
# ==========================================

class Balloon:

    def __init__(self):
        # Only spawn inside the active area
        self.x = random.uniform(X0 + 62, X1 - 62)

        # Balloon starts below the screen
        self.y = H + random.uniform(55, 220)

        self.rx = random.randint(32, 49)
        self.ry = self.rx * 1.30

        self.color = random.choice(COLORS)

        self.speed = random.uniform(95, 185)

        self.seed = random.random() * math.tau
        self.t = 0

    def step(self, dt):
        self.t += dt

        # Balloon rises upward
        self.y -= self.speed * dt

        # Gentle side-to-side floating motion
        self.x += (
            math.sin(self.t * 2 + self.seed)
            * 12 * dt
        )

        self.x = max(X0 + self.rx, min(X1 - self.rx, self.x))

    # ======================================
    # TAP DETECTION
    # ======================================

    def hit(self, x, y, extra=TAP_RADIUS_BONUS):
        return (
            ((x - self.x) / (self.rx + extra)) ** 2
            +
            ((y - self.y) / (self.ry + extra)) ** 2
        ) <= 1

    # ======================================
    # SWIPE DETECTION
    # ======================================

    def intersects(self, a, b, extra=TAP_RADIUS_BONUS // 2):
        rx = self.rx + extra
        ry = self.ry + extra

        ax = (a[0] - self.x) / rx
        ay = (a[1] - self.y) / ry

        bx = (b[0] - self.x) / rx
        by = (b[1] - self.y) / ry

        dx = bx - ax
        dy = by - ay

        denominator = dx * dx + dy * dy

        if denominator:
            t = max(
                0,
                min(
                    1,
                    -(ax * dx + ay * dy)
                    / denominator
                )
            )
        else:
            t = 0

        px = ax + dx * t
        py = ay + dy * t

        return px * px + py * py <= 1.0

    # ======================================
    # DRAW BALLOON
    # ======================================

    def draw(self, surface):
        x = int(self.x)
        y = int(self.y)

        rx = self.rx
        ry = int(self.ry)

        if y + ry < 0 or y - ry > H:
            return

        # String
        pygame.draw.line(
            surface,
            (95, 140, 164),
            (x, y + ry + 8),
            (x + 5, y + ry + 52),
            1
        )

        # Shadow
        pygame.draw.ellipse(
            surface,
            col(self.color, 0.45),
            (
                x - rx - 3,
                y - ry - 3,
                2 * rx + 6,
                2 * ry + 6
            )
        )

        # Balloon body
        pygame.draw.ellipse(
            surface,
            self.color,
            (
                x - rx,
                y - ry,
                2 * rx,
                2 * ry
            )
        )

        # Reflection highlight
        pygame.draw.ellipse(
            surface,
            col(self.color, 1.18),
            (
                x - rx // 2,
                y - ry // 2,
                rx // 3,
                ry // 2
            )
        )

        # Knot
        pygame.draw.polygon(
            surface,
            col(self.color, 0.9),
            [
                (x, y + ry - 1),
                (x - 7, y + ry + 10),
                (x + 7, y + ry + 10)
            ]
        )


# ==========================================
# PARTICLE EFFECT
# ==========================================

class Spark:

    def __init__(self, x, y, color):
        self.x = x
        self.y = y
        self.color = color

        angle = random.uniform(
            0, math.tau
        )

        speed = random.uniform(
            95, 390
        )

        self.vx = math.cos(angle) * speed
        self.vy = math.sin(angle) * speed

        self.life = random.uniform(
            0.25, 0.65
        )

        self.max_life = self.life
        self.r = random.randint(2, 6)

    def update(self, dt):
        self.x += self.vx * dt
        self.y += self.vy * dt

        self.vy += 175 * dt
        self.life -= dt

    def draw(self, screen):
        if self.life > 0:
            radius = max(
                1,
                int(
                    self.r * self.life
                    / self.max_life
                )
            )

            pygame.draw.circle(
                screen,
                self.color,
                (int(self.x), int(self.y)),
                radius
            )


# ==========================================
# BALLOON GAME
# ==========================================

class Game:

    def __init__(self):
        pygame.init()

        self.screen = pygame.display.set_mode(
            (W, H),
            pygame.SCALED
        )

        pygame.display.set_caption(
            "Balloon Atelier - LD19 Interactive Wall (1:1 map)"
        )

        self.clock = pygame.time.Clock()

        self.font_big = pygame.font.SysFont(
            "segoeui",
            48,
            bold=True
        )

        self.font_mid = pygame.font.SysFont(
            "segoeui",
            26,
            bold=True
        )

        self.font_small = pygame.font.SysFont(
            "segoeui",
            18
        )

        self.font_tiny = pygame.font.SysFont(
            "segoeui",
            14
        )

        # Receive touch coordinates from server.py
        self.sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM
        )

        self.sock.bind(UDP)
        self.sock.setblocking(False)

        self.fullscreen = False
        self.show_grid = False

        self.reset()

    # ======================================
    # RESET GAME
    # ======================================

    def reset(self):
        self.balloons = []
        self.sparks = []
        self.trails = []

        # ID -> {"pos", "t", "trail"}
        self.touches = {}

        self.score = 0
        self.combo = 0
        self.popped = 0
        self.best_combo = 0

        self.last_hit = 0
        self.spawn_timer = 0.3

        self.started = time.monotonic()

        self.running = True
        self.mouse_held = False
        self.last_touch = None

    # ======================================
    # POP BALLOON
    # ======================================

    def pop(self, balloon, mode):
        if balloon not in self.balloons:
            return

        self.balloons.remove(balloon)

        now = time.monotonic()

        if now - self.last_hit < 1.2:
            self.combo += 1
        else:
            self.combo = 1

        self.best_combo = max(
            self.best_combo,
            self.combo
        )

        self.last_hit = now

        bonus = min(
            50,
            (self.combo - 1) * 2
        )

        self.score += 10 + bonus
        self.popped += 1

        self.last_touch = (
            balloon.x,
            balloon.y,
            now,
            mode
        )

        # Explosion effect
        for _ in range(26):
            self.sparks.append(
                Spark(
                    balloon.x,
                    balloon.y,
                    balloon.color
                )
            )

    # ======================================
    # TAP TO POP
    # ======================================

    def tap(self, pos, mode="TAP"):
        for balloon in reversed(self.balloons):
            if balloon.hit(*pos):
                self.pop(balloon, mode)
                break

    # ======================================
    # SWIPE TO SLICE
    # ======================================

    def swipe(self, start, end):
        length = math.dist(
            start,
            end
        )

        # Reject tiny motions and tracking jumps
        if not (
            MIN_SLICE_PIXELS
            <= length
            <= MAX_SEGMENT_PIXELS
        ):
            return

        self.trails.append([
            start,
            end,
            0.20
        ])

        for balloon in self.balloons[:]:
            if balloon.intersects(start, end):
                self.pop(
                    balloon,
                    "SLICE"
                )

    # ======================================
    # TOUCH EVENT PROCESSING
    # ======================================

    def process_touch(self, tid, kind, pos):
        now = time.monotonic()

        if kind == "touch_up":
            self.touches.pop(
                tid,
                None
            )

        elif kind == "touch_down":
            self.touches[tid] = {
                "pos": pos,
                "t": now,
                "trail": [(pos[0], pos[1], now)]
            }

            # A touch pops a balloon immediately
            self.tap(pos)

        elif kind == "touch_move":
            touch = self.touches.get(tid)

            if touch is None:
                # Missed the touch_down: treat as a new touch
                self.process_touch(tid, "touch_down", pos)
                return

            old_pos = touch["pos"]
            old_time = touch["t"]

            touch["pos"] = pos
            touch["t"] = now
            touch["trail"].append((pos[0], pos[1], now))

            if now - old_time < SWIPE_WINDOW:
                self.swipe(old_pos, pos)

            # Hand sitting on a balloon also pops it
            self.tap(pos)

    # ======================================
    # RECEIVE UDP LIDAR EVENTS
    # ======================================

    def receive(self):
        # Limit packets processed per frame
        for _ in range(200):
            try:
                raw, _ = self.sock.recvfrom(4096)

            except BlockingIOError:
                break

            except OSError:
                break

            try:
                event = json.loads(
                    raw.decode("utf-8")
                )

                kind = event.get("type")

                if kind not in (
                    "touch_down",
                    "touch_move",
                    "touch_up"
                ):
                    continue

                tid = event.get("id")

                if tid is None:
                    continue

                if kind == "touch_up":
                    self.process_touch(
                        tid,
                        kind,
                        (0, 0)
                    )
                    continue

                # Same inch coordinates as the LiDAR map
                x_in = float(event["x_inches"])
                y_in = float(event["y_inches"])

                if not (
                    math.isfinite(x_in)
                    and math.isfinite(y_in)
                    and DEAD_LEFT_IN <= x_in <= FULL_W_IN - DEAD_RIGHT_IN
                    and 0 <= y_in <= HEIGHT_IN
                ):
                    continue

                # 1:1 -> inches to pixels
                self.process_touch(
                    tid,
                    kind,
                    (x_in * PPI, y_in * PPI)
                )

            except (
                UnicodeError,
                ValueError,
                TypeError,
                KeyError,
                AttributeError
            ):
                continue

    # ======================================
    # DRAW BACKGROUND
    # ======================================

    def draw_background(self, seconds):
        self.screen.fill(
            (8, 17, 37)
        )

        # Decorative lights
        for i in range(42):
            x = (i * 431 + 87) % W

            y = (
                i * 197 + int(seconds * 5)
            ) % H

            pygame.draw.circle(
                self.screen,
                (25, 51, 76),
                (x, y),
                i % 3 + 1
            )

        # Active play area (141 in x 80 in)
        pygame.draw.rect(
            self.screen,
            (14, 31, 57),
            (X0, 0, X1 - X0, H)
        )

        # Dead zones, 4 in each side
        for dx in (0, X1):
            pygame.draw.rect(
                self.screen,
                (80, 49, 67),
                (dx, 0, W - X1 if dx else X0, H)
            )

        # Active area boundaries
        for bx in (X0, X1):
            pygame.draw.line(
                self.screen,
                (255, 141, 158),
                (bx, 0),
                (bx, H),
                2
            )

        # Ambient floor
        pygame.draw.line(
            self.screen,
            (20, 89, 103),
            (40, H - 115),
            (W - 40, H - 115),
            2
        )

        # LiDAR sensor marker on the left edge
        sy = SENSOR_Y_IN * PPI

        pygame.draw.polygon(
            self.screen,
            (106, 230, 215),
            [(0, sy - 10), (18, sy), (0, sy + 10)]
        )

    # ======================================
    # INCH GRID (toggle with G)
    # ======================================

    def draw_grid(self):
        if not self.show_grid:
            return

        # Line every 10 inches, labelled
        for inch in range(0, FULL_W_IN + 1, 10):
            x = inch * PPI

            pygame.draw.line(
                self.screen,
                (40, 80, 105),
                (x, 0),
                (x, H),
                1
            )

            self.text(
                f"{inch}",
                self.font_tiny,
                (120, 170, 190),
                x + 3,
                H - 20
            )

        for inch in range(0, HEIGHT_IN + 1, 10):
            y = inch * PPI

            pygame.draw.line(
                self.screen,
                (40, 80, 105),
                (0, y),
                (W, y),
                1
            )

            self.text(
                f"{inch}",
                self.font_tiny,
                (120, 170, 190),
                X0 + 4,
                y + 2
            )

    # ======================================
    # TEXT HELPER
    # ======================================

    def text(self, text, font, color, x, y):
        rendered = font.render(
            text,
            True,
            color
        )

        self.screen.blit(
            rendered,
            (x, y)
        )

    # ======================================
    # HAND CURSOR INDICATOR
    # ======================================

    def draw_cursors(self, seconds):
        now = time.monotonic()

        for touch in self.touches.values():
            x, y = touch["pos"]
            x = int(x)
            y = int(y)

            # Fading motion trail behind the hand
            touch["trail"] = [
                p for p in touch["trail"]
                if now - p[2] < CURSOR_TRAIL_SECONDS
            ]

            trail = touch["trail"]

            for i in range(1, len(trail)):
                age = (now - trail[i][2]) / CURSOR_TRAIL_SECONDS
                fade = max(0.0, 1.0 - age)

                pygame.draw.line(
                    self.screen,
                    col((77, 244, 227), fade),
                    trail[i - 1][:2],
                    trail[i][:2],
                    max(2, int(8 * fade))
                )

            pulse = 3 * math.sin(seconds * 9)

            # Outer glow
            pygame.draw.circle(
                self.screen,
                (24, 82, 102),
                (x, y),
                int(38 + pulse),
                6
            )

            # Main ring
            pygame.draw.circle(
                self.screen,
                (77, 244, 227),
                (x, y),
                int(26 + pulse),
                3
            )

            # Crosshair
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                pygame.draw.line(
                    self.screen,
                    (239, 255, 255),
                    (x + dx * 16, y + dy * 16),
                    (x + dx * 36, y + dy * 36),
                    2
                )

            # Center dot
            pygame.draw.circle(
                self.screen,
                (255, 221, 134),
                (x, y),
                7
            )

            pygame.draw.circle(
                self.screen,
                (255, 255, 255),
                (x, y),
                7,
                2
            )

            # Inch readout - compare with the LiDAR studio map
            self.text(
                f"{x / PPI:.1f}, {y / PPI:.1f} in",
                self.font_small,
                (255, 221, 134),
                x + 44,
                y - 30
            )

    # ======================================
    # DRAW GAME HUD
    # ======================================

    def draw_hud(self):
        self.text(
            "BALLOON",
            self.font_big,
            (241, 247, 255),
            60,
            40
        )

        self.text(
            "ATELIER",
            self.font_big,
            (90, 230, 208),
            283,
            40
        )

        self.text(
            "TOUCH +10 | SWIPE TO SLICE",
            self.font_small,
            (156, 194, 207),
            64,
            101
        )

        # Score panel
        pygame.draw.rect(
            self.screen,
            (8, 26, 45),
            (W - 420, 46, 350, 108),
            border_radius=18
        )

        self.text(
            "SCORE",
            self.font_small,
            (163, 197, 214),
            W - 395,
            58
        )

        self.text(
            str(self.score),
            self.font_big,
            (255, 218, 138),
            W - 395,
            79
        )

        self.text(
            "COMBO",
            self.font_small,
            (163, 197, 214),
            W - 192,
            58
        )

        self.text(
            "x" + str(self.combo),
            self.font_mid,
            (108, 230, 209),
            W - 192,
            98
        )

        self.text(
            f"POPPED {self.popped}",
            self.font_mid,
            (241, 247, 255),
            65,
            H - 101
        )

        self.text(
            "G GRID | F11 FULLSCREEN | R RESET | ESC EXIT",
            self.font_small,
            (155, 190, 204),
            W - 560,
            H - 65
        )

        # Temporary tap/slice indication
        if self.last_touch:
            x, y, timestamp, mode = self.last_touch

            if time.monotonic() - timestamp < 1:
                self.text(
                    mode + "!",
                    self.font_mid,
                    (255, 221, 134),
                    int(x),
                    max(160, int(y) - 50)
                )

    # ======================================
    # RENDER GAME
    # ======================================

    def render(self, dt, seconds):
        self.draw_background(seconds)
        self.draw_grid()

        for balloon in self.balloons:
            balloon.draw(
                self.screen
            )

        for particle in self.sparks:
            particle.draw(
                self.screen
            )

        for start, end, life in self.trails:
            pygame.draw.line(
                self.screen,
                (28, 91, 111),
                start,
                end,
                13
            )

            pygame.draw.line(
                self.screen,
                (77, 244, 227),
                start,
                end,
                5
            )

            pygame.draw.line(
                self.screen,
                (239, 255, 255),
                start,
                end,
                2
            )

        self.draw_hud()

        # Hand indicator is drawn on top of everything
        self.draw_cursors(seconds)

        pygame.display.flip()

    # ======================================
    # FULLSCREEN MODE
    # ======================================

    def toggle_fullscreen(self):
        self.fullscreen = not self.fullscreen

        flags = pygame.SCALED

        if self.fullscreen:
            flags |= pygame.FULLSCREEN

        self.screen = pygame.display.set_mode(
            (W, H),
            flags
        )

    # ======================================
    # MAIN GAME LOOP
    # ======================================

    def run(self):
        while self.running:
            dt = min(
                0.05,
                self.clock.tick(FPS) / 1000
            )

            seconds = (
                time.monotonic()
                - self.started
            )

            # Keyboard and mouse events
            for event in pygame.event.get():

                if event.type == pygame.QUIT:
                    self.running = False

                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        self.running = False

                    elif event.key == pygame.K_F11:
                        self.toggle_fullscreen()

                    elif event.key == pygame.K_r:
                        self.reset()

                    elif event.key == pygame.K_g:
                        self.show_grid = not self.show_grid

                # Mouse tap
                elif (
                    event.type == pygame.MOUSEBUTTONDOWN
                    and event.button == 1
                ):
                    self.mouse_held = True

                    self.process_touch(
                        "mouse",
                        "touch_down",
                        event.pos
                    )

                # Mouse swipe
                elif (
                    event.type == pygame.MOUSEMOTION
                    and self.mouse_held
                ):
                    self.process_touch(
                        "mouse",
                        "touch_move",
                        event.pos
                    )

                elif (
                    event.type == pygame.MOUSEBUTTONUP
                    and event.button == 1
                ):
                    self.mouse_held = False

                    self.process_touch(
                        "mouse",
                        "touch_up",
                        event.pos
                    )

            # Receive real LiDAR coordinates
            self.receive()

            # Continuous balloon spawning
            self.spawn_timer += dt

            if (
                self.spawn_timer >= SPAWN_SECONDS
                and len(self.balloons) < MAX_BALLOONS
            ):
                self.spawn_timer -= SPAWN_SECONDS

                count = random.choice(
                    [1, 1, 2]
                )

                for _ in range(count):
                    if (
                        len(self.balloons)
                        < MAX_BALLOONS
                    ):
                        self.balloons.append(
                            Balloon()
                        )

            # Update balloon positions
            for balloon in self.balloons[:]:
                balloon.step(dt)

                if balloon.y < -120:
                    self.balloons.remove(
                        balloon
                    )

            # A balloon floating into a held hand pops too
            for touch in list(self.touches.values()):
                self.tap(touch["pos"])

            # Update particle physics
            for particle in self.sparks[:]:
                particle.update(dt)

                if particle.life <= 0:
                    self.sparks.remove(
                        particle
                    )

            # Fade swipe trails
            self.trails = [
                [a, b, life - dt]
                for a, b, life in self.trails
                if life > dt
            ]

            # Remove abandoned touch IDs
            now = time.monotonic()

            for tid, touch in list(
                self.touches.items()
            ):
                if (
                    tid != "mouse"
                    and now - touch["t"] > TOUCH_TIMEOUT
                ):
                    self.touches.pop(
                        tid, None
                    )

            self.render(
                dt,
                seconds
            )

        self.sock.close()
        pygame.quit()


# ==========================================
# START GAME
# ==========================================

if __name__ == "__main__":
    Game().run()
