import pygame
import math
import sys

pygame.init()

# --- Window setup ---
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("2D Robotic Arm with Multi-Point Trajectories")

clock = pygame.time.Clock()

# --- Arm parameters ---
base_pos = (WIDTH // 2, HEIGHT // 2 + 100)
L1 = 150
L2 = 100
theta1 = 45
theta2 = 45

# --- Joystick parameters ---
joy_center = (150, HEIGHT - 150)
joy_radius = 60
knob_radius = 20
knob_pos = list(joy_center)
dragging = False

# --- Trajectory and workspace ---
trajectory = []
workspace_surface = None

# --- Target selection ---
mode = None  # "2point", "3point", "multi"
target_points = []
current_path = []
path_index = 0
moving_along_path = False
multi_selecting = False

# --- Colors ---
SOFT_RED = (255, 120, 120, 60)
BG_COLOR = (30, 30, 40)
JOYSTICK_COLOR = (220, 220, 220)
LINE_COLOR = (0, 160, 255)
BUTTON_COLOR = (100, 100, 200)
BUTTON_ACTIVE = (150, 80, 220)

# --- Buttons ---
button_font = pygame.font.SysFont(None, 24)
button_line = pygame.Rect(WIDTH - 180, 40, 150, 40)
button_arc = pygame.Rect(WIDTH - 180, 90, 150, 40)
button_multi = pygame.Rect(WIDTH - 180, 140, 150, 40)
button_clear = pygame.Rect(WIDTH - 180, 190, 150, 40)

# --- Helper functions ---
def compute_workspace(base, L1, L2, a1_steps=60, a2_steps=60):
    points = []
    for i in range(a1_steps + 1):
        a1 = -90 + i * (240 / a1_steps)
        for j in range(a2_steps + 1):
            a2 = -135 + j * (270 / a2_steps)
            x = base[0] + L1 * math.cos(math.radians(a1)) + L2 * math.cos(math.radians(a1 + a2))
            y = base[1] - (L1 * math.sin(math.radians(a1)) + L2 * math.sin(math.radians(a1 + a2)))
            points.append((x, y))
    return points

def draw_workspace(surface):
    global workspace_surface
    if workspace_surface is None:
        points = compute_workspace(base_pos, L1, L2)
        workspace_surface = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
        pygame.draw.polygon(workspace_surface, SOFT_RED, points)
    surface.blit(workspace_surface, (0, 0))

def draw_arm(surface, base, a1, a2):
    x1 = base[0] + L1 * math.cos(math.radians(a1))
    y1 = base[1] - L1 * math.sin(math.radians(a1))
    x2 = x1 + L2 * math.cos(math.radians(a1 + a2))
    y2 = y1 - L2 * math.sin(math.radians(a1 + a2))

    pygame.draw.line(surface, (0, 255, 0), base, (x1, y1), 8)
    pygame.draw.line(surface, (0, 200, 255), (x1, y1), (x2, y2), 6)
    pygame.draw.circle(surface, (255, 0, 0), base, 10)
    pygame.draw.circle(surface, (255, 0, 0), (int(x1), int(y1)), 8)
    pygame.draw.circle(surface, (255, 255, 0), (int(x2), int(y2)), 8)
    return (x2, y2)

def draw_joystick(surface, center, knob, radius):
    pygame.draw.circle(surface, (100, 100, 100), center, radius, 3)
    pygame.draw.circle(surface, JOYSTICK_COLOR, knob, knob_radius)

def clamp_knob(center, pos, radius):
    dx = pos[0] - center[0]
    dy = pos[1] - center[1]
    dist = math.hypot(dx, dy)
    if dist > radius:
        dx = dx / dist * radius
        dy = dy / dist * radius
    return [center[0] + dx, center[1] + dy]

def inverse_kinematics(x, y):
    dx = x - base_pos[0]
    dy = base_pos[1] - y
    dist = math.hypot(dx, dy)
    dist = max(min(dist, L1 + L2), abs(L1 - L2))
    cos_a2 = (dx**2 + dy**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_a2 = max(-1, min(1, cos_a2))
    a2 = math.degrees(math.acos(cos_a2))
    k1 = L1 + L2 * cos_a2
    k2 = L2 * math.sin(math.radians(a2))
    a1 = math.degrees(math.atan2(dy, dx) - math.atan2(k2, k1))
    return a1, a2

def draw_button(surface, rect, text, active=False):
    color = BUTTON_ACTIVE if active else BUTTON_COLOR
    pygame.draw.rect(surface, color, rect, border_radius=8)
    txt = button_font.render(text, True, (255, 255, 255))
    surface.blit(txt, (rect.x + 15, rect.y + 10))

def quadratic_bezier_through_midpoint(p0, pmid, p2, steps=120):
    cx = 2 * pmid[0] - 0.5 * (p0[0] + p2[0])
    cy = 2 * pmid[1] - 0.5 * (p0[1] + p2[1])
    c = (cx, cy)
    path = []
    for i in range(steps + 1):
        t = i / steps
        x = (1 - t)**2 * p0[0] + 2 * (1 - t) * t * c[0] + t**2 * p2[0]
        y = (1 - t)**2 * p0[1] + 2 * (1 - t) * t * c[1] + t**2 * p2[1]
        path.append((x, y))
    return path

def catmull_rom_spline(points, steps=100):
    if len(points) < 2:
        return points
    path = []
    for i in range(-1, len(points) - 2):
        p0 = points[i]
        p1 = points[i + 1]
        p2 = points[i + 2]
        p3 = points[i + 3] if i + 3 < len(points) else p2
        for t in [x / steps for x in range(steps)]:
            t2 = t * t
            t3 = t2 * t
            x = 0.5 * ((2 * p1[0]) +
                       (-p0[0] + p2[0]) * t +
                       (2*p0[0] - 5*p1[0] + 4*p2[0] - p3[0]) * t2 +
                       (-p0[0] + 3*p1[0] - 3*p2[0] + p3[0]) * t3)
            y = 0.5 * ((2 * p1[1]) +
                       (-p0[1] + p2[1]) * t +
                       (2*p0[1] - 5*p1[1] + 4*p2[1] - p3[1]) * t2 +
                       (-p0[1] + 3*p1[1] - 3*p2[1] + p3[1]) * t3)
            path.append((x, y))
    path.append(points[-1])
    return path

# --- Precompute workspace ---
workspace_surface = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
pygame.draw.polygon(workspace_surface, SOFT_RED, compute_workspace(base_pos, L1, L2))

# --- Main loop ---
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

        elif event.type == pygame.MOUSEBUTTONDOWN:
            if button_line.collidepoint(event.pos):
                mode = "2point"
                target_points.clear()
                current_path.clear()
                moving_along_path = False
                multi_selecting = False

            elif button_arc.collidepoint(event.pos):
                mode = "3point"
                target_points.clear()
                current_path.clear()
                moving_along_path = False
                multi_selecting = False

            elif button_multi.collidepoint(event.pos):
                if not multi_selecting:
                    mode = "multi"
                    target_points.clear()
                    current_path.clear()
                    moving_along_path = False
                    multi_selecting = True
                else:
                    if len(target_points) >= 2:
                        current_path = catmull_rom_spline(target_points, steps=40)
                        path_index = 0
                        moving_along_path = True
                    mode = None
                    multi_selecting = False  # stop selecting but KEEP points visible

            elif button_clear.collidepoint(event.pos):
                trajectory.clear()
                target_points.clear()
                current_path.clear()
                moving_along_path = False
                mode = None
                multi_selecting = False

            elif mode == "2point":
                target_points.append(event.pos)
                if len(target_points) == 2:
                    current_path = [
                        (
                            target_points[0][0] + (target_points[1][0] - target_points[0][0]) * t / 100,
                            target_points[0][1] + (target_points[1][1] - target_points[0][1]) * t / 100,
                        )
                        for t in range(101)
                    ]
                    path_index = 0
                    moving_along_path = True
                    mode = None  # Keep control points visible

            elif mode == "3point":
                target_points.append(event.pos)
                if len(target_points) == 3:
                    current_path = quadratic_bezier_through_midpoint(target_points[0], target_points[1], target_points[2], 120)
                    path_index = 0
                    moving_along_path = True
                    mode = None  # Keep control points visible

            elif mode == "multi" and multi_selecting:
                target_points.append(event.pos)

            else:
                if math.hypot(event.pos[0] - knob_pos[0], event.pos[1] - knob_pos[1]) < knob_radius:
                    dragging = True

        elif event.type == pygame.MOUSEBUTTONUP:
            dragging = False
            knob_pos = list(joy_center)

        elif event.type == pygame.MOUSEMOTION and dragging:
            knob_pos = clamp_knob(joy_center, event.pos, joy_radius)

    # --- Movement logic ---
    if moving_along_path and current_path:
        px, py = current_path[path_index]
        theta1, theta2 = inverse_kinematics(px, py)
        path_index += 1
        if path_index >= len(current_path):
            moving_along_path = False
    else:
        dx = (knob_pos[0] - joy_center[0]) / joy_radius
        dy = (joy_center[1] - knob_pos[1]) / joy_radius
        theta1 += dy * 2.0
        theta2 += dx * 2.0
        theta1 = max(-90, min(150, theta1))
        theta2 = max(-135, min(135, theta2))

    # --- Drawing ---
    screen.fill(BG_COLOR)
    draw_workspace(screen)

    effector = draw_arm(screen, base_pos, theta1, theta2)

    trajectory.append(effector)
    if len(trajectory) > 2000:
        trajectory.pop(0)
    if len(trajectory) > 1:
        pygame.draw.lines(screen, LINE_COLOR, False, trajectory, 2)

    draw_joystick(screen, joy_center, knob_pos, joy_radius)
    draw_button(screen, button_line, "Set 2-Point Line", mode == "2point")
    draw_button(screen, button_arc, "Set 3-Point Arc", mode == "3point")
    draw_button(screen, button_multi, "Multi-Point Traj", multi_selecting)
    draw_button(screen, button_clear, "Clear Workspace")

    # Show only control points, NOT yellow connecting lines
    for p in target_points:
        pygame.draw.circle(screen, (255, 220, 0), p, 6)

    font = pygame.font.SysFont(None, 24)
    text = font.render(f"Joint1: {theta1:.1f}°   Joint2: {theta2:.1f}°", True, (255, 255, 255))
    screen.blit(text, (20, 20))

    pygame.display.flip()
    clock.tick(60)
