"""
Controlador para robot Khepera3 en laberinto
Universidad - Practica 2 IA
Algoritmo: Navegacion GPS + Odometria + Evasion reactiva de obstaculos
"""

from controller import Robot, DistanceSensor, Motor, PositionSensor, Compass, GPS
import math

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Robot constants (from maze_world.wbt)
MAX_SPEED   = 10.0    # rad/s  (motor max is 25, using 10 for stability)
BASE_SPEED  = 8.0     # Normal cruise speed
WHEEL_RADIUS = 0.0212 # meters (from world file Cylinder radius)
WHEEL_BASE   = 0.090  # meters (2 * 0.045, wheel y-offset)

# Goal position (DEF Meta Solid translation from maze_world.wbt)
# NOTE: assignment PDF says {2.0, 2.6, -0.04} but the world's Meta object
# is placed at (-2.06352, 2.25539). Use the world file value.
GOAL_X = -2.06352
GOAL_Y =  2.25539
GOAL_REACHED_DIST = 0.30  # meters — stop within 30 cm of goal

# Sensor thresholds
# IR sensors: ~1000 = max range (no obstacle), lower values = obstacle closer
OBSTACLE_THRESH   = 700   # below = obstacle detected
VERY_CLOSE_THRESH = 350   # below = very close, emergency avoidance

# ── Motor setup ──────────────────────────────────────────────────────────────
left_motor  = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

# CRITICAL FIX: switch from position control to velocity control mode
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# ── Wheel encoders ────────────────────────────────────────────────────────────
left_enc  = robot.getDevice("left wheel sensor")
right_enc = robot.getDevice("right wheel sensor")
left_enc.enable(timestep)
right_enc.enable(timestep)

# ── Distance sensors ─────────────────────────────────────────────────────────
# Sensor layout (robot-local frame, +X = forward, +Y = left, +Z = up):
#  ds0: (-0.0404, +0.0496) → rear-left
#  ds1: (+0.0148, +0.0630) → front-left
#  ds2: (+0.0488, +0.0454) → front-left  45°
#  ds3: (+0.0656, +0.0155) → front (slight left)
#  ds4: (+0.0656, -0.0155) → front (slight right)
#  ds5: (+0.0488, -0.0454) → front-right 45°
#  ds6: (+0.0148, -0.0630) → front-right
#  ds7: (-0.0404, -0.0496) → rear-right
#  ds8: (-0.0550,  0.0000) → rear
ds = []
for name in ["ds0", "ds1", "ds2", "ds3", "ds4", "ds5", "ds6", "ds7", "ds8"]:
    s = robot.getDevice(name)
    s.enable(timestep)
    ds.append(s)

# ── GPS (compass is unusable: WorldInfo northDirection=[0,0,-1] = straight down) ──
gps     = robot.getDevice("gps")
compass = robot.getDevice("compass")   # enabled for completeness, not used for heading
gps.enable(timestep)
compass.enable(timestep)

# ── Odometry state ────────────────────────────────────────────────────────────
# Initial heading derived from world file robot rotation:
#   rotation axis ≈ (0, 0, 1), angle ≈ -π  →  robot faces world -X  →  heading = π
robot_heading  = math.pi
prev_left_enc  = 0.0
prev_right_enc = 0.0

# Stuck-recovery state
stuck_counter    = 0
prev_pos         = None
recovery_counter = 0

# ── Helper functions ──────────────────────────────────────────────────────────

def normalize_angle(a):
    while a >  math.pi: a -= 2.0 * math.pi
    while a < -math.pi: a += 2.0 * math.pi
    return a

def clamp(v):
    return max(-MAX_SPEED, min(MAX_SPEED, v))

def update_odometry():
    global prev_left_enc, prev_right_enc, robot_heading
    cl = left_enc.getValue()
    cr = right_enc.getValue()
    dl = (cl - prev_left_enc) * WHEEL_RADIUS
    dr = (cr - prev_right_enc) * WHEEL_RADIUS
    # Standard differential drive: positive dtheta = CCW = turn left
    dtheta = (dr - dl) / WHEEL_BASE
    robot_heading = normalize_angle(robot_heading + dtheta)
    prev_left_enc = cl
    prev_right_enc = cr

def check_stuck(pos):
    """Return True if the robot has barely moved for many consecutive steps."""
    global stuck_counter, prev_pos
    if prev_pos is not None:
        moved = math.sqrt((pos[0] - prev_pos[0])**2 + (pos[1] - prev_pos[1])**2)
        if moved < 0.008:
            stuck_counter += 1
        else:
            stuck_counter = max(0, stuck_counter - 2)
    prev_pos = [pos[0], pos[1]]
    return stuck_counter > 60   # ~1.8 s at 32 ms timestep

# ── Main control function (ROBOT_MOVE equivalent) ─────────────────────────────

def ROBOT_MOVE():
    global recovery_counter

    # Update odometry heading
    update_odometry()

    # GPS position
    pos   = gps.getValues()
    px, py = pos[0], pos[1]

    # Distance to goal
    dx   = GOAL_X - px
    dy   = GOAL_Y - py
    dist = math.sqrt(dx * dx + dy * dy)

    if dist < GOAL_REACHED_DIST:
        return 0.0, 0.0   # Goal reached — stop

    # Sensor readings
    sensors = [d.getValue() for d in ds]

    # Grouped sensor aggregates
    front_c  = min(sensors[3], sensors[4])           # ds3, ds4 — directly ahead
    front_l  = sensors[2]                            # ds2 — front-left 45°
    front_r  = sensors[5]                            # ds5 — front-right 45°
    side_l   = sensors[1]                            # ds1 — front-left 90°
    side_r   = sensors[6]                            # ds6 — front-right 90°
    front_min = min(front_c, front_l, front_r)

    # ── Stuck recovery ────────────────────────────────────────────────────────
    if recovery_counter > 0:
        recovery_counter -= 1
        # Spin left to escape
        return -BASE_SPEED * 0.6, BASE_SPEED * 0.6

    if check_stuck(pos):
        recovery_counter = 40   # ~1.3 s of spinning
        stuck_counter = 0
        return -BASE_SPEED * 0.6, BASE_SPEED * 0.6

    # ── Emergency avoidance (very close obstacle) ─────────────────────────────
    if front_min < VERY_CLOSE_THRESH:
        if front_l <= front_r:
            return BASE_SPEED * 0.4, -BASE_SPEED * 0.6   # turn right
        else:
            return -BASE_SPEED * 0.6, BASE_SPEED * 0.4   # turn left

    # ── Normal obstacle avoidance ─────────────────────────────────────────────
    if front_min < OBSTACLE_THRESH:
        if front_l <= front_r:
            # Obstacle more on left → steer right
            ratio = front_l / OBSTACLE_THRESH            # 0 = very close, 1 = at threshold
            return BASE_SPEED, BASE_SPEED * ratio
        else:
            # Obstacle more on right → steer left
            ratio = front_r / OBSTACLE_THRESH
            return BASE_SPEED * ratio, BASE_SPEED

    # ── Goal-directed navigation ──────────────────────────────────────────────
    target_heading = math.atan2(dy, dx)
    angle_error    = normalize_angle(target_heading - robot_heading)

    # Proportional steering
    TURN_GAIN = 5.0
    turn = clamp(TURN_GAIN * angle_error)

    # Slow down while turning sharply so the odometry tracks better
    speed_factor = 1.0 - 0.4 * (abs(angle_error) / math.pi)
    base = BASE_SPEED * speed_factor

    left  = clamp(base - turn)
    right = clamp(base + turn)
    return left, right


# ── Main loop ─────────────────────────────────────────────────────────────────
print("Iniciando controlador Grupo_N...")
print(f"Meta: ({GOAL_X:.3f}, {GOAL_Y:.3f})")
print("Algoritmo: GPS + Odometria + Evasion reactiva")

iteration = 0
while robot.step(timestep) != -1:
    iteration += 1

    left_v, right_v = ROBOT_MOVE()
    left_motor.setVelocity(left_v)
    right_motor.setVelocity(right_v)

    if iteration % 30 == 0:
        pos     = gps.getValues()
        sensors = [int(d.getValue()) for d in ds]
        dist    = math.sqrt((GOAL_X - pos[0])**2 + (GOAL_Y - pos[1])**2)
        print(f"Pos: ({pos[0]:.2f}, {pos[1]:.2f}) | Dist meta: {dist:.2f}m "
              f"| HDG: {math.degrees(robot_heading):.0f}° "
              f"| Sens: {sensors[:7]} | V: L={left_v:.1f} R={right_v:.1f}")

    if left_v == 0.0 and right_v == 0.0:
        pos  = gps.getValues()
        dist = math.sqrt((GOAL_X - pos[0])**2 + (GOAL_Y - pos[1])**2)
        print(f"Meta alcanzada! Distancia final: {dist:.3f}m")
        break

print("Controlador finalizado.")
