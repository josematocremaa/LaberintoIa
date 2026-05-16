"""
Grupo_N controller - Python version (mirrors Grupo_N.c logic)
GPS navigation + odometry heading + LIDAR obstacle avoidance
Data logging for neural network training
"""

from controller import Robot
import math, csv, os

# ── Init ──────────────────────────────────────────────────────────────
robot    = Robot()
timestep = int(robot.getBasicTimeStep())

WHEEL_RADIUS = 0.0212
WHEEL_BASE   = 0.090
DT           = timestep / 1000.0
MAX_SPEED    = 24.0

GOAL_X = -2.06352
GOAL_Y =  2.25539

# Motors
lm = robot.getDevice("left wheel motor")
rm = robot.getDevice("right wheel motor")
lm.setPosition(float('inf'))
rm.setPosition(float('inf'))
lm.setVelocity(0.0)
rm.setVelocity(0.0)

# Sensors
gps     = robot.getDevice("gps");     gps.enable(timestep)
compass = robot.getDevice("compass"); compass.enable(timestep)
lidar   = robot.getDevice("lidar");   lidar.enable(timestep); lidar.enablePointCloud()
kb      = robot.getDevice("keyboard"); kb.enable(timestep)

ds = []
for name in ["ds0","ds1","ds2","ds3","ds4","ds5","ds6","ds7","ds8"]:
    s = robot.getDevice(name); s.enable(timestep); ds.append(s)

# ── Logging ───────────────────────────────────────────────────────────
LOG_FILE = "training_data.csv"
write_header = not os.path.exists(LOG_FILE) or os.path.getsize(LOG_FILE) == 0
log_f = open(LOG_FILE, "a", newline="")
log_w = csv.writer(log_f)
if write_header:
    log_w.writerow(["lf","fc","rf","ll","rr","sin_err","cos_err","dist_norm","left_out","right_out"])

# ── State ─────────────────────────────────────────────────────────────
heading   = math.pi   # robot initially faces -X
prev_x    = prev_y    = 0.0
prev_lspd = prev_rspd = 0.0
stuck_cnt = recov_cnt = 0
init_done = False
step_cnt  = 0
autopilot = False

def norm_angle(a):
    while a >  math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a

def lidar_scan():
    pts = lidar.getPointCloud()
    lf=fc=rf=ll=rr=0.60
    for p in pts:
        px, py = p.x, p.y
        if math.isnan(px) or math.isnan(py) or px < 0.03: continue
        dh = math.sqrt(px*px + py*py)
        if dh > 0.58: continue
        ang = math.atan2(py, px)
        if   ang >  0.52: ll = min(ll, dh)
        elif ang >  0.17: lf = min(lf, dh)
        elif ang > -0.17: fc = min(fc, dh)
        elif ang > -0.52: rf = min(rf, dh)
        else:             rr = min(rr, dh)
    return lf, fc, rf, ll, rr

# ── Main loop ─────────────────────────────────────────────────────────
print("Grupo_N ready. Arrow keys = manual, A = autopilot toggle")

while robot.step(timestep) != -1:
    step_cnt += 1

    # Keyboard
    key = kb.getKey()
    if key == ord('A'):
        autopilot = not autopilot
        print("auto control" if autopilot else "manual control")

    # GPS
    pos = gps.getValues()
    cx, cy, cz = pos[0], pos[1], pos[2]

    # Heading: odometry + GPS correction
    dh_odom = (prev_rspd - prev_lspd) * WHEEL_RADIUS / WHEEL_BASE * DT
    heading = norm_angle(heading + dh_odom)
    if init_done:
        dx, dy = cx - prev_x, cy - prev_y
        d = math.sqrt(dx*dx + dy*dy)
        if d > 0.005:
            gps_h = math.atan2(dy, dx)
            err   = norm_angle(gps_h - heading)
            heading = norm_angle(heading + 0.25*err)
            stuck_cnt = 0
        else:
            stuck_cnt += 1
    else:
        init_done = True
    prev_x, prev_y = cx, cy

    # LIDAR
    lf, fc, rf, ll, rr = lidar_scan()
    fmin = min(lf, fc, rf)

    # Goal
    gdx, gdy   = GOAL_X - cx, GOAL_Y - cy
    goal_dist  = math.sqrt(gdx*gdx + gdy*gdy)
    target_h   = math.atan2(gdy, gdx)
    aerr       = norm_angle(target_h - heading)

    # ── ROBOT_MOVE logic ─────────────────────────────────────────────
    if autopilot:
        if cz < -0.20:
            left_v, right_v =  MAX_SPEED*0.4, -MAX_SPEED*0.4

        elif goal_dist < 0.30:
            left_v = right_v = 0.0
            if step_cnt % 60 == 0: print(f"META ALCANZADA! {goal_dist:.3f}m")

        elif recov_cnt > 0:
            recov_cnt -= 1
            d = 1.0 if aerr >= 0 else -1.0
            left_v, right_v = -MAX_SPEED*0.35*d, MAX_SPEED*0.35*d

        elif stuck_cnt > 150:
            recov_cnt = 80; stuck_cnt = 0
            d = 1.0 if aerr >= 0 else -1.0
            left_v, right_v = -MAX_SPEED*0.35*d, MAX_SPEED*0.35*d

        elif fmin < 0.14:
            open_l = min(lf, ll); open_r = min(rf, rr)
            if open_l >= open_r:
                left_v, right_v = -MAX_SPEED*0.40, MAX_SPEED*0.50
            else:
                left_v, right_v =  MAX_SPEED*0.50, -MAX_SPEED*0.40

        elif fmin < 0.28:
            if lf <= rf:
                ratio = min(lf/0.28, 1.0)
                left_v, right_v = MAX_SPEED*0.55, MAX_SPEED*0.55*ratio
            else:
                ratio = min(rf/0.28, 1.0)
                left_v, right_v = MAX_SPEED*0.55*ratio, MAX_SPEED*0.55

        elif abs(aerr) > 1.30:
            d = 1.0 if aerr > 0 else -1.0
            left_v, right_v = -MAX_SPEED*0.30*d, MAX_SPEED*0.30*d

        else:
            turn  = max(-MAX_SPEED, min(MAX_SPEED, 4.0 * aerr))
            base  = MAX_SPEED * 0.65 * (1.0 - 0.5*abs(aerr)/math.pi)
            left_v  = max(-MAX_SPEED, min(MAX_SPEED, base - turn))
            right_v = max(-MAX_SPEED, min(MAX_SPEED, base + turn))

        lm.setVelocity(left_v)
        rm.setVelocity(right_v)
        prev_lspd, prev_rspd = left_v, right_v

    else:
        # Manual: read arrow keys (already handled by Webots motor assignment
        # via speed[] in C; in Python we re-read here)
        lv = rv = 0.0
        if   key == kb.UP:    lv, rv =  MAX_SPEED*0.5,  MAX_SPEED*0.5
        elif key == kb.DOWN:  lv, rv = -MAX_SPEED*0.4, -MAX_SPEED*0.4
        elif key == kb.LEFT:  lv, rv =  MAX_SPEED*0.2,  MAX_SPEED*0.5
        elif key == kb.RIGHT: lv, rv =  MAX_SPEED*0.5,  MAX_SPEED*0.2
        lm.setVelocity(lv)
        rm.setVelocity(rv)
        prev_lspd, prev_rspd = lv, rv
        left_v,  right_v    = lv, rv

    # ── Log (both modes, only when moving) ───────────────────────────
    vl = prev_lspd / MAX_SPEED
    vr = prev_rspd / MAX_SPEED
    if (vl*vl + vr*vr) > 0.01 and step_cnt % 3 == 0:
        dn = min(goal_dist / 5.0, 1.0)
        log_w.writerow([
            round(lf/0.6,4), round(fc/0.6,4), round(rf/0.6,4),
            round(ll/0.6,4), round(rr/0.6,4),
            round(math.sin(aerr),4), round(math.cos(aerr),4),
            round(dn,4),
            round(vl,4), round(vr,4)
        ])

    if step_cnt % 200 == 0:
        print(f"Pos:({cx:.2f},{cy:.2f}) Meta:{goal_dist:.2f}m "
              f"HDG:{math.degrees(heading):.0f}° "
              f"LiDAR:[{lf:.2f} {fc:.2f} {rf:.2f}] "
              f"V:({prev_lspd:.1f},{prev_rspd:.1f})")

log_f.close()
