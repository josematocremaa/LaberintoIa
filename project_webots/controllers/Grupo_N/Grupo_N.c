/*
 * Copyright 1996-2024 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/keyboard.h>
#include <webots/gps.h>
#include <webots/compass.h>
#include <webots/lidar.h>

#define MAX_SENSOR_NUMBER 16
#define RANGE (1024 / 2)
#define BOUND(x, a, b) (((x) < (a)) ? (a) : ((x) > (b)) ? (b) : (x))

#define LEFT  0
#define RIGHT 1
#define TIME_STEP 16
#define MAX_SPEED 10.0

/* ── Robot constants (from maze_world.wbt) ─────────────────────────── */
#define WHEEL_RADIUS 0.0212   /* meters */
#define WHEEL_BASE   0.090    /* meters */
#define DT           0.016    /* seconds (TIME_STEP = 16 ms) */

static WbDeviceTag sensors[MAX_SENSOR_NUMBER], left_motor, right_motor;
static double matrix[MAX_SENSOR_NUMBER][2];
static WbDeviceTag gps;
static WbDeviceTag compass;
static WbDeviceTag lidar;

static int    num_sensors;
static double range;
static int    time_step  = 0;
static double max_speed  = 0.0;
static double speed_unit = 1.0;
static bool   autopilot     = false;
static bool   old_autopilot = false;
static int    old_key       = -1;

double speed[2];
double sensors_value[MAX_SENSOR_NUMBER];

static double GOAL_POSITION[3] = {2.0, 2.6, -0.04};
const double       *comp;
const double       *position_3d;
const WbLidarPoint *lidar_points;

/* ── Data-logging file (set LOG_DATA 1 to collect training data) ──── */
#define LOG_DATA 1
static FILE *log_fp = NULL;

static void check_keyboard() {
  int key = wb_keyboard_get_key();
  if (key >= 0) {
    switch (key) {
      case WB_KEYBOARD_UP:
        speed[LEFT] = MAX_SPEED; speed[RIGHT] = MAX_SPEED; autopilot = false; break;
      case WB_KEYBOARD_DOWN:
        speed[LEFT] = -MAX_SPEED; speed[RIGHT] = -MAX_SPEED; autopilot = false; break;
      case WB_KEYBOARD_RIGHT:
        speed[LEFT] = MAX_SPEED; speed[RIGHT] = MAX_SPEED / 2; autopilot = false; break;
      case WB_KEYBOARD_LEFT:
        speed[LEFT] = MAX_SPEED / 2; speed[RIGHT] = MAX_SPEED; autopilot = false; break;
      case 'A':
        if (key != old_key) autopilot = !autopilot;
        break;
    }
  }
  if (autopilot != old_autopilot) {
    old_autopilot = autopilot;
    printf(autopilot ? "auto control\n" : "manual control\n");
  }
  old_key = key;
}

static void initialize() {
  wb_robot_init();
  time_step = wb_robot_get_basic_time_step();
  const char *robot_name = wb_robot_get_name();
  const char SIRI_name[] = "ds0";
  char sensors_name[5];
  const double(*temp_matrix)[2];
  range = RANGE;

  const double SIRI3_matrix[9][2] = {
    {-5000,-5000},{-20000,40000},{-30000,50000},{-70000,70000},
    {70000,-60000},{50000,-40000},{40000,-20000},{-5000,-5000},{-10000,-10000}
  };
  const double SIRI3_max_speed = 24;
  const double SIRI3_speed_unit = 0.00053429;

  num_sensors = 9;
  sprintf(sensors_name, "%s", SIRI_name);
  temp_matrix = SIRI3_matrix;
  range = 2000;  max_speed = SIRI3_max_speed;  speed_unit = SIRI3_speed_unit;

  int i;
  for (i = 0; i < num_sensors; i++) {
    sensors[i] = wb_robot_get_device(sensors_name);
    wb_distance_sensor_enable(sensors[i], time_step);
    if ((i+1) >= 10) {
      sensors_name[2] = '1'; sensors_name[3]++;
      if ((i+1) == 10) { sensors_name[3] = '0'; sensors_name[4] = '\0'; }
    } else sensors_name[2]++;
    int j;
    for (j = 0; j < 2; j++) matrix[i][j] = temp_matrix[i][j];
  }

  left_motor  = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor,  INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor,  0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  lidar = wb_robot_get_device("lidar");
  wb_lidar_enable(lidar, TIME_STEP);
  wb_lidar_enable_point_cloud(lidar);
  wb_keyboard_enable(TIME_STEP);

  gps     = wb_robot_get_device("gps");     wb_gps_enable(gps, TIME_STEP);
  compass = wb_robot_get_device("compass"); wb_compass_enable(compass, TIME_STEP);

  if (LOG_DATA) {
    log_fp = fopen("/home/josemanuel/Practica2_ia/training_data.csv", "a");
    /* Write header only if file is empty */
    if (log_fp) {
      fseek(log_fp, 0, SEEK_END);
      if (ftell(log_fp) == 0)
        fprintf(log_fp, "lf,fc,rf,ll,rr,sin_err,cos_err,dist_norm,left_out,right_out\n");
    }
  }

  printf("The %s robot is initialized!\n", robot_name);
  printf("Press 'a' for autopilot, arrows for manual, 'a' again to stop\n");
}

/* ═══════════════════════════════════════════════════════════════════════
 * ROBOT_MOVE  –  ONLY this function body is modified from the template.
 *
 * Algorithm: GPS navigation + odometry heading + LIDAR obstacle avoidance
 * + training-data logging.
 *
 * Heading is tracked with ODOMETRY (commanded wheel speeds) so it stays
 * accurate even while spinning, and corrected with GPS velocity when
 * the robot moves forward.
 * ═══════════════════════════════════════════════════════════════════════ */
void ROBOT_MOVE(double *SPEEDS, double *vals)
{
  /* ── Goal: update after loading new world (click DEF Meta Solid   */
  /* in Webots scene tree → Position tab to read the coordinates)    */
  static const double GOAL_X = -2.06352;   /* TODO: update for new map */
  static const double GOAL_Y =  2.25539;   /* TODO: update for new map */

  /* ── Mode: 1 = teleop with WASD + logging, 0 = autonomous + log  */
  #define TELEOP_MODE 0   /* 0=autonomous+logging  1=WASD teleop (unreliable in Webots) */

  /* ── Persistent state ─────────────────────────────────────────── */
  static double heading      = 3.14159265; /* initial: robot faces -X */
  static double prev_x       = 0.0,  prev_y = 0.0;
  static double prev_lspd    = 0.0,  prev_rspd = 0.0;  /* last applied speeds */
  static int    stuck_cnt    = 0;
  static int    recov_cnt    = 0;
  static int    init_done    = 0;
  static int    step_cnt     = 0;

  step_cnt++;

  /* ── GPS ──────────────────────────────────────────────────────── */
  double cx = position_3d[0];
  double cy = position_3d[1];
  double cz = position_3d[2];

  /* ── Heading: odometry first, then GPS correction ────────────── */
  /* Odometry: dheading = (Vr - Vl) * wheel_radius / wheel_base * dt */
  double dh_odom = (prev_rspd - prev_lspd) * WHEEL_RADIUS / WHEEL_BASE * DT;
  heading += dh_odom;
  while (heading >  3.14159265) heading -= 6.28318530;
  while (heading < -3.14159265) heading += 6.28318530;

  if (init_done) {
    double dx = cx - prev_x,  dy = cy - prev_y;
    double d  = sqrt(dx*dx + dy*dy);
    if (d > 0.005) {                    /* moving forward → trust GPS */
      double gps_h = atan2(dy, dx);
      double err   = gps_h - heading;
      while (err >  3.14159265) err -= 6.28318530;
      while (err < -3.14159265) err += 6.28318530;
      heading += 0.25 * err;            /* gentle GPS blend */
      while (heading >  3.14159265) heading -= 6.28318530;
      while (heading < -3.14159265) heading += 6.28318530;
      stuck_cnt = 0;
    } else {
      stuck_cnt++;
    }
  } else {
    init_done = 1;
  }
  prev_x = cx;  prev_y = cy;

  /* ── Fall detection ───────────────────────────────────────────── */
  if (cz < -0.20) {
    SPEEDS[LEFT] = max_speed * 0.4;  SPEEDS[RIGHT] = -max_speed * 0.4;
    prev_lspd = SPEEDS[LEFT];  prev_rspd = SPEEDS[RIGHT];
    return;
  }

  /* ── Goal check ───────────────────────────────────────────────── */
  double gdx = GOAL_X - cx,  gdy = GOAL_Y - cy;
  double goal_dist = sqrt(gdx*gdx + gdy*gdy);
  if (goal_dist < 0.30) {
    SPEEDS[LEFT] = SPEEDS[RIGHT] = 0.0;
    prev_lspd = prev_rspd = 0.0;
    if (step_cnt % 60 == 0) printf("META ALCANZADA! %.3f m\n", goal_dist);
    return;
  }

  /* ── TELEOP: read WASD (check_keyboard only intercepts arrows)  ──
   * Activate with 'A', then steer with W/A/S/D.  Data is logged.   */
  if (TELEOP_MODE) {
    int key = wb_keyboard_get_key();
    double tl = 0.0, tr = 0.0;
    if      (key == 'W') { tl =  max_speed * 0.60; tr =  max_speed * 0.60; }
    else if (key == 'S') { tl = -max_speed * 0.40; tr = -max_speed * 0.40; }
    else if (key == 'A') { tl =  max_speed * 0.15; tr =  max_speed * 0.55; }
    else if (key == 'D') { tl =  max_speed * 0.55; tr =  max_speed * 0.15; }
    /* else: hold still until a key is pressed */
    SPEEDS[LEFT]  = tl;
    SPEEDS[RIGHT] = tr;
    /* still need LIDAR for the log columns → fall through to scan  */
  }

  /* ── LIDAR scan ───────────────────────────────────────────────── */
  float lf = 0.60f, fc = 0.60f, rf = 0.60f, ll = 0.60f, rr = 0.60f;
  int n_pts = wb_lidar_get_number_of_points(lidar);
  int k;
  for (k = 0; k < n_pts; k++) {
    float px = lidar_points[k].x,  py = lidar_points[k].y;
    if (px != px || py != py) continue;          /* NaN */
    if (px < 0.03f)           continue;          /* behind */
    float dh = sqrtf(px*px + py*py);
    if (dh > 0.58f)           continue;          /* out of range */
    float ang = atan2f(py, px);
    if      (ang >  0.52f) { if (dh < ll) ll = dh; }  /* >30° left  */
    else if (ang >  0.17f) { if (dh < lf) lf = dh; }  /* 10-30° left */
    else if (ang > -0.17f) { if (dh < fc) fc = dh; }  /* ±10° center */
    else if (ang > -0.52f) { if (dh < rf) rf = dh; }  /* 10-30° right */
    else                   { if (dh < rr) rr = dh; }  /* >30° right  */
  }
  float fmin = lf < fc ? (lf < rf ? lf : rf) : (fc < rf ? fc : rf);

  /* ── Thresholds ───────────────────────────────────────────────── */
  double SAFE  = 0.28;   /* steer away */
  double EMERG = 0.14;   /* emergency turn */

  /* ── Goal direction ───────────────────────────────────────────── */
  double target_h = atan2(gdy, gdx);
  double aerr     = target_h - heading;
  while (aerr >  3.14159265) aerr -= 6.28318530;
  while (aerr < -3.14159265) aerr += 6.28318530;

  /* ── Skip autonomous logic in teleop mode ────────────────────── */
  if (TELEOP_MODE) goto done;

  /* ── Stuck recovery ───────────────────────────────────────────── */
  if (recov_cnt > 0) {
    recov_cnt--;
    /* Turn toward goal direction */
    double dir = (aerr >= 0.0) ? 1.0 : -1.0;
    SPEEDS[LEFT]  = -max_speed * 0.35 * dir;
    SPEEDS[RIGHT] =  max_speed * 0.35 * dir;
    goto done;
  }
  if (stuck_cnt > 150) {
    recov_cnt = 80;  stuck_cnt = 0;
    double dir = (aerr >= 0.0) ? 1.0 : -1.0;
    SPEEDS[LEFT]  = -max_speed * 0.35 * dir;
    SPEEDS[RIGHT] =  max_speed * 0.35 * dir;
    goto done;
  }

  /* ── Emergency avoidance ──────────────────────────────────────── */
  if (fmin < EMERG) {
    double open_l = (lf < ll) ? lf : ll;
    double open_r = (rf < rr) ? rf : rr;
    if (open_l >= open_r) {         /* left more open → turn left  */
      SPEEDS[LEFT]  = -max_speed * 0.40;
      SPEEDS[RIGHT] =  max_speed * 0.50;
    } else {                        /* right more open → turn right */
      SPEEDS[LEFT]  =  max_speed * 0.50;
      SPEEDS[RIGHT] = -max_speed * 0.40;
    }
    goto done;
  }

  /* ── Moderate obstacle: steer away ───────────────────────────── */
  if (fmin < SAFE) {
    double ratio;
    if (lf <= rf) {                 /* more open on right */
      ratio = lf / SAFE;  if (ratio > 1.0) ratio = 1.0;
      SPEEDS[LEFT]  = max_speed * 0.55;
      SPEEDS[RIGHT] = max_speed * 0.55 * ratio;
    } else {                        /* more open on left */
      ratio = rf / SAFE;  if (ratio > 1.0) ratio = 1.0;
      SPEEDS[LEFT]  = max_speed * 0.55 * ratio;
      SPEEDS[RIGHT] = max_speed * 0.55;
    }
    goto done;
  }

  /* ── Large angle error: turn in place toward goal ────────────── */
  {
    double abe = aerr < 0.0 ? -aerr : aerr;
    if (abe > 1.30) {               /* >75°: spin toward goal first  */
      double dir = (aerr > 0.0) ? 1.0 : -1.0;
      SPEEDS[LEFT]  = -max_speed * 0.30 * dir;
      SPEEDS[RIGHT] =  max_speed * 0.30 * dir;
      goto done;
    }

    /* ── Navigate toward goal ───────────────────────────────────── */
    double turn      = 4.0 * aerr;
    if (turn >  max_speed) turn =  max_speed;
    if (turn < -max_speed) turn = -max_speed;
    double spd_factor = 1.0 - 0.5 * abe / 3.14159265;
    double base       = max_speed * 0.65 * spd_factor;
    SPEEDS[LEFT]  = base - turn;
    SPEEDS[RIGHT] = base + turn;
    if (SPEEDS[LEFT]  >  max_speed) SPEEDS[LEFT]  =  max_speed;
    if (SPEEDS[LEFT]  < -max_speed) SPEEDS[LEFT]  = -max_speed;
    if (SPEEDS[RIGHT] >  max_speed) SPEEDS[RIGHT] =  max_speed;
    if (SPEEDS[RIGHT] < -max_speed) SPEEDS[RIGHT] = -max_speed;
  }

done:
  /* ── Log training data ────────────────────────────────────────── */
  /* Format: lf fc rf ll rr sin(err) cos(err) dist_norm left right  */
  /* lf/fc/rf/ll/rr normalised 0-1 (1=clear, 0=obstacle at sensor)  */
  if (LOG_DATA && log_fp && step_cnt % 3 == 0) {
    double aerr2 = aerr;   /* already normalised to [-pi,pi] */
    double dn    = goal_dist > 5.0 ? 1.0 : goal_dist / 5.0;
    fprintf(log_fp, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
            (double)lf/0.6, (double)fc/0.6, (double)rf/0.6,
            (double)ll/0.6, (double)rr/0.6,
            sin(aerr2), cos(aerr2), dn,
            SPEEDS[LEFT]  / max_speed,
            SPEEDS[RIGHT] / max_speed);
  }

  /* Debug every 200 steps */
  if (step_cnt % 200 == 0)
    printf("Pos:(%.2f,%.2f) Meta:%.2fm HDG:%.0f° tgt:%.0f° "
           "LiDAR[%.2f %.2f %.2f] V:(%.1f,%.1f)\n",
           cx, cy, goal_dist,
           heading * 57.2957795, target_h * 57.2957795,
           (double)lf, (double)fc, (double)rf,
           SPEEDS[LEFT], SPEEDS[RIGHT]);

  /* ── Store applied speeds for next-step odometry ─────────────── */
  prev_lspd = SPEEDS[LEFT];
  prev_rspd = SPEEDS[RIGHT];
  return;

  /* Suppress 'label at end of compound statement' warning */
  (void)0;
}

int main()
{
  int i;
  speed[0] = 0.0;  speed[1] = 0.0;
  initialize();

  while (wb_robot_step(time_step) != -1) {
    check_keyboard();
    position_3d  = wb_gps_get_values(gps);
    comp         = wb_compass_get_values(compass);
    for (i = 0; i < num_sensors; i++)
      sensors_value[i] = wb_distance_sensor_get_value(sensors[i]);
    lidar_points = wb_lidar_get_point_cloud(lidar);

    if (autopilot)
      ROBOT_MOVE(speed, sensors_value);

    wb_motor_set_velocity(left_motor,  speed[0]);
    wb_motor_set_velocity(right_motor, speed[1]);

    /* ── DATA COLLECTION: log sensor+speed every step (manual AND auto) ──
     * Remove this block once training data is collected.              */
    if (LOG_DATA && log_fp) {
      /* LIDAR scan (same logic as in ROBOT_MOVE) */
      float lf2=0.60f, fc2=0.60f, rf2=0.60f, ll2=0.60f, rr2=0.60f;
      int np = wb_lidar_get_number_of_points(lidar);
      int ki;
      for (ki = 0; ki < np; ki++) {
        float px = lidar_points[ki].x, py = lidar_points[ki].y;
        if (px != px || py != py || px < 0.03f) continue;
        float dh = sqrtf(px*px + py*py);
        if (dh > 0.58f) continue;
        float ang = atan2f(py, px);
        if      (ang >  0.52f) { if (dh < ll2) ll2 = dh; }
        else if (ang >  0.17f) { if (dh < lf2) lf2 = dh; }
        else if (ang > -0.17f) { if (dh < fc2) fc2 = dh; }
        else if (ang > -0.52f) { if (dh < rf2) rf2 = dh; }
        else                   { if (dh < rr2) rr2 = dh; }
      }
      /* GPS angle to goal */
      double cx2 = position_3d[0], cy2 = position_3d[1];
      double gdx2 = -2.06352 - cx2, gdy2 = 2.25539 - cy2;
      double gd2  = sqrt(gdx2*gdx2 + gdy2*gdy2);
      double th2  = atan2(gdy2, gdx2);
      /* heading approximation from GPS velocity is in ROBOT_MOVE;
         here we use goal angle directly as a proxy */
      double dn2  = gd2 > 5.0 ? 1.0 : gd2 / 5.0;
      /* only log when robot is actually moving (avoids static noise) */
      double vl = speed[0] / max_speed, vr = speed[1] / max_speed;
      if (vl*vl + vr*vr > 0.01)
        fprintf(log_fp, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                (double)lf2/0.6, (double)fc2/0.6, (double)rf2/0.6,
                (double)ll2/0.6, (double)rr2/0.6,
                sin(th2), cos(th2), dn2, vl, vr);
    }
  }

  if (log_fp) fclose(log_fp);
  return 0;
}
