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

/*
 * Description:  A controller moving various robots using the Braitenberg method.
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

#define LEFT 0
#define RIGHT 1
#define TIME_STEP 16
#define MAX_SPEED 10.0

static WbDeviceTag sensors[MAX_SENSOR_NUMBER], left_motor, right_motor;
static double matrix[MAX_SENSOR_NUMBER][2];
static WbDeviceTag gps;
static WbDeviceTag compass;
static WbDeviceTag lidar;

static int num_sensors;
static double range;
static int time_step = 0;
static double max_speed = 0.0;
static double speed_unit = 1.0;
static bool autopilot = false;
static bool old_autopilot = false;
static int old_key = -1;

double speed[2];
double sensors_value[MAX_SENSOR_NUMBER];

static double GOAL_POSITION[3] = {2.0, 2.6, -0.04};
const double *comp;
const double *position_3d;
const WbLidarPoint *lidar_points;

static void check_keyboard() {
  int key = wb_keyboard_get_key();
  if (key >= 0) {
    switch (key) {
      case WB_KEYBOARD_UP:
        speed[LEFT]  = MAX_SPEED;
        speed[RIGHT] = MAX_SPEED;
        autopilot = false;
        break;
      case WB_KEYBOARD_DOWN:
        speed[LEFT]  = -MAX_SPEED;
        speed[RIGHT] = -MAX_SPEED;
        autopilot = false;
        break;
      case WB_KEYBOARD_RIGHT:
        speed[LEFT]  = MAX_SPEED;
        speed[RIGHT] = MAX_SPEED / 2;
        autopilot = false;
        break;
      case WB_KEYBOARD_LEFT:
        speed[LEFT]  = MAX_SPEED / 2;
        speed[RIGHT] = MAX_SPEED;
        autopilot = false;
        break;
      case 'A':
        if (key != old_key)
          autopilot = !autopilot;
        break;
    }
  }
  if (autopilot != old_autopilot) {
    old_autopilot = autopilot;
    if (autopilot)
      printf("auto control\n");
    else
      printf("manual control\n");
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
    {-5000, -5000}, {-20000, 40000}, {-30000, 50000}, {-70000, 70000},
    {70000, -60000}, {50000, -40000}, {40000, -20000}, {-5000, -5000},
    {-10000, -10000}
  };

  const double SIRI3_max_speed = 24;
  const double SIRI3_speed_unit = 0.00053429;

  num_sensors = 9;
  sprintf(sensors_name, "%s", SIRI_name);
  temp_matrix = SIRI3_matrix;
  range = 2000;
  max_speed = SIRI3_max_speed;
  speed_unit = SIRI3_speed_unit;

  int i;
  for (i = 0; i < num_sensors; i++) {
    sensors[i] = wb_robot_get_device(sensors_name);
    wb_distance_sensor_enable(sensors[i], time_step);

    if ((i + 1) >= 10) {
      sensors_name[2] = '1';
      sensors_name[3]++;
      if ((i + 1) == 10) {
        sensors_name[3] = '0';
        sensors_name[4] = '\0';
      }
    } else
      sensors_name[2]++;

    int j;
    for (j = 0; j < 2; j++)
      matrix[i][j] = temp_matrix[i][j];
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

  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);

  compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, TIME_STEP);

  printf("The %s robot is initialized!\n", robot_name);
  printf("Press 'a' for autopilot, or arrows for manual driving\n");
}

/* ─────────────────────────────────────────────────────────────────────────
 * ROBOT_MOVE – only this function body is modified from the template.
 *
 * Algorithm: GPS navigation + GPS-velocity heading + LIDAR obstacle avoidance
 *
 * The LIDAR (maxRange=0.6m, aperture=pi/2 horizontal) provides reliable
 * obstacle detection. IR sensors are used only as backup for very close range.
 *
 * IR sensor convention: LOW value (~0) = close obstacle, HIGH (~1000) = clear.
 * The IR sensors are unreliable beyond ~10cm so LIDAR is the primary source.
 *
 * Sensor layout (robot frame, +X=forward, +Y=left, +Z=up):
 *   ds0 rear-left   ds1 front-left    ds2 front-left-45
 *   ds3 front       ds4 front         ds5 front-right-45
 *   ds6 front-right  ds7 rear-right   ds8 rear
 * ───────────────────────────────────────────────────────────────────────── */
void ROBOT_MOVE(double *SPEEDS, double *vals)
{
  /* ── Goal: DEF Meta Solid from maze_world.wbt ─────────────────── */
  static const double GOAL_X = -2.06352;
  static const double GOAL_Y =  2.25539;

  /* ── Persistent state ──────────────────────────────────────────── */
  static double prev_x    = 0.0;
  static double prev_y    = 0.0;
  static double heading   = 3.14159265; /* facing -X initially */
  static int    stuck_cnt = 0;
  static int    recov_cnt = 0;
  static int    init_done = 0;
  static int    step_cnt  = 0;

  step_cnt++;

  /* ── GPS position ──────────────────────────────────────────────── */
  double cx = position_3d[0];
  double cy = position_3d[1];
  double cz = position_3d[2];

  /* ── Heading from GPS velocity (updated every step) ───────────── */
  if (init_done) {
    double dx = cx - prev_x;
    double dy = cy - prev_y;
    double d  = sqrt(dx * dx + dy * dy);
    if (d > 0.003) {
      double gh  = atan2(dy, dx);
      double err = gh - heading;
      while (err >  3.14159265) err -= 6.28318530;
      while (err < -3.14159265) err += 6.28318530;
      heading += 0.35 * err;
      while (heading >  3.14159265) heading -= 6.28318530;
      while (heading < -3.14159265) heading += 6.28318530;
      stuck_cnt = 0;
    } else {
      stuck_cnt++;
    }
  } else {
    init_done = 1;
  }
  prev_x = cx;
  prev_y = cy;

  /* ── Hole/fall detection ────────────────────────────────────────── */
  if (cz < -0.20) {
    SPEEDS[LEFT]  =  max_speed * 0.4;
    SPEEDS[RIGHT] = -max_speed * 0.4;
    return;
  }

  /* ── Goal reached ──────────────────────────────────────────────── */
  double gdx       = GOAL_X - cx;
  double gdy       = GOAL_Y - cy;
  double goal_dist = sqrt(gdx * gdx + gdy * gdy);

  if (goal_dist < 0.30) {
    SPEEDS[LEFT]  = 0.0;
    SPEEDS[RIGHT] = 0.0;
    if (step_cnt % 100 == 0)
      printf("META ALCANZADA! dist=%.3f m\n", goal_dist);
    return;
  }

  /* ── LIDAR scan: find closest obstacle in front sectors ─────────
   * Lidar local frame: x=forward(tilted ~22° down), y=left, z=up
   * We use the horizontal distance sqrt(x²+y²) as the obstacle range.
   * Sectors based on lateral angle: left (y>0), center, right (y<0)  */
  float lf = 0.60f;   /* closest in front-left  sector */
  float fc = 0.60f;   /* closest in front-center sector */
  float rf = 0.60f;   /* closest in front-right sector */
  float ll = 0.60f;   /* closest in far-left (side) sector */
  float rr = 0.60f;   /* closest in far-right (side) sector */

  int n_pts = wb_lidar_get_number_of_points(lidar);
  int k;
  for (k = 0; k < n_pts; k++) {
    float px = lidar_points[k].x;
    float py = lidar_points[k].y;

    /* Skip NaN / infinity / behind the sensor */
    if (px != px || py != py)   continue;   /* NaN check */
    if (px < 0.03f)             continue;   /* behind or too close */
    float dh = sqrtf(px * px + py * py);    /* horizontal distance */
    if (dh > 0.58f)             continue;   /* beyond range */

    float ang = atan2f(py, px); /* lateral angle: + = left, - = right */

    if (ang > 0.40f) {          /* > ~23° left → side-left */
      if (dh < ll) ll = dh;
    } else if (ang > 0.12f) {   /* 7°–23° left → front-left */
      if (dh < lf) lf = dh;
    } else if (ang > -0.12f) {  /* ±7° center */
      if (dh < fc) fc = dh;
    } else if (ang > -0.40f) {  /* 7°–23° right → front-right */
      if (dh < rf) rf = dh;
    } else {                    /* > ~23° right → side-right */
      if (dh < rr) rr = dh;
    }
  }

  /* Closest reading across the full forward arc */
  float fmin = lf;
  if (fc < fmin) fmin = fc;
  if (rf < fmin) fmin = rf;

  /* ── Obstacle distances (meters) ───────────────────────────────── */
  double SAFE_DIST  = 0.30; /* steer away below this */
  double EMERG_DIST = 0.15; /* emergency turn below this */

  /* ── Stuck recovery ─────────────────────────────────────────────── */
  if (recov_cnt > 0) {
    recov_cnt--;
    SPEEDS[LEFT]  = -max_speed * 0.45;
    SPEEDS[RIGHT] =  max_speed * 0.45;
    return;
  }
  if (stuck_cnt > 120) {
    recov_cnt = 60;
    stuck_cnt = 0;
    SPEEDS[LEFT]  = -max_speed * 0.45;
    SPEEDS[RIGHT] =  max_speed * 0.45;
    return;
  }

  /* ── Emergency avoidance ────────────────────────────────────────── */
  if (fmin < EMERG_DIST) {
    /* Turn toward whichever side (left or right) has more space.
     * Use ll/rr (side sectors) as tiebreaker when lf ≈ rf.         */
    double open_left  = (lf < ll) ? lf : ll;
    double open_right = (rf < rr) ? rf : rr;
    if (open_left >= open_right) {  /* left is more open → turn left */
      SPEEDS[LEFT]  = -max_speed * 0.40;
      SPEEDS[RIGHT] =  max_speed * 0.50;
    } else {                        /* right is more open → turn right */
      SPEEDS[LEFT]  =  max_speed * 0.50;
      SPEEDS[RIGHT] = -max_speed * 0.40;
    }
    return;
  }

  /* ── Moderate obstacle: proportional steer ──────────────────────── */
  if (fmin < SAFE_DIST) {
    double ratio;
    if (lf <= rf) {             /* more open on right → steer right */
      ratio = lf / SAFE_DIST;
      if (ratio > 1.0) ratio = 1.0;
      SPEEDS[LEFT]  = max_speed * 0.55;
      SPEEDS[RIGHT] = max_speed * 0.55 * ratio;
    } else {                    /* more open on left → steer left */
      ratio = rf / SAFE_DIST;
      if (ratio > 1.0) ratio = 1.0;
      SPEEDS[LEFT]  = max_speed * 0.55 * ratio;
      SPEEDS[RIGHT] = max_speed * 0.55;
    }
    return;
  }

  /* ── Clear path: navigate toward goal ───────────────────────────── */
  double target_h = atan2(gdy, gdx);
  double aerr     = target_h - heading;
  while (aerr >  3.14159265) aerr -= 6.28318530;
  while (aerr < -3.14159265) aerr += 6.28318530;

  double turn_gain  = 5.0;
  double turn       = turn_gain * aerr;
  if (turn >  max_speed) turn =  max_speed;
  if (turn < -max_speed) turn = -max_speed;

  double abe        = (aerr < 0.0) ? -aerr : aerr;
  double spd_factor = 1.0 - 0.4 * abe / 3.14159265;
  double base       = max_speed * 0.65 * spd_factor;

  SPEEDS[LEFT]  = base - turn;
  SPEEDS[RIGHT] = base + turn;

  if (SPEEDS[LEFT]  >  max_speed) SPEEDS[LEFT]  =  max_speed;
  if (SPEEDS[LEFT]  < -max_speed) SPEEDS[LEFT]  = -max_speed;
  if (SPEEDS[RIGHT] >  max_speed) SPEEDS[RIGHT] =  max_speed;
  if (SPEEDS[RIGHT] < -max_speed) SPEEDS[RIGHT] = -max_speed;

  /* Debug every 200 steps */
  if (step_cnt % 200 == 0)
    printf("Pos:(%.2f,%.2f) Meta:%.2fm HDG:%.0f° "
           "LiDAR[lf=%.2f fc=%.2f rf=%.2f] V:(%.1f,%.1f)\n",
           cx, cy, goal_dist, heading * 57.2957795,
           (double)lf, (double)fc, (double)rf,
           SPEEDS[LEFT], SPEEDS[RIGHT]);
}

int main()
{
  int i;

  speed[0] = 0.0;
  speed[1] = 0.0;

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
  }

  return 0;
}
