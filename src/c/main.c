/*
 * Sensor Forwarding Test App
 *
 * Tests that accelerometer and compass sensor data forwarded from the
 * CloudPebble emulator (via QEMU) arrives correctly in the watch app.
 *
 * The emulator sends:
 *   Accel:   [sample_count][x_int16][y_int16][z_int16]...
 *             units: mg (milli-g), so 1g gravity = ~1000
 *   Compass: [heading_int32][calibration_byte]
 *             heading in TRIG_MAX_ANGLE (0x10000) units, calibration 0-3
 *
 * Test criteria:
 *   ACCEL PASS   - received accel data with |magnitude - 1000| < 350
 *                  (device at rest: gravity ≈ 1000 mg on one axis)
 *   COMPASS PASS - received compass data with heading in [0, TRIG_MAX_ANGLE)
 *                  and calibration level >= 1
 *
 * Note: tap is NOT forwarded from the Safari sensor page (qemu-sensors.html
 * only uses DeviceMotionEvent and DeviceOrientationEvent).
 *
 * Display layout (top to bottom):
 *   [Title]
 *   Accel X / Y / Z  (raw mg values)
 *   Magnitude        (|a| in mg)
 *   Compass heading  (degrees)
 *   Calibration      (0-3)
 *   Status line      (PASS / FAIL / WAITING per sensor)
 */

#include <pebble.h>

/* ── tuneable constants ─────────────────────────────────────────────────── */
#define ACCEL_SAMPLES_PER_UPDATE  5       /* batch size sent to handler      */
#define ACCEL_GRAVITY_MG          1000    /* expected resting magnitude (mg)  */
#define ACCEL_GRAVITY_TOLERANCE   350     /* ±mg before we call it "wrong"    */
#define TIMEOUT_SECONDS           10      /* seconds before marking TIMEOUT   */

/* ── test state ─────────────────────────────────────────────────────────── */
typedef enum {
  TEST_WAITING = 0,
  TEST_PASS,
  TEST_FAIL,
  TEST_TIMEOUT,
} TestResult;

static struct {
  /* raw data */
  int16_t  ax, ay, az;
  int32_t  magnitude;
  int32_t  compass_heading_deg;   /* 0-359 */
  uint8_t  compass_calibration;   /* 0-3   */

  /* test outcomes */
  TestResult accel_result;
  TestResult compass_result;

  /* counters / timing */
  uint32_t accel_callbacks;
  uint32_t compass_callbacks;
  time_t   start_time;
} s_state;

/* ── UI layers ──────────────────────────────────────────────────────────── */
static Window     *s_window;
static TextLayer  *s_title_layer;
static TextLayer  *s_accel_layer;
static TextLayer  *s_mag_layer;
static TextLayer  *s_compass_layer;
static TextLayer  *s_cal_layer;
static TextLayer  *s_status_layer;

/* ── string buffers (static to avoid stack churn) ───────────────────────── */
static char s_accel_buf[40];
static char s_mag_buf[32];
static char s_compass_buf[32];
static char s_cal_buf[24];
static char s_status_buf[32];

/* ── helpers ─────────────────────────────────────────────────────────────── */

static const char *result_str(TestResult r) {
  switch (r) {
    case TEST_WAITING: return "WAIT";
    case TEST_PASS:    return "PASS";
    case TEST_FAIL:    return "FAIL";
    case TEST_TIMEOUT: return "TOUT";
    default:           return "????";
  }
}

/* integer square root (no math.h needed on watch) */
static int32_t isqrt(int32_t n) {
  if (n <= 0) return 0;
  int32_t x = n;
  int32_t y = 1;
  while (x > y) {
    x = (x + y) / 2;
    y = n / x;
  }
  return x;
}

static void update_display(void) {
  snprintf(s_accel_buf, sizeof(s_accel_buf),
           "A: %d / %d / %d mg",
           (int)s_state.ax, (int)s_state.ay, (int)s_state.az);

  snprintf(s_mag_buf, sizeof(s_mag_buf),
           "|a|=%d mg (want ~%d)",
           (int)s_state.magnitude, ACCEL_GRAVITY_MG);

  snprintf(s_compass_buf, sizeof(s_compass_buf),
           "Hdg: %d deg", (int)s_state.compass_heading_deg);

  snprintf(s_cal_buf, sizeof(s_cal_buf),
           "Cal: %d/3", (int)s_state.compass_calibration);

  snprintf(s_status_buf, sizeof(s_status_buf),
           "Accel:%s  Compass:%s",
           result_str(s_state.accel_result),
           result_str(s_state.compass_result));

  text_layer_set_text(s_accel_layer,   s_accel_buf);
  text_layer_set_text(s_mag_layer,     s_mag_buf);
  text_layer_set_text(s_compass_layer, s_compass_buf);
  text_layer_set_text(s_cal_layer,     s_cal_buf);
  text_layer_set_text(s_status_layer,  s_status_buf);
}

/* ── sensor callbacks ────────────────────────────────────────────────────── */

static void accel_handler(AccelData *data, uint32_t num_samples) {
  if (num_samples == 0) return;

  /* use the most recent sample */
  AccelData *latest = &data[num_samples - 1];
  s_state.ax = latest->x;
  s_state.ay = latest->y;
  s_state.az = latest->z;
  s_state.accel_callbacks++;

  /* compute vector magnitude in mg */
  int32_t mag_sq = (int32_t)s_state.ax * s_state.ax
                 + (int32_t)s_state.ay * s_state.ay
                 + (int32_t)s_state.az * s_state.az;
  s_state.magnitude = isqrt(mag_sq);

  /* validate: at rest |a| ≈ 1g = 1000 mg */
  int32_t delta = s_state.magnitude - ACCEL_GRAVITY_MG;
  if (delta < 0) delta = -delta;

  if (s_state.accel_result != TEST_PASS) {
    s_state.accel_result = (delta <= ACCEL_GRAVITY_TOLERANCE)
                           ? TEST_PASS : TEST_FAIL;
  }

  APP_LOG(APP_LOG_LEVEL_DEBUG,
          "Accel cb#%lu  x=%d y=%d z=%d |a|=%d  [%s]",
          (unsigned long)s_state.accel_callbacks,
          (int)s_state.ax, (int)s_state.ay, (int)s_state.az,
          (int)s_state.magnitude,
          result_str(s_state.accel_result));

  update_display();
}

static void compass_handler(CompassHeadingData heading_data) {
  s_state.compass_callbacks++;
  s_state.compass_calibration = heading_data.compass_status;

  /*
   * CompassHeading is in TRIG_MAX_ANGLE units (0x10000 = 360 degrees).
   * Convert to degrees for display.
   */
  int32_t raw = heading_data.magnetic_heading;
  s_state.compass_heading_deg = (raw * 360) / TRIG_MAX_ANGLE;

  /*
   * Validate:
   *  - heading must be in [0, TRIG_MAX_ANGLE)
   *  - calibration >= 1 means the emulator sent a real heading
   *
   * The JS sends calibration=2 from qemu-sensors.html, so we expect >= 1.
   */
  bool heading_ok   = (raw >= 0 && raw < TRIG_MAX_ANGLE);
  bool cal_ok       = (heading_data.compass_status >= 1);

  if (s_state.compass_result != TEST_PASS) {
    s_state.compass_result = (heading_ok && cal_ok) ? TEST_PASS : TEST_FAIL;
  }

  APP_LOG(APP_LOG_LEVEL_DEBUG,
          "Compass cb#%lu  raw=%ld deg=%ld cal=%d  [%s]",
          (unsigned long)s_state.compass_callbacks,
          (long)raw,
          (long)s_state.compass_heading_deg,
          (int)heading_data.compass_status,
          result_str(s_state.compass_result));

  update_display();
}

/* ── timeout tick handler ────────────────────────────────────────────────── */

static void tick_handler(struct tm *tick_time, TimeUnits units_changed) {
  time_t now = time(NULL);
  int elapsed = (int)(now - s_state.start_time);

  if (elapsed >= TIMEOUT_SECONDS) {
    /* mark anything still WAITING as TIMEOUT */
    if (s_state.accel_result   == TEST_WAITING) s_state.accel_result   = TEST_TIMEOUT;
    if (s_state.compass_result == TEST_WAITING) s_state.compass_result = TEST_TIMEOUT;

    tick_timer_service_unsubscribe();

    APP_LOG(APP_LOG_LEVEL_WARNING,
            "Timeout after %ds — Accel:%s Compass:%s",
            elapsed,
            result_str(s_state.accel_result),
            result_str(s_state.compass_result));

    update_display();
  }
}

/* ── window lifecycle ────────────────────────────────────────────────────── */

static TextLayer *make_layer(Layer *root, GRect frame, GFont font,
                             GColor bg, GColor fg, GTextAlignment align) {
  TextLayer *tl = text_layer_create(frame);
  text_layer_set_background_color(tl, bg);
  text_layer_set_text_color(tl, fg);
  text_layer_set_font(tl, font);
  text_layer_set_text_alignment(tl, align);
  layer_add_child(root, text_layer_get_layer(tl));
  return tl;
}

static void window_load(Window *window) {
  Layer *root = window_get_root_layer(window);
  GRect bounds = layer_get_bounds(root);

  int w = bounds.size.w;
  int y = 0;

  GFont title_font = fonts_get_system_font(FONT_KEY_GOTHIC_14_BOLD);
  GFont body_font  = fonts_get_system_font(FONT_KEY_GOTHIC_14);
  GFont stat_font  = fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD);

  /* Title */
  s_title_layer = make_layer(root, GRect(0, y, w, 18),
                             title_font, GColorBlack, GColorWhite,
                             GTextAlignmentCenter);
  text_layer_set_text(s_title_layer, "Sensor Forward Test");
  y += 18;

  /* Accel XYZ */
  s_accel_layer = make_layer(root, GRect(0, y, w, 18),
                             body_font, GColorClear, GColorBlack,
                             GTextAlignmentLeft);
  text_layer_set_text(s_accel_layer, "A: -- / -- / --");
  y += 18;

  /* Magnitude */
  s_mag_layer = make_layer(root, GRect(0, y, w, 18),
                           body_font, GColorClear, GColorBlack,
                           GTextAlignmentLeft);
  text_layer_set_text(s_mag_layer, "|a|= waiting...");
  y += 18;

  /* Compass heading */
  s_compass_layer = make_layer(root, GRect(0, y, w, 18),
                               body_font, GColorClear, GColorBlack,
                               GTextAlignmentLeft);
  text_layer_set_text(s_compass_layer, "Hdg: waiting...");
  y += 18;

  /* Calibration */
  s_cal_layer = make_layer(root, GRect(0, y, w, 18),
                           body_font, GColorClear, GColorBlack,
                           GTextAlignmentLeft);
  text_layer_set_text(s_cal_layer, "Cal: -");
  y += 18;

  /* Status — bottom, larger font */
  s_status_layer = make_layer(root, GRect(0, bounds.size.h - 24, w, 24),
                              stat_font, GColorBlack, GColorWhite,
                              GTextAlignmentCenter);
  text_layer_set_text(s_status_layer, "Accel:WAIT  Compass:WAIT");
}

static void window_unload(Window *window) {
  text_layer_destroy(s_title_layer);
  text_layer_destroy(s_accel_layer);
  text_layer_destroy(s_mag_layer);
  text_layer_destroy(s_compass_layer);
  text_layer_destroy(s_cal_layer);
  text_layer_destroy(s_status_layer);
}

/* ── app init / deinit ───────────────────────────────────────────────────── */

static void init(void) {
  /* zero state */
  memset(&s_state, 0, sizeof(s_state));
  s_state.start_time = time(NULL);

  /* window */
  s_window = window_create();
  window_set_window_handlers(s_window, (WindowHandlers){
    .load   = window_load,
    .unload = window_unload,
  });
  window_stack_push(s_window, true);

  /*
   * Accelerometer
   * Request ACCEL_SAMPLING_10HZ so the emulator throttle (10 ms) lines up.
   * Batch 5 samples so we get a callback every 500 ms.
   */
  accel_data_service_subscribe(ACCEL_SAMPLES_PER_UPDATE, accel_handler);
  accel_service_set_sampling_rate(ACCEL_SAMPLING_10HZ);

  /* Compass */
  compass_service_subscribe(compass_handler);

  /* Timeout watchdog — fires every second */
  tick_timer_service_subscribe(SECOND_UNIT, tick_handler);

  APP_LOG(APP_LOG_LEVEL_INFO, "Sensor test app started. Timeout in %ds.", TIMEOUT_SECONDS);
}

static void deinit(void) {
  accel_data_service_unsubscribe();
  compass_service_unsubscribe();
  tick_timer_service_unsubscribe();
  window_destroy(s_window);
}

int main(void) {
  init();
  app_event_loop();
  deinit();
  return 0;
}