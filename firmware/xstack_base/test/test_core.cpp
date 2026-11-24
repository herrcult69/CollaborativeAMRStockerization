// Host unit tests for xstack_core. Build: see firmware/xstack_base/README.md
#include <cassert>
#include <cmath>
#include <cstdio>
#include "../lib/xstack_core/diff_drive.h"
#include "../lib/xstack_core/lift.h"
#include "../lib/xstack_core/pid.h"
#include "../lib/xstack_core/watchdog.h"

using namespace xstack;
static int fails = 0;
#define CHECK_NEAR(a, b, tol) do { double _a=(a),_b=(b); if (std::fabs(_a-_b) > (tol)) { \
  std::printf("FAIL %s:%d  %s=%f expected %f\n", __FILE__, __LINE__, #a, _a, _b); ++fails; } } while (0)
#define CHECK(c) do { if (!(c)) { std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); ++fails; } } while (0)

static const DriveGeometry G{0.075f, 0.40f, 4096.0f};

static void test_twist_to_wheels() {
  WheelSpeeds s = twistToWheels(G, 0.3f, 0.0f, 1.0f);
  CHECK_NEAR(s.left_mps, 0.3, 1e-6); CHECK_NEAR(s.right_mps, 0.3, 1e-6);
  s = twistToWheels(G, 0.0f, 1.0f, 1.0f);           // spin left
  CHECK_NEAR(s.left_mps, -0.2, 1e-6); CHECK_NEAR(s.right_mps, 0.2, 1e-6);
  s = twistToWheels(G, 0.5f, 1.0f, 0.35f);          // saturate, keep ratio
  CHECK_NEAR(s.right_mps, 0.35, 1e-6);
  CHECK_NEAR(s.left_mps / s.right_mps, 0.3 / 0.7, 1e-5);
}

static int32_t ticksFor(float meters) {
  return (int32_t)std::lround(meters / (2 * M_PI * G.wheel_radius_m) * G.ticks_per_rev);
}

static void test_odom_straight_and_spin() {
  Odometry o(G);
  for (int i = 0; i < 100; ++i) o.update(ticksFor(0.01f), ticksFor(0.01f), 0.02f);
  CHECK_NEAR(o.x(), 1.0, 2e-3); CHECK_NEAR(o.y(), 0.0, 1e-6); CHECK_NEAR(o.theta(), 0.0, 1e-6);
  CHECK_NEAR(o.v(), 0.5, 5e-3);

  Odometry r(G);  // in-place rotation must give zero translation at drive_center
  const float arc = (float)(M_PI / 2) * 0.5f * G.wheel_separation_m;
  long sent = 0;
  for (int i = 1; i <= 50; ++i) { long t = ticksFor(arc * i / 50.0f); r.update(-(t - sent), t - sent, 0.02f); sent = t; }
  CHECK_NEAR(r.theta(), M_PI / 2, 2e-3); CHECK_NEAR(r.x(), 0.0, 1e-6); CHECK_NEAR(r.y(), 0.0, 1e-6);
}

static void test_odom_circle_closes() {
  Odometry o(G);
  const float R = 1.0f, L = 2 * (float)M_PI * R, N = 2000;
  const float half = 0.5f * G.wheel_separation_m;
  double accL = 0, accR = 0; long sentL = 0, sentR = 0;
  for (int i = 0; i < N; ++i) {      // carry rounding so ticks stay exact
    accL += (L / N) * (R - half) / R; accR += (L / N) * (R + half) / R;
    long tl = ticksFor(accL), tr = ticksFor(accR);
    o.update(tl - sentL, tr - sentR, 0.01f); sentL = tl; sentR = tr;
  }
  CHECK_NEAR(o.x(), 0.0, 5e-3); CHECK_NEAR(o.y(), 0.0, 5e-3);
}

static void test_pid_converges_and_clamps() {
  PidGains g; g.kp = 2.0f; g.ki = 5.0f; g.kff = 0.0f; g.out_min = -1; g.out_max = 1;
  Pid pid(g);
  float y = 0;                        // first-order plant: y' = (u*0.6 - y)/0.1
  for (int i = 0; i < 1000; ++i) { float u = pid.update(0.4f, y, 0.01f); y += (u * 0.6f - y) / 0.1f * 0.01f; }
  CHECK_NEAR(y, 0.4, 1e-3);
  Pid sat(g);                         // huge error -> clamped, integral bounded
  for (int i = 0; i < 1000; ++i) CHECK(std::fabs(sat.update(100.0f, 0.0f, 0.01f)) <= 1.0f);
  CHECK(sat.integral() < 2.0f);
}

static void test_lift() {
  LiftConfig c{0.0f, 0.60f, 0.08f, 0.0001f, 0.005f, -0.3f, 10.0f};
  PidGains g; g.kp = 8.0f; g.ki = 0.0f; g.out_min = -1; g.out_max = 1;
  Lift lift(c, g);
  CHECK(lift.update(0, false, 0.01f) == 0.0f);          // unhomed: motor off
  lift.startHoming();
  CHECK_NEAR(lift.update(500, false, 0.01f), -0.3, 1e-6);
  CHECK(lift.update(480, true, 0.01f) == 0.0f);
  CHECK(lift.state() == LiftState::kReady);
  lift.setTarget(5.0f); CHECK_NEAR(lift.target(), 0.60, 1e-6);  // clamped
  lift.setTarget(0.25f);
  long ticks = 480; float h = 0;
  for (int i = 0; i < 1000; ++i) {                      // ideal velocity plant
    float u = lift.update(ticks, false, 0.01f);
    h += u * 0.1f * 0.01f; ticks = 480 + std::lround(h / c.meters_per_tick);
  }
  CHECK(lift.atTarget()); CHECK_NEAR(lift.height(), 0.25, 0.005);
  lift.setTarget(0.0f);                                  // bottom switch blocks downward drive
  CHECK(lift.update(ticks, true, 0.01f) >= 0.0f);
}

static void test_watchdog() {
  CmdWatchdog w(500);
  CHECK(w.expired(0));
  w.feed(1000); CHECK(!w.expired(1400)); CHECK(w.expired(1501));
  w.feed(0xFFFFFF00u); CHECK(!w.expired(0x00000010u));  // millis() wrap
}

int main() {
  test_twist_to_wheels(); test_odom_straight_and_spin(); test_odom_circle_closes();
  test_pid_converges_and_clamps(); test_lift(); test_watchdog();
  std::printf(fails ? "%d FAILED\n" : "all tests passed\n", fails);
  return fails ? 1 : 0;
}
