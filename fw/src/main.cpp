// Valve actuator firmware for the LilyGO TTGO T-Motor.
//
// Drives a NEMA17 through the onboard TMC2209: STEP/DIR for motion,
// single-wire UART for current, microstepping and chopper configuration.
// A serial console (115200 baud) moves the valve and tunes current/speed.

#include <Arduino.h>
#include <AccelStepper.h>
#include <TMCStepper.h>

#include "config.h"
#include "pins.h"

namespace {

constexpr uint8_t TMC2209_VERSION = 0x21;
constexpr uint32_t DRIVER_POLL_MS = 1000;

HardwareSerial TmcSerial(1);

// One instance per possible UART address, so the DIP switch setting can be
// detected at runtime.
TMC2209Stepper drivers[4] = {
    TMC2209Stepper(&TmcSerial, config::R_SENSE, 0b00),
    TMC2209Stepper(&TmcSerial, config::R_SENSE, 0b01),
    TMC2209Stepper(&TmcSerial, config::R_SENSE, 0b10),
    TMC2209Stepper(&TmcSerial, config::R_SENSE, 0b11),
};
TMC2209Stepper *driver = nullptr;

AccelStepper stepper(AccelStepper::DRIVER, pins::TMC_STEP, pins::TMC_DIR);

struct Settings {
  uint16_t run_current_ma;
  float hold_fraction;
  float max_speed_deg_s;
  float accel_deg_s2;
  bool spreadcycle;
  bool disable_when_idle;
};

Settings settings = {
    config::RUN_CURRENT_MA,
    config::HOLD_CURRENT_FRACTION,
    config::MAX_SPEED_DEG_S,
    config::ACCEL_DEG_S2,
    config::USE_SPREADCYCLE,
    config::DISABLE_WHEN_IDLE,
};

bool moving = false;
uint32_t last_driver_poll = 0;

// Debounced limit switch input. Only reported for now; motion does not stop
// on it until the switches are fitted.
struct LimitSwitch {
  const char *name;
  uint8_t pin;
  bool tripped;
  bool raw;
  uint32_t raw_since;
};

LimitSwitch limits[] = {
    {"limit 1", pins::LIMIT_1, false, false, 0},
    {"limit 2", pins::LIMIT_2, false, false, 0},
};
constexpr uint32_t LIMIT_DEBOUNCE_MS = 20;

constexpr float stepsPerDeg() {
  return config::FULL_STEPS_PER_REV * config::MICROSTEPS * config::GEAR_RATIO / 360.0f;
}

long degToSteps(float deg) { return lroundf(deg * stepsPerDeg()); }
float stepsToDeg(long steps) { return steps / stepsPerDeg(); }

// --- TMC2209 ----------------------------------------------------------------

bool driverResponds(TMC2209Stepper &d) { return d.version() == TMC2209_VERSION; }

TMC2209Stepper *findDriver() {
  if (config::DRIVER_ADDRESS >= 0) {
    TMC2209Stepper &d = drivers[config::DRIVER_ADDRESS & 0b11];
    return driverResponds(d) ? &d : nullptr;
  }
  for (auto &d : drivers) {
    if (driverResponds(d)) return &d;
  }
  return nullptr;
}

void applyCurrent() {
  uint16_t ma = min(settings.run_current_ma, config::MAX_RUN_CURRENT_MA);
  driver->rms_current(ma, settings.hold_fraction);
}

void applyDriverConfig() {
  driver->GSTAT(0b111);           // clear reset/error flags
  driver->push();                 // library defaults into every register
  driver->pdn_disable(true);      // PDN_UART is the UART, not power-down
  driver->mstep_reg_select(true); // microsteps from MRES, not MS1/MS2 pins
  driver->I_scale_analog(false);  // ignore the VREF pot, current set over UART
  driver->microsteps(config::MICROSTEPS);
  driver->intpol(true);
  driver->shaft(config::INVERT_DIRECTION);
  driver->en_spreadCycle(settings.spreadcycle);
  applyCurrent();
}

void connectDriver() {
  driver = findDriver();
  if (driver == nullptr) return;
  applyDriverConfig();
  int addr = driver - drivers;
  Serial.printf("TMC2209 found at UART address 0b%d%d, %u mA RMS\n", (addr >> 1) & 1, addr & 1,
                driver->rms_current());
}

// Called while idle: notice the driver appearing, vanishing (motor supply
// off) or resetting (supply brown-out wipes its registers).
void pollDriver() {
  if (driver == nullptr) {
    connectDriver();
    return;
  }
  if (!driverResponds(*driver)) {
    Serial.println("TMC2209 stopped responding (motor supply off?)");
    driver = nullptr;
    return;
  }
  uint8_t gstat = driver->GSTAT();
  if (gstat & 0b001) {
    Serial.println("TMC2209 reset detected, reapplying configuration");
    applyDriverConfig();
  } else if (gstat & 0b010) {
    Serial.printf("TMC2209 driver error, DRV_STATUS=0x%08lX\n",
                  (unsigned long)driver->DRV_STATUS());
    driver->GSTAT(0b010);
  }
}

// --- Motion -----------------------------------------------------------------

void applyMotion() {
  stepper.setMaxSpeed(settings.max_speed_deg_s * stepsPerDeg());
  stepper.setAcceleration(settings.accel_deg_s2 * stepsPerDeg());
}

void startMove(long target_steps) {
  if (driver == nullptr) {
    Serial.println("No TMC2209, check the 9-12 V motor supply");
    return;
  }
  stepper.enableOutputs();
  stepper.moveTo(target_steps);
  moving = true;
}

void updateMotion() {
  stepper.run();
  if (moving && stepper.distanceToGo() == 0) {
    moving = false;
    if (settings.disable_when_idle) stepper.disableOutputs();
    Serial.printf("At %.1f deg\n", stepsToDeg(stepper.currentPosition()));
  }
}

// --- Limit switches ---------------------------------------------------------

void initLimits() {
  for (auto &l : limits) {
    pinMode(l.pin, INPUT);
    l.raw = l.tripped = digitalRead(l.pin) == HIGH;
    l.raw_since = millis();
  }
  pinMode(pins::P1_SPARE, INPUT);
}

void updateLimits() {
  for (auto &l : limits) {
    bool raw = digitalRead(l.pin) == HIGH;
    if (raw != l.raw) {
      l.raw = raw;
      l.raw_since = millis();
    } else if (raw != l.tripped && millis() - l.raw_since >= LIMIT_DEBOUNCE_MS) {
      l.tripped = raw;
      Serial.printf("%s %s\n", l.name, l.tripped ? "open (at limit or wire broken)" : "closed");
    }
  }
}

// --- Console ----------------------------------------------------------------

void printHelp() {
  Serial.println(
      "Commands:\n"
      "  goto <deg>        move to absolute valve angle\n"
      "  move <deg>        move relative to current angle\n"
      "  zero              call the current angle 0\n"
      "  stop              decelerate and stop\n"
      "  current <mA>      run current, RMS (100 to " +
      String(config::MAX_RUN_CURRENT_MA) +
      ")\n"
      "  hold <0-1>        hold current as a fraction of run current\n"
      "  speed <deg/s>     max speed\n"
      "  accel <deg/s^2>   acceleration\n"
      "  mode spread|stealth\n"
      "  idle off|hold     cut power after moves, or keep holding\n"
      "  status");
}

void printStatus() {
  Serial.printf("position %.1f deg, target %.1f deg, %s\n",
                stepsToDeg(stepper.currentPosition()),
                stepsToDeg(stepper.targetPosition()), moving ? "moving" : "idle");
  Serial.printf("run %u mA, hold %.0f%%, speed %.1f deg/s, accel %.1f deg/s^2\n",
                settings.run_current_ma, settings.hold_fraction * 100,
                settings.max_speed_deg_s, settings.accel_deg_s2);
  Serial.printf("mode %s, idle %s\n", settings.spreadcycle ? "spreadCycle" : "stealthChop",
                settings.disable_when_idle ? "off" : "hold");
  Serial.printf("%s %s, %s %s\n", limits[0].name, limits[0].tripped ? "open" : "closed",
                limits[1].name, limits[1].tripped ? "open" : "closed");
  if (driver == nullptr) {
    Serial.println("driver: not found");
    return;
  }
  uint32_t drv_status = driver->DRV_STATUS();
  Serial.printf("driver: %u mA RMS, DRV_STATUS 0x%08lX%s%s%s\n", driver->rms_current(),
                (unsigned long)drv_status, (drv_status & (1 << 0)) ? " overtemp-warning" : "",
                (drv_status & (1 << 1)) ? " overtemp-shutdown" : "",
                (drv_status & (0b1111 << 2)) ? " short" : "");
}

bool parseFloat(const char *arg, float &out) {
  if (arg == nullptr) return false;
  char *end;
  out = strtof(arg, &end);
  return end != arg;
}

void handleCommand(char *line) {
  char *cmd = strtok(line, " \t");
  char *arg = strtok(nullptr, " \t");
  if (cmd == nullptr) return;
  float value;

  if (!strcmp(cmd, "goto") && parseFloat(arg, value)) {
    startMove(degToSteps(value));
  } else if (!strcmp(cmd, "move") && parseFloat(arg, value)) {
    startMove(stepper.targetPosition() + degToSteps(value));
  } else if (!strcmp(cmd, "zero")) {
    stepper.setCurrentPosition(0);
  } else if (!strcmp(cmd, "stop")) {
    stepper.stop();
  } else if (!strcmp(cmd, "current") && parseFloat(arg, value) && value >= 100) {
    settings.run_current_ma = min<uint16_t>(value, config::MAX_RUN_CURRENT_MA);
    if (driver) applyCurrent();
  } else if (!strcmp(cmd, "hold") && parseFloat(arg, value) && value >= 0 && value <= 1) {
    settings.hold_fraction = value;
    if (driver) applyCurrent();
  } else if (!strcmp(cmd, "speed") && parseFloat(arg, value) && value > 0) {
    settings.max_speed_deg_s = value;
    applyMotion();
  } else if (!strcmp(cmd, "accel") && parseFloat(arg, value) && value > 0) {
    settings.accel_deg_s2 = value;
    applyMotion();
  } else if (!strcmp(cmd, "mode") && arg && (!strcmp(arg, "spread") || !strcmp(arg, "stealth"))) {
    settings.spreadcycle = !strcmp(arg, "spread");
    if (driver) driver->en_spreadCycle(settings.spreadcycle);
  } else if (!strcmp(cmd, "idle") && arg && (!strcmp(arg, "off") || !strcmp(arg, "hold"))) {
    settings.disable_when_idle = !strcmp(arg, "off");
    if (!moving) {
      if (settings.disable_when_idle) stepper.disableOutputs();
      else if (driver) stepper.enableOutputs();
    }
  } else if (!strcmp(cmd, "status")) {
    printStatus();
    return;
  } else {
    printHelp();
    return;
  }
  Serial.println("ok");
}

void readConsole() {
  static char buf[64];
  static size_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (len == 0) continue;
      buf[len] = '\0';
      len = 0;
      handleCommand(buf);
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }
}

}  // namespace

void setup() {
  // Keep the motor unpowered until the driver is configured.
  pinMode(pins::TMC_EN, OUTPUT);
  digitalWrite(pins::TMC_EN, HIGH);

  pinMode(pins::TMC_CLK, OUTPUT);
  digitalWrite(pins::TMC_CLK, LOW);
  pinMode(pins::TMC_SPREAD, OUTPUT);
  digitalWrite(pins::TMC_SPREAD, LOW);

  initLimits();

  Serial.begin(115200);
  TmcSerial.begin(config::TMC_UART_BAUD, SERIAL_8N1, pins::TMC_UART_RX, pins::TMC_UART_TX);
  delay(100);
  Serial.println("\nthermostat valve actuator");

  stepper.setEnablePin(pins::TMC_EN);
  stepper.setPinsInverted(false, false, true);  // ENN is active low
  stepper.disableOutputs();
  applyMotion();

  connectDriver();
  if (driver == nullptr) Serial.println("No TMC2209 yet, check the 9-12 V motor supply");
  if (driver && !settings.disable_when_idle) stepper.enableOutputs();
  printHelp();
}

void loop() {
  updateMotion();
  updateLimits();
  readConsole();

  // UART register access blocks for a few ms, so only poll between moves.
  if (!moving && millis() - last_driver_poll >= DRIVER_POLL_MS) {
    last_driver_poll = millis();
    pollDriver();
  }
}
