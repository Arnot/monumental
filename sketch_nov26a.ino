#include <TMCStepper.h>

#include <Wire.h>
#include "ADS1X15.h"
#include <Adafruit_PWMServoDriver.h>

constexpr pin_size_t pin_chipSelA = 21; // Mot0
constexpr pin_size_t pin_chipSelB = 18; // Enc0
constexpr pin_size_t pin_chipSelC = 25; // Mot1
constexpr pin_size_t pin_chipSelD = 24; // Enc1

constexpr pin_size_t pin_SPI0SCK = 22;
constexpr pin_size_t pin_SPI0MOSI = 19;
constexpr pin_size_t pin_SPI0MISO = 20;

constexpr pin_size_t in1Pin = 1;
constexpr pin_size_t in2Pin = 0;

// Stepper Motor OConfig
constexpr uint16_t MOTOR_CURRENT = 4000;
constexpr uint8_t DEFAULT_IRUN = 31;
constexpr uint8_t DEFAULT_IHOLD = 16;
constexpr uint8_t TMC_TOFFRUN = 4;

#define SPEED_RPS 1.0
#define RATIO_ACCEL 5.0
#define RAIIO_DECEL 5.0

constexpr float FULL_STEPS_PER_REV = 200.0;
constexpr float MICROSTEP_FACTOR = 256.0;

constexpr float DEFAULT_VEL = (FULL_STEPS_PER_REV * MICROSTEP_FACTOR * SPEED_RPS);
constexpr float DEFAULT_ACCEL = (FULL_STEPS_PER_REV * MICROSTEP_FACTOR / RATIO_ACCEL);
constexpr float DEFAULT_DECEL = (FULL_STEPS_PER_REV * MICROSTEP_FACTOR / RAIIO_DECEL);

constexpr float RS = 0.05;

#define LINEAR_STEP_TIME 100
#define ROTATE_STEP_CW (MICROSTEP_FACTOR)
#define MIN_LINEAR_DELTA 2
#define MIN_LINEAR_DELAY 30
#define DELAY_DELTA_FACTOR 0.1f

TMC5160Stepper stepperDriver(pin_chipSelA, RS);

Adafruit_PWMServoDriver linearDriver(PCA9685_I2C_ADDRESS, Wire1);
ADS1015 ADS(0x48, &Wire1);

void setupMotor() {
  stepperDriver.setSPISpeed(1000000);
  stepperDriver.reset();
  stepperDriver.toff(0);
  stepperDriver.rms_current(MOTOR_CURRENT);
  stepperDriver.ihold(DEFAULT_IHOLD);
  stepperDriver.irun(DEFAULT_IRUN);
  stepperDriver.en_pwm_mode(false);
  stepperDriver.VSTOP(10);
  stepperDriver.RAMPMODE(0);
  stepperDriver.TZEROWAIT(0);

  stepperDriver.shaft(0);
  stepperDriver.en_softstop(0);
  stepperDriver.shortdelay(true);
  stepperDriver.shortfilter(2);
  //
  // Sets the internal motion profile —- see datasheet
  stepperDriver.v1(0); // Use Trapezoid Only, disable first ramps
  stepperDriver.a1(DEFAULT_ACCEL);
  stepperDriver.d1(DEFAULT_DECEL);
  stepperDriver.AMAX(DEFAULT_ACCEL);
  stepperDriver.VMAX(DEFAULT_VEL);
  stepperDriver.DMAX(DEFAULT_DECEL);

  stepperDriver.toff(TMC_TOFFRUN);
}

void setupI2C() {
  Wire1.setSDA(2);
  Wire1.setSCL(3);
  Wire1.begin();

  Serial.println("Wire 1 Begin");
  if (!ADS.begin())
    Serial.println("ADS Error");

  if (!linearDriver.begin())
    Serial.println("PWM Error");

  linearDriver.setPin(in1Pin, 0);
  linearDriver.setPin(in2Pin, 0);

  Serial.println("I2C Done");
}

void setup() {
  pinMode(pin_chipSelA, OUTPUT);
  SPI.setTX(pin_SPI0MOSI);
  SPI.setRX(pin_SPI0MISO);
  SPI.setSCK(pin_SPI0SCK);

  SPI.begin();

  Serial.begin(9600);
  while (!Serial); // Wait for USB monitor to open
  Serial.println("Online");

  setupMotor();
  setupI2C();

  if (stepperDriver.test_connection() != 0)
  {
    Serial.println("Driver not connected");
    while (1);
  }
}


int readPosition() {
  return ADS.readADC(1);
}

void linearRetract(uint16_t speed = 4095)
{
  linearDriver.setPin(in1Pin, speed);
  linearDriver.setPin(in2Pin, 0);
}

void linearExtend(uint16_t speed = 4095)
{
  linearDriver.setPin(in1Pin, 0);
  linearDriver.setPin(in2Pin, speed);
}

void linearStop() {
  linearDriver.setPin(in1Pin, 0);
  linearDriver.setPin(in2Pin, 0);
}

char readChar() {
  if (Serial.available() <= 0) {
    return '\0';
  }

  return Serial.read();
}

String readLine() {
  String result = String("");
  while (Serial.available() > 0) {
    delay(3); // delay to allow buffer to fill

    if (Serial.available() > 0) {
      char c = Serial.read();
      result += c;

      if (c == '\n') {
        break;
      }
    }
  }

  /* if (result.length() > 0) { */
  /*   Serial.print("Received "); Serial.println(result); */
  /* } */

  return result;
}

void rotate(int steps) {
  stepperDriver.XTARGET(stepperDriver.XACTUAL() + steps); // This is in Microsteps

  const unsigned long start = millis();
  const unsigned long timeout_ms = 3000; // 3 seconds, tweak as needed

  while (!stepperDriver.position_reached()) {
    if (millis() - start > timeout_ms) {
      break;
    }
    delay(50);
  }
}

void linearExtendStep() {
  linearExtendStep(LINEAR_STEP_TIME);
}

void linearRetractStep() {
  linearRetractStep(LINEAR_STEP_TIME);
}

void linearExtendStep(int delayTime) {
  linearExtend();
  delay(delayTime);
  linearStop();
}

void linearRetractStep(int delayTime) {
  linearRetract();
  delay(delayTime);
  linearStop();
}

void linearMoveTo(int where) {
    int currentPosition = readPosition();
    int delta = currentPosition - where;
    /* int lastDelta = 0; */

    int iterations = 0;
    int maxIterations = 200;
    while (abs(delta) > MIN_LINEAR_DELTA) {
        if (iterations++ > maxIterations) {
            break;
        }

        if (delta > 0) {
            linearRetract();
        } else {
            linearExtend();
        }

        delay(max(MIN_LINEAR_DELAY, (int)(DELAY_DELTA_FACTOR * (float) abs(delta))));
        linearStop();
        delta = readPosition() - where;

        /* if (abs(delta - lastDelta) < 1) { // basically not changing */
        /*     break; */
        /* } */
        /* lastDelta = delta; */
    }

    linearStop();
}

void processCommand(String cmd) {
  if (cmd.length() <= 1) {
    /* Serial.print(cmd); */
    /* Serial.println(" - Missing argument"); */
    return;
  }

  char action = cmd.charAt(0);
  int amount = cmd.substring(1, cmd.length()).toInt();

  switch (action) {
    case 'r':
    case 'R':
      rotate(amount * ROTATE_STEP_CW);
      break;
    case 'l':
    case 'L':
      linearMoveTo(amount);
      break;
    case 'p':
    case 'P':
      Serial.println(readPosition());
      break;
  }
}

void loop() {
  /* Serial.println(readPosition()); */
  String cmd = readLine();
  if (cmd.length() <= 1) {
    Serial.println("no cmd");
    delay(500);
  } else {
    processCommand(cmd);
  }
}
