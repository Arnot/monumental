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


// Example of using the functions to control things
void loop() {
  Serial.println(readPosition()); // Read "FL" Port
  linearRetract();
  delay(1000);
  linearExtend();
  delay(1000);
  linearStop();
  delay(1000);

  // Move 10 full steps from current position
  stepperDriver.XTARGET(stepperDriver.XACTUAL() + (MICROSTEP_FACTOR * 10) ); // This is in Microsteps
  // Wait until done
  while (!stepperDriver.position_reached()) {
    Serial.println("Moving");
    delay(1000);
  }
}
