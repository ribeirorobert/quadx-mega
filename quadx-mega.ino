/*    
  RobCopter v1.0 Firmware
  Created by Robert R. Gomes, May 17, 2023. 
  All rights reserved.
*/

/*libraries used*/
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>

#include "VL53L1X.h"
#include "Servo.h"

/*uncomment which type of multicopter will be used*/
#define QUAD_X
//#define TRICOPTER

/*uncomment which battery cell number will be used*/
//#define BATT_2_CELLS
#define BATT_3_CELLS
//#define BATT_4_CELLS

/*uncomment which sensors will be used*/
#define VL53L1x                 /*optical distance sensor*/
//#define BMP180                  /*barometric pressure sensor*/
//#define MS561101BA              /*barometric pressure sensor*/

///#define LONG_RANGE 
#define SHORT_RANGE 
//#define HIGH_ACCURACY 
//#define HIGH_SPEED

/*uncomment which rate loop will be used*/
//#define RATE_LOOP_250HZ
#define RATE_LOOP_500HZ

#define HARD_SOFT_PWM       0   /*1 - if dont use Servo.h library to run motors, 0 - otherwise*/

/*communication settings, do not change*/
#define SERIAL_BAUD_RATE    38400UL
#define I2C_CLOCK_SPEED     400000UL
#define I2C_TIMEOUT         4000UL

#define serialTelemetry     Serial1   

#define TELEMETRY_ENABLE    0   /*enable to use telemetry, not implemented yet*/
#define MPU6050_ENABLE      1   /*enable if you have the sensor on your board, disable GY87_ENABLE*/
#define SERIAL_ENABLE       1   /*enable serial communication*/
#define PRINTF_ENABLE       1   /*enable printf function*/
#define BMP180_ENABLE       0   /*enable if you have the sensor on your board*/
#define MS5611_ENABLE       0   /*enable if you have the sensor on your board*/
#define GY87_ENABLE         0   /*enable if you have the sensor on your board, disable MPU6050_ENABLE*/ 
                                /*GY-87 / HW-290 10DOF Accelerometer, Gyroscope, Magnetometer and Barometer Sensor*/

/*flight modes*/
#define ALT_HOLD_MODE       0   /*enable altitude hold mode*/
#define HOVER_MODE          1   /*enable hover mode*/

#define FAIL_SAFE           0   /*enable fail safe functions, not implemented yet*/

#define DEBUG_STATUS        0   /*enable to see copter initialize status logs*/
#define DEBUG_EEPROM        0   /*enable to debug EEPROM*/
#define DEBUG_IMU           0   /*enable to debug IMU*/
#define DEBUG_ESC           0   /*enable to debug ESCs calibration*/

/*sensors I2C addresses, do not change*/
#define MPU6050_ADDRESS     0x68
#define COMPASS_ADDRESS     0x1E
#define VL53L1X_ADDRESS     0x29
#define BMP180_ADDRESS      0x77
#define MS5611_ADDRESS      0x77
#define GYRO_ADDRESS        0x69
#define ACC_ADDRESS         0x53

/*frame LEDs pins, use the schematic no see another pins*/
#define LED_FRONT_LEFT      5
#define LED_FRONT_RIGHT     62
#define LED_REAR_LEFT       11
#define LED_REAR_RIGHT      48
#define VIRTUAL_GND         63

/*board LEDs pins, use the schematic no see another pins*/
#define LED_YELLOW_PIN      42
#define LED_GREEN_PIN       40
#define LED_BLUE_PIN        44
#define LED_RED_PIN         38
#define BUZZER_PIN          23

/*voltage measurement pin and 7seg display pins*/
#define BATTERY_PIN         A0
#define CLOCK_PIN           A14
#define LATCH_PIN           A10
#define DATA_PIN            A12 

/*time constants, change carefully if needed*/
#define WAITING_SAFETY_PERIOD       120000UL
#define CHANNEL_READ_PERIOD         100000UL
#define READ_BATTERY_PERIOD         2000000UL
#define PWM_SAFETY_PERIOD           2100UL
#define HOVER_CTRL_PERIOD           35000UL
#define TELEMETRY_PERIOD            1000000UL
#define BLINK_LED_PERIOD            1000000UL
#define DISP_7SEG_PERIOD            4000UL
#define BUZZER_PERIOD               1000000UL
#define LEDS_PERIOD                 500000UL

#if defined(RATE_LOOP_250HZ)
#define CTRL_LOOP_PERIOD            4000UL
#else
#define CTRL_LOOP_PERIOD            2000UL
#endif

#define MAX_RUNNING_TIME            900000UL


/*general constants, change carefully if needed*/
#define SENSITIVITY_SCALE_FACTOR    0.015267F     /*0.015267 = 1/65.5 [º/s]*/

#if defined(QUAD_X)
#define NUMBER_MOTORS     4
#elif defined(TRICOPTER)
#define NUMBER_MOTORS     3
#else
#define NUMBER_MOTORS     6
#endif

#if defined(BATT_2_CELLS)
#define BATTERY_UPPER_THRESHOLD     7.0F          /*max. battery voltage discharge*/
#define BATTERY_LOWER_THRESHOLD     6.0F          /*min. battery voltage*/
#define BATTERY_COMPENSATION        0.000222F     /*1/4500*/
#define BATTERY_LOWER_TRIGGER       6.0F          /*lower voltage to compensation*/
#define BATTERY_UPPER_TRIGGER       8.2F          /*upper voltage to compensation*/
#elif defined(BATT_3_CELLS)
#define BATTERY_UPPER_THRESHOLD     10.5F         /*max. battery voltage discharge*/
#define BATTERY_LOWER_THRESHOLD     6.0F          /*min. battery voltage*/
#define BATTERY_COMPENSATION        0.000571F     /*1/3500*/
#define BATTERY_LOWER_TRIGGER       8.0F          /*lower voltage to compensation*/
#define BATTERY_UPPER_TRIGGER       12.4F         /*upper voltage to compensation*/
#elif defined(BATT_4_CELLS)
#define BATTERY_UPPER_THRESHOLD     10.7F         /*max. battery voltage discharge*/
#define BATTERY_LOWER_THRESHOLD     6.0F          /*min. battery voltage*/
#define BATTERY_COMPENSATION        0.000571F     /*1/3500*/
#define BATTERY_LOWER_TRIGGER       10.0F         /*lower voltage to compensation*/
#define BATTERY_UPPER_TRIGGER       16.5F         /*upper voltage to compensation*/
#endif

#define LEVEL_ADJUST_FACTOR         13.0F         /*level adjust -> (stick travel-dead band)/angle setting = (500 - 10)/13 ~ 37°*/
#define DEGREES_TO_RADIAN           0.000001066F  /*0.000001066 = 0.0000611*(PI/180)*/
#define RADIAN_TO_DEGREES           57.295779F    /*57.295779 = 180/PI*/

#if defined(RATE_LOOP_250HZ)
#define ZERO_RATE_OUTPUT            0.0000610687F /*1/(250Hz/65.5) [°]*/
#else
#define ZERO_RATE_OUTPUT            0.0000305343F /*1/(500Hz/65.5) [°]*/
#endif

#define VOLTAGE_FACTOR              0.015285F     /*R2/R1+R2 = K = 0.319727, (5.0/1023) / K = 0.015285*/
#define HALF_LOOP_RATE              0.002F        /*sample time/2.0*/

#define FAILSAFE_THRESHOLD          990           /*pulse width to detect radio fail safe, not implemented yet*/
#define AUTO_LEVEL_MODE             1             /*enable auto level*/

/*copter status*/
#define COPTER_STATUS_UNDEFINED      0b00000000  
#define COPTER_RADIO_STARTUP_FAIL    0b00000001
#define COPTER_ESC_NOT_CALIBRATED    0b00000010
#define COPTER_EEPROM_READ_FAIL      0b00000100 
#define COPTER_GYRO_CALIBRATED       0b00001000
#define COPTER_ACC_CALIBRATED        0b00010000
#define COPTER_MPU6050_ERROR         0b00100000
#define COPTER_BMP180_ERROR          0b01000000
#define COPTER_MS5611_ERROR          0b01000000
#define COPTER_STATUS_OK             0b10000000

/*BMP180 registers*/
#if defined(BMP180)
#define BMP180_READ_TEMPERATURE     0x2E
#define BMP180_TEMPERATURE_DATA     0xF6
#define BMP180_READ_PRESSURE        0x34
#define BMP180_PRESSURE_DATA        0xF6
#define BMP180_AC1_ADDRESS          0xAA
#define BMP180_CONTROL              0xF4
#define BMP180_OSS                  0x03

#define BMP180_WAIT_UT_READ         6000UL
#define BMP180_WAIT_UP_READ         21000UL
#endif

/*MS561101BA registers*/
#if defined(MS561101BA)
#define MS5611_TEMPERATURE_ADDR     0x50
#define MS5611_PRESSURE_ADDR        0x40
#define MS5611_REG_RESET            0x1E
#define MS5611_OSR_4096             0x08

#define MS5611_WAIT_UT_UP_READ      10000UL
#endif

/*EEPROM addresses*/
#define GYROX_OFFSET_EEPROM_ADDR    0x00
#define GYROY_OFFSET_EEPROM_ADDR    0x04
#define GYROZ_OFFSET_EEPROM_ADDR    0x08
#define ACCX_OFFSET_EEPROM_ADDR     0x0C
#define ACCY_OFFSET_EEPROM_ADDR     0x10
#define ACCZ_OFFSET_EEPROM_ADDR     0x14
#define ESC_CAL_EEPROM_ADDR         0xFF

/*moving average indexes*/
#define TEMPERATURE_INDEX     5U
#define PARACHUTE_INDEX       30U
#define ALT_ERROR_INDEX       10U
#define PRESSURE_INDEX        20U

#define GYRO_OFFSET_INDEX     1000UL
#define ACC_OFFSET_INDEX      100U

#define ACCZ_SHORT_INDEX      25U
#define ACCZ_LONG_INDEX       50U
#define ACCZ_THRESHOLD        800UL

/*radio pulses width*/
#define STICK_INIT_VAL    1300UL
#define STICK_MIDPOINT    1500UL
#define STICK_UPPER_DB    1510UL
#define STICK_LOWER_DB    1490UL
#define MIN_THROTTLE      1000UL
#define MAX_THROTTLE      1800UL
#define MIN_THROTTLE      1000UL
#define LOCK_SPEED        1000UL   
#define IDLE_SPEED        1200UL 
#define FULL_SPEED        2000UL  
#define HOVER_FACTOR      1300UL

/*motors status*/
#define LOCK_THROTTLE     0x00
#define IDLE_THROTTLE     0x01
#define FULL_THROTTLE     0x02

/*general constants*/
#define MAX_PID_OUTPUT    400.0F
#define MAX_STICK_REF     150.0F


/*LEDs status*/
#define ALL_LEDS_OFF  0b00000000
#define ALL_LEDS_ON   0b00000001
#define LED_YELLOW    0b00000010
#define LED_GREEN     0b00000100
#define LED_BLUE      0b00001000
#define LED_RED       0b00010000
#define LED_FRONT     0b00100000
#define LED_REAR      0b01000000

/*PID controllers gains*/
#define NUMBER_CTRL   5

#define KP_ALT_HOLD   2.0F
#define KI_ALT_HOLD   0.4F
#define KD_ALT_HOLD   0.75F

#if defined(RATE_LOOP_250HZ)
#define KP_PITCH      1.2F
#define KI_PITCH      0.04F
#define KD_PITCH      20.0F

#define KP_ROLL       1.2F
#define KI_ROLL       0.04F
#define KD_ROLL       20.0F

#define KP_YAW        4.0F
#define KI_YAW        0.02F
#define KD_YAW        0.0F
#elif defined(RATE_LOOP_500HZ)
#define KP_PITCH      1.1F
#define KI_PITCH      0.02F
#define KD_PITCH      14.0F

#define KP_ROLL       1.1F
#define KI_ROLL       0.02F
#define KD_ROLL       14.0F

#define KP_YAW        4.0F
#define KI_YAW        0.02F
#define KD_YAW        0.0F

#define KP_HOVER      0.12F
#define KI_HOVER      0.01F
#define KD_HOVER      1.30F //1.83
#endif

/*structures*/
struct {
  uint8_t pageIndex = 1;
  uint8_t status;
  uint8_t motors;
  uint8_t leds;
  bool buzzer;
  float batteryVoltage;
} copter;

struct {
  union {int32_t val; uint8_t raw[4];} gyroXOffset;
  union {int32_t val; uint8_t raw[4];} gyroYOffset;
  union {int32_t val; uint8_t raw[4];} gyroZOffset;

  union {int32_t val; uint8_t raw[4];} accXOffset;
  union {int32_t val; uint8_t raw[4];} accYOffset;

  int32_t gyroX;
  int32_t gyroY;
  int32_t gyroZ;

  int16_t temp;

  int32_t accX;
  int32_t accY;
  int32_t accZ;
} IMU;

struct {
  float error[NUMBER_CTRL], prev_error[NUMBER_CTRL], sat_error[NUMBER_CTRL];
  float p_term[NUMBER_CTRL], i_term[NUMBER_CTRL], d_term[NUMBER_CTRL];
  float output[NUMBER_CTRL], prev_output[NUMBER_CTRL], presat_output[NUMBER_CTRL];
  float delta[NUMBER_CTRL], prev_delta[NUMBER_CTRL];
  float prev_feedback[NUMBER_CTRL];

  float kp[NUMBER_CTRL] = {KP_ROLL, KP_PITCH, KP_YAW, KP_HOVER, KP_ALT_HOLD};
  float ki[NUMBER_CTRL] = {KI_ROLL, KI_PITCH, KI_YAW, KI_HOVER, KI_ALT_HOLD};
  float kd[NUMBER_CTRL] = {KD_ROLL, KD_PITCH, KD_YAW, KD_HOVER, KD_ALT_HOLD};
  float ku[NUMBER_CTRL] = {0.8, 0.8, 0.8, 0.8, 0.5};
  const float sample_time = 1.0;
  const float gain_factor = 1.0;
  const float tau = 0.001;

  int16_t lim_max_error[NUMBER_CTRL] = { 100,  100,  100,  150,  100};
  int16_t lim_min_error[NUMBER_CTRL] = {-100, -100, -100, -150, -100};

  const int16_t lim_max_integr =  300;
  const int16_t lim_min_integr = -300;

  const int16_t lim_max_output[NUMBER_CTRL] = { 400,  400,  400,  400,  400};
  const int16_t lim_min_output[NUMBER_CTRL] = {-400, -400, -400, -400, -400};
} PID;

#if defined(BMP180)
struct {
  int16_t ac1;
  int16_t ac2;
  int16_t ac3;
  uint16_t ac4;
  uint16_t ac5;
  uint16_t ac6;
  int16_t b1;
  int16_t b2;
  int16_t mb;
  int16_t mc;
  int16_t md;

  union {
    int16_t val;
    int8_t raw[2];
  } UT;

  union {
    int32_t val;
    int8_t raw[4];
  } UP;

  int32_t temperature;
  int32_t pressure;

  unsigned long timeOut;
  bool status;
} BMP180;
#endif

#if defined(MS561101BA) 
struct {
  uint16_t c[7];

  union {
    int32_t val;
    int8_t raw[4];
  } ut;

  union {
    uint32_t val;
    int8_t raw[4];
  } up;

  int32_t temperature;
  int32_t pressure;

  unsigned long timeOut;
  bool status;
} MS5611;
#endif

/*general arrays*/
const uint8_t numbers[10] = {
  0b00000011, 0b10011111, 0b00100101, 0b00001101, 0b10011001, 
  0b01001001, 0b01000001, 0b00011111, 0b00000001, 0b00001001
};

const uint8_t fail[5] = {
  0b01110001, 0b00010001, 0b10011111, 0b11100011, 0b11111111
};

const uint8_t eeprom[10] = {
  0b01100001, 0b01100001, 0b00110001, 0b11110101, 0b11000101, 
  0b11010101, 0b11111111, 0b11111111, 0b11111111, 0b11111111
};

const uint8_t mpu[11] = {
  0b11010101, 0b00110001, 0b11000111, 0b01000001, 0b00000011, 
  0b01001001, 0b00000011, 0b11111111, 0b11111111, 0b11111111, 
  0b11111111
};

const uint8_t bmp[10] = {
  0b11000001, 0b11010101, 0b00110001, 0b10011111, 0b00000001, 
  0b00000011, 0b11111111, 0b11111111, 0b11111111, 0b11111111
};

const uint8_t ms[11] = {
  0b11010101, 0b01001001, 0b11111111, 0b01001001, 0b01000001, 
  0b10011111, 0b10011111, 0b11111111, 0b11111111, 0b11111111, 
  0b11111111
};

const uint8_t radio[9] = {
  0b11110101, 0b00010001, 0b10000101, 0b11011111, 0b11000101, 
  0b11111111, 0b11111111, 0b11111111, 0b11111111
};

const uint8_t esc[7] = {
  0b01100001, 0b01001001, 0b01100011, 0b11111111, 0b11111111, 
  0b11111111, 0b11111111
};

uint8_t msg[6*sizeof(fail)+sizeof(eeprom)+sizeof(mpu)+sizeof(bmp)+sizeof(ms)+sizeof(radio)+sizeof(esc)];

/*status variables*/
bool statusCalibrationEsc = false;
bool statusManualAltitude = false;
bool statusAltitudeHold = false;

uint8_t counterFailSafe = 0;
uint8_t muxIndex = 0;

volatile uint16_t channel[9];

/*throttle variables*/
uint16_t mainThrottle, initialThrottle, hoverThrottle;
float adjustThrottle;
float manualThrottle;

/*motors*/
int16_t motor[8]; /*six motor + two servo*/
/*using Servo.h library to run motors*/
Servo pwm_motor[8]; /*six motor + two servo*/

/*roll, pitch and yaw variables*/
float gyroRollInput, gyroPitchInput, gyroYawInput;
float rollAngle, pitchAngle;

float rollLevelAdjust, pitchLevelAdjust;
float rollAngleAcc, pitchAngleAcc;

/*roll, pitch and yaw ctrl variables*/
float rollSetpoint, pitchSetpoint, yawSetpoint;

float errorRoll, errorPitch, errorYaw;
float pTermRoll, pTermPitch, pTermYaw;
float iTermRoll, iTermPitch, iTermYaw;
float dTermRoll, dTermPitch, dTermYaw;

float errorRollAnt, errorPitchAnt, errorYawAnt;
float pidRoll, pidPitch, pidYaw;

/*altitude hold ctrl variables*/
float altHoldSetPoint, errorAltHold, pidAltHold;
float pTermAltHold, iTermAltHold, dTermAltHold;

/*altitude hold variables*/
uint8_t altitudeErrorIndex = 0;
uint8_t temperatureIndex = 0;
uint8_t parachuteIndex = 0;
uint8_t pressureIndex = 0;

int32_t rawTemperature;
int32_t parachuteThrottle;

int32_t temperatureSum;
int32_t pressureSum;

int32_t temperatureBuffer[TEMPERATURE_INDEX];
int32_t parachuteBuffer[PARACHUTE_INDEX];
int32_t pressureBuffer[PRESSURE_INDEX];

float altitudeErrorBuffer[ALT_ERROR_INDEX];

float altitudeErrorSum;
float altitudeError;

float actualPressure, groundPressure;
float actualPressureFast, actualPressureSlow;
float actualPressureDiff;

float previousParachutePressure;

/*time variables*/
unsigned long previousReadVoltagePeriod;
unsigned long previousIgnoreCmdsPeriod;
unsigned long previousCtrlLoopPeriod;
unsigned long previousBuzzerPeriod;
unsigned long previousBlinkPeriod;
unsigned long startCopterTimer;

int16_t tiltMiddle = 1500;
int16_t tiltOffset = 0;


bool statusHover = false;
float filteredDistance, measure_0;
float targetDistance = 200;
uint16_t hoverCounter = 0; 

float hoverFactor = 0;
float otoNewSet = 0;

VL53L1X sensor;

void setup() {
  /*initialize serial debug port*/
#if SERIAL_ENABLE  
  Serial.begin(SERIAL_BAUD_RATE);
#endif

  /*telemetry serial port*/
#if TELEMETRY_ENABLE
  serialTelemetry.begin(9600); //Rx, Tx
#endif

  /*initialize I2C communication*/
  Wire.begin();
  Wire.setClock(I2C_CLOCK_SPEED); /*TWBR = 12; set the I2C clock speed to 400kHz*/

  /*inputs and outputs*/
  pinMode(BATTERY_PIN, INPUT);

  pinMode(LED_FRONT_LEFT,  OUTPUT);
  pinMode(LED_FRONT_RIGHT, OUTPUT);
  pinMode(LED_REAR_LEFT,   OUTPUT);
  pinMode(LED_REAR_RIGHT,  OUTPUT);
  pinMode(VIRTUAL_GND,     OUTPUT);

  pinMode(LED_YELLOW_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN,  OUTPUT); 
  pinMode(LED_BLUE_PIN,   OUTPUT); 
  pinMode(LED_RED_PIN,    OUTPUT);
  pinMode(BUZZER_PIN,     OUTPUT);
  
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(DATA_PIN,  OUTPUT);

  /*virtual GND*/
  digitalWrite(VIRTUAL_GND, LOW);

#if HARD_SOFT_PWM
  /*motors PWM pins, chance carefully if needed*/
  /*functions used -> copter_init() and write_pwm()*/
  DDRE |= bit(DDE5); /*data direction register PORTE.5 (motor 1)*/
  DDRB |= bit(DDB7); /*data direction register PORTB.7 (motor 2)*/

  DDRH |= bit(DDH4); /*data direction register PORTH.4 (motor 3)*/
  DDRH |= bit(DDH6); /*data direction register PORTH.6 (motor 4)*/
#if defined(TRICOPTER)
  DDRB |= bit(DDB3); /*data direction register PORTB.3 (tilt servo)*/
#endif
#else
#if defined(QUAD_X)
  pwm_motor[0].attach(3,  1000, 2000);
  pwm_motor[1].attach(13, 1000, 2000);
  pwm_motor[2].attach(7,  1000, 2000);
  pwm_motor[3].attach(9,  1000, 2000);

  pwm_motor[0].writeMicroseconds(LOCK_SPEED);
  pwm_motor[1].writeMicroseconds(LOCK_SPEED);
  pwm_motor[2].writeMicroseconds(LOCK_SPEED);
  pwm_motor[3].writeMicroseconds(LOCK_SPEED);
#elif defined(TRICOPTER)
  pwm_motor[0].attach(3,  1000, 2000);
  pwm_motor[1].attach(13, 1000, 2000);
  pwm_motor[2].attach(7,  1000, 2000);
  pwm_motor[3].attach(50, 1000, 2000);

  pwm_motor[0].writeMicroseconds(LOCK_SPEED);
  pwm_motor[1].writeMicroseconds(LOCK_SPEED);
  pwm_motor[2].writeMicroseconds(LOCK_SPEED);
  pwm_motor[3].writeMicroseconds(tiltMiddle);
#endif
#endif

  /*input PPM signal pin*/
  PCMSK0 |= bit(PCINT2); /*want pin B2*/
  PCIFR  |= bit(PCIF0);  /*clear any outstanding interrupts*/
  PCICR  |= bit(PCIE0);  /*enable pin change interrupts PCMSK0*/

  /*blink LEDs*/
  //blink_leds();

  /*copter init*/
  copter_init();

  /*update timers*/
  previousReadVoltagePeriod = micros();
  previousCtrlLoopPeriod = micros();
  previousBuzzerPeriod = micros();
  previousBlinkPeriod = micros();
  startCopterTimer = millis();
}

void loop() {
  /*ctrl loop period (4ms=250Hz), do not change*/
  if (micros() - previousCtrlLoopPeriod >= CTRL_LOOP_PERIOD) { 
    previousCtrlLoopPeriod = micros();

    static uint8_t taskOrder = 0;
    (taskOrder >= 7) ? taskOrder = 0 : taskOrder ++;

    switch (taskOrder) {
      case 0: { /*hover, not work yet*/
#if HOVER_MODE
        hover_mode();
#endif
      };
      break;
 
      case 1: { /*read pressure and temperature, do not change*/
#if BMP180_ENABLE    
        BMP180_update();
#elif MS5611_ENABLE    
        MS5611_update();
#endif  
      };
      break;

      case 2: { /*altitude hold, not work yet*/
#if ALT_HOLD_MODE  
        altitude_hold(3);
#endif  
      }
      break;

      case 3: { /*read battery voltage, change carefully if needed*/
        read_battery_voltage(); 
      }
      break;

      case 4: { /*check alarms*/
        check_alarms();
      }
      break;

      case 5: { /*show on display the copter status*/
        show_display_status();
      }
      break;

      case 6: { /*print variables on serial monitor, keep disable when fly, change if needed*/
#if PRINTF_ENABLE
        //print_parameters(8); //copter.pageIndex
#endif
      }
      break;

      case 7: { /*telemetry data, change if needed*/
#if TELEMETRY_ENABLE
        update_telemetry_data();
#endif
      }
      break;
    }

    /*menu: calibrate IMU, calibrate ESCs, start/stop motors, change carefully if needed*/
    menu_functions();

    /*read IMU data, do not change*/
    IMU_compute();

    /*accelerometer and gyroscope Euler angles, do not change*/
    euler_angles();

    /*roll, pitch and yaw PID controllers, change carefully if needed*/
    run_controllers();

    /*runs motors, change carefully if needed*/
    run_motors();  
  }

}

/*******************************************************
  copter status
/*******************************************************/
void copter_init(void) {
#if DEBUG_STATUS
  Serial.println(F("COPTER STARTING..."));
#endif

  /*copter default status*/
  copter.status |= COPTER_STATUS_UNDEFINED;
  copter.motors = LOCK_THROTTLE;
  copter.leds = ALL_LEDS_OFF;
  copter.buzzer = false;

  /*read EEPROM data*/
#if DEBUG_STATUS
  Serial.print(F("->READING EEPROM: "));
#endif
  
  if (!read_eeprom()) {
#if DEBUG_STATUS
    Serial.println(F("ERROR"));
#endif
    copter.status |= COPTER_EEPROM_READ_FAIL;
    fail_message();
  } else {
#if DEBUG_STATUS
    Serial.println(F("OK"));
#endif
    copter.status |= COPTER_GYRO_CALIBRATED;
    copter.status |= COPTER_ACC_CALIBRATED;
  }

  /*ESCs calibration*/
#if DEBUG_STATUS
  Serial.print(F("->ESC CALIBRATION: "));
#endif

  if (statusCalibrationEsc) {
    if (!calibrate_escs()) {
#if DEBUG_STATUS
      Serial.println(F("ERROR"));
#endif
      copter.status |= COPTER_ESC_NOT_CALIBRATED;
    } else {
#if DEBUG_STATUS
      Serial.println(F("OK"));
#endif
    }

    EEPROM.write(ESC_CAL_EEPROM_ADDR, 0);
  } else {
#if DEBUG_STATUS
    Serial.println(F("SKIPED"));
#endif
  }

  /*check if IMU is connected*/
#if DEBUG_STATUS
  Serial.print(F("->CHECKING IMU: "));
#endif
  
  for (uint8_t i = 0; i < 10; i ++) {
    Wire.beginTransmission(MPU6050_ADDRESS);
    uint8_t status = Wire.endTransmission();

    if (status != 0) {
#if DEBUG_STATUS
      Serial.println(F("ERROR"));
#endif
      copter.status |= COPTER_MPU6050_ERROR;
    } else {
#if DEBUG_STATUS
      Serial.println(F("OK"));
#endif
      IMU_init(); //IMU configuration
      copter.status &= ~COPTER_MPU6050_ERROR;
      break;
    }
    delay(500);
  }

  /*check if BMP180 is connected*/
#if defined(BMP180)  
#if DEBUG_STATUS
  Serial.print(F("->CHECKING BMP180: "));
#endif

  for (uint8_t i = 0; i < 10; i ++) {
    Wire.beginTransmission(BMP180_ADDRESS);
    uint8_t status = Wire.endTransmission();

    if (status != 0) {
#if DEBUG_STATUS
      Serial.println(F("ERROR"));
#endif
      copter.status |= COPTER_BMP180_ERROR;
    } else {
#if DEBUG_STATUS
      Serial.println(F("OK"));
#endif      
      BMP180_init();
      copter.status &= ~COPTER_BMP180_ERROR;
      break;
    }
    delay(500);
  }

  //stablish barometer
  for (uint8_t i = 0; i < 100; i ++) {    
    BMP180_update();                   
    altitude_hold(2);                                           
    delayMicroseconds(4000);                                                   
  }
  actualPressure = 0; 
#endif

  /*check if MS561101BA is connected*/
#if defined(MS561101BA)
#if DEBUG_STATUS
  Serial.print(F("->CHECKING MS561101BA: "));
#endif

  for (uint8_t i = 0; i < 10; i ++) {
    Wire.beginTransmission(MS5611_ADDRESS);
    uint8_t status = Wire.endTransmission();

    if (status != 0) {
#if DEBUG_STATUS
      Serial.println(F("ERROR"));
#endif
      copter.status |= COPTER_MS5611_ERROR;
    } else {
#if DEBUG_STATUS
      Serial.println(F("OK"));
#endif      
      MS5611_init();
      copter.status &= ~COPTER_MS5611_ERROR;
      break;
    }
    delay(500);
  }
#endif

  /*check if VL53L1X is connected*/
#if defined(VL53L1x) && defined(HOVER_MODE)
#if DEBUG_STATUS
  Serial.print(F("->CHECKING VL53L1X: "));
#endif

  for (uint8_t i = 0; i < 10; i ++) {
    Wire.beginTransmission(VL53L1X_ADDRESS);
    uint8_t status = Wire.endTransmission();

    if (status != 0) {
#if DEBUG_STATUS
      Serial.println(F("ERROR"));
#endif
      //copter.status |= COPTER_MS5611_ERROR;
    } else {
#if DEBUG_STATUS
      Serial.println(F("OK"));
#endif      
      VL53L1X_init();
      //copter.status &= ~COPTER_MS5611_ERROR;
      break;
    }
    delay(500);
  }
#endif

  /*wait radio signals*/ 
#if DEBUG_STATUS  
  Serial.print(F("->WAITING RADIO SIGNALS: "));
#endif

  if (!wait_radio_start()) {
#if DEBUG_STATUS
    Serial.println(F("ERROR"));
#endif
    copter.status |= COPTER_RADIO_STARTUP_FAIL;
    fail_message();
  } else {
#if DEBUG_STATUS
    Serial.println(F("OK"));
#endif
  }

  /*bip motors*/
#if DEBUG_STATUS
  Serial.println(F("->BIP MOTORS"));
#endif

  for (uint16_t i = 0; i < 100; i ++) {
    PORTE |= (1 << PORTE5); /*motor 1*/
    PORTB |= (1 << PORTB7); /*motor 2*/
    PORTH |= (1 << PORTH4); /*motor 3*/
    PORTH |= (1 << PORTH6); /*motor 4*/
    delayMicroseconds(1000);

    PORTE &= ~(1 << PORTE5); /*motor 1*/
    PORTB &= ~(1 << PORTB7); /*motor 2*/
    PORTH &= ~(1 << PORTH4); /*motor 3*/
    PORTH &= ~(1 << PORTH6); /*motor 4*/
    delayMicroseconds(3000);

    update_display(0b11000001, 0b10011111, 0b00110001, 0b11111111);
  }

  /*error message, EEPROM*/
  if (copter.status & COPTER_EEPROM_READ_FAIL) {
    for (uint8_t i = 0; i < sizeof(fail); i ++) 
      msg[i] = fail[i];
    muxIndex += sizeof(fail);

    for (uint8_t i = muxIndex, k = 0; i < muxIndex + sizeof(eeprom); i ++, k ++) 
      msg[i] = eeprom[k];
    muxIndex += sizeof(eeprom);
  }

  /*error message, MPU6050*/
  if (copter.status & COPTER_MPU6050_ERROR) {
    for (uint8_t i = muxIndex, k = 0; i < muxIndex + sizeof(fail); i ++, k ++) 
      msg[i] = fail[k];
    muxIndex += sizeof(fail);

    for (uint8_t i = muxIndex, k = 0; i < muxIndex + sizeof(mpu); i ++, k ++) 
      msg[i] = mpu[k];
    muxIndex += sizeof(mpu);
  }

#if defined(BMP180)
  /*error message, BMP180*/
  if (copter.status & COPTER_BMP180_ERROR) {
    for (uint8_t i = muxIndex, k = 0; i < muxIndex + sizeof(fail); i ++, k ++) 
      msg[i] = fail[k];
    muxIndex += sizeof(fail);

    for (uint8_t i = muxIndex, k = 0; i < muxIndex + sizeof(bmp); i ++, k ++) 
      msg[i] = bmp[k];
    muxIndex += sizeof(bmp);
  }
#endif

#if defined(MS561101BA)
  /*error message, MS561101BA*/
  if (copter.status & COPTER_MS5611_ERROR) {
    for (uint8_t i = muxIndex, k = 0; i < muxIndex + sizeof(fail); i ++, k ++) 
      msg[i] = fail[k];
    muxIndex += sizeof(fail);

    for (uint8_t i = muxIndex, k = 0; i < muxIndex + sizeof(ms); i ++, k ++) 
      msg[i] = ms[k];
    muxIndex += sizeof(ms);
  }
#endif

  /*error message, radio not start*/
  if (copter.status & COPTER_RADIO_STARTUP_FAIL) {
    for (uint8_t i = muxIndex, k = 0; i < muxIndex + sizeof(fail); i ++, k ++) 
      msg[i] = fail[k];
    muxIndex += sizeof(fail);

    for (uint8_t i = muxIndex, k = 0; i < muxIndex + sizeof(radio); i ++, k ++) 
      msg[i] = radio[k];
    muxIndex += sizeof(radio);
  }

  /*error message, ESCs calibration*/
  if (copter.status & COPTER_ESC_NOT_CALIBRATED) {
    for (uint8_t i = muxIndex, k = 0; i < muxIndex + sizeof(fail); i ++, k ++) 
      msg[i] = fail[k];
    muxIndex += sizeof(fail);

    for (uint8_t i = muxIndex, k = 0; i < muxIndex + sizeof(esc); i ++, k ++) 
      msg[i] = esc[k];
    muxIndex += sizeof(esc);
  }

  if (muxIndex == 0) {
#if DEBUG_STATUS
    Serial.println(F("RESULT: SUCESS"));
#endif
    copter.status |= COPTER_STATUS_OK;
  } else {
#if DEBUG_STATUS
    Serial.println(F("RESULT: ERROR"));
#endif
  }
  
  /*turn off LEDs*/
  copter.leds &= ~LED_YELLOW & ~LED_FRONT & ~LED_REAR;

  /*read battery voltage*/
  copter.batteryVoltage = analogRead(BATTERY_PIN) * VOLTAGE_FACTOR;
}

/*******************************************************
  IMU configuration
/*******************************************************/
void IMU_init(void) {
  Wire.beginTransmission(MPU6050_ADDRESS);     
  Wire.write(0x6B); /*PWR_MGMT_1*/
  Wire.write(0x00); /*DEVICE_RESET; SLEEP; CYCLE; TEMP_DIS; CLKSEL (00000000, all disable)*/
  Wire.endTransmission();      

  Wire.beginTransmission(MPU6050_ADDRESS);     
  Wire.write(0x1A); /*CONFIG*/
  Wire.write(0x03); /*EXT_SYNC_SET (000, FSYNC input disable); DLPF_CFG (011, ACC bandwidth = 44Hz and GYRO bandwidth = 42Hz)*/
  Wire.endTransmission();         

  Wire.beginTransmission(MPU6050_ADDRESS);     
  Wire.write(0x1B); /*GYRO_CONFIG*/
  Wire.write(0x08); /*XG_ST,YG_ST,ZG_ST (000, disable self test); FS_SEL (1, 01, +- 500°/s [65.5 LSB/°/s] full scale range); ---;*/
  Wire.endTransmission();  

  Wire.beginTransmission(MPU6050_ADDRESS);     
  Wire.write(0x1C); /*ACCEL_CONFIG*/
  Wire.write(0x10); /*XA_ST,YA_ST,ZA_ST (000, disable self test); AFS_SEL (2, 10, +- 8g [4096 LSB/g] full scale range); ---;*/
  Wire.endTransmission();                  
}

/*******************************************************
  read IMU data (accelerometer, gyroscope, subtract offset)
/*******************************************************/
bool IMU_compute(void) {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x3B);                      /*start reading register 43h and auto increment with every read*/
  Wire.endTransmission();
  Wire.requestFrom(MPU6050_ADDRESS, 14); /*request 14 bytes from the MPU6050*/

  if (Wire.available() != 14) return false;

  IMU.accX  = (Wire.read() << 8 | Wire.read());      
  IMU.accY  = (Wire.read() << 8 | Wire.read());  
  IMU.accZ  = (Wire.read() << 8 | Wire.read());     
  IMU.temp  = (Wire.read() << 8 | Wire.read());      
  IMU.gyroX = (Wire.read() << 8 | Wire.read()); 
  IMU.gyroY = (Wire.read() << 8 | Wire.read()); 
  IMU.gyroZ = (Wire.read() << 8 | Wire.read()); 

  /*invert the direction of the axis*/
  IMU.gyroX *= -1.0;
  IMU.gyroY *=  1.0;
  IMU.gyroZ *= -1.0;

  /*subtract gyroscope calibration value*/
  IMU.gyroX -= IMU.gyroXOffset.val;
  IMU.gyroY -= IMU.gyroYOffset.val;
  IMU.gyroZ -= IMU.gyroZOffset.val;

  /*subtract accelerometer calibration value*/
  IMU.accX -= IMU.accXOffset.val;
  IMU.accY -= IMU.accYOffset.val;

  return true;
}

/*******************************************************
  Euler angles
/*******************************************************/
void euler_angles(void) {
  /*gyro angles filtered*/
  gyroRollInput  = (gyroRollInput  * 0.7) + (IMU.gyroX * SENSITIVITY_SCALE_FACTOR * 0.3);
  gyroPitchInput = (gyroPitchInput * 0.7) + (IMU.gyroY * SENSITIVITY_SCALE_FACTOR * 0.3);
  gyroYawInput   = (gyroYawInput   * 0.7) + (IMU.gyroZ * SENSITIVITY_SCALE_FACTOR * 0.3);

  /*gyro angles calculation*/
  rollAngle  += (IMU.gyroX * ZERO_RATE_OUTPUT); 
  pitchAngle += (IMU.gyroY * ZERO_RATE_OUTPUT);

  pitchAngle -= rollAngle  * sin(IMU.gyroZ * DEGREES_TO_RADIAN); /*if the IMU has yawed transfer the pitch angle to the roll angel*/
  rollAngle  += pitchAngle * sin(IMU.gyroZ * DEGREES_TO_RADIAN); /*if the IMU has yawed transfer the roll angle to the pitch angel*/

  /*accelerometer angles calculations*/
  float accTotalVector = sqrt((IMU.accX * IMU.accX) + (IMU.accY * IMU.accY) + (IMU.accZ * IMU.accZ));
  if (abs(IMU.accY) < accTotalVector) {
    pitchAngleAcc = asin((float)IMU.accY / accTotalVector) * RADIAN_TO_DEGREES;
  }
  if (abs(IMU.accX) < accTotalVector) {
    rollAngleAcc = asin((float)IMU.accX / accTotalVector) * RADIAN_TO_DEGREES * (-1); 
  }

  rollAngle  = (rollAngle  * 0.9996) + (rollAngleAcc  * 0.0004); /*correct the drift of the gyro roll angle with the accelerometer roll angle*/
  pitchAngle = (pitchAngle * 0.9996) + (pitchAngleAcc * 0.0004); /*correct the drift of the gyro pitch angle with the accelerometer pitch angle*/

  /*calculate roll and pitch angles correction*/
#if AUTO_LEVEL_MODE  
  rollLevelAdjust  = rollAngle  * (float)LEVEL_ADJUST_FACTOR;
  pitchLevelAdjust = pitchAngle * (float)LEVEL_ADJUST_FACTOR;
#else
  rollLevelAdjust  = 0;
  pitchLevelAdjust = 0;
#endif
}

/*******************************************************
  read commands from radio(take off, calibration, ...)
/*******************************************************/
bool menu_functions(void) {
  if ((copter.status & COPTER_RADIO_STARTUP_FAIL) || (millis() < 2000)) return false;

  /*debug control*/
  // if (copter.motors == LOCK_THROTTLE) {
  //   if (channel[1] > 1900) { /*next debug page*/
  //     while (channel[1] > 1900) waiting_knob_release();
  //     copter.pageIndex ++;
  //   }
    
  //   if (channel[1] < 1100) { /*previous debug page*/
  //     while (channel[1] < 1100) waiting_knob_release();
  //     copter.pageIndex --;
  //   }

  //   copter.pageIndex = constrain(copter.pageIndex, 1, 8);
  // }
  
  /*frame LEDs switch*/
  if (channel[5] > 1600) {
    copter.leds |= LED_YELLOW | LED_FRONT | LED_REAR;
  } else {
    copter.leds &= ~LED_YELLOW & ~LED_FRONT & ~LED_REAR;
  }

  /*flight modes*/
  if (copter.motors == LOCK_THROTTLE) {
    /*altitude hold*/
#if ALT_HOLD_MODE
    if (channel[8] > 1600) {
      statusAltitudeHold = true; 
    } else {
      statusAltitudeHold = false; 
    }
#endif

    /*hover*/
#if HOVER_MODE
    if (channel[8] > 1600) {
      statusHover = true; 
    } else {
      statusHover = false; 
    }
#endif
  }

  /*calibration options*/
  if (copter.motors == LOCK_THROTTLE) {
    /*gyroscope calibration*/
    if ((channel[1] > 1900) && (channel[2] > 1900)) {
      while ((channel[1] > 1900) && (channel[2] > 1900)) waiting_knob_release();

      while (true) {
        if ((channel[1] > 1900) && (channel[2] > 1900)) {
          if (calibrate_gyroscope()) {
            copter.status |= COPTER_GYRO_CALIBRATED;
          } else {
            copter.status &= ~COPTER_GYRO_CALIBRATED;
            fail_message();
          }
          break;
        }

        if ((channel[1] > 1900) && (channel[2] < 1100)) {
          while ((channel[1] > 1900) && (channel[2] < 1100)) waiting_knob_release();
          break;
        }

        update_display(0b01000001, 0b11110111, 0b11110101, 0b11000101);
      }

      copter.leds |= LED_YELLOW | LED_FRONT | LED_REAR;
      copter.buzzer = true;
    }

    /*accelerometer calibration*/
    if ((channel[1] < 1100) && (channel[2] > 1900)) { 
      while ((channel[1] < 1100) && (channel[2] > 1900)) waiting_knob_release();

      while (true) { 
        if ((channel[1] < 1100) && (channel[2] > 1900)) {
          while ((channel[1] < 1100) && (channel[2] > 1900)) waiting_knob_release();
          if (calibrate_accelerometer()) {
            copter.status |= COPTER_ACC_CALIBRATED;
          } else {
            copter.status &= ~COPTER_ACC_CALIBRATED;
            fail_message();
          }
          break;
        }

        if ((channel[1] > 1900) && (channel[2] < 1100)) {
          while ((channel[1] > 1900) && (channel[2] < 1100)) waiting_knob_release();
          break;
        }

        update_display(0b00010001, 0b01100011, 0b01100011, 0b11111111);
      }

      copter.leds |= LED_YELLOW | LED_FRONT | LED_REAR;
      copter.buzzer = true;
    }

    /*ESCs calibration*/
    if ((channel[1] < 1100) && (channel[2] < 1100)) {
      while ((channel[1] < 1100) && (channel[2] < 1100)) waiting_knob_release();

      while (true) {
        if ((channel[1] < 1100) && (channel[2] < 1100)) { /*confirm operation*/
          while ((channel[1] < 1100) && (channel[2] < 1100)) waiting_knob_release();

          EEPROM.write(ESC_CAL_EEPROM_ADDR, 1); /*save on EEPROM, when reinitialize calibrate_escs() will be executed*/
          break;
        }

        if ((channel[1] > 1900) && (channel[2] < 1100)) { /*abort operation*/
          while ((channel[1] > 1900) && (channel[2] < 1100)) waiting_knob_release();

          EEPROM.write(ESC_CAL_EEPROM_ADDR, 0); /*save on EEPROM*/
          break;
        }

        update_display(0b01100001, 0b01001001, 0b01100011, 0b11111111);
      }

      copter.leds |= LED_YELLOW | LED_FRONT | LED_REAR;
      copter.buzzer = true;
    }
  }

  /*start/stop motors*/
  if ((copter.status & COPTER_STATUS_OK) && (copter.status & COPTER_GYRO_CALIBRATED) && 
      (copter.status & COPTER_ACC_CALIBRATED)) {
    /*idle speed*/
    if ((copter.motors == LOCK_THROTTLE) && (channel[3] < 1100) && (channel[4] < 1100)) {
      while ((channel[3] < 1100) && (channel[4] < 1100)) waiting_knob_release();

      copter.motors = IDLE_THROTTLE;
      copter.buzzer = true;
    } 

    /*full speed*/
    if ((copter.motors == IDLE_THROTTLE) && (channel[3] < 1100) && (channel[4] < 1100)) {
      while ((channel[3] < 1100) && (channel[4] < 1100)) waiting_knob_release();

      /*reset PID variables*/
      pid_controller_init();
      copter.motors = FULL_THROTTLE;
    } 
    
    /*stop motors*/
    if ((channel[3] < 1100) && (channel[4] > 1900)) {
      copter.motors = LOCK_THROTTLE;
    } 
  }

  return true;
}

/*******************************************************
  waiting knob release
/*******************************************************/
void waiting_knob_release(void) {
  static unsigned long counterWaitingKnob = millis();
  static bool flagKnob = false;

  if (millis() - counterWaitingKnob >= 200) {
    counterWaitingKnob = millis();
    flagKnob = !flagKnob;
  }

  if (flagKnob) update_display(0b11111101, 0b11111101, 0b11111101, 0b11111101);
  else update_display(0b11111111, 0b11111111, 0b11111111, 0b11111111);
}

/*******************************************************
  run controllers
/*******************************************************/
void run_controllers(void) {
  /*ROLL*/
  /*reference*/
  rollSetpoint = (((channel[1] - (float)STICK_MIDPOINT) - rollLevelAdjust) / 3.0);

  /*dead band*/
  rollSetpoint = apply_dead_band(rollSetpoint, 15);

  /*update*/
  pid_controller_update(0, rollSetpoint, gyroRollInput);


  /*PITCH*/
  /*reference*/
  pitchSetpoint = (((channel[2] - (float)STICK_MIDPOINT) - pitchLevelAdjust) / 3.0);

  /*dead band*/
  pitchSetpoint = apply_dead_band(pitchSetpoint, 15);  

  /*update*/
  pid_controller_update(1, pitchSetpoint, gyroPitchInput);


  /*YAW*/
  /*reference*/
  (channel[3] > 1100) ? yawSetpoint = ((channel[4] - (float)STICK_MIDPOINT) / 3.0) : yawSetpoint = 0;

  /*dead band*/
  yawSetpoint = apply_dead_band(yawSetpoint, 20); 

  /*update*/
  pid_controller_update(2, yawSetpoint, gyroYawInput);

  // PID.kp[0] = (float)(constrain(map(channel[6], 990, 2000, 1, 30), 1, 30) / 10.0);
  // PID.ki[0] = 0.02;
  // PID.kd[0] = (float)(constrain(map(channel[7], 990, 2000, 1, 50), 1, 50) / 1.0);

  // PID.kp[1] = PID.kp[0];
  // PID.ki[1] = PID.ki[0];
  // PID.kd[1] = PID.kd[0];
}

/*******************************************************
  PID controller update
/*******************************************************/
void pid_controller_update(uint8_t axis, float reference, float feedback) {
  /*error*/
  if (axis == 3) {
    PID.error[axis] = reference - feedback;
  } else {
    PID.error[axis] = feedback - reference; /*inverted*/
  }

  /*error limit*/
  PID.error[axis] = constrain(PID.error[axis], PID.lim_min_error[axis], PID.lim_max_error[axis]);

  /*proportial term*/
  PID.p_term[axis] = PID.kp[axis] * PID.error[axis];

  /*integral term*/
  PID.i_term[axis] = PID.i_term[axis] 
                  + (PID.ki[axis] * PID.sample_time * (PID.error[axis] + PID.prev_error[axis]) * 0.5) 
                  + (PID.ku[axis] * PID.sample_time * PID.sat_error[axis]);

  /*integral saturation*/
  //PID.i_term[axis] = constrain(PID.i_term[axis], PID.lim_min_integr, PID.lim_max_integr);

  /*reset integral term*/
  //if (abs(PID.error[axis]) > 2.0) PID.i_term[axis] = 0;

  /*derivative term*/
  //PID.d_term[axis] = (PID.kd[axis] / PID.gain_factor) * ((PID.error[axis] - PID.prev_error[axis]) / PID.sample_time); //work well with +D not with -D

  // PID.delta[axis] = ((feedback - PID.prev_feedback[axis]) / PID.sample_time); IMU.gyroX
  // PID.delta[axis] = (0.6 * PID.delta[axis]) + (0.4 * PID.prev_delta[axis]);
  // PID.d_term[axis] = (PID.kd[axis] / PID.gain_factor) * PID.delta[axis]; //not work yet

  PID.delta[axis] = ((PID.error[axis] - PID.prev_error[axis]) / PID.sample_time);
  PID.delta[axis] = (0.6 * PID.delta[axis]) + (0.4 * PID.prev_delta[axis]);
  PID.d_term[axis] = (PID.kd[axis] / PID.gain_factor) * PID.delta[axis]; //work well with +D not with -D

  // PID.d_term[axis] = ((2.0 * PID.kd[axis] * (PID.error[axis] - PID.prev_error[axis]))
  //                  + ((2.0 * PID.tau - PID.sample_time) * PID.d_term[axis]))
  //                  /  (2.0 * PID.tau + PID.sample_time); //work well with +D not with -D

  /*output*/
  PID.output[axis] = PID.p_term[axis] + PID.i_term[axis] + PID.d_term[axis]; /*P + I + D*/

  /*pre saturation output*/
  PID.presat_output[axis] = PID.output[axis];

  /*output saturation*/
  PID.output[axis] = constrain(PID.output[axis], PID.lim_min_output[axis], PID.lim_max_output[axis]);

  /*saturation error*/
  PID.sat_error[axis] = PID.output[axis] - PID.presat_output[axis];

  /*update previous variables*/
  PID.prev_error[axis] = PID.error[axis];
  PID.prev_feedback[axis] = feedback;
  PID.prev_delta[axis] = PID.delta[axis];

  /*hover mode*/
  if (axis == 3) {
    // if (PID.error[axis] < 0) {
    //   PID.kd[axis] = (float)(map(PID.error[axis], -100, 0, 25, 90) / 100.0);
    // } else {
    //   PID.kd[axis] = (float)(map(PID.error[axis], 0, 100, 90, 25) / 100.0);
    // }
     
    // if (PID.error[axis] < 0) {
    //   PID.kp[axis] = (float)(map(PID.error[axis], -100, 0, 24, 12) / 10.0);
    // } else {
    //   PID.kp[axis] = (float)(map(PID.error[axis], 0, 100, 12, 24) / 10.0);
    // }  
   
    // if (PID.error[axis] < 0) {
    //   PID.ki[axis] = (float)(map(PID.error[axis], -100, 0, 20, 10) / 10.0);
    // } else {
    //   PID.ki[axis] =(float)(map(PID.error[axis], 0, 100, 10, 20) / 10.0);
    // }   
   }

  /*(see this work: https://drive.google.com/file/d/1I4uQgPqj5LlNNIw-dCfZhPc1CPUJ-44N/view?usp=sharing)*/
}

/*******************************************************
  PID controller init
/*******************************************************/
void pid_controller_init(void) {
  /*updated angles quickly*/
  rollAngle  = rollAngleAcc;
  pitchAngle = pitchAngleAcc;


  /*to do, remove*/
  errorRoll = errorPitch = errorYaw = 0;
  errorRollAnt = errorPitchAnt = errorYawAnt = 0;

  pTermRoll = pTermPitch = pTermYaw = 0;
  iTermRoll = iTermPitch = iTermYaw = 0;
  dTermRoll = dTermPitch = dTermYaw = 0;

  pidRoll = pidPitch = pidYaw = 0;

  errorAltHold = 0;

  pTermAltHold = 0;
  iTermAltHold = 0;
  dTermAltHold = 0;

  pidAltHold = 0;


  /*reset PID variables*/
  for (uint8_t i = 0; i < NUMBER_CTRL; i ++) {
    PID.error[i] = 0;
    PID.prev_error[i] = 0;
    PID.sat_error[i] = 0;
    PID.p_term[i] = 0;
    PID.i_term[i] = 0;
    PID.d_term[i] = 0;
    PID.output[i] = 0;
    PID.prev_output[i] = 0;
    PID.presat_output[i] = 0;
    PID.delta[i] = 0;
    PID.prev_delta[i] = 0;
    PID.prev_feedback[i] = 0;
  }
}

/*******************************************************
  apply dead band
/*******************************************************/
float apply_dead_band(float value, uint8_t deadband) {
  if (abs(value) < deadband) {
    value = 0;
  } else if (value > 0) {
    value -= deadband;
  } else if (value < 0) {
    value += deadband;
  }

  value = constrain(value, -MAX_STICK_REF, MAX_STICK_REF);

  return value;
}

/*******************************************************
  read battery voltage
/*******************************************************/
void read_battery_voltage(void) {
  if (micros() - previousReadVoltagePeriod >= READ_BATTERY_PERIOD) {
    previousReadVoltagePeriod = micros();

    float voltage = analogRead(BATTERY_PIN) * VOLTAGE_FACTOR;
    copter.batteryVoltage = (copter.batteryVoltage * 0.95) + (voltage * 0.05);
  }
}

/*******************************************************
  show on display status
/*******************************************************/
void show_display_status(void) {
  static unsigned long previousCopterPeriod = millis();
  static uint8_t copterIndex = 1;

  if (copter.status & COPTER_STATUS_OK) {
    if ((millis() - previousCopterPeriod >= 5000) && (copterIndex == 0)) {
      previousCopterPeriod = millis();
      copterIndex = 1;
    } else if ((millis() - previousCopterPeriod >= 1000) && (copterIndex == 1)) {
      previousCopterPeriod = millis();
      copterIndex = 2;
    } else if ((millis() - previousCopterPeriod >= 5000) && (copterIndex == 2)) {
      previousCopterPeriod = millis();
      copterIndex = 3;
    } else if ((millis() - previousCopterPeriod >= 1000) && (copterIndex == 3)) {
      previousCopterPeriod = millis();
      copterIndex = 0;
    }

    if ((copterIndex == 0) || (copterIndex == 2)) {
      uint16_t volt = (uint16_t)(copter.batteryVoltage * 100);

      uint8_t n1 = (volt / 1000);
      uint8_t n2 = (volt % 1000) * 0.01;
      uint8_t n3 = (volt % 100) * 0.1;
      uint8_t n4 = (volt % 10);

      update_display(numbers[n1], numbers[n2] & 0b11111110, numbers[n3], numbers[n4]);
    } else if (copterIndex == 1) {
      update_display(0b11000001, 0b00010001, 0b11100001, 0b11100001);
    } else if (copterIndex == 3) {
      if (copter.motors == LOCK_THROTTLE) update_display(0b11100011, 0b11000101, 0b11100101, 0b11110001);
      if (copter.motors == IDLE_THROTTLE) update_display(0b10011111, 0b10000101, 0b11100011, 0b01100001);
      if (copter.motors == FULL_THROTTLE) update_display(0b01110001, 0b10000011, 0b11100011, 0b11100011);
    }
  } else {
    display_mux();
  }
}

/*******************************************************
  check alarms
/*******************************************************/
void check_alarms(void) {
  static unsigned long previousBipBuzzerPeriod = millis();
  static uint8_t buzzerCounter = 0;

  /*bip buzzer->low battery voltage or after 3 minutes stopped*/
  if ((copter.batteryVoltage > BATTERY_LOWER_THRESHOLD) && (copter.batteryVoltage < BATTERY_UPPER_THRESHOLD)) {
    copter.leds &= ~LED_BLUE;
    if (micros() - previousBuzzerPeriod >= BUZZER_PERIOD) {
      previousBuzzerPeriod = micros();
      digitalWrite(BUZZER_PIN, !digitalRead(BUZZER_PIN));
    }
  } else if ((!copter.buzzer) && (millis() - startCopterTimer < MAX_RUNNING_TIME)) {
    copter.leds |= LED_BLUE;
    digitalWrite(BUZZER_PIN, LOW);
  }

  /*bip buzzer->general*/
  if ((copter.buzzer) && (copter.batteryVoltage > BATTERY_UPPER_THRESHOLD) && 
      (millis() - previousBipBuzzerPeriod >= 50)) {
    previousBipBuzzerPeriod = millis();
    digitalWrite(BUZZER_PIN, !digitalRead(BUZZER_PIN));

    buzzerCounter ++;
    if (buzzerCounter >= 5) {
      digitalWrite(BUZZER_PIN, LOW);
      copter.buzzer = false;
      buzzerCounter = 0;
    }
  }

  /*bip buzzer->after 3 minutes stopped*/
  if ((millis() - startCopterTimer >= MAX_RUNNING_TIME) && 
      (millis() - previousBipBuzzerPeriod >= 200)) {
    previousBipBuzzerPeriod = millis();
    digitalWrite(BUZZER_PIN, !digitalRead(BUZZER_PIN));
  }

  /*blink LED green*/
  if (micros() - previousBlinkPeriod >= BLINK_LED_PERIOD) {
    previousBlinkPeriod = micros();
    copter.leds ^= LED_GREEN;
  }

  /*if rate loop exceed 4ms (250Hz), turn on LED red*/ 
  (micros() - previousCtrlLoopPeriod > CTRL_LOOP_PERIOD) ? copter.leds |= LED_RED : copter.leds &= ~LED_RED;

  /*switch LEDs state*/
  toggle_leds();
}

/*******************************************************
  run motors
/*******************************************************/
void run_motors(void) {
  #define PIDMIX(X, Y, Z) (uint16_t)(mainThrottle + (PID.output[0] * X) + (PID.output[1] * Y) + (PID.output[2] * Z))
  #define SERVODIR(x) ((x > 0) ? 1 : -1)
  #define TILT_SERVO_MIDDLE 1500UL

  /*
  tiltOffset = map(channel[6], 1000, 2000, -300, 300);
  tiltServo.writeMicroseconds(channel[4] - tiltOffset);
  */
  
  /*lock*/
  if (copter.motors == LOCK_THROTTLE) {
    for (uint8_t i = 0; i < NUMBER_MOTORS; i ++) {
      motor[i] = LOCK_SPEED; 
    }

#if defined(TRICOPTER)
    tiltOffset = map(channel[6], 1000, 2000, -300, 300);
    tiltMiddle = (1500 - tiltOffset);
    
    motor[3] = tiltMiddle; /*to do, save on eeprom*/
#endif
  }

  /*idle*/
  if (copter.motors == IDLE_THROTTLE) {
    for (uint8_t i = 0; i < NUMBER_MOTORS; i ++) {
      motor[i] = IDLE_SPEED; 
    }

#if defined(TRICOPTER)
    motor[3] = tiltMiddle;
#endif
  } 
  
  /*full*/
  if (copter.motors == FULL_THROTTLE) {
    /*throttle signal*/
    if (statusAltitudeHold) { /*altitude hold mode*/
      mainThrottle = (uint16_t)(1500.0 + adjustThrottle + manualThrottle + pidAltHold);
    } else if (statusHover) { /*hover mode*/
      mainThrottle = hoverThrottle;
    } else { /*default mode*/
      mainThrottle = channel[3];
    }

    mainThrottle = constrain(mainThrottle, MIN_THROTTLE, MAX_THROTTLE);

#if defined(QUAD_X) /*QUAD CW X*/
    motor[0] = PIDMIX(+1, -1, -1); /*motor 1 (front-right - CCW)*/
    motor[1] = PIDMIX(+1, +1, +1); /*motor 2 (rear-right - CW)*/
    motor[2] = PIDMIX(-1, +1, -1); /*motor 3 (rear-left - CCW)*/
    motor[3] = PIDMIX(-1, -1, +1); /*motor 4 (front-left - CW)*/
#elif defined(TRICOPTER) /*TRICOPTER Y*/
    motor[0] = PIDMIX(-1, -2/3, 0); /*motor 1 (front-right - CW)*/
    motor[1] = PIDMIX(+1, -2/3, 0); /*motor 2 (front-left - CCW)*/
    motor[2] = PIDMIX( 0, +4/3, 0); /*motor 3 (rear - CCW)*/
    //esc4 = (channel[4] - tiltOffset);
    motor[3] = (SERVODIR(channel[4]) * pidYaw) + (TILT_SERVO_MIDDLE - tiltOffset); //TILT_SERVO_MIDDLE /*tilt servo*/
#endif

    /*battery voltage compensation*/
    if (((copter.batteryVoltage > BATTERY_LOWER_TRIGGER) && (copter.batteryVoltage < BATTERY_UPPER_TRIGGER)) && (!statusAltitudeHold)) {
      for (uint8_t i = 0; i < NUMBER_MOTORS; i ++) {
        motor[i] += motor[i] * (float(BATTERY_UPPER_TRIGGER - copter.batteryVoltage) * BATTERY_COMPENSATION);
      }
    }

    /*speed limit*/
    for (uint8_t i = 0; i < NUMBER_MOTORS; i ++) {
      motor[i] = constrain(motor[i], IDLE_SPEED, FULL_SPEED);
    }

#if defined(TRICOPTER)
    if (mainThrottle < 1100) motor[3] = tiltMiddle;
#endif
  } else {
    mainThrottle  = MIN_THROTTLE;
    hoverThrottle = MIN_THROTTLE;
  }

  write_pwm();
}

/*******************************************************
  write PWM
/*******************************************************/
void write_pwm(void) {
  unsigned long startTime = micros(); 
  unsigned long currTime;
  uint8_t flagPwm = (0xFF >> NUMBER_MOTORS); /*0xFF >> 4 = 0x0F*/

#if HARD_SOFT_PWM
  /*set all PWM pins to high*/
  PORTE |= (1 << PORTE5); /*motor 1*/
  PORTB |= (1 << PORTB7); /*motor 2*/
  PORTH |= (1 << PORTH4); /*motor 3*/

#if defined(QUAD_X)
  PORTH |= (1 << PORTH6); /*motor 4*/
#elif defined(TRICOPTER)
  PORTB |= (1 << PORTB3); /*tilt servo*/
#endif

  while (flagPwm > 0x00) {
    currTime = micros();

    if (currTime >= startTime + motor[0]) {
      PORTE &= ~(1 << PORTE5);
      flagPwm &= 0b00001110;
    }

    if (currTime >= startTime + motor[1]) {
      PORTB &= ~(1 << PORTB7);
      flagPwm &= 0b00001101;
    }

    if (currTime >= startTime + motor[2]) {
      PORTH &= ~(1 << PORTH4);
      flagPwm &= 0b00001011;
    }

#if defined(QUAD_X)
    if (currTime >= startTime + motor[3]) {
      PORTH &= ~(1 << PORTH6);
      flagPwm &= 0b00000111;
    }
#elif defined(TRICOPTER)
    if (currTime >= startTime + motor[3]) {
      PORTB &= ~(1 << PORTB3);
      flagPwm &= 0b00000111;
    }
#endif

    if (currTime - startTime >= PWM_SAFETY_PERIOD) {
      PORTE &= ~(1 << PORTE5); /*motor 1*/
      PORTB &= ~(1 << PORTB7); /*motor 2*/
      PORTH &= ~(1 << PORTH4); /*motor 3*/
      PORTH &= ~(1 << PORTH6); /*motor 4*/
      PORTB &= ~(1 << PORTB3); /*tilt servo*/

      flagPwm &= 0x00;
      break;
    }
  }
#else
  pwm_motor[0].writeMicroseconds(motor[0]);
  pwm_motor[1].writeMicroseconds(motor[1]);
  pwm_motor[2].writeMicroseconds(motor[2]);
  pwm_motor[3].writeMicroseconds(motor[3]);
#endif
}

/*******************************************************
  calibration ESCs
/*******************************************************/
bool calibrate_escs(void) {
  unsigned long startEscCalibrationTime = millis();
  uint8_t flagThrottleMoved = 0;
  uint16_t i = 0;
  uint8_t k = 0;

  while (true) {
    for (uint8_t i = 0; i < NUMBER_MOTORS; i ++) {
      motor[i] = channel[3];
    }
    
#if defined(TRICOPTER)
    motor[3] = tiltMiddle;
#endif

    write_pwm();

    if ((flagThrottleMoved == 0) && (channel[3] > 1850)) flagThrottleMoved = 1;
    if ((flagThrottleMoved == 1) && (channel[3] < 1100)) flagThrottleMoved = 2;

    /*exit*/
    if ((channel[1] > 1900) && (channel[2] < 1100)) {
      for (uint8_t i = 0; i < NUMBER_MOTORS; i ++) {
        motor[i] = LOCK_SPEED;
      }

#if defined(TRICOPTER)
      motor[3] = tiltMiddle;
#endif

      write_pwm();

      if (flagThrottleMoved == 2) return true;
      else return false;
    }

    /*if passed 1min, exit*/
    if (millis() - startEscCalibrationTime >= WAITING_SAFETY_PERIOD) {
      return false;
    }

    i ++;
    if (i % 50 == 0) {
      (k >= 2) ? k = 0 : k ++;
    }

    if (k == 0) update_display(0b01111111, 0b01111111, 0b01111111, 0b01111111);
    if (k == 1) update_display(0b11111101, 0b11111101, 0b11111101, 0b11111101);
    if (k == 2) update_display(0b11101111, 0b11101111, 0b11101111, 0b11101111);
    
    delayMicroseconds(4000);
  }
}

/*******************************************************
  calibrate gyroscope
/*******************************************************/
bool calibrate_gyroscope(void) {
  uint8_t k = 0;

  int32_t gyroXCalib = 0;
  int32_t gyroYCalib = 0;
  int32_t gyroZCalib = 0;

  /*reset previous offset*/
  IMU.gyroXOffset.val = 0;
  IMU.gyroYOffset.val = 0;
  IMU.gyroZOffset.val = 0;

#if DEBUG_IMU
  Serial.println(F("CALIBRATING GYROSCOPE..."));
#endif

  for (uint16_t i = 0; i < GYRO_OFFSET_INDEX; i ++) {
    if (i % 50 == 0) {
      if (channel[5] > 1500) copter.leds ^= LED_GREEN | LED_FRONT | LED_REAR;
      else copter.leds &= ~LED_YELLOW & ~LED_FRONT & ~LED_REAR;

      toggle_leds();

      (k >= 2) ? k = 0 : k ++;
    }

    if (k == 0) update_display(0b01111111, 0b01111111, 0b01111111, 0b01111111);
    if (k == 1) update_display(0b11111101, 0b11111101, 0b11111101, 0b11111101);
    if (k == 2) update_display(0b11101111, 0b11101111, 0b11101111, 0b11101111);

#if DEBUG_IMU
    Serial.print(F("["));
    Serial.print(i + 1);
    Serial.print(F("]: "));
    Serial.print(IMU.gyroX);
    Serial.print(F("  "));
    Serial.print(IMU.gyroY);
    Serial.print(F("  "));
    Serial.println(IMU.gyroZ);
#endif

    IMU_compute();    

    for (uint8_t i = 0; i < NUMBER_MOTORS; i ++) {
      motor[i] = LOCK_SPEED;
    }

#if defined(TRICOPTER)
    motor[3] = tiltMiddle;
#endif

    write_pwm();

    gyroXCalib += IMU.gyroX;   
    gyroYCalib += IMU.gyroY;  
    gyroZCalib += IMU.gyroZ;   
    delayMicroseconds(4000);                      
  }

  IMU.gyroXOffset.val = (int32_t)((float)gyroXCalib / (float)GYRO_OFFSET_INDEX);
  IMU.gyroYOffset.val = (int32_t)((float)gyroYCalib / (float)GYRO_OFFSET_INDEX);
  IMU.gyroZOffset.val = (int32_t)((float)gyroZCalib / (float)GYRO_OFFSET_INDEX);

#if DEBUG_IMU
  Serial.println(F("GYROSCOPE OFFSETs"));
  Serial.print(F("X: "));
  Serial.print(IMU.gyroXOffset.val);
  Serial.print(F(" Y: "));
  Serial.print(IMU.gyroYOffset.val);
  Serial.print(F(" Z: "));
  Serial.println(IMU.gyroZOffset.val);
#endif

  /*high calibration value? error*/
  if ((IMU.gyroXOffset.val < -255) || (IMU.gyroXOffset.val > 255) ||
      (IMU.gyroYOffset.val < -255) || (IMU.gyroYOffset.val > 255) ||
      (IMU.gyroZOffset.val < -255) || (IMU.gyroZOffset.val > 255)) {
#if DEBUG_IMU
    Serial.println(F("GYROSCOPE NOT CALIBRATED"));
#endif
    IMU.gyroXOffset.val = 0;
    IMU.gyroYOffset.val = 0;
    IMU.gyroZOffset.val = 0;

    for (uint8_t i = GYROX_OFFSET_EEPROM_ADDR; i <= GYROZ_OFFSET_EEPROM_ADDR+3; i ++) {
      EEPROM.write(i, 0);
    }
    return false;
  } else {
#if DEBUG_IMU
    Serial.println(F("GYROSCOPE CALIBRATED"));
#endif

    /*save on EEPROM calibration values*/
    EEPROM.write(GYROX_OFFSET_EEPROM_ADDR,   IMU.gyroXOffset.raw[3]);
    EEPROM.write(GYROX_OFFSET_EEPROM_ADDR+1, IMU.gyroXOffset.raw[2]);
    EEPROM.write(GYROX_OFFSET_EEPROM_ADDR+2, IMU.gyroXOffset.raw[1]);
    EEPROM.write(GYROX_OFFSET_EEPROM_ADDR+3, IMU.gyroXOffset.raw[0]);

    EEPROM.write(GYROY_OFFSET_EEPROM_ADDR,   IMU.gyroYOffset.raw[3]);
    EEPROM.write(GYROY_OFFSET_EEPROM_ADDR+1, IMU.gyroYOffset.raw[2]);
    EEPROM.write(GYROY_OFFSET_EEPROM_ADDR+2, IMU.gyroYOffset.raw[1]);
    EEPROM.write(GYROY_OFFSET_EEPROM_ADDR+3, IMU.gyroYOffset.raw[0]);

    EEPROM.write(GYROZ_OFFSET_EEPROM_ADDR,   IMU.gyroZOffset.raw[3]);
    EEPROM.write(GYROZ_OFFSET_EEPROM_ADDR+1, IMU.gyroZOffset.raw[2]);
    EEPROM.write(GYROZ_OFFSET_EEPROM_ADDR+2, IMU.gyroZOffset.raw[1]);
    EEPROM.write(GYROZ_OFFSET_EEPROM_ADDR+3, IMU.gyroZOffset.raw[0]);

    return true;
  }
}

/*******************************************************
  calibrate accelerometer
/*******************************************************/
bool calibrate_accelerometer(void) {
  uint8_t k = 0;

  int32_t accXCal = 0;
  int32_t accYCal = 0;

  //reset previous offset
  IMU.accXOffset.val = 0; 
  IMU.accYOffset.val = 0;

#if DEBUG_IMU
  Serial.println(F("CALIBRATING ACCELEROMETER..."));
#endif

  for (uint8_t i = 0; i < ACC_OFFSET_INDEX; i ++) {
    if (i % 10 == 0) {
      if (channel[5] > 1500) copter.leds ^= LED_GREEN | LED_FRONT | LED_REAR;
      else copter.leds &= ~LED_YELLOW & ~LED_FRONT & ~LED_REAR;

      (k >= 2) ? k = 0 : k ++;
    }

    if (k == 0) update_display(0b01111111, 0b01111111, 0b01111111, 0b01111111);
    if (k == 1) update_display(0b11111101, 0b11111101, 0b11111101, 0b11111101);
    if (k == 2) update_display(0b11101111, 0b11101111, 0b11101111, 0b11101111);

#if DEBUG_IMU
    Serial.print(F("["));
    Serial.print(i + 1);
    Serial.print(F("]: "));
    Serial.print(IMU.accX);
    Serial.print(F("  "));
    Serial.println(IMU.accY);
#endif

    IMU_compute();    

    for (uint8_t i = 0; i < NUMBER_MOTORS; i ++) {
      motor[i] = LOCK_SPEED;
    }
    
#if defined(TRICOPTER)
    motor[3] = tiltMiddle;
#endif

    write_pwm();

    accXCal += IMU.accX;   
    accYCal += IMU.accY;  

    /*high value of calibration? error*/
    if ((IMU.accX < -500) || (IMU.accX > 500) || 
        (IMU.accY < -500) || (IMU.accY > 500)) {
#if DEBUG_IMU
      Serial.println(F("ACCELEROMETER NOT CALIBRATED"));
#endif
      IMU.accXOffset.val = 0;   
      IMU.accYOffset.val = 0;  

      for (uint8_t i = ACCX_OFFSET_EEPROM_ADDR; i <= ACCY_OFFSET_EEPROM_ADDR+3; i ++) {
        EEPROM.write(i, 0);
      }
      return false;
    }

    delayMicroseconds(4000);                      
  }

  IMU.accXOffset.val = (int32_t)((float)accXCal / (float)ACC_OFFSET_INDEX);
  IMU.accYOffset.val = (int32_t)((float)accYCal / (float)ACC_OFFSET_INDEX);

#if DEBUG_IMU
  Serial.println(F("ACCELEROMETER OFFSETs"));
  Serial.print(F("ACC X: "));
  Serial.print(IMU.accXOffset.val);
  Serial.print(F(" ACC Y: "));
  Serial.println(IMU.accYOffset.val);
#endif

#if DEBUG_IMU
  Serial.println(F("ACCELEROMETER CALIBRATED"));
#endif

  /*save on EEPROM offset values*/
  EEPROM.write(ACCX_OFFSET_EEPROM_ADDR,   IMU.accXOffset.raw[3]);
  EEPROM.write(ACCX_OFFSET_EEPROM_ADDR+1, IMU.accXOffset.raw[2]);
  EEPROM.write(ACCX_OFFSET_EEPROM_ADDR+2, IMU.accXOffset.raw[1]);
  EEPROM.write(ACCX_OFFSET_EEPROM_ADDR+3, IMU.accXOffset.raw[0]);

  EEPROM.write(ACCY_OFFSET_EEPROM_ADDR,   IMU.accYOffset.raw[3]);
  EEPROM.write(ACCY_OFFSET_EEPROM_ADDR+1, IMU.accYOffset.raw[2]);
  EEPROM.write(ACCY_OFFSET_EEPROM_ADDR+2, IMU.accYOffset.raw[1]);
  EEPROM.write(ACCY_OFFSET_EEPROM_ADDR+3, IMU.accYOffset.raw[0]);

  return true;
}

/*******************************************************
  read EEPROM
/*******************************************************/
bool read_eeprom(void) {
  uint8_t a, b, c, d;

  /*gyroscope X offset*/
  a = (uint8_t)EEPROM.read(GYROX_OFFSET_EEPROM_ADDR);
  b = (uint8_t)EEPROM.read(GYROX_OFFSET_EEPROM_ADDR+1);
  c = (uint8_t)EEPROM.read(GYROX_OFFSET_EEPROM_ADDR+2);
  d = (uint8_t)EEPROM.read(GYROX_OFFSET_EEPROM_ADDR+3);
  IMU.gyroXOffset.val = (int16_t)((a << 32) | (b << 16) | (c << 8) | d); /*to do, use union*/

  /*gyroscope Y offset*/
  a = (uint8_t)EEPROM.read(GYROY_OFFSET_EEPROM_ADDR);
  b = (uint8_t)EEPROM.read(GYROY_OFFSET_EEPROM_ADDR+1);
  c = (uint8_t)EEPROM.read(GYROY_OFFSET_EEPROM_ADDR+2);
  d = (uint8_t)EEPROM.read(GYROY_OFFSET_EEPROM_ADDR+3);
  IMU.gyroYOffset.val = (int16_t)((a << 32) | (b << 16) | (c << 8) | d);

  /*gyroscope Z offset*/
  a = (uint8_t)EEPROM.read(GYROZ_OFFSET_EEPROM_ADDR);
  b = (uint8_t)EEPROM.read(GYROZ_OFFSET_EEPROM_ADDR+1);
  c = (uint8_t)EEPROM.read(GYROZ_OFFSET_EEPROM_ADDR+2);
  d = (uint8_t)EEPROM.read(GYROZ_OFFSET_EEPROM_ADDR+3);
  IMU.gyroZOffset.val = (int16_t)((a << 32) | (b << 16) | (c << 8) | d);

  /*accelerometer X offset*/
  a = (uint8_t)EEPROM.read(ACCX_OFFSET_EEPROM_ADDR);
  b = (uint8_t)EEPROM.read(ACCX_OFFSET_EEPROM_ADDR+1);
  c = (uint8_t)EEPROM.read(ACCX_OFFSET_EEPROM_ADDR+2);
  d = (uint8_t)EEPROM.read(ACCX_OFFSET_EEPROM_ADDR+3);
  IMU.accXOffset.val = (int16_t)((a << 32) | (b << 16) | (c << 8) | d);

  /*accelerometer Y offset*/
  a = (uint8_t)EEPROM.read(ACCY_OFFSET_EEPROM_ADDR);
  b = (uint8_t)EEPROM.read(ACCY_OFFSET_EEPROM_ADDR+1);
  c = (uint8_t)EEPROM.read(ACCY_OFFSET_EEPROM_ADDR+2);
  d = (uint8_t)EEPROM.read(ACCY_OFFSET_EEPROM_ADDR+3);
  IMU.accYOffset.val = (int16_t)((a << 32) | (b << 16) | (c << 8) | d);

  /*flag to calibrate ESCs*/
  statusCalibrationEsc = (bool)EEPROM.read(ESC_CAL_EEPROM_ADDR);
  
#if DEBUG_EEPROM
  Serial.println(F("\nMPU6050 OFFSETs"));
  Serial.print(F("GYRO X: ")); Serial.println(IMU.gyroXOffset.val);
  Serial.print(F("GYRO Y: ")); Serial.println(IMU.gyroYOffset.val);
  Serial.print(F("GYRO Z: ")); Serial.println(IMU.gyroZOffset.val);
  Serial.print(F("ACC X: ")); Serial.println(IMU.accXOffset.val);
  Serial.print(F("ACC Y: ")); Serial.println(IMU.accYOffset.val);
  Serial.print(F("ESC CALIBRATION STATUS: ")); Serial.println(statusCalibrationEsc);
#endif 

  /*at least one high offset value? reset all values*/
  if ((IMU.gyroXOffset.val < -255) || (IMU.gyroXOffset.val > 255) ||
      (IMU.gyroYOffset.val < -255) || (IMU.gyroYOffset.val > 255) || 
      (IMU.gyroZOffset.val < -255) || (IMU.gyroZOffset.val > 255) ||
      (IMU.accXOffset.val  < -500) || (IMU.accXOffset.val  > 500) ||
      (IMU.accYOffset.val  < -500) || (IMU.accYOffset.val  > 500)) {
    IMU.gyroXOffset.val = 0;
    IMU.gyroYOffset.val = 0;
    IMU.gyroZOffset.val = 0;
    IMU.accXOffset.val = 0;
    IMU.accYOffset.val = 0;
    return false;
  } else {
    return true;
  }
}

/*******************************************************
  waiting radio start
/*******************************************************/
bool wait_radio_start(void) {
  unsigned long startWaitRadioTime = millis();
  uint16_t i = 0;
  uint8_t k = 0;

  for (uint8_t i = 0; i < 9; i ++) {
    channel[i] = STICK_INIT_VAL;
  }

  while (true) {
    uint8_t counter = 0;
    for (uint8_t i = 0; i < 9; i ++) {
      if (channel[i] != STICK_INIT_VAL) counter ++;
    }
    
    if (counter >= 3) {
      return true;
    } else if (millis() - startWaitRadioTime >= WAITING_SAFETY_PERIOD) {
      return false;
    }
 
    i ++;
    if (i % 50 == 0) {
      (k >= 2) ? k = 0 : k ++;
    }

    if (k == 0) update_display(0b01111111, 0b01111111, 0b01111111, 0b01111111);
    if (k == 1) update_display(0b11111101, 0b11111101, 0b11111101, 0b11111101);
    if (k == 2) update_display(0b11101111, 0b11101111, 0b11101111, 0b11101111);
    
    delayMicroseconds(4000);
  }
}

/*******************************************************
  update display
/*******************************************************/
void update_display(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4) {
  static uint8_t index = 0;
  (index > 3) ? index = 0 : index ++;

  /*multiplex display*/
  switch (index) {
    case 0: {
      digitalWrite(LATCH_PIN, LOW);
      shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, d1);
      shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, 0b00010000);
      digitalWrite(LATCH_PIN, HIGH);
    }
    break;
    
    case 1: {
      digitalWrite(LATCH_PIN, LOW);
      shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, d2);
      shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, 0b00100000);
      digitalWrite(LATCH_PIN, HIGH);
    }
    break;
    
    case 2: {
      digitalWrite(LATCH_PIN, LOW);
      shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, d3);
      shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, 0b01000000);
      digitalWrite(LATCH_PIN, HIGH);
    }
    break; 
       
    case 3: {
      digitalWrite(LATCH_PIN, LOW);
      shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, d4);
      shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, 0b10000000);
      digitalWrite(LATCH_PIN, HIGH);
    }
    break;
  }
}

/*******************************************************
  display mux
/*******************************************************/
bool display_mux(void) {
  static unsigned long previousSlidePeriod = millis();
  static uint8_t index = 0;
  static bool flag = false;

  (index >= 3) ? index = 0 : index ++;

  switch (index) {
    case 0: {
      digitalWrite(LATCH_PIN, LOW);
      shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, msg[0]);
      shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, B00010000);
      digitalWrite(LATCH_PIN, HIGH);
    }
    break;

    case 1: {
      digitalWrite(LATCH_PIN, LOW);
      shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, msg[1]);
      shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, B00100000);
      digitalWrite(LATCH_PIN, HIGH);
    }
    break;

    case 2: {
      digitalWrite(LATCH_PIN, LOW);
      shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, msg[2]);
      shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, B01000000);
      digitalWrite(LATCH_PIN, HIGH);
    }
    break;

    case 3: {
      digitalWrite(LATCH_PIN, LOW);
      shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, msg[3]);
      shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, B10000000);
      digitalWrite(LATCH_PIN, HIGH);
    }
    break;
  }

  /*circular buffer*/
  if (millis() - previousSlidePeriod >= 300) {
    previousSlidePeriod = millis();

    if (!flag) {
      flag = true;
      index = 0;
    } else {
      uint8_t aux = msg[0];
      uint8_t end = muxIndex - 1;
      for (uint8_t i = 0; i < end; i ++) msg[i] = msg[i + 1];
      msg[end] = aux;
    }

    /*blink LEDs*/
    copter.leds ^= LED_YELLOW | LED_FRONT | LED_REAR;
    toggle_leds();
  }
}

/*******************************************************
  fail message
/*******************************************************/
void fail_message(void) {
  for (uint16_t i = 0; i < 500; i ++) {
    update_display(0b01110001, 0b00010001, 0b10011111, 0b11100011);
    delayMicroseconds(4000);
  }
}

/*******************************************************
  print parameters
/*******************************************************/
void print_parameters(uint8_t index) {
  static uint8_t printIndex = 0;

  switch (index) {
    case 1: {
      (printIndex <= 0x0F) ? printIndex ++ : printIndex = 0;
      if (printIndex == 0x00) Serial.print(F("GX "));
      if (printIndex == 0x01) Serial.print(IMU.gyroX);
      if (printIndex == 0x02) Serial.print(F(" GY "));
      if (printIndex == 0x03) Serial.print(IMU.gyroY);
      if (printIndex == 0x04) Serial.print(F(" ACCX "));
      if (printIndex == 0x05) Serial.print(IMU.accX);
      if (printIndex == 0x06) Serial.print(F(" ACCY "));
      if (printIndex == 0x07) Serial.print(IMU.accY);
      if (printIndex == 0x08) Serial.print(F(" OFF GX "));
      if (printIndex == 0x09) Serial.print(IMU.gyroXOffset.val);
      if (printIndex == 0x0A) Serial.print(F(" OFF GY "));
      if (printIndex == 0x0B) Serial.print(IMU.gyroYOffset.val);
      if (printIndex == 0x0C) Serial.print(F(" OFF ACCX "));
      if (printIndex == 0x0D) Serial.print(IMU.accXOffset.val);
      if (printIndex == 0x0E) Serial.print(F(" OFF ACCY "));
      if (printIndex == 0x0F) Serial.println(IMU.accYOffset.val);
    }
    break;

    case 2: {
      (printIndex <= 0x0B) ? printIndex ++ : printIndex = 0;
      if (printIndex == 0x00) Serial.print(F("ROLL SP "));
      if (printIndex == 0x01) Serial.print(rollSetpoint);
      if (printIndex == 0x02) Serial.print(F(" ADJ "));
      if (printIndex == 0x03) Serial.print(rollLevelAdjust);
      if (printIndex == 0x04) Serial.print(F(" GYRO "));
      if (printIndex == 0x05) Serial.print(gyroRollInput);
      if (printIndex == 0x06) Serial.print(F(" ACC "));
      if (printIndex == 0x07) Serial.print(rollAngleAcc);
      if (printIndex == 0x08) Serial.print(F(" ANG "));
      if (printIndex == 0x09) Serial.print(rollAngle);
      if (printIndex == 0x0A) Serial.print(F(" OUT "));
      if (printIndex == 0x0B) Serial.println(pidRoll);
    }
    break;

    case 3: {
      (printIndex <= 0x0B) ? printIndex ++ : printIndex = 0;
      if (printIndex == 0x00) Serial.print(F("PITCH SP "));
      if (printIndex == 0x01) Serial.print(pitchSetpoint);
      if (printIndex == 0x02) Serial.print(F(" ADJ "));
      if (printIndex == 0x03) Serial.print(pitchLevelAdjust);
      if (printIndex == 0x04) Serial.print(F(" GYRO "));
      if (printIndex == 0x05) Serial.print(gyroPitchInput);
      if (printIndex == 0x06) Serial.print(F(" ACC "));
      if (printIndex == 0x07) Serial.print(pitchAngleAcc);
      if (printIndex == 0x08) Serial.print(F(" ANG "));
      if (printIndex == 0x09) Serial.print(pitchAngle);
      if (printIndex == 0x0A) Serial.print(F(" OUT "));
      if (printIndex == 0x0B) Serial.println(pidPitch);
    }
    break;

    case 4: {
      (printIndex <= 0x0B) ? printIndex ++ : printIndex = 0;
      if (printIndex == 0x00) Serial.print(F("YAW SP "));
      if (printIndex == 0x01) Serial.print(yawSetpoint);
      if (printIndex == 0x02) Serial.print(F(" GYRO "));
      if (printIndex == 0x03) Serial.print(gyroYawInput);
      if (printIndex == 0x04) Serial.print(F(" P "));
      if (printIndex == 0x05) Serial.print(PID.p_term[2]);
      if (printIndex == 0x06) Serial.print(F(" I "));
      if (printIndex == 0x07) Serial.print(PID.i_term[2]);
      if (printIndex == 0x08) Serial.print(F(" OUT "));
      if (printIndex == 0x09) Serial.print(PID.output[2]);
      if (printIndex == 0x0A) Serial.print(F(" M[3] "));
      if (printIndex == 0x0B) Serial.println(motor[3]);
    }
    break;
    
    case 5: {
      (printIndex <= 0x0F) ? printIndex ++ : printIndex = 0;
      if (printIndex == 0x00) Serial.print(F("CH1 "));
      if (printIndex == 0x01) Serial.print(channel[1]);
      if (printIndex == 0x02) Serial.print(F(" CH2 "));
      if (printIndex == 0x03) Serial.print(channel[2]);
      if (printIndex == 0x04) Serial.print(F(" CH3 "));
      if (printIndex == 0x05) Serial.print(channel[3]);
      if (printIndex == 0x06) Serial.print(F(" CH4 "));
      if (printIndex == 0x07) Serial.print(channel[4]);
      if (printIndex == 0x08) Serial.print(F(" CH5 "));
      if (printIndex == 0x09) Serial.print(channel[5]);
      if (printIndex == 0x0A) Serial.print(F(" CH6 "));
      if (printIndex == 0x0B) Serial.print(channel[6]);
      if (printIndex == 0x0C) Serial.print(F(" CH7 "));
      if (printIndex == 0x0D) Serial.print(channel[7]);
      if (printIndex == 0x0E) Serial.print(F(" CH8 "));
      if (printIndex == 0x0F) Serial.println(channel[8]);
    }
    break;

    case 6: {
      (printIndex <= 0x06) ? printIndex ++ : printIndex = 0;
      if (printIndex == 0x01) Serial.print(F("COPTER STS "));
      if (printIndex == 0x02) Serial.print(copter.status, BIN);
      if (printIndex == 0x03) Serial.print(F(" MOTOR STS "));
      if (printIndex == 0x04) Serial.print(copter.motors);
      if (printIndex == 0x05) Serial.print(F(" BATT "));
      if (printIndex == 0x06) Serial.println(copter.batteryVoltage);
    }
    break;

    case 7: {
      (printIndex <= 0x15) ? printIndex ++ : printIndex = 0;
      if (printIndex == 0x00) Serial.print(F("ALTH "));
      if (printIndex == 0x01) Serial.print(statusAltitudeHold);
      if (printIndex == 0x02) Serial.print(F(" CH3 "));
      if (printIndex == 0x03) Serial.print(channel[3]);
      if (printIndex == 0x04) Serial.print(F(" THR "));
      if (printIndex == 0x05) Serial.print(mainThrottle);
      if (printIndex == 0x06) Serial.print(F(" MTHR "));
      if (printIndex == 0x07) Serial.print(manualThrottle);
      if (printIndex == 0x08) Serial.print(F(" ITHR "));
      if (printIndex == 0x09) Serial.print(initialThrottle);
      if (printIndex == 0x0A) Serial.print(F(" CORR "));
      if (printIndex == 0x0B) Serial.print(adjustThrottle);
      if (printIndex == 0x0C) Serial.print(F(" REF "));
      if (printIndex == 0x0D) Serial.print(altHoldSetPoint);
      if (printIndex == 0x0E) Serial.print(F(" P "));
      if (printIndex == 0x0F) Serial.print(actualPressure);
      if (printIndex == 0x10) Serial.print(F(" EALT "));
      if (printIndex == 0x11) Serial.print(altitudeError);
      if (printIndex == 0x12) Serial.print(F(" E "));
      if (printIndex == 0x13) Serial.print(errorAltHold);
      if (printIndex == 0x14) Serial.print(F(" PID "));
      if (printIndex == 0x15) Serial.println(pidAltHold);
    }
    break;

    case 8: {
      (printIndex <= 0x09) ? printIndex ++ : printIndex = 0;
      if (printIndex == 0x00) Serial.print(F("THR "));
      if (printIndex == 0x01) Serial.print(mainThrottle);
      if (printIndex == 0x02) Serial.print(F(" HVF "));
      if (printIndex == 0x03) Serial.print(hoverFactor);
      if (printIndex == 0x04) Serial.print(F(" OTO "));
      if (printIndex == 0x05) Serial.print(otoNewSet);
      if (printIndex == 0x06) Serial.print(F(" INP "));
      if (printIndex == 0x07) Serial.print(filteredDistance);
      if (printIndex == 0x08) Serial.print(F(" PID "));
      if (printIndex == 0x09) Serial.println(PID.output[3]);
    }
    break;
  }
}

/*******************************************************
  toggle LEDs
/*******************************************************/
void toggle_leds(void) {
  /*board LEDs*/
  (copter.leds & LED_YELLOW) ? digitalWrite(LED_YELLOW_PIN, HIGH) : digitalWrite(LED_YELLOW_PIN, LOW);
  (copter.leds & LED_GREEN) ? digitalWrite(LED_GREEN_PIN, HIGH) : digitalWrite(LED_GREEN_PIN, LOW);
  (copter.leds & LED_BLUE) ? digitalWrite(LED_BLUE_PIN, HIGH) : digitalWrite(LED_BLUE_PIN, LOW);
  (copter.leds & LED_RED) ? digitalWrite(LED_RED_PIN, HIGH) : digitalWrite(LED_RED_PIN, LOW);

  /*front frame LEDs*/
  if (copter.leds & LED_FRONT) {
    digitalWrite(LED_FRONT_LEFT, HIGH);
    digitalWrite(LED_FRONT_RIGHT, HIGH);
  } else {
    digitalWrite(LED_FRONT_LEFT, LOW);
    digitalWrite(LED_FRONT_RIGHT, LOW);
  }

  /*rear frame LEDs*/
  if (copter.leds & LED_REAR) {
    digitalWrite(LED_REAR_LEFT, HIGH);
    digitalWrite(LED_REAR_RIGHT, HIGH);
  } else {
    digitalWrite(LED_REAR_LEFT, LOW);
    digitalWrite(LED_REAR_RIGHT, LOW);
  }
}

/*******************************************************
  blink LEDs
/*******************************************************/
void blink_leds(void) {
  unsigned long previousDisplayPeriod;
  unsigned long previousBlinkPeriod = millis();
  uint8_t indexBlink = 0;

  copter.leds |= LED_BLUE | LED_YELLOW | LED_FRONT | LED_REAR;
  toggle_leds();

  while (true) {
    /*blink LEDs*/
    if (millis() - previousBlinkPeriod >= 100) {
      previousBlinkPeriod = millis();

      copter.leds ^= LED_YELLOW | LED_FRONT | LED_REAR;
      toggle_leds();

      indexBlink ++;
    }

    /*to do*/
    if (micros() - previousDisplayPeriod >= 5000) {
      previousDisplayPeriod = micros();
      update_display(0b10011111, 0b11010101, 0b11011111, 0b11100001);
    }

    if (indexBlink >= 10) break;
  }
}

#if defined(BMP180)
/*******************************************
  BMP180 read calibration
/*******************************************/
bool BMP180_read_calibration(void) {
  Wire.beginTransmission(BMP180_ADDRESS);
  Wire.write(BMP180_AC1_ADDRESS); /*first address of calibration values*/
  Wire.endTransmission();
  Wire.requestFrom(BMP180_ADDRESS, 22, true);

  if (Wire.available() != 22) return false;

  BMP180.ac1 = (Wire.read() << 8) | Wire.read();
  BMP180.ac2 = (Wire.read() << 8) | Wire.read();
  BMP180.ac3 = (Wire.read() << 8) | Wire.read();
  BMP180.ac4 = (Wire.read() << 8) | Wire.read();
  BMP180.ac5 = (Wire.read() << 8) | Wire.read();
  BMP180.ac6 = (Wire.read() << 8) | Wire.read();
  BMP180.b1  = (Wire.read() << 8) | Wire.read();
  BMP180.b2  = (Wire.read() << 8) | Wire.read();
  BMP180.mb  = (Wire.read() << 8) | Wire.read();
  BMP180.mc  = (Wire.read() << 8) | Wire.read();
  BMP180.md  = (Wire.read() << 8) | Wire.read();

  return true;
}

/*******************************************
  BMP180 muxIndex measurement
/*******************************************/
void BMP180_init(void) {
  BMP180_read_calibration();
  delay(100);
  BMP180_UT_start();
  BMP180.timeOut = micros() + BMP180_WAIT_UT_READ;
  BMP180.status = true;
}

/*******************************************************
  request uncompensated temperature
/*******************************************************/
void BMP180_UT_start(void) {
  Wire.beginTransmission(BMP180_ADDRESS);
  Wire.write(BMP180_CONTROL); 
  Wire.write(BMP180_READ_TEMPERATURE); 
  Wire.endTransmission();
}

/*******************************************************
  read uncompensated temperature
/*******************************************************/
bool BMP180_UT_read(void) {
  Wire.beginTransmission(BMP180_ADDRESS);
  Wire.write(BMP180_TEMPERATURE_DATA); 
  Wire.endTransmission();
  Wire.requestFrom(BMP180_ADDRESS, 2, true);

  if (Wire.available() != 2) return false;

  BMP180.UT.raw[1] = Wire.read();
  BMP180.UT.raw[0] = Wire.read();

  return true;
}

/*******************************************************
  request uncompensated pressure
/*******************************************************/
void BMP180_UP_start(void) {
  Wire.beginTransmission(BMP180_ADDRESS);
  Wire.write(BMP180_CONTROL); 
  Wire.write(BMP180_READ_PRESSURE + (BMP180_OSS << 6)); 
  Wire.endTransmission();
}

/*******************************************************
  read uncompensated pressure
/*******************************************************/
bool BMP180_UP_read(void) {
  Wire.beginTransmission(BMP180_ADDRESS);
  Wire.write(BMP180_PRESSURE_DATA);
  Wire.endTransmission();
  Wire.requestFrom(BMP180_ADDRESS, 3, true);

  if (Wire.available() != 3) return false;

  BMP180.UP.raw[2] = Wire.read();
  BMP180.UP.raw[1] = Wire.read();
  BMP180.UP.raw[0] = Wire.read();

  return true;
}

/*******************************************************
  read barometer
/*******************************************************/
void BMP180_calculate(void) {
  int32_t x1, x2, x3, b3, b5, b6, p, tmp;
  uint32_t b4, b7;

  /*temperature calculation*/
  x1 = ((int32_t)BMP180.UT.val - BMP180.ac6) * BMP180.ac5 >> 15;
  x2 = ((int32_t)BMP180.mc << 11) / (x1 + BMP180.md);
  b5 = x1 + x2;
  BMP180.temperature = (b5 * 10 + 8) >> 4; /*in 0.01 degC (same as MS5611 temperature)*/

  /*pressure calculation*/
  b6 = b5 - 4000;
  x1 = (BMP180.b2 * (b6 * b6 >> 12)) >> 11; 
  x2 = BMP180.ac2 * b6 >> 11;
  x3 = x1 + x2;
  tmp = BMP180.ac1;
  tmp = (tmp*4 + x3) << BMP180_OSS;
  b3 = (tmp+2)/4;
  x1 = BMP180.ac3 * b6 >> 13;
  x2 = (BMP180.b1 * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (BMP180.ac4 * (uint32_t)(x3 + 32768)) >> 15;

  b7 = ((uint32_t) (BMP180.UP.val >> (8 - BMP180_OSS)) - b3) * (50000UL >> BMP180_OSS);

  p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  BMP180.pressure = p + ((x1 + x2 + 3791) >> 4);
}

/*******************************************************
  update values
/*******************************************************/
uint8_t BMP180_update(void) {
  if (micros() < BMP180.timeOut) return 0;
  BMP180.timeOut = micros() + BMP180_WAIT_UT_READ; /*wait 4.5ms to read temperature + 1.5ms margin (6ms)*/

  if (BMP180.status) {
    BMP180_UT_read();
    BMP180_UP_start();
    BMP180.status = false;
    BMP180.timeOut += BMP180_WAIT_UP_READ; /*wait 27ms (6ms+21ms) to read pressure*/
    return 1;
  } else {
    BMP180_UP_read();
    BMP180_UT_start();
    BMP180_calculate();
    BMP180.status = true;
    return 2;
  }
}
#endif

#if defined(MS561101BA)
/*******************************************
  reset the MS5611
/*******************************************/
void MS5611_reset(void) {
  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(MS5611_REG_RESET);
  Wire.endTransmission();
}

/*******************************************
  muxIndex MS5611 measurement
/*******************************************/
void MS5611_init(void) {
  MS5611_reset();
  delay(100);
  MS5611_read_calibration();
  delay(100);
  MS5611_UT_start();
  MS5611.timeOut = micros() + MS5611_WAIT_UT_UP_READ;
  MS5611.status = true;
}

/*******************************************
  read calibration values
/*******************************************/
void MS5611_read_calibration(void) {
  for (uint8_t i = 1; i < 7; i ++) {
    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.write(0xA0 + (i * 2)); /*first address of calibration data*/
    Wire.endTransmission();

    Wire.requestFrom(MS5611_ADDRESS, 2, true);
    while (Wire.available() < 2);

    MS5611.c[i] = (uint16_t)((Wire.read() << 8) | Wire.read());
  }
}

/*******************************************
  read uncompensated temperature
/*******************************************/
void MS5611_UT_start(void) {
  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(MS5611_TEMPERATURE_ADDR + MS5611_OSR_4096); 
  Wire.endTransmission();
}

/*******************************************
  read temperature
/*******************************************/
bool MS5611_UT_read(void) {
  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(0x00); 
  Wire.endTransmission();

  Wire.requestFrom(MS5611_ADDRESS, 3, true);
  if (Wire.available() != 3) return false;

  MS5611.ut.raw[2] = Wire.read();
  MS5611.ut.raw[1] = Wire.read();
  MS5611.ut.raw[0] = Wire.read();

  return true;
}

/*******************************************
  read uncompensated pressure
/*******************************************/
void MS5611_UP_start(void) {
  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(MS5611_PRESSURE_ADDR + MS5611_OSR_4096); 
  Wire.endTransmission();
}

/*******************************************
  read pressure
/*******************************************/
bool MS5611_UP_read(void) {
  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(0x00); 
  Wire.endTransmission();

  Wire.requestFrom(MS5611_ADDRESS, 3, true);
  if (Wire.available() != 3) return false;

  MS5611.up.raw[2] = Wire.read();
  MS5611.up.raw[1] = Wire.read();
  MS5611.up.raw[0] = Wire.read();

  return true;
}

/*******************************************
  calculate values
/*******************************************/
bool MS5611_calculate(void) {
  int32_t OFF2, SENS2, delt;

  /*temperature calculation*/
  int64_t dT = (int32_t)MS5611.ut.val - ((int32_t)MS5611.c[5] << 8);
  MS5611.temperature = 2000 + ((dT * MS5611.c[6]) >> 23);

  /*pressure calculation*/
  int64_t OFF  = ((uint32_t)MS5611.c[2] << 16) + ((dT * MS5611.c[4]) >> 7);
  int64_t SENS = ((uint32_t)MS5611.c[1] << 15) + ((dT * MS5611.c[3]) >> 8);

  /*second order temperature compensation*/
  if (MS5611.temperature < 2000) { /*temperature lower than 20st.C*/ 
    delt = MS5611.temperature - 2000;
    delt  = (5 * delt * delt);
    OFF2  = delt >> 1;
    SENS2 = delt >> 2;
    if (MS5611.temperature < -1500) { /*temperature lower than -15st.C*/
      delt  = MS5611.temperature + 1500;
      delt  = (delt * delt);
      OFF2  += (7 * delt);
      SENS2 += (11 * delt) >> 1;
    }
    OFF  -= OFF2; 
    SENS -= SENS2;
  }

  MS5611.pressure = (((MS5611.up.val * SENS) >> 21) - OFF) >> 15;
}

/*******************************************
  altitude estimated
/*******************************************/
float MS5611_alt_est(float pressure) {
  float altitude = 44330.0 * (1.0 - pow(pressure / 101325.0, 0.1903));
  return (altitude * 100);
}

/*******************************************
  update values
/*******************************************/
uint8_t MS5611_update(void) {
  if (micros() < MS5611.timeOut) return 0;
  MS5611.timeOut = micros() + MS5611_WAIT_UT_UP_READ; /*wait 8.5ms to read temperature and pressure + 1.5ms margin (10ms)*/

  if (MS5611.status) {
    MS5611_UT_read();
    MS5611_UP_start();
    MS5611.status = false;
    return 1;
  } else {
    MS5611_UP_read();
    MS5611_UT_start();
    MS5611_calculate();
    MS5611.status = true;
    return 2;
  }
}
#endif

#if defined(VL53L1x)
/*******************************************************
  VL53L1X init
/*******************************************************/
void VL53L1X_init(void) {
  sensor.init();
  sensor.setTimeout(500);

  /*
  Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  You can change these settings to adjust the performance of the sensor, but
  the minimum timing budget is 20 ms for short distance mode and 33 ms for
  medium and long distance modes. See the VL53L1X datasheet for more
  information on range and timing limits.
  */
  sensor.setDistanceMode(VL53L1X::Short);
  sensor.setMeasurementTimingBudget(20000);

  /*
  Start continuous readings at a rate of one measurement every 50 ms (the inter-measurement period).
  This period should be at least as long as the timing budget.
  */
  sensor.startContinuous(50);
}
#endif

#if ALT_HOLD_MODE
/*******************************************************
  altitude hold mode
/*******************************************************/
void altitude_hold(uint8_t index) {
  static uint8_t altHoldIndex  = 0;
  static uint8_t readTempIndex = 0;

  /*temperature buffer*/
  if (altHoldIndex == 0) {
    if (readTempIndex == 0) {
      MS5611_UT_read();

      temperatureSum -= temperatureBuffer[temperatureIndex];
#if defined(BMP180)
      temperatureBuffer[temperatureIndex] = BMP180.temperature;
#endif
#if defined(MS561101BA)
      temperatureBuffer[temperatureIndex] = MS5611.ut.val;
#endif
      temperatureSum += temperatureBuffer[temperatureIndex];
      
      temperatureIndex ++;
      if (temperatureIndex >= TEMPERATURE_INDEX) {
        temperatureIndex = 0;
      }

      rawTemperature = (temperatureSum / (float)TEMPERATURE_INDEX);
    } else {
      MS5611_UP_read();
    }

    readTempIndex ++;
    if (readTempIndex >= 20) {
      readTempIndex = 0;
      MS5611_UT_start();
    } else {
      MS5611_UP_start();
    }
  }

  /*pressure buffer*/
  if (altHoldIndex == 1) { 
    int32_t OFF2, SENS2, delt;

    /*temperature calculation*/
    int64_t dT = (int32_t)rawTemperature - ((int32_t)MS5611.c[5] << 8);
    MS5611.temperature = 2000 + ((dT * MS5611.c[6]) >> 23);

    /*pressure calculation*/
    int64_t OFF  = ((uint32_t)MS5611.c[2] << 16) + ((dT * MS5611.c[4]) >> 7);
    int64_t SENS = ((uint32_t)MS5611.c[1] << 15) + ((dT * MS5611.c[3]) >> 8);

    /*second order temperature compensation*/
    if (MS5611.temperature < 2000) { /*temperature lower than 20st.C*/
      delt = MS5611.temperature - 2000;
      delt  = (5 * delt * delt);
      OFF2  = delt >> 1;
      SENS2 = delt >> 2;
      if (MS5611.temperature < -1500) { /*temperature lower than -15st.C*/
        delt  = MS5611.temperature + 1500;
        delt  = (delt * delt);
        OFF2  += (7 * delt);
        SENS2 += (11 * delt) >> 1;
      }
      OFF  -= OFF2; 
      SENS -= SENS2;
    }

    MS5611.pressure = (((MS5611.up.val * SENS) >> 21) - OFF) >> 15;

    pressureSum -= pressureBuffer[pressureIndex];
#if defined(BMP180)    
    pressureBuffer[pressureIndex] = BMP180.pressure;
#endif    
#if defined(MS561101BA)
    pressureBuffer[pressureIndex] = MS5611.pressure;
#endif    
    pressureSum += pressureBuffer[pressureIndex];

    pressureIndex ++;
    if (pressureIndex >= PRESSURE_INDEX) {
      pressureIndex = 0;
    }

    actualPressureFast = (float)(pressureSum / (float)PRESSURE_INDEX);

    /*complementary filter*/
    actualPressureSlow = (float)((actualPressureSlow * 0.985) + (actualPressureFast * 0.015));
    actualPressureDiff = (float)(actualPressureSlow - actualPressureFast);
    actualPressureDiff = constrain(actualPressureDiff, -8, 8);

    if ((actualPressureDiff > 1) || (actualPressureDiff < -1)) {
      actualPressureSlow -= (actualPressureDiff / 6.0);
    }

    /*minimum oscilation pressure*/
    actualPressure = actualPressureSlow;
  }

  /*altitude hold PID*/
  if (altHoldIndex == 2) { 
    if (statusManualAltitude) {
      previousParachutePressure = (actualPressure * 10);
    }

    parachuteThrottle -= parachuteBuffer[parachuteIndex];
    parachuteBuffer[parachuteIndex] = ((actualPressure * 10) - previousParachutePressure);
    parachuteThrottle += parachuteBuffer[parachuteIndex];
    previousParachutePressure = (actualPressure * 10);

    parachuteIndex ++;
    if (parachuteIndex >= PARACHUTE_INDEX) {
      parachuteIndex = 0;
    }

    /*altidude hold on?*/
    if (statusAltitudeHold) {
      if (altHoldSetPoint == 0) {
        altHoldSetPoint = actualPressure;
      }

      /*manual mainThrottle*/
      statusManualAltitude = false;                                                    
      manualThrottle = 0;                                                           
      if (channel[3] > 1600) {                                                        
        statusManualAltitude = true;                                                  
        altHoldSetPoint = actualPressure;                                     
        manualThrottle = (float)((channel[3] - 1600.0) / 2.5);                              
      }
      if (channel[3] < 1400) {                                                        
        statusManualAltitude = true;                                                  
        altHoldSetPoint = actualPressure;                                     
        manualThrottle = (float)((channel[3] - 1400.0) / 2.5);                              
      }

      /*auto correction mainThrottle*/
      altitudeErrorSum -= altitudeErrorBuffer[altitudeErrorIndex];
      altitudeErrorBuffer[altitudeErrorIndex] = MS5611_alt_est(altHoldSetPoint) - MS5611_alt_est(actualPressure);
      altitudeErrorSum += altitudeErrorBuffer[altitudeErrorIndex];

      altitudeErrorIndex ++;
      if (altitudeErrorIndex >= ALT_ERROR_INDEX) {
        altitudeErrorIndex = 0;
      }

      altitudeError = (altitudeErrorSum / (float)ALT_ERROR_INDEX);

      if (abs(altitudeError) > 8) {
        if (altHoldSetPoint > actualPressure) adjustThrottle -= 2.0;
        if (altHoldSetPoint < actualPressure) adjustThrottle += 2.0;

        adjustThrottle = constrain(adjustThrottle, -120, 120);
      }

      /*altitude hold PID*/                               
      errorAltHold = actualPressure - altHoldSetPoint;         

      uint8_t kp  = constrain(map(channel[6], 990, 2000, 1, 15), 1, 15);    
      uint8_t kix = map(channel[7], 990, 2000, 100, 1);      

      float altHoldErrorGain = 0;                                                   
      if ((errorAltHold > 10.0) || (errorAltHold < -10.0)) {    
        altHoldErrorGain = ((abs(errorAltHold) - 10.0) / 20.0);                    
        if (altHoldErrorGain > 3.0) altHoldErrorGain = 3.0;            
      }

      pTermAltHold = (kp + altHoldErrorGain) * errorAltHold;
      iTermAltHold += ((KI_ALT_HOLD / kix) * errorAltHold);
      dTermAltHold = (KD_ALT_HOLD * parachuteThrottle);

      iTermAltHold = constrain(iTermAltHold, -MAX_PID_OUTPUT, MAX_PID_OUTPUT);

      pidAltHold = pTermAltHold + iTermAltHold + dTermAltHold;
      pidAltHold = constrain(pidAltHold, -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
    } else {
      initialThrottle = mainThrottle;
      statusManualAltitude = true;
      adjustThrottle = 0;
      altHoldSetPoint = 0;
      manualThrottle = 0;
      altitudeError = 0;
      errorAltHold = 0;
      pTermAltHold = 0;
      iTermAltHold = 0;
      dTermAltHold = 0;
      pidAltHold = 0;
    }
  }

  altHoldIndex ++;
  if (altHoldIndex >= index) altHoldIndex = 0;
}

#endif

#if HOVER_MODE
/*******************************************************
  hover mode
/*******************************************************/
void hover_mode(void) {
  static unsigned long previousHoverPeriod = micros();

  /*read distance only once per 35ms*/
  if (micros() - previousHoverPeriod >= HOVER_CTRL_PERIOD) {
    previousHoverPeriod = micros();

    /*read distance in millimeters*/
    uint16_t rawDistance = sensor.readRangeContinuousMillimeters();

    /*measured distance between valid range?*/
    if ((rawDistance > 10) && (rawDistance < 2001)) {
      filteredDistance = rawDistance;

      /*filter distance measured*/
      filteredDistance = (0.9 * filteredDistance) + (0.1 * measure_0);
      measure_0 = filteredDistance;

      /*hover controller enabled*/
      if (statusHover) {
        uint8_t dist = constrain(map(channel[6], 990, 1990, 20, 50), 20, 50);
        targetDistance = (float)round((0.8 * targetDistance) + (0.2 * dist * 10));
        //targetDistance = 200;

        /*Kp=0.12,Ki=0.01,Kd=1.3-1.83, lim=+-150, hoverF=1300, step=+-3*/
        PID.kp[3] = 0.12; //(float)(constrain(map(channel[6], 990, 2000, 10, 30), 10, 30) / 10.0);
        PID.ki[3] = 0.01;
        PID.kd[3] = 1.30; //(float)(constrain(map(channel[7], 990, 2000, 100, 200), 100, 200) / 100.0); //1.83 e 1.30

        pid_controller_update(3, targetDistance, filteredDistance);

        /*1300 +-50 +-400, 1000, 1800*/ 
        hoverFactor = constrain(HOVER_FACTOR + otoNewSet + PID.output[3], MIN_THROTTLE, MAX_THROTTLE); 
        hoverCounter ++;
      }

      /*take off*/
      if (hoverCounter == 35) {
        if ((200 - filteredDistance) < 0) {
          otoNewSet = otoNewSet - map(abs(200-filteredDistance), 0, 200, 0, 50); //50
        } else {
          otoNewSet = otoNewSet + map(abs(200-filteredDistance), 0, 200, 0, 50); //50
        }   

        otoNewSet = constrain(otoNewSet, -100, 100);
      }
    } else if (rawDistance > 2000) { /*measured distance over max. value?*/
      filteredDistance = round(targetDistance * 1.3);
    } else if (rawDistance < 0) { /*measured distance under min. value?*/
      filteredDistance = 0;
    }
  }

  if (statusHover) { /*hover mode enable*/
    if (hoverThrottle < round(hoverFactor)) hoverThrottle += 3;
    if (hoverThrottle > round(hoverFactor)) hoverThrottle -= 3;

    hoverThrottle = constrain(hoverThrottle, MIN_THROTTLE, MAX_THROTTLE);
  } else { /*hover mode disable*/
    hoverThrottle = MIN_THROTTLE;
    targetDistance = 200;

    hoverCounter = 0;
    hoverFactor = 0;
    otoNewSet = 0;

    PID.output[3] = 0;
    PID.i_term[3] = 0;
  }
}
#endif

/*******************************************************
  pin change interrupt (radio in PPM mode)
/*******************************************************/
ISR(PCINT0_vect) {
  static unsigned long previousPulseWidth = micros();
  static uint8_t channelIndex = 0;

  /*read pulse width PPM mode (PCINT3)*/
  if (PINB & 0b00000100) { 
    unsigned long pulseWidth = micros() - previousPulseWidth;
    if (pulseWidth < 0) pulseWidth += 0xFFFFFFFF;
    previousPulseWidth = micros();

    if (pulseWidth > 3000) channelIndex = 0;
    else channelIndex ++;

    channel[channelIndex] = pulseWidth;
  }

#if FAIL_SAFE
  if ((channelIndex >= 1) && (channelIndex <= 4)) {
    if (pulseWidth > FAILSAFE_THRESHOLD) {
      goodPulses |= (1 << channelIndex);
      if (goodPulses == 0x0F) counterFailSafe = 0;
    } else {
      counterFailSafe ++;
    }
  }
#endif
}
