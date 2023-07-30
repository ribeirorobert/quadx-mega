/*    
  QuadX Mega - v1.0.
  Created by Robert R. Gomes, May 17, 2023. 
  All rights reserved.
*/

#include <EEPROM.h>
#include <Wire.h>

#define SERIAL_BAUD_RATE    38400UL
#define I2C_CLOCK_SPEED     400000UL
#define I2C_TIMEOUT         4000UL

#define serialXBEE          Serial1   

#define TELEMETRY_ENABLE    0
#define SERIAL_ENABLE       0
#define PRINTF_ENABLE       0
#define BMP180_ENABLE       0
#define MS5611_ENABLE       1
#define FAIL_SAFE           0

#define DEBUG_STATUS        0
#define DEBUG_EEPROM        0
#define DEBUG_MPU           0
#define DEBUG_ESC           0

//#define BMP180
#define MS561101BA

#define MPU6050_ADDRESS     0x68
#define COMPASS_ADDRESS     0x1E
#define BMP180_ADDRESS      0x77
#define MS5611_ADDRESS      0x77
#define GYRO_ADDRESS        0x69
#define ACC_ADDRESS         0x53

#define LED_FRONT_LEFT      5
#define LED_FRONT_RIGHT     62
#define LED_REAR_LEFT       11
#define LED_REAR_RIGHT      48
#define VIRTUAL_GND         63

#define LED_YELLOW_PIN      42
#define LED_GREEN_PIN       40
#define LED_BLUE_PIN        44
#define LED_RED_PIN         38
#define BUZZER_PIN          23

#define BATTERY_PIN         A0
#define CLOCK_PIN           A14
#define LATCH_PIN           A10
#define DATA_PIN            A12 

#define WAITING_SAFETY_PERIOD       120000UL
#define CHANNEL_READ_PERIOD         100000UL
#define READ_BATTERY_PERIOD         2000000UL
#define IGNORE_CMDS_PERIOD          1000000UL
#define PWM_SAFETY_PERIOD           2100UL
#define TELEMETRY_PERIOD            1000000UL
#define BLINK_LED_PERIOD            1000000UL
#define CTRL_LOOP_PERIOD            4000UL
#define DISP_7SEG_PERIOD            4000UL
#define BUZZER_PERIOD               1000000UL
#define LEDS_PERIOD                 500000UL

#define MAX_RUNNING_TIME            900000UL

#define SENSITIVITY_SCALE_FACTOR    0.015267F //0.015267 = 1/65.5 [º/s]
#define BATTERY_UPPER_THRESHOLD     10.7F
#define BATTERY_LOWER_THRESHOLD     6.0F
#define BATTERY_COMPENSATION        0.000571F //1/3500
#define LEVEL_ADJUST_FACTOR         15.0F
#define DEGREES_TO_RADIAN           0.000001066F //0.000001066 = 0.0000611*(PI/180)
#define RADIAN_TO_DEGREES           57.295779F //57.295779 = 180/PI
#define ZERO_RATE_OUTPUT            0.0000611F //0.0000611 = 1/(250Hz/65.5) [º]
#define VOLTAGE_FACTOR              0.015285F //R2/R1+R2 = K = 0.319727, (5.0/1023) / K = 0.015285
#define HALF_LOOP_RATE              0.002F //sample time / 2

#define FAILSAFE_THRESHOLD          990
#define AUTO_LEVEL_MODE             1

#define QUADX_STATUS_UNDEFINED      0b00000000  
#define QUADX_RADIO_STARTUP_FAIL    0b00000001
#define QUADX_ESC_NOT_CALIBRATED    0b00000010
#define QUADX_EEPROM_READ_FAIL      0b00000100 
#define QUADX_GYRO_CALIBRATED       0b00001000
#define QUADX_ACC_CALIBRATED        0b00010000
#define QUADX_MPU6050_ERROR         0b00100000
#define QUADX_BMP180_ERROR          0b01000000
#define QUADX_MS5611_ERROR          0b01000000
#define QUADX_STATUS_OK             0b10000000

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

#if defined(MS561101BA)
#define MS5611_TEMPERATURE_ADDR     0x50
#define MS5611_PRESSURE_ADDR        0x40
#define MS5611_REG_RESET            0x1E
#define MS5611_OSR_4096             0x08

#define MS5611_WAIT_UT_UP_READ      10000UL
#endif

#define GYROX_OFFSET_EEPROM_ADDR    0x00
#define GYROY_OFFSET_EEPROM_ADDR    0x04
#define GYROZ_OFFSET_EEPROM_ADDR    0x08
#define ACCX_OFFSET_EEPROM_ADDR     0x0C
#define ACCY_OFFSET_EEPROM_ADDR     0x10
#define ACCZ_OFFSET_EEPROM_ADDR     0x14
#define ESC_CAL_EEPROM_ADDR         0xFF

#define TEMPERATURE_INDEX     5U
#define PARACHUTE_INDEX       30U
#define ALT_ERROR_INDEX       10U
#define PRESSURE_INDEX        20U

#define GYRO_OFFSET_INDEX     1000UL
#define ACC_OFFSET_INDEX      100U

#define ACCZ_SHORT_INDEX      25U
#define ACCZ_LONG_INDEX       50U
#define ACCZ_THRESHOLD        800UL

#define STICK_MIDPOINT    1500UL
#define STICK_UPPER_DB    1510.0F
#define STICK_LOWER_DB    1490.0F
#define MIN_THROTTLE      1000UL
#define MAX_THROTTLE      1800UL
#define LOCK_SPEED        1000UL   
#define IDLE_SPEED        1200UL 
#define FULL_SPEED        2000UL  

#define LOCK_THROTTLE     0x00
#define IDLE_THROTTLE     0x01
#define FULL_THROTTLE     0x02

#define MAX_PID_OUTPUT    400.0F
#define MAX_STICK_REF     150.0F

#define ALL_LEDS_OFF  0b00000000
#define ALL_LEDS_ON   0b00000001
#define LED_YELLOW    0b00000010
#define LED_GREEN     0b00000100
#define LED_BLUE      0b00001000
#define LED_RED       0b00010000
#define LED_FRONT     0b00100000
#define LED_REAR      0b01000000

// #define KP_ALT_HOLD   1.4
// #define KI_ALT_HOLD   0.2
// #define KD_ALT_HOLD   0.75

#define KP_ALT_HOLD   2.0
#define KI_ALT_HOLD   0.4
#define KD_ALT_HOLD   0.75

#define KP_PITCH      1.2
#define KI_PITCH      0.04
#define KD_PITCH      20.0

#define KP_ROLL       1.2
#define KI_ROLL       0.04
#define KD_ROLL       20.0

#define KP_YAW        4.0
#define KI_YAW        0.02

struct {
  uint8_t status;
  uint8_t motors;
  uint8_t leds;
  bool buzzer;
} quadX;

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
} MPU6050;

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

bool statusCalibrationEsc = false;
bool statusManualAltitude = false;
bool statusAltitudeHold = false;

uint8_t counterFailSafe = 0;
uint8_t debugPageIndex = 1;
uint8_t muxIndex = 0;

volatile uint16_t channel[9];

uint16_t currentThrottle, initialThrottle;
float adjustThrottle;
float manualThrottle;

float batteryVoltage;

//roll, pitch and yaw variables
float gyroRollInput, gyroPitchInput, gyroYawInput;
float rollAngle, pitchAngle;

float rollLevelAdjust, pitchLevelAdjust;
float rollAngleAcc, pitchAngleAcc;

//roll, pitch and yaw ctrl variables
float rollSetPoint, pitchSetPoint, yawSetPoint;

float errorRoll, errorPitch, errorYaw;
float pTermRoll, pTermPitch, pTermYaw;
float iTermRoll, iTermPitch, iTermYaw;
float dTermRoll, dTermPitch, dTermYaw;

float errorRollAnt, errorPitchAnt, errorYawAnt;
float pidRoll, pidPitch, pidYaw;

//altitude hold ctrl variables
float altHoldSetPoint, errorAltHold, pidAltHold;
float pTermAltHold, iTermAltHold, dTermAltHold;

//altitude hold variables
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

//time variables
unsigned long previousReadVoltagePeriod;
unsigned long previousIgnoreCmdsPeriod;
unsigned long previousCtrlLoopPeriod;
unsigned long previousBuzzerPeriod;
unsigned long previousBlinkPeriod;
unsigned long startQuadXTimer;

void setup() {
  //initialize serial
#if SERIAL_ENABLE  
  Serial.begin(SERIAL_BAUD_RATE);
  delay(1000);
#endif

#if TELEMETRY_ENABLE
  serialXBEE.begin(9600); //Rx, Tx
#endif

  //initialize I2C
  Wire.begin();
  Wire.setClock(I2C_CLOCK_SPEED); //TWBR = 12; set the I2C clock speed to 400kHz

  //inputs and outputs
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

  //virtual GND
  digitalWrite(VIRTUAL_GND, LOW);

  DDRE |= bit(DDE5); //data direction register PORTE.5 (ESC1)
  DDRB |= bit(DDB7); //data direction register PORTB.7 (ESC2)

  DDRH |= bit(DDH4); //data direction register PORTH.4 (ESC3)
  DDRH |= bit(DDH6); //data direction register PORTH.6 (ESC4)

  PCMSK0 |= bit(PCINT2); //want pin B2
  PCIFR |= bit(PCIF0); //clear any outstanding interrupts
  PCICR |= bit(PCIE0); //enable pin change interrupts PCMSK0

  //blink LEDs
  blink_leds();

  //quadX status
  quadX_status();

  //get battery voltage
  batteryVoltage = analogRead(BATTERY_PIN) * VOLTAGE_FACTOR;

  //update timers
  previousReadVoltagePeriod = micros();
  previousIgnoreCmdsPeriod = micros();
  previousCtrlLoopPeriod = micros();
  previousBuzzerPeriod = micros();
  previousBlinkPeriod = micros();
  startQuadXTimer = millis();
}

void loop() {
  //reading accelerometer and gyroscope angles
  acc_gyro_values();

  //accelerometer and gyroscope euler angles
  euler_angles();

  //read commands from radio   
  read_radio_commands();

  //read pressure and temperature
#if BMP180_ENABLE    
  BMP180_update();
#endif  

#if MS5611_ENABLE    
  //MS5611_update();
#endif  

  //altitude hold
#if MS5611_ENABLE  
  altitude_hold(3);
#endif  

  //roll, pitch and yaw PIDs controllers
  execute_pid_controllers();

  //read battery voltage
  read_battery_voltage();

  //show on display quadX status
  show_quad_status();

  //check alarms
  check_alarms();

  //ctrl loop period
  while (micros() - previousCtrlLoopPeriod < CTRL_LOOP_PERIOD);
  previousCtrlLoopPeriod = micros();

  //ESCs PWM signals
  esc_pwm_signals();  

  //print data on serial monitor debugPageIndex
#if PRINTF_ENABLE
  print_info(7);
#endif

  //telemetry parameters
#if TELEMETRY_ENABLE
  update_telemetry_parameters();
#endif
}

/*******************************************************
  quadX status
/*******************************************************/
void quadX_status(void) {
#if DEBUG_STATUS
  Serial.println(F("QUADX STARTING..."));
#endif

  //quadX default status
  quadX.status |= QUADX_STATUS_UNDEFINED;
  quadX.motors = LOCK_THROTTLE;
  quadX.leds = ALL_LEDS_OFF;
  quadX.buzzer = false;

  //read EEPROM data
#if DEBUG_STATUS
  Serial.print(F("->READING EEPROM: "));
#endif
  
  if (!read_eeprom()) {
#if DEBUG_STATUS
    Serial.println(F("ERROR"));
#endif
    quadX.status |= QUADX_EEPROM_READ_FAIL;
    fail_message();
  } else {
#if DEBUG_STATUS
    Serial.println(F("OK"));
#endif
    quadX.status |= QUADX_GYRO_CALIBRATED;
    quadX.status |= QUADX_ACC_CALIBRATED;
  }

  //ESCs calibration
#if DEBUG_STATUS
  Serial.print(F("->ESC CALIBRATION: "));
#endif

  if (statusCalibrationEsc) {
    if (!esc_calibration_signals()) {
#if DEBUG_STATUS
      Serial.println(F("ERROR"));
#endif
      quadX.status |= QUADX_ESC_NOT_CALIBRATED;
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

  //check if MPU6050 is ok
#if DEBUG_STATUS
  Serial.print(F("->CHECKING MPU6050: "));
#endif
  
  for (uint8_t i = 0; i < 10; i ++) {
    Wire.beginTransmission(MPU6050_ADDRESS);
    uint8_t status = Wire.endTransmission();

    if (status != 0) {
#if DEBUG_STATUS
      Serial.println(F("ERROR"));
#endif
      quadX.status |= QUADX_MPU6050_ERROR;
    } else {
#if DEBUG_STATUS
      Serial.println(F("OK"));
#endif
      MPU6050_init(); //IMU configuration
      quadX.status &= ~QUADX_MPU6050_ERROR;
      break;
    }
    delay(500);
  }

  //check if BMP180 is ok
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
      quadX.status |= QUADX_BMP180_ERROR;
    } else {
#if DEBUG_STATUS
      Serial.println(F("OK"));
#endif      
      BMP180_init();
      quadX.status &= ~QUADX_BMP180_ERROR;
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

  //check if MS561101BA is ok
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
      quadX.status |= QUADX_MS5611_ERROR;
    } else {
#if DEBUG_STATUS
      Serial.println(F("OK"));
#endif      
      MS5611_init();
      quadX.status &= ~QUADX_MS5611_ERROR;
      break;
    }
    delay(500);
  }

#endif

  //wait radio signals 
// #if DEBUG_STATUS  
//   Serial.print(F("->WAITING RADIO SIGNALS: "));
// #endif

//   if (!wait_radio_signals()) {
// #if DEBUG_STATUS
//     Serial.println(F("ERROR"));
// #endif
//     quadX.status |= QUADX_RADIO_STARTUP_FAIL;
//     fail_message();
//   } else {
// #if DEBUG_STATUS
//     Serial.println(F("OK"));
// #endif
//   }

  //bip motors
#if DEBUG_STATUS
  Serial.println(F("->BIP MOTORS"));
#endif

  for (uint16_t i = 0; i < 500; i ++) {
    PORTE |= (1 << PORTE5); PORTB |= (1 << PORTB7);
    PORTH |= (1 << PORTH4); PORTH |= (1 << PORTH6);
    delayMicroseconds(1000);

    PORTE &= ~(1 << PORTE5); PORTB &= ~(1 << PORTB7);
    PORTH &= ~(1 << PORTH4); PORTH &= ~(1 << PORTH6);
    delayMicroseconds(3000);

    seven_seg(0b11000001, 0b10011111, 0b00110001, 0b11111111);
  }

  if (quadX.status & QUADX_EEPROM_READ_FAIL) {
    for (uint8_t i = 0; i < sizeof(fail); i ++) 
      msg[i] = fail[i];
    muxIndex += sizeof(fail);

    for (uint8_t i = muxIndex, k = 0; i < muxIndex + sizeof(eeprom); i ++, k ++) 
      msg[i] = eeprom[k];
    muxIndex += sizeof(eeprom);
  }

  if (quadX.status & QUADX_MPU6050_ERROR) {
    for (uint8_t i = muxIndex, k = 0; i < muxIndex + sizeof(fail); i ++, k ++) 
      msg[i] = fail[k];
    muxIndex += sizeof(fail);

    for (uint8_t i = muxIndex, k = 0; i < muxIndex + sizeof(mpu); i ++, k ++) 
      msg[i] = mpu[k];
    muxIndex += sizeof(mpu);
  }

#if defined(BMP180)
  if (quadX.status & QUADX_BMP180_ERROR) {
    for (uint8_t i = muxIndex, k = 0; i < muxIndex + sizeof(fail); i ++, k ++) 
      msg[i] = fail[k];
    muxIndex += sizeof(fail);

    for (uint8_t i = muxIndex, k = 0; i < muxIndex + sizeof(bmp); i ++, k ++) 
      msg[i] = bmp[k];
    muxIndex += sizeof(bmp);
  }
#endif

#if defined(MS561101BA)
  if (quadX.status & QUADX_MS5611_ERROR) {
    for (uint8_t i = muxIndex, k = 0; i < muxIndex + sizeof(fail); i ++, k ++) 
      msg[i] = fail[k];
    muxIndex += sizeof(fail);

    for (uint8_t i = muxIndex, k = 0; i < muxIndex + sizeof(ms); i ++, k ++) 
      msg[i] = ms[k];
    muxIndex += sizeof(ms);
  }
#endif

  if (quadX.status & QUADX_RADIO_STARTUP_FAIL) {
    for (uint8_t i = muxIndex, k = 0; i < muxIndex + sizeof(fail); i ++, k ++) 
      msg[i] = fail[k];
    muxIndex += sizeof(fail);

    for (uint8_t i = muxIndex, k = 0; i < muxIndex + sizeof(radio); i ++, k ++) 
      msg[i] = radio[k];
    muxIndex += sizeof(radio);
  }

  if (quadX.status & QUADX_ESC_NOT_CALIBRATED) {
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
    quadX.status |= QUADX_STATUS_OK;
  } else {
#if DEBUG_STATUS
    Serial.println(F("RESULT: ERROR"));
#endif
  }
  
  quadX.leds |= LED_YELLOW | LED_FRONT | LED_REAR;
}

/*******************************************************
  MPU6050 configuration
/*******************************************************/
void MPU6050_init(void) {
  Wire.beginTransmission(MPU6050_ADDRESS);     
  Wire.write(0x6B); //PWR_MGMT_1
  Wire.write(0x00); //DEVICE_RESET; SLEEP; CYCLE; TEMP_DIS; CLKSEL (00000000, all disable)
  Wire.endTransmission();      

  Wire.beginTransmission(MPU6050_ADDRESS);     
  Wire.write(0x1A); //CONFIG
  Wire.write(0x03); //EXT_SYNC_SET (000, FSYNC input disable); DLPF_CFG (011, ACC bandwidth = 44Hz and GYRO bandwidth = 42Hz)
  Wire.endTransmission();         

  Wire.beginTransmission(MPU6050_ADDRESS);     
  Wire.write(0x1B); //GYRO_CONFIG
  Wire.write(0x08); //XG_ST,YG_ST,ZG_ST (000, disable self test); FS_SEL (1, 01, +- 500°/s [65.5 LSB/°/s] full scale range); ---;
  Wire.endTransmission();  

  Wire.beginTransmission(MPU6050_ADDRESS);     
  Wire.write(0x1C); //ACCEL_CONFIG
  Wire.write(0x10); ///XA_ST,YA_ST,ZA_ST (000, disable self test); AFS_SEL (2, 10, +- 8g [4096 LSB/g] full scale range); ---;
  Wire.endTransmission();                  
}

/*******************************************************
  accelerometer and gyroscope values
/*******************************************************/
bool acc_gyro_values(void) {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x3B);                      //start reading register 43h and auto increment with every read
  Wire.endTransmission();
  Wire.requestFrom(MPU6050_ADDRESS, 14); //request 14 bytes from the MPU6050

  //reading the low and high bytes of each angle
  if (Wire.available() != 14) return false;

  MPU6050.accX  = (Wire.read() << 8 | Wire.read());      
  MPU6050.accY  = (Wire.read() << 8 | Wire.read());  
  MPU6050.accZ  = (Wire.read() << 8 | Wire.read());     
  MPU6050.temp  = (Wire.read() << 8 | Wire.read());      
  MPU6050.gyroX = (Wire.read() << 8 | Wire.read()); 
  MPU6050.gyroY = (Wire.read() << 8 | Wire.read()); 
  MPU6050.gyroZ = (Wire.read() << 8 | Wire.read()); 

  //invert the direction of the axis
  MPU6050.gyroX *= -1.0;
  MPU6050.gyroY *=  1.0;
  MPU6050.gyroZ *= -1.0;

  //subtract gyroscope calibration value 
  MPU6050.gyroX -= MPU6050.gyroXOffset.val;
  MPU6050.gyroY -= MPU6050.gyroYOffset.val;
  MPU6050.gyroZ -= MPU6050.gyroZOffset.val;

  //subtract accelerometer calibration value
  MPU6050.accX -= MPU6050.accXOffset.val;
  MPU6050.accY -= MPU6050.accYOffset.val;

  return true;
}

/*******************************************************
  euler angles
/*******************************************************/
void euler_angles(void) {
  //PID gyro angles filtered
  gyroRollInput  = (gyroRollInput  * 0.7) + (MPU6050.gyroX * SENSITIVITY_SCALE_FACTOR * 0.3);
  gyroPitchInput = (gyroPitchInput * 0.7) + (MPU6050.gyroY * SENSITIVITY_SCALE_FACTOR * 0.3);
  gyroYawInput   = (gyroYawInput   * 0.7) + (MPU6050.gyroZ * SENSITIVITY_SCALE_FACTOR * 0.3);

  //gyro angles calculation
  rollAngle  += (MPU6050.gyroX * ZERO_RATE_OUTPUT); 
  pitchAngle += (MPU6050.gyroY * ZERO_RATE_OUTPUT);

  pitchAngle -= rollAngle  * sin(MPU6050.gyroZ * DEGREES_TO_RADIAN); //if the IMU has yawed transfer the pitch angle to the roll angel
  rollAngle  += pitchAngle * sin(MPU6050.gyroZ * DEGREES_TO_RADIAN); //if the IMU has yawed transfer the roll angle to the pitch angel

  //accelerometer angles calculations
  float accTotalVector = sqrt((MPU6050.accX * MPU6050.accX) + (MPU6050.accY * MPU6050.accY) + (MPU6050.accZ * MPU6050.accZ));
  if (abs(MPU6050.accY) < accTotalVector) {
    pitchAngleAcc = asin((float)MPU6050.accY / accTotalVector) * RADIAN_TO_DEGREES;
  }
  if (abs(MPU6050.accX) < accTotalVector) {
    rollAngleAcc = asin((float)MPU6050.accX / accTotalVector) * RADIAN_TO_DEGREES * (-1); 
  }

  rollAngle  = (rollAngle  * 0.9996) + (rollAngleAcc  * 0.0004); //correct the drift of the gyro roll angle with the accelerometer roll angle
  pitchAngle = (pitchAngle * 0.9996) + (pitchAngleAcc * 0.0004); //correct the drift of the gyro pitch angle with the accelerometer pitch angle

  //calculate roll and pitch angle correction
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
bool read_radio_commands(void) {
  if (micros() - previousIgnoreCmdsPeriod < IGNORE_CMDS_PERIOD) return false;
  if (quadX.status & QUADX_RADIO_STARTUP_FAIL) return false;

  //debug pages control
  // if (quadX.motors == LOCK_THROTTLE) {
  //   if (channel[1] > 1900) { //next debug page
  //     while (channel[1] > 1900) waiting_knob_release();
  //     debugPageIndex ++;
  //   }
    
  //   if (channel[1] < 1100) { //previous debug page
  //     while (channel[1] < 1100) waiting_knob_release();
  //     debugPageIndex --;
  //   }

  //   debugPageIndex = constrain(debugPageIndex, 1, 7);
  // }

  //altitude hold control
#if MS5611_ENABLE
  if (channel[8] > 1600) {
    statusAltitudeHold = true; 
  } else {
    statusAltitudeHold = false; 
  }
#endif
  
  //frame LEDs control
  if (channel[5] > 1600) {
    quadX.leds |= LED_YELLOW | LED_FRONT | LED_REAR;
  } else {
    quadX.leds &= ~LED_YELLOW & ~LED_FRONT & ~LED_REAR;
  }

  //calibration options
  if (quadX.motors == LOCK_THROTTLE) {
    //gyroscope calibration
    if ((channel[1] > 1900) && (channel[2] > 1900)) {
      while ((channel[1] > 1900) && (channel[2] > 1900)) waiting_knob_release();

      while (true) {
        if ((channel[1] > 1900) && (channel[2] > 1900)) {
          if (calibrate_gyroscope()) {
            quadX.status |= QUADX_GYRO_CALIBRATED;
          } else {
            quadX.status &= ~QUADX_GYRO_CALIBRATED;
            fail_message();
          }
          break;
        }

        if ((channel[1] > 1900) && (channel[2] < 1100)) {
          while ((channel[1] > 1900) && (channel[2] < 1100)) waiting_knob_release();
          break;
        }

        seven_seg(0b01000001, 0b11110111, 0b11110101, 0b11000101);
      }

      quadX.leds |= LED_YELLOW | LED_FRONT | LED_REAR;
      quadX.buzzer = true;
    }

    //accelerometer calibration
    if ((channel[1] < 1100) && (channel[2] > 1900)) { 
      while ((channel[1] < 1100) && (channel[2] > 1900)) waiting_knob_release();

      while (true) { 
        if ((channel[1] < 1100) && (channel[2] > 1900)) {
          while ((channel[1] < 1100) && (channel[2] > 1900)) waiting_knob_release();
          if (calibrate_accelerometer()) {
            quadX.status |= QUADX_ACC_CALIBRATED;
          } else {
            quadX.status &= ~QUADX_ACC_CALIBRATED;
            fail_message();
          }
          break;
        }

        if ((channel[1] > 1900) && (channel[2] < 1100)) {
          while ((channel[1] > 1900) && (channel[2] < 1100)) waiting_knob_release();
          break;
        }

        seven_seg(0b00010001, 0b01100011, 0b01100011, 0b11111111);
      }

      quadX.leds |= LED_YELLOW | LED_FRONT | LED_REAR;
      quadX.buzzer = true;
    }

    //ESCs calibration
    if ((channel[1] < 1100) && (channel[2] < 1100)) {
      while ((channel[1] < 1100) && (channel[2] < 1100)) waiting_knob_release();

      while (true) {
        if ((channel[1] < 1100) && (channel[2] < 1100)) { //yes
          while ((channel[1] < 1100) && (channel[2] < 1100)) waiting_knob_release();

          EEPROM.write(ESC_CAL_EEPROM_ADDR, 1);
          break;
        }

        if ((channel[1] > 1900) && (channel[2] < 1100)) { //no
          while ((channel[1] > 1900) && (channel[2] < 1100)) waiting_knob_release();

          EEPROM.write(ESC_CAL_EEPROM_ADDR, 0);
          break;
        }

        seven_seg(0b01100001, 0b01001001, 0b01100011, 0b11111111);
      }

      quadX.leds |= LED_YELLOW | LED_FRONT | LED_REAR;
      quadX.buzzer = true;
    }
  }

  //brushless control
  if ((quadX.status & QUADX_STATUS_OK) && (quadX.status & QUADX_GYRO_CALIBRATED) && 
      (quadX.status & QUADX_ACC_CALIBRATED)) {
    //idle brushless motors
    if ((quadX.motors == LOCK_THROTTLE) && (channel[3] < 1100) && (channel[4] < 1100)) {
      while ((channel[3] < 1100) && (channel[4] < 1100)) waiting_knob_release();

      quadX.motors = IDLE_THROTTLE;
      quadX.buzzer = true;
    } 

    //brushless motors running
    if ((quadX.motors == IDLE_THROTTLE) && (channel[3] < 1100) && (channel[4] < 1100)) {
      while ((channel[3] < 1100) && (channel[4] < 1100)) waiting_knob_release();

      reset_variables();
      quadX.motors = FULL_THROTTLE;
    } 
    
    //turn off brushless motors
    if ((channel[3] < 1100) && (channel[4] > 1900)) {
      quadX.motors = LOCK_THROTTLE;
    } 
  }

  return true;
}

/*******************************************************
  waiting knob release
/*******************************************************/
void waiting_knob_release(void) {
  static unsigned long counterWaitingKnob;
  static bool flagWaitingKnob = false;

  if (millis() - counterWaitingKnob >= 200) {
    counterWaitingKnob = millis();
    flagWaitingKnob = !flagWaitingKnob;
  }

  if (flagWaitingKnob) seven_seg(0b11111101, 0b11111101, 0b11111101, 0b11111101);
  else seven_seg(0b11111111, 0b11111111, 0b11111111, 0b11111111);
}

/*******************************************************
  execute PID controllers
/*******************************************************/
void execute_pid_controllers(void) {
  /*ROLL PID*/

  /*reference*/
  rollSetPoint = (((channel[1] - (float)STICK_MIDPOINT) - rollLevelAdjust) / 3.0);

  /*dead band*/
  rollSetPoint = apply_dead_band(rollSetPoint, 15);

  /*error*/
  errorRoll = gyroRollInput - rollSetPoint;

  /*proportional term*/
  pTermRoll =  KP_ROLL * errorRoll;

  /*integral term*/
  iTermRoll += KI_ROLL * (errorRoll + errorRollAnt) * 0.5;

  /*anti-windup*/
  iTermRoll = constrain(iTermRoll, -MAX_PID_OUTPUT, MAX_PID_OUTPUT);

  /*derivative term*/
  dTermRoll =  KD_ROLL * (errorRoll - errorRollAnt);

  /*output*/
  pidRoll = pTermRoll + iTermRoll + dTermRoll;

  /*output saturation*/
  pidRoll = constrain(pidRoll, -MAX_PID_OUTPUT, MAX_PID_OUTPUT);

  /*update previous variables*/
  errorRollAnt = errorRoll;


  /*PITCH PID*/

  /*reference*/
  pitchSetPoint = (((channel[2] - (float)STICK_MIDPOINT) - pitchLevelAdjust) / 3.0);

  /*dead band*/
  pitchSetPoint = apply_dead_band(pitchSetPoint, 15);  

  /*error*/
  errorPitch = gyroPitchInput - pitchSetPoint;

  /*proportional term*/
  pTermPitch =  KP_PITCH * errorPitch;

  /*integral term*/
  iTermPitch += KI_PITCH * (errorPitch + errorPitchAnt) * 0.5;

  /*anti-windup*/
  iTermPitch = constrain(iTermPitch, -MAX_PID_OUTPUT, MAX_PID_OUTPUT);

  /*derivative term*/
  dTermPitch =  KD_PITCH * (errorPitch - errorPitchAnt);

  /*output*/
  pidPitch = pTermPitch + iTermPitch + dTermPitch;

  /*output saturantion*/
  pidPitch = constrain(pidPitch, -MAX_PID_OUTPUT, MAX_PID_OUTPUT);

  /*update previous varibles*/
  errorPitchAnt = errorPitch;


  /*YAW PID*/

  /*reference*/
  (channel[3] > 1100) ? yawSetPoint = ((channel[4] - (float)STICK_MIDPOINT) / 3.0) : yawSetPoint = 0;

  /*dead band*/
  yawSetPoint = apply_dead_band(yawSetPoint, 20); 

  /*error*/
  errorYaw = gyroYawInput - yawSetPoint;

  /*proportional term*/
  pTermYaw =  KP_YAW * errorYaw;

  /*integral term*/
  iTermYaw += KI_YAW * (errorYaw + errorYawAnt) * 0.5;

  /*anti-windup*/
  iTermYaw = constrain(iTermYaw, -MAX_PID_OUTPUT, MAX_PID_OUTPUT);

  /*output*/
  pidYaw = pTermYaw + iTermYaw;

  /*output saturation*/
  pidYaw = constrain(pidYaw, -MAX_PID_OUTPUT, MAX_PID_OUTPUT);

  /*update previous variables*/
  errorYawAnt = errorYaw;
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
  read_battery_voltage
/*******************************************************/
void read_battery_voltage(void) {
  if (micros() - previousReadVoltagePeriod >= READ_BATTERY_PERIOD) {
    previousReadVoltagePeriod = micros();

    float voltage = analogRead(BATTERY_PIN) * VOLTAGE_FACTOR;
    batteryVoltage = (batteryVoltage * 0.95) + (voltage * 0.05);
  }
}

/*******************************************************
  show quad status
/*******************************************************/
void show_quad_status(void) {
  static unsigned long previousStatusQuadPeriod;
  static uint8_t quadIndex = 1;

  if (quadX.status & QUADX_STATUS_OK) {
    if ((millis() - previousStatusQuadPeriod >= 5000) && (quadIndex == 0)) {
      previousStatusQuadPeriod = millis();
      quadIndex = 1;
    } else if ((millis() - previousStatusQuadPeriod >= 1000) && (quadIndex == 1)) {
      previousStatusQuadPeriod = millis();
      quadIndex = 2;
    } else if ((millis() - previousStatusQuadPeriod >= 5000) && (quadIndex == 2)) {
      previousStatusQuadPeriod = millis();
      quadIndex = 3;
    } else if ((millis() - previousStatusQuadPeriod >= 1000) && (quadIndex == 3)) {
      previousStatusQuadPeriod = millis();
      quadIndex = 0;
    }

    if ((quadIndex == 0) || (quadIndex == 2)) {
      uint16_t volt = (uint16_t)(batteryVoltage * 100);

      uint8_t n1 = (volt / 1000);
      uint8_t n2 = (volt % 1000) * 0.01;
      uint8_t n3 = (volt % 100) * 0.1;
      uint8_t n4 = (volt % 10);

      seven_seg(numbers[n1], numbers[n2] & 0b11111110, numbers[n3], numbers[n4]);
    } else if (quadIndex == 1) {
      seven_seg(0b11000001, 0b00010001, 0b11100001, 0b11100001);
    } else if (quadIndex == 3) {
      if (quadX.motors == LOCK_THROTTLE) seven_seg(0b11100011, 0b11000101, 0b11100101, 0b11110001);
      if (quadX.motors == IDLE_THROTTLE) seven_seg(0b10011111, 0b10000101, 0b11100011, 0b01100001);
      if (quadX.motors == FULL_THROTTLE) seven_seg(0b01110001, 0b10000011, 0b11100011, 0b11100011);
    }
  } else {
    seven_seg_mux();
  }
}

/*******************************************************
  check alarms
/*******************************************************/
void check_alarms(void) {
  static unsigned long previousBipBuzzerPeriod;
  static uint8_t buzzerCounter = 0;

  //bip buzzer, low battery or after 3 minutes
  if ((batteryVoltage > BATTERY_LOWER_THRESHOLD) && (batteryVoltage < BATTERY_UPPER_THRESHOLD)) {
    quadX.leds &= ~LED_BLUE;
    if (micros() - previousBuzzerPeriod >= BUZZER_PERIOD) {
      previousBuzzerPeriod = micros();
      digitalWrite(BUZZER_PIN, !digitalRead(BUZZER_PIN));
    }
  } else if ((!quadX.buzzer) && (millis() - startQuadXTimer < MAX_RUNNING_TIME)) {
    quadX.leds |= LED_BLUE;
    digitalWrite(BUZZER_PIN, LOW);
  }

  //bip buzzer
  if ((quadX.buzzer) && (batteryVoltage > BATTERY_UPPER_THRESHOLD) && 
      (millis() - previousBipBuzzerPeriod >= 50)) {
    previousBipBuzzerPeriod = millis();
    digitalWrite(BUZZER_PIN, !digitalRead(BUZZER_PIN));

    buzzerCounter ++;
    if (buzzerCounter >= 5) {
      digitalWrite(BUZZER_PIN, LOW);
      quadX.buzzer = false;
      buzzerCounter = 0;
    }
  }

  //bip buzzer
  if ((millis() - startQuadXTimer >= MAX_RUNNING_TIME) && 
      (millis() - previousBipBuzzerPeriod >= 200)) {
    previousBipBuzzerPeriod = millis();
    digitalWrite(BUZZER_PIN, !digitalRead(BUZZER_PIN));
  }

  //blink LED green
  if (micros() - previousBlinkPeriod >= BLINK_LED_PERIOD) {
    previousBlinkPeriod = micros();
    quadX.leds ^= LED_GREEN;
  }

  //turn on LED red if ctrl loop exceed 4ms
  (micros() - previousCtrlLoopPeriod > CTRL_LOOP_PERIOD) ? quadX.leds |= LED_RED : quadX.leds &= ~LED_RED;

  //switch LEDs states
  toggle_leds();
}

/*******************************************************
  esc pwm signals
/*******************************************************/
void esc_pwm_signals(void) {
  uint16_t esc1, esc2, esc3, esc4;
  
  //lock
  if (quadX.motors == LOCK_THROTTLE) {
    esc1 = LOCK_SPEED; esc2 = LOCK_SPEED;
    esc3 = LOCK_SPEED; esc4 = LOCK_SPEED;
  }

  //idle
  if (quadX.motors == IDLE_THROTTLE) {
    esc1 = IDLE_SPEED; esc2 = IDLE_SPEED;
    esc3 = IDLE_SPEED; esc4 = IDLE_SPEED;
  } 
  
  //full
  if (quadX.motors == FULL_THROTTLE) {
    if (statusAltitudeHold) {
      currentThrottle = 1500.0 + adjustThrottle + manualThrottle + pidAltHold;
    } else {
      currentThrottle = channel[3];
    }

    if (currentThrottle > MAX_THROTTLE) currentThrottle = MAX_THROTTLE;

    esc1 = (uint16_t)(currentThrottle - pidPitch + pidRoll - pidYaw); //pulse width for ESC 1 front-right - CCW
    esc2 = (uint16_t)(currentThrottle + pidPitch + pidRoll + pidYaw); //pulse width for ESC 2 rear-right  - CW
    esc3 = (uint16_t)(currentThrottle + pidPitch - pidRoll - pidYaw); //pulse width for ESC 3 rear-left   - CCW
    esc4 = (uint16_t)(currentThrottle - pidPitch - pidRoll + pidYaw); //pulse width for ESC 4 front-left  - CW

    if (((batteryVoltage > 8.0) && (batteryVoltage < 12.40)) && (!statusAltitudeHold)) {
      esc1 += esc1 * ((12.40 - batteryVoltage) * BATTERY_COMPENSATION);
      esc2 += esc2 * ((12.40 - batteryVoltage) * BATTERY_COMPENSATION);
      esc3 += esc3 * ((12.40 - batteryVoltage) * BATTERY_COMPENSATION);
      esc4 += esc4 * ((12.40 - batteryVoltage) * BATTERY_COMPENSATION);
    }

    //lower and upper speed limit
    esc1 = constrain(esc1, IDLE_SPEED, FULL_SPEED);
    esc2 = constrain(esc2, IDLE_SPEED, FULL_SPEED);
    esc3 = constrain(esc3, IDLE_SPEED, FULL_SPEED);
    esc4 = constrain(esc4, IDLE_SPEED, FULL_SPEED);
  } 

  virtual_pwm_generate(esc1, esc2, esc3, esc4);
}

/*******************************************************
  virtual pwm generate
/*******************************************************/
void virtual_pwm_generate(uint16_t esc1, uint16_t esc2, uint16_t esc3, uint16_t esc4) {
  unsigned long startTime = micros(); 
  unsigned long currTime;
  uint8_t flagEscLoop = 0x0F;

  PORTE |= (1 << PORTE5); PORTB |= (1 << PORTB7);
  PORTH |= (1 << PORTH4); PORTH |= (1 << PORTH6);

  while (flagEscLoop > 0x00) {
    currTime = micros();
    if (currTime >= startTime + esc1) {
      PORTE &= ~(1 << PORTE5);
      flagEscLoop &= 0b00001110;
    }

    if (currTime >= startTime + esc2) {
      PORTB &= ~(1 << PORTB7);
      flagEscLoop &= 0b00001101;
    }

    if (currTime >= startTime + esc3) {
      PORTH &= ~(1 << PORTH4);
      flagEscLoop &= 0b00001011;
    }

    if (currTime >= startTime + esc4) {
      PORTH &= ~(1 << PORTH6);
      flagEscLoop &= 0b00000111;
    }

    if (currTime - startTime >= PWM_SAFETY_PERIOD) {
      PORTE &= ~(1 << PORTE5); PORTB &= ~(1 << PORTB7);
      PORTH &= ~(1 << PORTH4); PORTH &= ~(1 << PORTH6);
      flagEscLoop &= 0b00000000;
      break;
    }
  }
}

/*******************************************************
  esc calibration signals
/*******************************************************/
bool esc_calibration_signals(void) {
  unsigned long startEscCalibrationTime = millis();
  uint8_t flagThrottleMoved = 0;
  uint16_t i = 0;
  uint8_t k = 0;

  while (true) {
    virtual_pwm_generate(channel[3], channel[3], channel[3], channel[3]);

    if ((flagThrottleMoved == 0) && (channel[3] > 1850)) flagThrottleMoved = 1;
    if ((flagThrottleMoved == 1) && (channel[3] < 1100)) flagThrottleMoved = 2;

    //exit
    if ((channel[1] > 1900) && (channel[2] < 1100)) {
      virtual_pwm_generate(LOCK_SPEED, LOCK_SPEED, LOCK_SPEED, LOCK_SPEED);
      if (flagThrottleMoved == 2) return true;
      else return false;
    }

    //if passed 1min, exit
    if (millis() - startEscCalibrationTime >= WAITING_SAFETY_PERIOD) {
      return false;
    }

    i ++;
    if (i % 50 == 0) {
      (k >= 2) ? k = 0 : k ++;
    }

    if (k == 0) seven_seg(0b01111111, 0b01111111, 0b01111111, 0b01111111);
    if (k == 1) seven_seg(0b11111101, 0b11111101, 0b11111101, 0b11111101);
    if (k == 2) seven_seg(0b11101111, 0b11101111, 0b11101111, 0b11101111);
    
    delayMicroseconds(4000);
  }
}

/*******************************************************
  reset PID variables
/*******************************************************/
void reset_variables(void) {
  rollAngle  = rollAngleAcc;
  pitchAngle = pitchAngleAcc;

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
}

/*******************************************************
  calibrate gyroscope
/*******************************************************/
bool calibrate_gyroscope(void) {
  uint8_t k = 0;

  int32_t gyroXCalib = 0;
  int32_t gyroYCalib = 0;
  int32_t gyroZCalib = 0;

  //reset previous offset
  MPU6050.gyroXOffset.val = 0;
  MPU6050.gyroYOffset.val = 0;
  MPU6050.gyroZOffset.val = 0;

#if DEBUG_MPU
  Serial.println(F("CALIBRATING GYROSCOPE..."));
#endif

  for (uint16_t i = 0; i < GYRO_OFFSET_INDEX; i ++) {
    if (i % 50 == 0) {
      if (channel[5] > 1500) quadX.leds ^= LED_GREEN | LED_FRONT | LED_REAR;
      else quadX.leds &= ~LED_YELLOW & ~LED_FRONT & ~LED_REAR;

      toggle_leds();

      (k >= 2) ? k = 0 : k ++;
    }

    if (k == 0) seven_seg(0b01111111, 0b01111111, 0b01111111, 0b01111111);
    if (k == 1) seven_seg(0b11111101, 0b11111101, 0b11111101, 0b11111101);
    if (k == 2) seven_seg(0b11101111, 0b11101111, 0b11101111, 0b11101111);

#if DEBUG_MPU
    Serial.print(F("["));
    Serial.print(i + 1);
    Serial.print(F("]: "));
    Serial.print(MPU6050.gyroX);
    Serial.print(F("  "));
    Serial.print(MPU6050.gyroY);
    Serial.print(F("  "));
    Serial.println(MPU6050.gyroZ);
#endif

    acc_gyro_values();    
    virtual_pwm_generate(LOCK_SPEED, LOCK_SPEED, LOCK_SPEED, LOCK_SPEED);

    gyroXCalib += MPU6050.gyroX;   
    gyroYCalib += MPU6050.gyroY;  
    gyroZCalib += MPU6050.gyroZ;   
    delayMicroseconds(4000);                      
  }

  MPU6050.gyroXOffset.val = (int32_t)((float)gyroXCalib / (float)GYRO_OFFSET_INDEX);
  MPU6050.gyroYOffset.val = (int32_t)((float)gyroYCalib / (float)GYRO_OFFSET_INDEX);
  MPU6050.gyroZOffset.val = (int32_t)((float)gyroZCalib / (float)GYRO_OFFSET_INDEX);

#if DEBUG_MPU
  Serial.println(F("GYROSCOPE OFFSETs"));
  Serial.print(F("X: "));
  Serial.print(MPU6050.gyroXOffset.val);
  Serial.print(F(" Y: "));
  Serial.print(MPU6050.gyroYOffset.val);
  Serial.print(F(" Z: "));
  Serial.println(MPU6050.gyroZOffset.val);
#endif

  if ((MPU6050.gyroXOffset.val < -255) || (MPU6050.gyroXOffset.val > 255) ||
      (MPU6050.gyroYOffset.val < -255) || (MPU6050.gyroYOffset.val > 255) ||
      (MPU6050.gyroZOffset.val < -255) || (MPU6050.gyroZOffset.val > 255)) {
#if DEBUG_MPU
    Serial.println(F("GYROSCOPE NOT CALIBRATED"));
#endif
    MPU6050.gyroXOffset.val = 0;
    MPU6050.gyroYOffset.val = 0;
    MPU6050.gyroZOffset.val = 0;

    for (uint8_t i = GYROX_OFFSET_EEPROM_ADDR; i <= GYROZ_OFFSET_EEPROM_ADDR+3; i ++) {
      EEPROM.write(i, 0);
    }
    return false;
  } else {
#if DEBUG_MPU
    Serial.println(F("GYROSCOPE CALIBRATED"));
#endif

    EEPROM.write(GYROX_OFFSET_EEPROM_ADDR,   MPU6050.gyroXOffset.raw[3]);
    EEPROM.write(GYROX_OFFSET_EEPROM_ADDR+1, MPU6050.gyroXOffset.raw[2]);
    EEPROM.write(GYROX_OFFSET_EEPROM_ADDR+2, MPU6050.gyroXOffset.raw[1]);
    EEPROM.write(GYROX_OFFSET_EEPROM_ADDR+3, MPU6050.gyroXOffset.raw[0]);

    EEPROM.write(GYROY_OFFSET_EEPROM_ADDR,   MPU6050.gyroYOffset.raw[3]);
    EEPROM.write(GYROY_OFFSET_EEPROM_ADDR+1, MPU6050.gyroYOffset.raw[2]);
    EEPROM.write(GYROY_OFFSET_EEPROM_ADDR+2, MPU6050.gyroYOffset.raw[1]);
    EEPROM.write(GYROY_OFFSET_EEPROM_ADDR+3, MPU6050.gyroYOffset.raw[0]);

    EEPROM.write(GYROZ_OFFSET_EEPROM_ADDR,   MPU6050.gyroZOffset.raw[3]);
    EEPROM.write(GYROZ_OFFSET_EEPROM_ADDR+1, MPU6050.gyroZOffset.raw[2]);
    EEPROM.write(GYROZ_OFFSET_EEPROM_ADDR+2, MPU6050.gyroZOffset.raw[1]);
    EEPROM.write(GYROZ_OFFSET_EEPROM_ADDR+3, MPU6050.gyroZOffset.raw[0]);

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
  MPU6050.accXOffset.val = 0; 
  MPU6050.accYOffset.val = 0;

#if DEBUG_MPU
  Serial.println(F("CALIBRATING ACCELEROMETER..."));
#endif

  for (uint8_t i = 0; i < ACC_OFFSET_INDEX; i ++) {
    if (i % 10 == 0) {
      if (channel[5] > 1500) quadX.leds ^= LED_GREEN | LED_FRONT | LED_REAR;
      else quadX.leds &= ~LED_YELLOW & ~LED_FRONT & ~LED_REAR;

      (k >= 2) ? k = 0 : k ++;
    }

    if (k == 0) seven_seg(0b01111111, 0b01111111, 0b01111111, 0b01111111);
    if (k == 1) seven_seg(0b11111101, 0b11111101, 0b11111101, 0b11111101);
    if (k == 2) seven_seg(0b11101111, 0b11101111, 0b11101111, 0b11101111);

#if DEBUG_MPU
    Serial.print(F("["));
    Serial.print(i + 1);
    Serial.print(F("]: "));
    Serial.print(MPU6050.accX);
    Serial.print(F("  "));
    Serial.println(MPU6050.accY);
#endif

    acc_gyro_values();    
    virtual_pwm_generate(LOCK_SPEED, LOCK_SPEED, LOCK_SPEED, LOCK_SPEED);

    accXCal += MPU6050.accX;   
    accYCal += MPU6050.accY;  

    if ((MPU6050.accX < -500) || (MPU6050.accX > 500) || 
        (MPU6050.accY < -500) || (MPU6050.accY > 500)) {
#if DEBUG_MPU
      Serial.println(F("ACCELEROMETER NOT CALIBRATED"));
#endif
      MPU6050.accXOffset.val = 0;   
      MPU6050.accYOffset.val = 0;  

      for (uint8_t i = ACCX_OFFSET_EEPROM_ADDR; i <= ACCY_OFFSET_EEPROM_ADDR+3; i ++) {
        EEPROM.write(i, 0);
      }
      return false;
    }

    delayMicroseconds(4000);                      
  }

  MPU6050.accXOffset.val = (int32_t)((float)accXCal / (float)ACC_OFFSET_INDEX);
  MPU6050.accYOffset.val = (int32_t)((float)accYCal / (float)ACC_OFFSET_INDEX);

#if DEBUG_MPU
  Serial.println(F("ACCELEROMETER OFFSETs"));
  Serial.print(F("ACC X: "));
  Serial.print(MPU6050.accXOffset.val);
  Serial.print(F(" ACC Y: "));
  Serial.println(MPU6050.accYOffset.val);
#endif

#if DEBUG_MPU
  Serial.println(F("ACCELEROMETER CALIBRATED"));
#endif

  EEPROM.write(ACCX_OFFSET_EEPROM_ADDR,   MPU6050.accXOffset.raw[3]);
  EEPROM.write(ACCX_OFFSET_EEPROM_ADDR+1, MPU6050.accXOffset.raw[2]);
  EEPROM.write(ACCX_OFFSET_EEPROM_ADDR+2, MPU6050.accXOffset.raw[1]);
  EEPROM.write(ACCX_OFFSET_EEPROM_ADDR+3, MPU6050.accXOffset.raw[0]);

  EEPROM.write(ACCY_OFFSET_EEPROM_ADDR,   MPU6050.accYOffset.raw[3]);
  EEPROM.write(ACCY_OFFSET_EEPROM_ADDR+1, MPU6050.accYOffset.raw[2]);
  EEPROM.write(ACCY_OFFSET_EEPROM_ADDR+2, MPU6050.accYOffset.raw[1]);
  EEPROM.write(ACCY_OFFSET_EEPROM_ADDR+3, MPU6050.accYOffset.raw[0]);

  return true;
}

/*******************************************************
  reading EEPROM
/*******************************************************/
bool read_eeprom(void) {
  uint8_t a, b, c, d;

  a = (uint8_t)EEPROM.read(GYROX_OFFSET_EEPROM_ADDR);
  b = (uint8_t)EEPROM.read(GYROX_OFFSET_EEPROM_ADDR+1);
  c = (uint8_t)EEPROM.read(GYROX_OFFSET_EEPROM_ADDR+2);
  d = (uint8_t)EEPROM.read(GYROX_OFFSET_EEPROM_ADDR+3);
  MPU6050.gyroXOffset.val = (int16_t)((a << 32) | (b << 16) | (c << 8) | d);

  a = (uint8_t)EEPROM.read(GYROY_OFFSET_EEPROM_ADDR);
  b = (uint8_t)EEPROM.read(GYROY_OFFSET_EEPROM_ADDR+1);
  c = (uint8_t)EEPROM.read(GYROY_OFFSET_EEPROM_ADDR+2);
  d = (uint8_t)EEPROM.read(GYROY_OFFSET_EEPROM_ADDR+3);
  MPU6050.gyroYOffset.val = (int16_t)((a << 32) | (b << 16) | (c << 8) | d);

  a = (uint8_t)EEPROM.read(GYROZ_OFFSET_EEPROM_ADDR);
  b = (uint8_t)EEPROM.read(GYROZ_OFFSET_EEPROM_ADDR+1);
  c = (uint8_t)EEPROM.read(GYROZ_OFFSET_EEPROM_ADDR+2);
  d = (uint8_t)EEPROM.read(GYROZ_OFFSET_EEPROM_ADDR+3);
  MPU6050.gyroZOffset.val = (int16_t)((a << 32) | (b << 16) | (c << 8) | d);

  a = (uint8_t)EEPROM.read(ACCX_OFFSET_EEPROM_ADDR);
  b = (uint8_t)EEPROM.read(ACCX_OFFSET_EEPROM_ADDR+1);
  c = (uint8_t)EEPROM.read(ACCX_OFFSET_EEPROM_ADDR+2);
  d = (uint8_t)EEPROM.read(ACCX_OFFSET_EEPROM_ADDR+3);
  MPU6050.accXOffset.val = (int16_t)((a << 32) | (b << 16) | (c << 8) | d);

  a = (uint8_t)EEPROM.read(ACCY_OFFSET_EEPROM_ADDR);
  b = (uint8_t)EEPROM.read(ACCY_OFFSET_EEPROM_ADDR+1);
  c = (uint8_t)EEPROM.read(ACCY_OFFSET_EEPROM_ADDR+2);
  d = (uint8_t)EEPROM.read(ACCY_OFFSET_EEPROM_ADDR+3);
  MPU6050.accYOffset.val = (int16_t)((a << 32) | (b << 16) | (c << 8) | d);

  statusCalibrationEsc = (bool)EEPROM.read(ESC_CAL_EEPROM_ADDR);
  
#if DEBUG_EEPROM
  Serial.println(F("\nMPU6050 OFFSETs"));
  Serial.print(F("GYRO X: ")); Serial.println(MPU6050.gyroXOffset.val);
  Serial.print(F("GYRO Y: ")); Serial.println(MPU6050.gyroYOffset.val);
  Serial.print(F("GYRO Z: ")); Serial.println(MPU6050.gyroZOffset.val);
  Serial.print(F("ACC X: ")); Serial.println(MPU6050.accXOffset.val);
  Serial.print(F("ACC Y: ")); Serial.println(MPU6050.accYOffset.val);
  Serial.print(F("ESC CALIBRATION STATUS: ")); Serial.println(statusCalibrationEsc);
#endif 

  if ((MPU6050.gyroXOffset.val < -255) || (MPU6050.gyroXOffset.val > 255) ||
      (MPU6050.gyroYOffset.val < -255) || (MPU6050.gyroYOffset.val > 255) || 
      (MPU6050.gyroZOffset.val < -255) || (MPU6050.gyroZOffset.val > 255) ||
      (MPU6050.accXOffset.val  < -500) || (MPU6050.accXOffset.val  > 500) ||
      (MPU6050.accYOffset.val  < -500) || (MPU6050.accYOffset.val  > 500)) {
    MPU6050.gyroXOffset.val = 0;
    MPU6050.gyroYOffset.val = 0;
    MPU6050.gyroZOffset.val = 0;
    MPU6050.accXOffset.val = 0;
    MPU6050.accYOffset.val = 0;
    return false;
  } else {
    return true;
  }
}

/*******************************************************
  waiting radio signals
/*******************************************************/
bool wait_radio_signals(void) {
  unsigned long startWaitRadioTime = millis();
  uint8_t flagWaitRadio = 0xFF;
  uint16_t i = 0;
  uint8_t k = 0;

  while (true) {
    if (channel[1] < 1100) flagWaitRadio &= 0b11111110;
    if (channel[1] > 1900) flagWaitRadio &= 0b11111101;
    if (channel[2] < 1100) flagWaitRadio &= 0b11111011;
    if (channel[2] > 1900) flagWaitRadio &= 0b11110111;
    if (channel[3] < 1100) flagWaitRadio &= 0b11101111;
    if (channel[3] > 1900) flagWaitRadio &= 0b11011111;
    if (channel[4] < 1100) flagWaitRadio &= 0b10111111;
    if (channel[4] > 1900) flagWaitRadio &= 0b01111111;
    
    if (flagWaitRadio == 0x00) return true;
    if (millis() - startWaitRadioTime >= WAITING_SAFETY_PERIOD) return false;
 
    i ++;
    if (i % 50 == 0) {
      (k >= 2) ? k = 0 : k ++;
    }

    if (k == 0) seven_seg(0b01111111, 0b01111111, 0b01111111, 0b01111111);
    if (k == 1) seven_seg(0b11111101, 0b11111101, 0b11111101, 0b11111101);
    if (k == 2) seven_seg(0b11101111, 0b11101111, 0b11101111, 0b11101111);
    
    delayMicroseconds(4000);
  }
}

/*******************************************************
  seven seg display
/*******************************************************/
void seven_seg(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4) {
  static uint8_t index = 0;

  index ++;
  if (index > 3) index = 0;

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
  seven seg muxtiplex
/*******************************************************/
bool seven_seg_mux(void) {
  static unsigned long previousSlidePeriod;
  static unsigned long previousBlinkPeriod;
  static uint8_t index = 0;
  static bool flag = false;

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

  index ++;
  if (index >= 4) index = 0;

  //circular buffer
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
  }

  if (millis() - previousBlinkPeriod >= 500) {
    previousBlinkPeriod = millis();
    
    quadX.leds ^= LED_YELLOW | LED_FRONT | LED_REAR;
    toggle_leds();
  }
}

/*******************************************************
  fail message
/*******************************************************/
void fail_message(void) {
  for (uint16_t i = 0; i < 500; i ++) {
    seven_seg(0b01110001, 0b00010001, 0b10011111, 0b11100011);
    delayMicroseconds(4000);
  }
}

/*******************************************************
  print info
/*******************************************************/
void print_info(uint8_t index) {
  static uint8_t printIndex = 0;

  switch (index) {
    case 1: {
      (printIndex <= 0x0F) ? printIndex ++ : printIndex = 0;
      if (printIndex == 0x00) Serial.print(F("GX "));
      if (printIndex == 0x01) Serial.print(MPU6050.gyroX);
      if (printIndex == 0x02) Serial.print(F(" GY "));
      if (printIndex == 0x03) Serial.print(MPU6050.gyroY);
      if (printIndex == 0x04) Serial.print(F(" ACCX "));
      if (printIndex == 0x05) Serial.print(MPU6050.accX);
      if (printIndex == 0x06) Serial.print(F(" ACCY "));
      if (printIndex == 0x07) Serial.print(MPU6050.accY);
      if (printIndex == 0x08) Serial.print(F(" OFF GX "));
      if (printIndex == 0x09) Serial.print(MPU6050.gyroXOffset.val);
      if (printIndex == 0x0A) Serial.print(F(" OFF GY "));
      if (printIndex == 0x0B) Serial.print(MPU6050.gyroYOffset.val);
      if (printIndex == 0x0C) Serial.print(F(" OFF ACCX "));
      if (printIndex == 0x0D) Serial.print(MPU6050.accXOffset.val);
      if (printIndex == 0x0E) Serial.print(F(" OFF ACCY "));
      if (printIndex == 0x0F) Serial.println(MPU6050.accYOffset.val);
    }
    break;

    case 2: {
      (printIndex <= 0x0B) ? printIndex ++ : printIndex = 0;
      if (printIndex == 0x00) Serial.print(F("ROLL SP "));
      if (printIndex == 0x01) Serial.print(rollSetPoint);
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
      if (printIndex == 0x01) Serial.print(pitchSetPoint);
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
      (printIndex <= 0x05) ? printIndex ++ : printIndex = 0;
      if (printIndex == 0x00) Serial.print(F("YAW SP "));
      if (printIndex == 0x01) Serial.print(yawSetPoint);
      if (printIndex == 0x02) Serial.print(F(" GYRO "));
      if (printIndex == 0x03) Serial.print(gyroYawInput);
      if (printIndex == 0x04) Serial.print(F(" OUT "));
      if (printIndex == 0x05) Serial.println(pidYaw);
    }
    break;
    
    case 5: {
      (printIndex < 16) ? printIndex ++ : printIndex = 0;
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
      (printIndex < 7) ? printIndex ++ : printIndex = 0;
      if (printIndex == 0x01) Serial.print(F("QUADX STS "));
      if (printIndex == 0x02) Serial.print(quadX.status, BIN);
      if (printIndex == 0x03) Serial.print(F(" MOTOR STS "));
      if (printIndex == 0x04) Serial.print(quadX.motors);
      if (printIndex == 0x05) Serial.print(F(" BATT "));
      if (printIndex == 0x06) Serial.println(batteryVoltage);
    }
    break;

    case 7: {
      (printIndex <= 0x15) ? printIndex ++ : printIndex = 0;
      if (printIndex == 0x00) Serial.print(F("ALTH "));
      if (printIndex == 0x01) Serial.print(statusAltitudeHold);
      if (printIndex == 0x02) Serial.print(F(" CH3 "));
      if (printIndex == 0x03) Serial.print(channel[3]);
      if (printIndex == 0x04) Serial.print(F(" THR "));
      if (printIndex == 0x05) Serial.print(currentThrottle);
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
  }
}

/*******************************************************
  toggle LEDs
/*******************************************************/
void toggle_leds(void) {
  (quadX.leds & LED_YELLOW) ? digitalWrite(LED_YELLOW_PIN, HIGH) : digitalWrite(LED_YELLOW_PIN, LOW);
  (quadX.leds & LED_GREEN) ? digitalWrite(LED_GREEN_PIN, HIGH) : digitalWrite(LED_GREEN_PIN, LOW);
  (quadX.leds & LED_BLUE) ? digitalWrite(LED_BLUE_PIN, HIGH) : digitalWrite(LED_BLUE_PIN, LOW);
  (quadX.leds & LED_RED) ? digitalWrite(LED_RED_PIN, HIGH) : digitalWrite(LED_RED_PIN, LOW);

  if (quadX.leds & LED_FRONT) {
    digitalWrite(LED_FRONT_LEFT, HIGH);
    digitalWrite(LED_FRONT_RIGHT, HIGH);
  } else {
    digitalWrite(LED_FRONT_LEFT, LOW);
    digitalWrite(LED_FRONT_RIGHT, LOW);
  }

  if (quadX.leds & LED_REAR) {
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
  unsigned long previousBlinkPeriod;
  uint8_t indexBlink = 0;

  quadX.leds |= LED_BLUE | LED_YELLOW | LED_FRONT | LED_REAR;
  toggle_leds();

  while (true) {
    if (millis() - previousBlinkPeriod >= 100) {
      previousBlinkPeriod = millis();

      quadX.leds ^= LED_YELLOW | LED_FRONT | LED_REAR;
      toggle_leds();

      indexBlink ++;
    }

    if (micros() - previousDisplayPeriod >= 5000) {
      previousDisplayPeriod = micros();
      seven_seg(0b10011111, 0b11010101, 0b11011111, 0b11100001);
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
  Wire.write(BMP180_AC1_ADDRESS); //first address of calibration values
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

  //temperature calculation
  x1 = ((int32_t)BMP180.UT.val - BMP180.ac6) * BMP180.ac5 >> 15;
  x2 = ((int32_t)BMP180.mc << 11) / (x1 + BMP180.md);
  b5 = x1 + x2;
  BMP180.temperature = (b5 * 10 + 8) >> 4; //in 0.01 degC (same as MS5611 temperature)

  //pressure calculation
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
  BMP180.timeOut = micros() + BMP180_WAIT_UT_READ; //wait 4.5ms to read temperature + 1.5ms margin (6ms)

  if (BMP180.status) {
    BMP180_UT_read();
    BMP180_UP_start();
    BMP180.status = false;
    BMP180.timeOut += BMP180_WAIT_UP_READ; //wait 27ms (6ms+21ms) to read pressure 
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
    Wire.write(0xA0 + (i * 2)); //first address of calibration data
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

  //temperature calculation
  int64_t dT = (int32_t)MS5611.ut.val - ((int32_t)MS5611.c[5] << 8);
  MS5611.temperature = 2000 + ((dT * MS5611.c[6]) >> 23);

  //pressure calculation
  int64_t OFF  = ((uint32_t)MS5611.c[2] << 16) + ((dT * MS5611.c[4]) >> 7);
  int64_t SENS = ((uint32_t)MS5611.c[1] << 15) + ((dT * MS5611.c[3]) >> 8);

  //second order temperature compensation
  if (MS5611.temperature < 2000) { //temperature lower than 20st.C 
    delt = MS5611.temperature - 2000;
    delt  = (5 * delt * delt);
    OFF2  = delt >> 1;
    SENS2 = delt >> 2;
    if (MS5611.temperature < -1500) { //temperature lower than -15st.C
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
  MS5611.timeOut = micros() + MS5611_WAIT_UT_UP_READ; //wait 8.5ms to read temperature and pressure + 1.5ms margin (10ms)

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

/*******************************************************
  altitude hold pid
/*******************************************************/
void altitude_hold(uint8_t index) {
  static uint8_t altHoldIndex  = 0;
  static uint8_t readTempIndex = 0;

  //temperature buffer
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

  //pressure buffer
  if (altHoldIndex == 1) { 
    int32_t OFF2, SENS2, delt;

    //temperature calculation
    int64_t dT = (int32_t)rawTemperature - ((int32_t)MS5611.c[5] << 8);
    MS5611.temperature = 2000 + ((dT * MS5611.c[6]) >> 23);

    //pressure calculation
    int64_t OFF  = ((uint32_t)MS5611.c[2] << 16) + ((dT * MS5611.c[4]) >> 7);
    int64_t SENS = ((uint32_t)MS5611.c[1] << 15) + ((dT * MS5611.c[3]) >> 8);

    //second order temperature compensation
    if (MS5611.temperature < 2000) { //temperature lower than 20st.C 
      delt = MS5611.temperature - 2000;
      delt  = (5 * delt * delt);
      OFF2  = delt >> 1;
      SENS2 = delt >> 2;
      if (MS5611.temperature < -1500) { //temperature lower than -15st.C
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

    //complementary filter
    actualPressureSlow = (float)((actualPressureSlow * 0.985) + (actualPressureFast * 0.015));
    actualPressureDiff = (float)(actualPressureSlow - actualPressureFast);
    actualPressureDiff = constrain(actualPressureDiff, -8, 8);

    if ((actualPressureDiff > 1) || (actualPressureDiff < -1)) {
      actualPressureSlow -= (actualPressureDiff / 6.0);
    }

    //minimum oscilation pressure
    actualPressure = actualPressureSlow;
  }

  //altitude hold pid
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

    //altidude hold on?
    if (statusAltitudeHold) {
      if (altHoldSetPoint == 0) {
        altHoldSetPoint = actualPressure;
      }

      //manual currentThrottle
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

      //auto correction currentThrottle
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

      //altitude hold PID                               
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
      initialThrottle = currentThrottle;
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

/*******************************************************
  pin change interrupt (reading radio channels)
/*******************************************************/
ISR(PCINT0_vect) {
  static unsigned long previousPulseWidth;
  static uint8_t channelIndex = 0;

  //read pulse width PPM mode (PCINT3)
  if (PINB & B00000100) { 
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
