/*
 * Initial firmware provided by SparkFun and was modified to include a Blue Robotics Bar 30 temperature/pressure sensor. 
 * It is standard setup to only include the pressure readings, which can then be converted into depth. All output is read
 * through the serial monitor as well as written to an SD card if there is one available.
 * 
 * Designed to measure very quick movements of spawning Grouper off the Gulf Coast of Florida. Sample rate is set at 1000 Hz.
 * 
 * Last Modified: July 20th, 2017 by Nicholas Hukriede
 * 
 * Mote Marine Laboratory: Ocean Technology Program
 */

#include <MS5837.h>
#include <SparkFunMPU9250-DMP.h>
#include <SD.h>
#include "config.h"

#ifdef ENABLE_NVRAM_STORAGE
#include <FlashStorage.h>
#endif

MPU9250_DMP imu; 
MS5837 sensor;

// LOGGING CONTROL GLOBALS
bool enableSDLogging = ENABLE_SD_LOGGING;
bool enableSerialLogging = ENABLE_UART_LOGGING;
bool enableTimeLog = ENABLE_TIME_LOG;
bool enableCalculatedValues = ENABLE_CALCULATED_LOG;
bool enableAccel = ENABLE_ACCEL_LOG;
bool enableSensor = ENABLE_SENSOR_LOG;
bool enableGyro = ENABLE_GYRO_LOG;
bool enableCompass = ENABLE_MAG_LOG;
bool enableQuat = ENABLE_QUAT_LOG;
bool enableEuler = ENABLE_EULER_LOG;
bool enableHeading = ENABLE_HEADING_LOG;
unsigned short accelFSR = IMU_ACCEL_FSR;
unsigned short gyroFSR = IMU_GYRO_FSR;
unsigned short fifoRate = DMP_SAMPLE_RATE;

// SD CARD GLOBALS
bool sdCardPresent = false; 
String logFileName; 
String logFileBuffer; 

// LED BLINK CONTROLS
uint32_t lastBlink = 0;
void blinkLED() {
  static bool ledState = false;
  digitalWrite(HW_LED_PIN, ledState);
  ledState = !ledState;
}

// FLASH STORAGE GLOBALS
#ifdef ENABLE_NVRAM_STORAGE
  FlashStorage(flashEnableSDLogging, bool);
  FlashStorage(flashFirstRun, bool);
  FlashStorage(flashEnableSD, bool);
  FlashStorage(flashEnableSerialLogging, bool);
  FlashStorage(flashenableTime, bool);
  FlashStorage(flashEnableCalculatedValues, bool);
  FlashStorage(flashEnableAccel, bool);
  FlashStorage(flashEnableSensor, bool);
  FlashStorage(flashEnableGyro, bool);
  FlashStorage(flashEnableCompass, bool);
  FlashStorage(flashEnableQuat, bool);
  FlashStorage(flashEnableEuler, bool);
  FlashStorage(flashEnableHeading, bool);
  FlashStorage(flashAccelFSR, unsigned short);
  FlashStorage(flashGyroFSR, unsigned short);
  FlashStorage(flashLogRate, unsigned short);
#endif

void setup() {
  initHardware(); 
#ifdef ENABLE_NVRAM_STORAGE
  initLoggingParams();
#endif

  if (!initIMU()) {
    LOG_PORT.println("Error connecting to MPU-9250");
    while (1) ; 
  }

  if (initSD()) {
    sdCardPresent = true;
    logFileName = nextLogFile(); 
  }

  if (initSens()) {
    sensor.setModel(MS5837::MS5837_30BA);
    sensor.setFluidDensity(1029);
  }
}

void loop() {
  
  if (LOG_PORT.available()) {
    parseSerialInput(LOG_PORT.read()); 
  }

  if (!imu.fifoAvailable()) {
    return;                   
  }
  
  if (imu.dmpUpdateFifo() != INV_SUCCESS) {
    return; 
  }
  
  if ((enableCompass || enableHeading) && (imu.updateCompass() != INV_SUCCESS)) {
    return; 
  }
  
  if ( enableSerialLogging || enableSDLogging) {
    logIMUData();
  }

}

/*
 * Creates a string to log data received from the IMU and Bar30
 * and outputs that string onto the serial monitor at 115200 baud rate.
 */
void logIMUData(void) {
  String imuLog = ""; 
  if (enableTimeLog) {
    imuLog += String(imu.time) + ", "; 
  }
  if (enableAccel) {
    if (enableCalculatedValues) {
      imuLog += String(imu.calcAccel(imu.ax)) + ", ";
      imuLog += String(imu.calcAccel(imu.ay)) + ", ";
      imuLog += String(imu.calcAccel(imu.az)) + ", ";
    } else {
      imuLog += String(imu.ax) + ", ";
      imuLog += String(imu.ay) + ", ";
      imuLog += String(imu.az) + ", ";
    }
  }
  if (enableGyro) {
    if (enableCalculatedValues) {
      imuLog += String(imu.calcGyro(imu.gx)) + ", ";
      imuLog += String(imu.calcGyro(imu.gy)) + ", ";
      imuLog += String(imu.calcGyro(imu.gz)) + ", ";
    } else {
      imuLog += String(imu.gx) + ", ";
      imuLog += String(imu.gy) + ", ";
      imuLog += String(imu.gz) + ", ";
    }
  }
  if (enableCompass) {
    if (enableCalculatedValues) {
      imuLog += String(imu.calcMag(imu.mx)) + ", ";
      imuLog += String(imu.calcMag(imu.my)) + ", ";
      imuLog += String(imu.calcMag(imu.mz)) + ", ";    
    } else {
      imuLog += String(imu.mx) + ", ";
      imuLog += String(imu.my) + ", ";
      imuLog += String(imu.mz) + ", ";
    }
  }
  if (enableQuat) {
    if (enableCalculatedValues) {
      imuLog += String(imu.calcQuat(imu.qw), 4) + ", ";
      imuLog += String(imu.calcQuat(imu.qx), 4) + ", ";
      imuLog += String(imu.calcQuat(imu.qy), 4) + ", ";
      imuLog += String(imu.calcQuat(imu.qz), 4) + ", ";
    } else {
      imuLog += String(imu.qw) + ", ";
      imuLog += String(imu.qx) + ", ";
      imuLog += String(imu.qy) + ", ";
      imuLog += String(imu.qz) + ", ";      
    }
  }
  if (enableEuler) {
    imu.computeEulerAngles();
    imuLog += String(imu.pitch, 2) + ", ";
    imuLog += String(imu.roll, 2) + ", ";
    imuLog += String(imu.yaw, 2) + ", ";
  }
  if (enableHeading) {
    imuLog += String(imu.computeCompassHeading(), 2) + ", ";
  }

  if (enableSensor) {
    sensor.read();
    imuLog += String(sensor.pressure()) + ", ";
    
    // Uncomment these lines below to add temperature and depth to the output settings:
    
    //imuLog += String(sensor.temperature()) + ", "; 
    //imuLog += String(sensor.depth()) + ", ";

  }
  
  imuLog.remove(imuLog.length() - 2, 2);
  imuLog += "\r\n"; // Add a new line

  if (enableSerialLogging) {
    LOG_PORT.print(imuLog); 
  }
  
  if ( sdCardPresent && enableSDLogging) {
    if (imuLog.length() + logFileBuffer.length() >= SD_LOG_WRITE_BUFFER_SIZE) {
      sdLogString(logFileBuffer); 
      logFileBuffer = ""; 
      blinkLED(); 
    }
    logFileBuffer += imuLog;
  } else {
    if (millis() > lastBlink + UART_BLINK_RATE) {
      blinkLED(); 
      lastBlink = millis();
    }
  }
}

/*
 * Initializes hardware settings and blink control.
 */
void initHardware(void) {
  pinMode(HW_LED_PIN, OUTPUT);
  digitalWrite(HW_LED_PIN, LOW);
  pinMode(MPU9250_INT_PIN, INPUT_PULLUP);
  LOG_PORT.begin(SERIAL_BAUD_RATE);
}

/*
 * Initializes the IMU and sets global variable standards.
 * returns success/failure
 */
bool initIMU(void) {
  if (imu.begin() != INV_SUCCESS) {
    return false;
  }
  imu.enableInterrupt(); 
  imu.setIntLevel(1);    
  imu.setIntLatched(1);
  imu.setGyroFSR(gyroFSR);
  imu.setAccelFSR(accelFSR);
  imu.setLPF(IMU_AG_LPF); 
  imu.setSampleRate(IMU_AG_SAMPLE_RATE); 
  imu.setCompassSampleRate(IMU_COMPASS_SAMPLE_RATE); 

  unsigned short dmpFeatureMask = 0;
  if (ENABLE_GYRO_CALIBRATION) {
    dmpFeatureMask |= DMP_FEATURE_SEND_CAL_GYRO;
  } else {
    dmpFeatureMask |= DMP_FEATURE_SEND_RAW_GYRO;
  }
  dmpFeatureMask |= DMP_FEATURE_SEND_RAW_ACCEL;
  dmpFeatureMask |= DMP_FEATURE_6X_LP_QUAT;
  imu.dmpBegin(dmpFeatureMask, fifoRate);

  return true; 
}

/*
 * Initializes the SD card for writing and storage.
 * returns success/failure
 */
bool initSD(void) {
  if (!SD.begin(SD_CHIP_SELECT_PIN)) {
    return false;
  }
  return true;
}

/*
 * Initialize the pressure sensor.
 * return success/failure
 */
bool initSens() {
  Wire.begin();
  if (!sensor.init()) {
    return false;
  }
  return true;
}

/*
 * Logs the string of data to a log file on the SD card.
 * returns success/failure
 */
bool sdLogString(String toLog) {
  File logFile = SD.open(logFileName, FILE_WRITE);
  if (logFile.size() > (SD_MAX_FILE_SIZE - toLog.length())) {
    logFileName = nextLogFile();
    logFile = SD.open(logFileName, FILE_WRITE);
  }

  if (logFile) {
    logFile.print(toLog);
    logFile.close();
    return true; 
  }
  return false; 
}

/*
 * Creates a new log file if the max has not yet been reached.
 * returns the name of the new log file
 */
String nextLogFile(void)  {
  String filename;
  int logIndex = 0;

  for (int i = 0; i < LOG_FILE_INDEX_MAX; i++) {
    filename = String(LOG_FILE_PREFIX);
    filename += String(logIndex);
    filename += ".";
    filename += String(LOG_FILE_SUFFIX);

    if (!SD.exists(filename)) {
      return filename;
    }
    logIndex++;
  }

  return "";
}

/*
 * Reads serial input for logging commands.
 */
void parseSerialInput(char c) {
  unsigned short temp;
  switch (c)
  {
  case PAUSE_LOGGING: 
    enableSerialLogging = !enableSerialLogging;
#ifdef ENABLE_NVRAM_STORAGE
    flashEnableSerialLogging.write(enableSerialLogging);
#endif
    break;
  case ENABLE_TIME: 
    enableTimeLog = !enableTimeLog;
#ifdef ENABLE_NVRAM_STORAGE
    flashenableTime.write(enableTimeLog);
#endif
    break;
  case ENABLE_ACCEL: 
    enableAccel = !enableAccel;
#ifdef ENABLE_NVRAM_STORAGE
    flashEnableAccel.write(enableAccel);
#endif
    break;
  case ENABLE_SENSOR:
    enableSensor = !enableSensor;
#ifdef ENABLE_NVRAM_STORAGE
    flashEnableSensor.write(enableSensor);
#endif
    break;
  case ENABLE_GYRO: 
    enableGyro = !enableGyro;
#ifdef ENABLE_NVRAM_STORAGE
    flashEnableGyro.write(enableGyro);
#endif
    break;
  case ENABLE_COMPASS: 
    enableCompass = !enableCompass;
#ifdef ENABLE_NVRAM_STORAGE
    flashEnableCompass.write(enableCompass);
#endif
    break;
  case ENABLE_CALC: 
    enableCalculatedValues = !enableCalculatedValues;
#ifdef ENABLE_NVRAM_STORAGE
    flashEnableCalculatedValues.write(enableCalculatedValues);
#endif
    break;
  case ENABLE_QUAT: 
    enableQuat = !enableQuat;
#ifdef ENABLE_NVRAM_STORAGE
    flashEnableQuat.write(enableQuat);
#endif
    break;
  case ENABLE_EULER: 
    enableEuler = !enableEuler;
#ifdef ENABLE_NVRAM_STORAGE
    flashEnableEuler.write(enableEuler);
#endif
    break;
  case ENABLE_HEADING: 
    enableHeading = !enableHeading;
#ifdef ENABLE_NVRAM_STORAGE
    flashEnableHeading.write(enableHeading);
#endif
    break;
  case SET_LOG_RATE: 
    temp = imu.dmpGetFifoRate(); 
    if (temp == 1) 
      temp = 10;
    else
      temp += 10; 
    if (temp > 100)  
      temp = 1;
    imu.dmpSetFifoRate(temp); 
    temp = imu.dmpGetFifoRate(); 
#ifdef ENABLE_NVRAM_STORAGE
    flashLogRate.write(temp); 
#endif
    LOG_PORT.println("IMU rate set to " + String(temp) + " Hz");
    break;
  case SET_ACCEL_FSR: 
    temp = imu.getAccelFSR();      
    if (temp == 2) temp = 4;       
    else if (temp == 4) temp = 8;  
    else if (temp == 8) temp = 16; 
    else temp = 2;                 
    imu.setAccelFSR(temp); 
    temp = imu.getAccelFSR(); 
#ifdef ENABLE_NVRAM_STORAGE
    flashAccelFSR.write(temp); 
#endif
    LOG_PORT.println("Accel FSR set to +/-" + String(temp) + " g");
    break;
  case SET_GYRO_FSR:
    temp = imu.getGyroFSR();           
    if (temp == 250) temp = 500;       
    else if (temp == 500) temp = 1000; 
    else if (temp == 1000) temp = 2000;
    else temp = 250;                   
    imu.setGyroFSR(temp); 
    temp = imu.getGyroFSR(); 
#ifdef ENABLE_NVRAM_STORAGE
    flashGyroFSR.write(temp); 
#endif
    LOG_PORT.println("Gyro FSR set to +/-" + String(temp) + " dps");
    break;
  case ENABLE_SD_LOGGING: 
    enableSDLogging = !enableSDLogging;
#ifdef ENABLE_NVRAM_STORAGE
    flashEnableSDLogging.write(enableSDLogging);
#endif
    break;
  default: 
    break;
  }
}

#ifdef ENABLE_NVRAM_STORAGE
  void initLoggingParams(void)  {
    if (flashFirstRun.read() == 0) {
      flashEnableSDLogging.write(enableSDLogging);
      flashEnableSerialLogging.write(enableSerialLogging);
      flashenableTime.write(enableTimeLog);
      flashEnableCalculatedValues.write(enableCalculatedValues);
      flashEnableAccel.write(enableAccel);
      flashEnableSensor.write(enableSensor);
      flashEnableGyro.write(enableGyro);
      flashEnableCompass.write(enableCompass);
      flashEnableQuat.write(enableQuat);
      flashEnableEuler.write(enableEuler);
      flashEnableHeading.write(enableHeading);
      flashAccelFSR.write(accelFSR);
      flashGyroFSR.write(gyroFSR);
      flashLogRate.write(fifoRate);
      flashFirstRun.write(1); 
    } else {
      enableSDLogging = flashEnableSDLogging.read();
      enableSerialLogging = flashEnableSerialLogging.read();
      enableTimeLog = flashenableTime.read();
      enableCalculatedValues = flashEnableCalculatedValues.read();
      enableAccel = flashEnableAccel.read();
      enableSensor = flashEnableSensor.read();
      enableGyro = flashEnableGyro.read();
      enableCompass = flashEnableCompass.read();
      enableQuat = flashEnableQuat.read();
      enableEuler = flashEnableEuler.read();
      enableHeading = flashEnableHeading.read();
      accelFSR = flashAccelFSR.read();
      gyroFSR = flashGyroFSR.read();
      fifoRate = flashLogRate.read();
    }
  }
#endif
  
