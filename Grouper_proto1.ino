/*
 * Original SparkFun Razor IMU firmware provided by SparkFun.
 * Modified by Nicholas Hukriede of Mote Marine Laboratories Ocean Technology Program.
 * Now has capabilities to display and read from a Blue Robotics Bar 30 pressure and temperature sensor.
 * 
 * Last Modified: July 20th, 2017
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

/////////////////////////////
// Logging Control Globals //
/////////////////////////////
// Defaults all set in config.h
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

/////////////////////
// SD Card Globals //
/////////////////////
bool sdCardPresent = false; 
String logFileName; 
String logFileBuffer; 

#ifdef ENABLE_NVRAM_STORAGE
  ///////////////////////////
  // Flash Storage Globals //
  ///////////////////////////
  // Logging parameters are all stored in non-volatile memory.
  // They should maintain the previously set config value.
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

void setup()  {
  
  initHardware(); 
  
#ifdef ENABLE_NVRAM_STORAGE
  initLoggingParams();
#endif

  if (!initIMU()) {
    LOG_PORT.println("Error connecting to MPU-9250");
    while (1);
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
  if (!imu.fifoAvailable()) {
    return;                   
  }
  
  if (imu.dmpUpdateFifo() != INV_SUCCESS) {
    return;
  }
  
  if ((enableCompass || enableHeading) && (imu.updateCompass() != INV_SUCCESS)) {
    return; 
  }
 
  if (enableSerialLogging || enableSDLogging)  {
    logIMUData();
  }
}

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
    imuLog += String(sensor.temperature()) + ", ";
    imuLog += String(sensor.depth()) + ", ";
  }
 
  imuLog.remove(imuLog.length() - 2, 2);
  imuLog += "\r\n"; 

  if (enableSerialLogging)  {
    LOG_PORT.print(imuLog); 
  }
    
  if ( sdCardPresent && enableSDLogging)  {
    if (imuLog.length() + logFileBuffer.length() >= SD_LOG_WRITE_BUFFER_SIZE) {
      sdLogString(logFileBuffer);
      logFileBuffer = "";
    }
    logFileBuffer += imuLog;
  }
}

// Initialize hardware options
void initHardware(void) {
  pinMode(MPU9250_INT_PIN, INPUT_PULLUP);
  LOG_PORT.begin(SERIAL_BAUD_RATE);
}

/* Initialize pressure/temperature sensor
 * returns success/failure
 */
bool initSens() {
  Wire.begin();
  if (!sensor.init()) {
    Serial.println("Error Connecting to the pressure sensor");
    return false;
  }
  return true;
}

/* Initialize the IMU and set up global variables
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

/* Initialize SD card for storage
 * returns success/failure
 */
bool initSD(void) {
  if (!SD.begin(SD_CHIP_SELECT_PIN)) {
    return false;
  }
  return true;
}


/* Log a string to the SD card
 * returns success/failure
 */
bool sdLogString(String toLog)  {
  File logFile = SD.open(logFileName, FILE_WRITE);
  if (logFile.size() > (SD_MAX_FILE_SIZE - toLog.length())) {
    
    logFileName = nextLogFile();
    logFile = SD.open(logFileName, FILE_WRITE);
  }

  if (logFile)  {
    
    logFile.print(toLog);
    logFile.close();

    return true;
  }
  return false;
}

/* Finds the next log file name
 * returns the filename or null if exceeded the max
 */
String nextLogFile(void)  {
  String filename;
  int logIndex = 0;

  for (int i = 0; i < LOG_FILE_INDEX_MAX; i++)  {
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


#ifdef ENABLE_NVRAM_STORAGE
  void initLoggingParams(void)  {
    if (flashFirstRun.read() == 0) 
    {
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
    }
    else
    {
      
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


