#include "Constants.h"
#include "HardwareConfig.h"
#include "MSP.h"

 
// Task
TaskHandle_t mainTask;
TaskHandle_t mspTask;

//MSP port
MSP msp;

void setup() { 
  Serial.begin(BAUDRATE);
  // Waiting for serial
  while (!Serial) {}

  //Init hardware pins
  InitHardwarePins();
  
  //MSP port using Softserial
  Serial2.begin(MSP_BAUDRATE, SERIAL_8N1, MSP_RXPIN, MSP_TXPIN);
  msp.begin(Serial2);
  Serial.println("MSP coomunication started");

  
  // Task to control loops
  // This should be defined at the end because they will car the task functions as soon are defined
  xTaskCreatePinnedToCore(
                    mainLoop,   /* Task function. */
                    "mainLoop",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &mainTask,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500); 

  xTaskCreatePinnedToCore(
                    mspLoop,   /* Task function. */
                    "mspLoop",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &mspTask,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
  delay(500); 


}

// TODO: change these functions 
// Join functions send the same request (faster)
// receive the data to be changed (easy to impement and use)
//int32_t getBaroAltitude(){
//  msp_altitude_t alt;
//  if (msp.request(MSP_ALTITUDE, &alt, sizeof(alt))) {
//     return alt.estimatedActualPosition;
//  }
//  return MSP_INT32_ERROR;
//}
//
//int8_t getVoltage(){
//  msp_analog_t v;
//  if (msp.request(MSP_ANALOG, &v, sizeof(v))){
//     return v.vbat;
//  }
//  return MSP_INT32_ERROR;
//
//}
//
//int8_t getMisc(){
//  msp_misc_t v;
//  if (msp.request(MSP_MISC, &v, sizeof(v))){
//     return v.vbatscale;
//  }
//  return MSP_INT32_ERROR;
//}
//
//int32_t getSonarAltitude(){
//  msp_sonar_altitude_t sonar_alt;
//  if (msp.request(MSP_SONAR_ALTITUDE, &sonar_alt, sizeof(sonar_alt))){
//    return sonar_alt.altitude;
//  }
//  return MSP_INT32_ERROR;
//}
//
//int16_t getGyroX(){
//  msp_raw_imu_t gyro;
//  if (msp.request(MSP_RAW_IMU, &gyro, sizeof(gyro))){
//    return gyro.gyro[0];
//  }
//  return MSP_INT32_ERROR;
//}
//
//int16_t getGyroY(){
//  msp_raw_imu_t gyro;
//  if (msp.request(MSP_RAW_IMU, &gyro, sizeof(gyro))){
//    return gyro.gyro[1];
//  }
//  return MSP_INT32_ERROR;
//}
//
//int16_t getGyroZ(){
//  msp_raw_imu_t gyro;
//  if (msp.request(MSP_RAW_IMU, &gyro, sizeof(gyro))){
//    return gyro.gyro[2];
//  }
//  return MSP_INT32_ERROR;
//}
//
//
//int16_t * getAcc(){
//  msp_raw_imu_t acc;
//  if (msp.request(MSP_RAW_IMU, &acc, sizeof(acc))){
//    return acc.acc;
//  }
//  return MSP_INT32_ERROR;
//}
//
//int16_t getAccY(){
//  msp_raw_imu_t acc;
//  if (msp.request(MSP_RAW_IMU, &acc, sizeof(acc))){
//    return acc.acc[1];
//  }
//  return MSP_INT32_ERROR;
//}
//
//int16_t getAccZ(){
//  msp_raw_imu_t acc;
//  if (msp.request(MSP_RAW_IMU, &acc, sizeof(acc))){
//    return acc.acc[2];
//  }
//  return MSP_INT32_ERROR;
//}


//Get gyro data
bool getGyroData(int16_t rdata[]){
  msp_raw_imu_t gyro;
  if (msp.request(MSP_RAW_IMU, &gyro, sizeof(gyro))){
     rdata[0] = gyro.gyro[0];
     rdata[1] = gyro.gyro[1];
     rdata[2] = gyro.gyro[2];
     return true;
  }
  return false;
}


//Get drone data
void showDroneData(){
    int32_t sonar_alt;
    int8_t vbat;
    int8_t vbatscale;
    int8_t volt;
    int32_t sonar;
    int16_t gyro[3]={0,0,0};
    int16_t acc[3]={0,0,0};

    //Get gyro data
    if(getGyroData(gyro)){
      Serial.print("GyroX: ");
      Serial.print(gyro[0]);
      Serial.print(" GyroY: ");
      Serial.print(gyro[1]);
      Serial.print(" GyroZ: ");
      Serial.println(gyro[2]);
    }
}


// Loop, not used but it should be present
// Known issue https://github.com/espressif/arduino-esp32/issues/595
// https://esp32.com/viewtopic.php?f=2&t=809&p=10191&hilit=esp_task_wdt_feed#p10191
void loop(){
  vTaskDelay(10);
}


// Main loop
void mainLoop(void * pvParameters) {
  for(;;){
    vTaskDelay(10);
  }
}

//Main loop where msp request are
//**** Telemetry loop
void mspLoop(void * pvParameters) {
  for(;;){
    showDroneData();
    vTaskDelay(10);
  }  
}
