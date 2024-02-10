//**** Includes
#include "sbus.h"
#include "MSP.h"
#include "BluetoothSerial.h"
#include "Constants.h"
#include "HardwareConfig.h"
#include "SimpleAutonomous.h"
#define DEBUG 1

//**** Drone data
int32_t  drone_sonarAltitude = 0; //Altitude in centimeters (32 bits saved)
uint8_t  drone_vbat = 0; //Main analog voltage (from the battery pads)
uint16_t drone_mah = 0;
uint8_t  drone_hwHealthy = 0; //Hardware ok


// Raw data
// To convert these data into pitch,roll,yaw data: 
// https://engineering.stackexchange.com/questions/3348/calculating-pitch-yaw-and-roll-from-mag-acc-and-gyro-data
int16_t drone_gyro[3]={0,0,0};
int16_t drone_acc[3]={0,0,0};
int16_t drone_mag[3]={0,0,0};

//**** Tasks 
TaskHandle_t mainTask;
TaskHandle_t mspTask;

//**** SBus setup 
bfs::SbusRx sbus_rx(&Serial1);
bfs::SbusTx sbus_tx(&Serial1); 

//**** BT setup
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif


//**** Variables
BluetoothSerial SerialBT;
static uint8_t cBuf[BLUETOOTH_BUFFER_SIZE];
uint8_t BTBuff[BLUETOOTH_BUFFER_SIZE];

//Status
uint8_t system_status = WAITING_FOR_DEVICES;
uint8_t flight_mode = MANUAL_MODE;
uint8_t bt_status = BT_DISCONNECTED;
uint8_t rc_status = RC_DISCONNECTED;

//Decoded channel information from SBUS and BT (16 channels)
static std::array<int16_t, bfs::SbusRx::NUM_CH()> channels;
//Bluetooth data 
static std::array<int16_t, NUM_RC_DATA> bt_data;
//SBus data frame
std::array<int16_t, bfs::SbusRx::NUM_CH()> sbus_data;

// Communication with the FC using MSP protocol
MSP msp;

// **** Setup 
void setup() { 
  Serial.begin(BAUDRATE);
  while (!Serial) {} //Waiting for serial

  //**** Init hardware pins
  InitHardwarePins();
  
  // ---- Bluetooth 
  bluetooth_init();
  delay(500);

  // ----- SBUS communication 
  sbus_rx.Begin(SBUS_RXPIN,SBUS_TXPIN);
  sbus_tx.Begin(SBUS_RXPIN,SBUS_TXPIN);
  delay(1000);
  PRINTS("Receiver device started");
  resetChannels();
  sendChannelsData(); //Prevent randon values being sent to the FC
  delay(500);

  //****  MSP communication
  Serial2.begin(MSP_BAUDRATE, SERIAL_8N1, MSP_RXPIN, MSP_TXPIN);
  while (!Serial2) {} //Waiting for serial
  msp.begin(Serial2);
  PRINTS("MSP communication started");
  delay(500);
  
  //**** All ready
  #if DEBUG
  PRINTS("The device is ready.. Waiting connections");
  #endif
 
  //**** Task to control loops
  xTaskCreatePinnedToCore(
                    mainLoop,    /* Task function. */
                    "mainLoop",  /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &mainTask,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500); 

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    mspLoop,     /* Task function. */
                    "mspLoop",   /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &mspTask,    /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
  delay(500); 
}

//**** Send channels data to the FC
void sendChannelsData(){
  sbus_tx.ch(channels);
  sbus_tx.Write();
}

//**** Call back to check if the bluetooth was disconnected
void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
  if(event == ESP_SPP_SRV_OPEN_EVT){
    bt_status = BT_CONNECTED;
    PRINTS("Bluetooth is connected");
  }
  if(event == ESP_SPP_CLOSE_EVT ){
    bt_status = BT_DISCONNECTED;
    PRINTS("Bluetooth is disconnected");
  }
}

//**** Init bluetooth
void bluetooth_init() {
  if(!SerialBT.begin(AIRCRAFT_NAME)){
    PRINTS("Bluetooth device can not be initialized");
  }else{
    PRINTS("Bluetooth device started");
    SerialBT.register_callback(callback);
  }
  //Set all data lo low values
  initBTData();
}

//**** Update altitude
void updateSonarAltitude(){
   msp_sonar_altitude_t sonar_alt;
   if (msp.request(MSP_SONAR_ALTITUDE, &sonar_alt, sizeof(sonar_alt))){
       drone_sonarAltitude = sonar_alt.altitude; //int32_t
   }
}

//**** Update hardware status
void updateHwHealthy(){
   msp_sensor_status_t msp_status;
   if (msp.request(MSP_SENSOR_STATUS, &msp_status, sizeof(msp_status))){
       drone_hwHealthy = msp_status.isHardwareHealthy;
   }
}

//**** Update analgo data (battery,mAhDrawn) 
void updateAnalogData(){
   msp_analog_t adata;
   if (msp.request(MSP_ANALOG, &adata, sizeof(adata))){
      drone_vbat = adata.vbat;
      drone_mah = adata.mAhDrawn;
   }
}

//**** Update altitude
void updateIMUData(){
  msp_raw_imu_t imu_data;
  if (msp.request(MSP_RAW_IMU, &imu_data, sizeof(imu_data))){
    //Gyro data
    drone_gyro[0]=imu_data.gyro[0];
    drone_gyro[1]=imu_data.gyro[1];
    drone_gyro[2]=imu_data.gyro[2];
    //Acc
    drone_acc[0]=imu_data.acc[0];
    drone_acc[1]=imu_data.acc[1];
    drone_acc[2]=imu_data.acc[2];
    //Mag
    drone_mag[0]=imu_data.mag[0];
    drone_mag[1]=imu_data.mag[2];
    drone_mag[2]=imu_data.mag[2];
  }
}

//**** Update drone status from FC
void updateDroneTelem(){
  updateIMUData();
  updateAnalogData();
  updateSonarAltitude();
  updateHwHealthy();
}

//**** Init BT data properly
void initBTData(){
  //Min throtle
  bt_data[THROTTLE]=MIN_SBUS_VALUE; 
  //Rest in the middle
  bt_data[PITCH]=MID_SBUS_VALUE;
  bt_data[ROLL]=MID_SBUS_VALUE;
  bt_data[YAW]=MID_SBUS_VALUE;
  //Aux channels
  for (int i=4;i<NUM_RC_DATA;i++){
      bt_data[i]=MIN_SBUS_VALUE;
  } 
  //Set autonomous mode (just in case)
  //channels[CTRL] = MAX_SBUS_VALUE;
}

//**** Copy RC Transmitter data
void copyRxData(){
  //Copy the RC position commands
  for (int i=0;i<bfs::SbusRx::NUM_CH();i++){
      channels[i]=sbus_data[i];
  } 
}

//**** Copy RC BT data
void copyBTData(){
  //Copy the RC position commands
  for (int i=0;i<NUM_RC_DATA;i++){
      channels[i]=bt_data[i];
  } 
  //Set autonomous mode (just in case)
  //channels[CTRL] = MAX_SBUS_VALUE;
}

//**** Check is the aircraft is ready to flight
void checkConnectionStatus(){
  if(system_status == DRONE_READY){
    if(bt_status == BT_DISCONNECTED ||
      rc_status == RC_DISCONNECTED || rc_status == RC_FAILSAFE){
      //This is a failsafe situation
      if(system_status != FAILSAFE){
        PRINTS("Drone is in failsafe mode.. Drone lost BT or RC connections!! Emergency landing");
      }
      system_status = FAILSAFE;
    }
  }
  // Check if failsafe condition is present
  if(bt_status == BT_CONNECTED && rc_status == RC_CONNECTED){
    if(system_status != DRONE_READY){
      system_status = DRONE_READY;
        PRINTS("Devices connected.. Drone is ready to fly.");
    }
  }
}

//**** Check flight mode selection based on the MODE channel 
void checkFlightMode(int value){ 
  // Flight mode status
  if(value < MID_SBUS_VALUE  && flight_mode!=MANUAL_MODE){
     flight_mode = MANUAL_MODE;
     PRINTS("Manual mode enabled");
  }
  else if(value > MID_SBUS_VALUE  && flight_mode!=AUTONOMOUS_MODE){
     flight_mode = AUTONOMOUS_MODE;
       PRINTS("Autonomous mode enabled");
  }
}

//**** Show channel values on serial USB out
void printDroneData(){
    PRINT("T:",channels[THROTTLE]);
    PRINT(" P:",channels[PITCH]);
    PRINT(" R:",channels[ROLL]);
    PRINT(" Y:",channels[YAW]);
    PRINT(" Arm:",channels[ARM]);
    PRINT(" Modes:",channels[MODES]);
    PRINT(" Ctrl:",channels[CTRL]);
    PRINT(" Beep:",channels[BEEP]);
    PRINT(" - [Hw:",drone_hwHealthy);
    PRINT(" Sy:",system_status);
    PRINT(" Bt:",drone_vbat);
    PRINT(" Al:",drone_sonarAltitude);
    PRINT(" Mh:",drone_mah);
    PRINTS("]");
}

//**** Compute the CRC from the received frame
int computeRC_CRC(std::array<int16_t, NUM_RC_DATA> buff){
   int sum1 = 0;
   int sum2 = 0;
   int crc = 0;
   for (int i=0;i<NUM_RC_DATA;i++){
        sum1 = (sum1 + buff[i]) % 255;
        sum2 = (sum2 + sum1) % 255;
   }     
   crc = sum1*256 + sum2;
// D
//   if(crc<0)
//      return 65535+crc;
//   else
//      
  return  crc;
}

//**** Copy BT to buffer data
void copyRCBuff(std::array<int16_t, NUM_RC_DATA> buff){
  for(int i=0;i<NUM_RC_DATA;i++){
    bt_data[i]=buff[i];
  }
}

//**** Update LED status
void updateLEDStatus(){
    if(system_status != DRONE_READY){
      blinkLED(STARTING_BLINK);
    }
    else{
      blinkLED(RUNNING_BLINK);
    }
}

//**** Reset channels to minimum value
void resetChannels(){
  for(int i=0; i<bfs::SbusRx::NUM_CH(); i++){
    channels[i] = MIN_SBUS_VALUE;
  }
}

////**** Format the data
//TODO: Implement CRC
void copyStatusData(uint8_t sdata[7]){
   static uint8_t fmt_altitude=0;  
   static uint8_t fmt_hwHealthy=0;

   //Check the data
   if(drone_sonarAltitude >= 0){
      fmt_altitude = (uint8_t) drone_sonarAltitude;
   }
   if(drone_hwHealthy >= 0){
      fmt_hwHealthy = (uint8_t) drone_hwHealthy;
   }
    
   //Status command at the beginning
   sdata[0] = BT_START_DELIMITER;
   //Data
   sdata[1] = STATUS_CMD;
   //Hw healthy
   sdata[2] = fmt_hwHealthy;
   //System status
   sdata[3] = system_status;
   //Vbat data
   sdata[4] = drone_vbat;
   //Altitude data
   sdata[5] = fmt_altitude;
   //End
   sdata[6] = BT_END_DELIMITER;   
}

//**** Send drone status
void sendDroneStatus(){
   uint8_t statusData[7];

   copyStatusData(statusData);
   SerialBT.write(statusData,7);
}

////**** Decode BT RC data
void processRCCommand(uint8_t cBuf[BLUETOOTH_BUFFER_SIZE]){
  std::array<int16_t, NUM_RC_DATA> bt_data;
  unsigned int b1 = 0;
  unsigned int b2 = 0;
  int val,crc,i;

  //Decode RC data
  for(i=0;i<NUM_RC_DATA;i++){
    b1 = cBuf[2*i];
    b2 = cBuf[2*i+1];
    bt_data[i] = b2*256+b1;
  }
  //CRC
  b1 = cBuf[2*i];
  b2 = cBuf[2*i+1];
  crc =  b2*256+b1;

  //If the RC data is OK the RC data is copied to the buffer
  if(crc==computeRC_CRC(bt_data)){
     copyRCBuff(bt_data);
  } 
}
    
//Get gyro data
bool getGyroData(int16_t rdata[]){
  msp_raw_imu_t gyro;
  if (msp.request(MSP_RAW_IMU, &gyro, sizeof(gyro))){
     rdata[0] = gyro.gyro[0];
     rdata[1] = gyro.gyro[1];
     rdata[2] = gyro.gyro[2];
     return true;
  }
  return true;
}

//**** Decode BT RC data
void decodeRCBT(){
  static std::array<int16_t, NUM_RC_DATA> bt_data;
  unsigned int b1 = 0;
  unsigned int b2 = 0;
  int val,crc;
  static int pk_index = 0;

  //BT_END_DELIMITER '\n' we are not using this
  //A frame with BT data is comming, we read the channel data from the frame
  while(pk_index < NUM_RC_DATA+1){ //Capture the frame data including the CRC 
    if (SerialBT.available()){
      b1 = SerialBT.read();
      b2 = SerialBT.read();
      val = b2*256 + b1;
      if(pk_index < NUM_RC_DATA) //Capture the channel data from the frame
        bt_data[pk_index] = val;  
      if(pk_index>=NUM_RC_DATA-1){ //Last byte on frame is the CRC
        crc = val;  
      }  
      pk_index = pk_index+1;
    } 
  }
  pk_index=0;

  //Check RC data
  if(crc==computeRC_CRC(bt_data)){
     copyRCBuff(bt_data);
  } 
}

//**** Bluetooth update
void bluetooth_update() {
    unsigned int b = 0;    
    b = SerialBT.read(); //Start 
    if(b==BT_START_DELIMITER){ //Check the command     
      b = SerialBT.read();
      switch(b){
        case(RC_CMD):
          decodeRCBT();
          break;
        case(STATUS_CMD):
          sendDroneStatus();
          break;
        default:
          PRINTS("Received an unrecognized command");
          break;
       }
   }
}

//**** Telemetry loop
void mspLoop(void * pvParameters){
  //Update telemetry info
  for(;;){
     //Get telemetry data
     updateDroneTelem();
     vTaskDelay(10);
  }
}

//**** Main loop
void mainLoop(void * pvParameters) {
  for(;;){
    droneControl();
    vTaskDelay(10);
  }
}

//**** Fucntions to control the drone
void droneControl(){  
  //Update the LED status
  updateLEDStatus();
  //Update bluetooth
  bluetooth_update();
     
  //If all devices are connected
  if(system_status == DRONE_READY){
    // Always SBUS is reading from RC commands   
    if (sbus_rx.Read() && rc_status == RC_CONNECTED) {
      // Grab the received data 
      sbus_data = sbus_rx.ch();

      //If it is not in failsafe and the frame is valid
      if(!sbus_rx.lost_frame()){      
        //Check for failsafe
        if(sbus_rx.failsafe()){      
          if(rc_status != RC_FAILSAFE){
                rc_status = RC_FAILSAFE;
                PRINTS("Radio control failsafe"); 
          }
        }
        else if(flight_mode==MANUAL_MODE){ //When is in manual mode, all the channels are send directly to the FC
          // Send the rc data to the FC
          copyRxData();
          sbus_tx.ch(sbus_data);
          sbus_tx.Write();
        } 
        else if (flight_mode == AUTONOMOUS_MODE && bt_status == BT_CONNECTED){
          //Copy BT data to channels
          copyBTData();
          sbus_tx.ch(channels);
          sbus_tx.Write();
        }
        printDroneData();
        //Check if flight mode has been changed by RC commands switch
        checkFlightMode(sbus_data[CTRL]); 
      }
    }
  }
  //Failsafe emergency landing procedure
  else if(system_status == FAILSAFE){
    //Lower the throttle and beeping
    channels[THROTTLE] = MIN_SBUS_VALUE;
    channels[BEEP] = MAX_SBUS_VALUE;
    //Disarm when reach the ground
    if(drone_sonarAltitude <=MIN_ALT){
      channels[MODES] = MIN_SBUS_VALUE;
      channels[ARM] = MIN_SBUS_VALUE;
    }
    sbus_tx.ch(channels);
    sbus_tx.Write();
    
  }
  
  // if it is not ready to flight, check if RC is ready
  else{
    //Check if the RC data is receiving and it is valid 
    //When the receiver is connected, it sends null data to sbus channel
    if (sbus_rx.Read()) {
      //Check if the data is null
      sbus_data = sbus_rx.ch();
      //The three first channels is more than enough
      if(sbus_data[0]>=MIN_SBUS_VALUE && sbus_data[0]>=MIN_SBUS_VALUE && sbus_data[0]>=MIN_SBUS_VALUE){ 
         if(!sbus_rx.failsafe() && !sbus_rx.lost_frame()){
            if(rc_status != RC_CONNECTED){
              rc_status = RC_CONNECTED;
              PRINTS("Transmitter is connected"); 
            }
         }
       }
    }
    //Bluettoh connection lost is controlled by a callback
  }
  checkConnectionStatus();
}

//**** Not used 
void loop(){
  //This delay is important to avoid blocking
  vTaskDelay(10);
}


//******************************************************************************//
// OLDER IMPLEMENTATIONS
//**** Bluetooth init
//bool getCommand() {
//  unsigned int b = 0;
//  static enum { ST_IDLE, ST_CMD, ST_DATA, ST_END } state = ST_IDLE;
//  static uint8_t countTarget, countActual=0;
//  bool r = false;
//
//  if(SerialBT.available()){
//    b = SerialBT.read();
//    switch(state)
//    {
//      case ST_IDLE:
//        if(b==BT_START_DELIMITER){
//          countActual=0;
//          state = ST_CMD;
//          cmd_received=NONE;
//        }
//        break;
//      case ST_CMD:
//        switch(b)
//        {
//          case RC_CMD:
//            state = ST_DATA;
//            countTarget = 2*(NUM_RC_DATA+1); //Last is the CRC
//            cmd_received = RC_CMD;
//            break;
//          case STATUS_CMD:
//            state = ST_END;
//            cmd_received = STATUS_CMD;
//            break;  
//        }
//        break;
//      case ST_DATA:
//        cBuf[countActual++]=b;
//        if(countActual >= countTarget) {//Data obtained
//          state = ST_END;
//        }
//        break;
//      case ST_END:
//        state = ST_IDLE;
//        r = (b == BT_END_DELIMITER);
////        if(r==true){
////          //Check the command and then process
////          switch(cmd){
////            case(RC_CMD):
////              processRCCommand(cBuf);
////              break;
////            case(STATUS_CMD):
////              sendDroneStatus();
////              break;
////          }
////        }
//        if(!b){
//          PRINTS("Format not correct");
//        }
//        break;
//      default:
//        state = ST_IDLE;
//        break;
//    }      
//  }
//  return r;
//}
//

////**** Bluetooth get Command
//bool getCommand() {
//  unsigned int b = 0;
//  static unsigned int b1,b2;
//  static std::array<int16_t, NUM_RC_DATA+1> bt_data;
//  static enum { ST_IDLE, ST_CMD, ST_RCDATA, ST_END } state = ST_IDLE;
//  static enum { BT_LSB, BT_MSB } bit_state = BT_LSB;
//  static int cmd = NONE;
//  static uint8_t countTarget, countActual=0;
//  bool r = false;
//
//  if(SerialBT.available()){
//    b = SerialBT.read();
//    switch(state)
//    {
//      case ST_IDLE:
//        if(b==BT_START_DELIMITER){
//          countActual=0;
//          state = ST_CMD;
//          cmd=NONE;
//        }
//        break;
//      case ST_CMD:
//        switch(b)
//        {
//          case RC_CMD:
//            state = ST_RCDATA;
//            countTarget = NUM_RC_DATA+1; 
//            cmd = RC_CMD;
//            break;
//          case STATUS_CMD:
//            state = ST_END;
//            cmd = STATUS_CMD;
//            break;  
//        }
//        break;
//      case ST_RCDATA:
//        switch(bit_state){
//          case BT_LSB:
//            b1=b;
//            bit_state = BT_MSB;
//            break;
//          case BT_MSB:
//            b2=b;
//            bt_data[countActual++] = b2*256+b1;
//            bit_state = BT_LSB;
//            break;
//        }
//        if(countActual >= countTarget) {//Data obtained
//          state = ST_END;
//        }
//        break;
//      case ST_END:
//        state = ST_IDLE;
//        r = (b == BT_END_DELIMITER);
//        if(r==true){
//          //Check the command and then process
//          switch(cmd){
//            case(RC_CMD):
//              processRCCommand(bt_data);
//              break;
//            case(STATUS_CMD):
//              sendDroneStatus();
//              break;
//          }
//        }
//        else{
//          PRINTS("Format not correct");
//        }
//        break;
//      default:
//        state = ST_IDLE;
//        break;
//    }      
//  }
//  return r;
//}

////**** Process RC command 
//void processRCCommand(std::array<int16_t,  NUM_RC_DATA> bt_data){
//  int crc;
//  if(validateRCData(bt_data)){
//     copyBTRCData(bt_data);
//     Serial.println("OK");
//  } 
//}


////**** Process the commands received
//void bluetooth_update(){
//  //If a command is received
//  if(getCommand()){
//    switch(cmd_received){
//      case(RC_CMD):
//        processRCCommand(cBuf);
//        PRINTS("RC command");
//        break;
//      case(STATUS_CMD):
//        sendDroneStatus();
//        PRINTS("Status command");
//        break;
//    }
//  }
//}
//uint8_t bytes [sizeof(int)] = 
//{
//  ((uint16_t)i >> 0) & 0xFF,  // shift by 0 not needed, of course, just stylistic
//  ((uint16_t)i >> 8) & 0xFF,
//};
//// 32 bit system  
//uint8_t bytes [sizeof(int)] = 
//{
//  ((uint32_t)i >>  0) & 0xFF,
//  ((uint32_t)i >>  8) & 0xFF,
//  ((uint32_t)i >> 16) & 0xFF,
//  ((uint32_t)i >> 24) & 0xFF,
//};
