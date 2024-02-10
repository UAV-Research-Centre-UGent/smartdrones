/*
 * There are three serial ports on the ESP known as U0UXD, U1UXD and U2UXD.
 * 
 * U0UXD is used to communicate with the ESP32 for programming and during reset/boot.
 * U1UXD is unused and can be used for your projects. Some boards use this port for SPI Flash access though
 * U2UXD is unused and can be used for your projects.
 * 
*/

#include "sbus.h"
#include "BluetoothSerial.h"
#include "Constants.h"
#include "HardwareConfig.h"

// SBus setup ------------------------------------------------------------------
bfs::SbusRx sbus_rx(&Serial1);
bfs::SbusTx sbus_tx(&Serial1); 

// BT setup---------------------------------------------------------------------
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
 
// Variables-------------------------------------------------------------------
BluetoothSerial SerialBT;

uint8_t system_status = WAITING_FOR_DEVICES;
uint8_t drone_status = DRONE_DISARMED;
uint8_t flight_mode = MANUAL_MODE;
uint8_t bt_status = BT_DISCONNECTED;
uint8_t rc_status = RC_DISCONNECTED;
//Decoded channel information from SBUS and BT
static std::array<int16_t, bfs::SbusRx::NUM_CH()> channels;
//Bluetooth data frame. The frame only transmit the number of channels in this version
static std::array<int16_t, BT_DATA_FRAME_SIZE> bt_data;
static std::array<int16_t, NUM_CHANNELS> bt_channels;
//SBus data frame
std::array<int16_t, bfs::SbusRx::NUM_CH()> sbus_data;

int pk_index = 0;

void setup() { 
  Serial.begin(BAUDRATE);
  // Waiting for serial
  while (!Serial) {}
  
  InitHardwarePins();
  
  // Bluetooth setup
  if(!SerialBT.begin(AIRCRAFT_NAME)){
    Serial.println("An error occurred initializing Bluetooth");
    bt_status = BT_ERROR;
  }else{
    Serial.println("Bluetooth device started");
    SerialBT.register_callback(callback);
  }
  delay(100);

  // Begin the SBUS communication 
  sbus_rx.Begin(SBUS_RXPIN,SBUS_TXPIN);
  sbus_tx.Begin(SBUS_RXPIN,SBUS_TXPIN);
  Serial.println("Receiver device started");
  delay(100);

  Serial.println("The device started, now you can pair it with bluetooth!");
  Serial.println("System is waiting for the devices (FlightCOMPuter/GPS/RadioControl)"); 
}

// Call back to check if the bluetooth was disconnected
void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
  if(event == ESP_SPP_SRV_OPEN_EVT){
    bt_status = BT_CONNECTED;
    Serial.println("FCOM (Bluetooth) is connected");
  }
  if(event == ESP_SPP_CLOSE_EVT ){
    bt_status = BT_DISCONNECTED;
    Serial.println("FCOM (Bluetooth) is disconnected");
  }
}

// Copy SBus data to channel, this is mainly for visualize the data
void sBUSToChannel(){
  for(int i =0; i<bfs::SbusRx::NUM_CH(); i++){
    channels[i] = sbus_data[i];
  }
}

// Check is the aircraft is ready to flight
// Check list:
// - Bluetooth connection
// - Radio connection (not in failsafe)
// - TODO:
// - MSP connection 
// - AIRCRAFT ready (GPS fixed, all devices working properlly)
void checkConnectionStatus(){
  if(system_status == READY_TO_FLIGHT){
    if((bt_status == BT_DISCONNECTED || bt_status == BT_ERROR) && (rc_status == RC_DISCONNECTED || rc_status == RC_FAILSAFE)){
      //This is a failsafe mode
      if(system_status != FAILSAFE){
        Serial.println("Failsafe mode enabled.. Drone lost BT and RC connections!!");
      }
      system_status = FAILSAFE;
    }
  }
  if(bt_status == BT_CONNECTED && rc_status == RC_CONNECTED){
    if(system_status != READY_TO_FLIGHT){
      system_status = READY_TO_FLIGHT;
      Serial.println("Devices connected.. Drone is ready to flight.");
    }
  }
}

// Check flight mode selection based on the MODE channel from the transmitter (manual)
// Flight mode can be change from the transmitter
void checkFlightMode(int value){ 
  // Flight mode status
  if(value < MID_SBUS_VALUE  && flight_mode!=MANUAL_MODE){
     flight_mode = MANUAL_MODE;
     Serial.println("Manual mode enabled");
  }
  else if(value > MID_SBUS_VALUE  && flight_mode!=AUTONOMOUS_MODE){
     flight_mode = AUTONOMOUS_MODE;
     Serial.println("Autonomous mode enabled");
  }
}

//Shoe channel values on serial USB out
void showChannelValues(){
    Serial.print(" T:");
    Serial.print(channels[THROTTLE]);
    Serial.print(" P:");
    Serial.print(channels[PITCH]);
    Serial.print(" R:");
    Serial.print(channels[ROLL]);
    Serial.print(" Y:");
    Serial.print(channels[YAW]);
    Serial.print(" A1:");
    Serial.print(channels[AUX1]);
    Serial.print(" A2:");
    Serial.print(channels[AUX2]);
    Serial.print(" A3(*):");
    Serial.print(channels[AUX3]);
}

//Copy BT frame data to channels
void copyBTData(std::array<int16_t, BT_DATA_FRAME_SIZE> buff){
  //Copy channel data
  for(int i =0; i<NUM_CHANNELS; i++){
    channels[i] = buff[i];
  }
  for(int i=NUM_CHANNELS; i<bfs::SbusRx::NUM_CH(); i++){
    channels[i] = MIN_SBUS_VALUE;
  }
  //Copy rest of the data
  //TODO:
  //Aircraft status, GPS data, altitude data, velocity data, sensor data
}

// Decode data recceived by Bluetooth
int decodeBT(){
  unsigned int b1 = 0;
  unsigned int b2 = 0;
  int val,crc;

  //A frame with BT data is comming, we read the channel data from the frame
  while(pk_index < BT_DATA_FRAME_SIZE){ //Capture the frame data including the CRC 
    if (SerialBT.available()){
      b1 = SerialBT.read();
      b2 = SerialBT.read();
      val = b2*256 + b1;
      if(pk_index < NUM_CHANNELS) //Capture the channel data from the frame
        bt_data[pk_index] = val;  
      if(pk_index>=BT_DATA_FRAME_SIZE-1){ //Last byte on frame is the CRC
        crc = val;  
      }  
      pk_index = pk_index+1;
    } 
  }
  pk_index=0;
  return crc;
}

//Compute the CRC from the received frame
int computeCRC(std::array<int16_t, BT_DATA_FRAME_SIZE> buff){
   int sum1 = 0;
   int sum2 = 0;
   for (int i=0;i<BT_DATA_FRAME_SIZE-1;i++){
        sum1 = (sum1 + buff[i]) % 255;
        sum2 = (sum2 + sum1) % 255;
   }     
   return sum1*256 + sum2;
}

// Update LED status
void updateLEDStatus(){
    if(system_status != READY_TO_FLIGHT){
      blinkLED(STARTING_BLINK);
    }
    else{
      blinkLED(RUNNING_BLINK);
    }
}

// Main loop
void loop() {
  unsigned int b1 = 0;
  unsigned int b2 = 0;
  int val,crc;

  //Update the LED status
  updateLEDStatus();
  
  //If all devices are connected
  if(system_status == READY_TO_FLIGHT){

    // Always SBUS is reading from RC commands   
    if (sbus_rx.Read() && rc_status == RC_CONNECTED) {
      // Grab the received data 
      sbus_data = sbus_rx.ch();
      if(sbus_rx.lost_frame()==false){      
        if(flight_mode==MANUAL_MODE){ //When is in manual mode, all the channels are send directly to the FC
          sbus_tx.ch(sbus_data);
          // Write the data to the FC 
          sbus_tx.Write();
          sBUSToChannel();
          Serial.print("[RC commands] ");
          showChannelValues();
          Serial.println("");
        } 
        //Check if flight mode has been changed by RC commands switch
        checkFlightMode(sbus_data[AUX3]); //AUX 3 is used to take control of the drone, this channel is only used by the transmitter
      }
    }

    //Bluetooth reading 
    if (flight_mode == AUTONOMOUS_MODE && SerialBT.available()) { 
      //Bluettoh decode
      b1 = SerialBT.read();
      b2 = SerialBT.read();
      val = b2*256 + b1;
      // This is to prevent getting more frames
      if(bt_status == BT_CONNECTED){
         if(val==START_BT){
            if (decodeBT() == computeCRC(bt_data)){
                copyBTData(bt_data);
                Serial.print("[BT commands] ");
                showChannelValues();
                Serial.println("");
                //Send commands to the FC
                sbus_tx.ch(channels);
                // Write the data to the FC 
                sbus_tx.Write();
            }
         }
      }
    }  
  }
  else{
    //Check if the RC data is receiving and it is in not failsafe mode
    if (sbus_rx.Read()) {
       if(sbus_rx.failsafe() != 1){
          if(rc_status != RC_CONNECTED){
            rc_status = RC_CONNECTED;
            Serial.println("Transmitter is connected"); 
          }
       }
    }
    //Bluettoh connection lost is controlled by a callback
  }
  checkConnectionStatus();
}
