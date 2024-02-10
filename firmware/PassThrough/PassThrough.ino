#include "sbus.h"
#include "Constants.h"
#include "HardwareConfig.h"

// SBus setup ------------------------------------------------------------------
// Using the same serial port
bfs::SbusRx sbus_rx(&Serial1);
bfs::SbusTx sbus_tx(&Serial1); 

uint8_t system_status = WAITING_FOR_DEVICES;
uint8_t rc_status = RC_DISCONNECTED;
//SBus data frame
std::array<int16_t, bfs::SbusRx::NUM_CH()> sbus_data;


void setup() { 
  Serial.begin(BAUDRATE);
  // Waiting for serial
  while (!Serial) {}
  
  InitHardwarePins();
  
  // Begin the SBUS communication 
  sbus_rx.Begin(SBUS_RXPIN,SBUS_TXPIN);
  sbus_tx.Begin(SBUS_RXPIN,SBUS_TXPIN);
  #ifdef USE_SERIAL_OUTPUT
    Serial.println("Receiver device started");
    initTXSBus();
    delay(100);
    Serial.println("The device started, Waiting for the RC connection");
  #endif
 }


// Check is the aircraft is ready to flight
// Check list:
// - Radio connection (not in failsafe)
void checkConnectionStatus(){
  if(system_status == READY){
    if(rc_status == RC_DISCONNECTED || rc_status == RC_FAILSAFE){
      //This is a failsafe mode
      if(system_status != FAILSAFE){
        #ifdef USE_SERIAL_OUTPUT
          Serial.println("Failsafe mode enabled.. Drone lost RC connections!!");
        #endif
        system_status = WAITING_FOR_DEVICES;
      }
      system_status = FAILSAFE;
    }
  }
  if(rc_status == RC_CONNECTED){
    if(system_status != READY){
      system_status = READY;
      #ifdef USE_SERIAL_OUTPUT
        Serial.println("RC receiver connected..");
      #endif
    }
  }
}


//Show channel values on serial USB out
void showRCValues(){
    Serial.print(" T:");
    Serial.print(sbus_data[THROTTLE]);
    Serial.print(" P:");
    Serial.print(sbus_data[PITCH]);
    Serial.print(" R:");
    Serial.print(sbus_data[ROLL]);
    Serial.print(" Y:");
    Serial.print(sbus_data[YAW]);
    Serial.print(" M:");
    Serial.print(sbus_data[MODES]);
    Serial.print(" AUX1:");
    Serial.print(sbus_data[AUX1]);
    Serial.print(" ");
    Serial.print(" AUX2:");
    Serial.print(sbus_data[AUX2]);
    Serial.print(" ");
    Serial.print(" AUX3:");
    Serial.print(sbus_data[AUX3]);
    Serial.print(" ");
    Serial.print(" AUX4:");
    Serial.println(sbus_data[AUX4]);
    
}


// Update LED status
void updateLEDStatus(){
    if(system_status != READY){
      blinkLED(STARTING_BLINK);
    }
    else{
      blinkLED(RUNNING_BLINK);
    }
}

// Init sbus data and send to the FC
void initTXSBus(){
  //Center sticks and lower the throttle
  sbus_data[THROTTLE]=MIN_SBUS_VALUE;
  sbus_data[PITCH]=MID_SBUS_VALUE;
  sbus_data[ROLL]=MID_SBUS_VALUE;
  sbus_data[YAW]=MID_SBUS_VALUE;
  //Aux channels to min value
  for(int i=4;i<bfs::SbusRx::NUM_CH();i++){
    sbus_data[i]=MIN_SBUS_VALUE;
  }
}

// Main loop
void loop() {
  //Update the LED status
  updateLEDStatus();
  
  //If all devices are connected
  if(system_status == READY){

    // Always SBUS is reading from RC commands   
    if (sbus_rx.Read() && rc_status == RC_CONNECTED) {
      // Grab the received data 
      sbus_data = sbus_rx.ch();
      if(sbus_rx.lost_frame()==false){      
          sbus_tx.ch(sbus_data);
          // Write the data to the FC 
          sbus_tx.Write();
          rc_status = RC_CONNECTED;
          #ifdef USE_SERIAL_OUTPUT
            Serial.print("[RC commands] ");
            showRCValues();
            Serial.println(""); 
          #endif    
      }
    } 
  }
  else{
    //Check if the RC data is receiving and it is in not failsafe mode
    if (sbus_rx.Read()) {
      //Check if the data is null
      sbus_data = sbus_rx.ch();
      //The three first channels is more than enough
      if(sbus_data[0]>=MIN_SBUS_VALUE && sbus_data[0]>=MIN_SBUS_VALUE && sbus_data[0]>=MIN_SBUS_VALUE){ 
         if(!sbus_rx.failsafe() && !sbus_rx.lost_frame()){
            if(rc_status != RC_CONNECTED){
              rc_status = RC_CONNECTED;
              #ifdef USE_SERIAL_OUTPUT
                Serial.println("Transmitter is connected"); 
              #endif  
            }
         }
      }
    }
  }
  checkConnectionStatus();
}
