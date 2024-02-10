//**** Includes
#include "BluetoothSerial.h"
#include "Constants.h"
#include "HardwareConfig.h"
#include "MSP.h"
#include "sbus.h"

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

//**** Drone data
int32_t altitude = 0; //Altitude in centimeters
uint8_t drone_vbat = 0; //Main analog voltage (from the battery pads)
uint16_t drone_mah = 0;
// Raw data
// To convert these data into pitch,roll,yaw data: 
// https://engineering.stackexchange.com/questions/3348/calculating-pitch-yaw-and-roll-from-mag-acc-and-gyro-data
int16_t drone_gyro[3]={0,0,0};
int16_t drone_acc[3]={0,0,0};
int16_t drone_mag[3]={0,0,0};

// Status
uint8_t system_status = WAITING_FOR_DEVICES;
uint8_t drone_status = DRONE_DISARMED;
uint8_t flight_mode = MANUAL_MODE;
uint8_t bt_status = BT_DISCONNECTED;
uint8_t rc_status = RC_DISCONNECTED;

// Decoded channel information from SBUS and BT
static std::array<int16_t, bfs::SbusRx::NUM_CH()> channels;
// Bluetooth data frame
static std::array<int16_t, BT_DATA_FRAME_SIZE> bt_data;
static std::array<int16_t, NUM_CHANNELS> bt_channels;
// SBus data frame
std::array<int16_t, bfs::SbusRx::NUM_CH()> sbus_data;

// Communication with the FC using MSP protocol
MSP msp;

void setup() {

  Serial.begin(BAUDRATE);

  //**** Waiting for serial
  while (!Serial) {
  }

  //**** Init hardware pins
  InitHardwarePins();

  // ---- Bluetooth
  bluetooth_init();
  delay(500);

  // Begin the SBUS communication
  sbus_rx.Begin(SBUS_RXPIN, SBUS_TXPIN);
  sbus_tx.Begin(SBUS_RXPIN, SBUS_TXPIN);
#ifdef USE_SERIAL_OUTPUT
  Serial.println("Receiver device started");
#endif
  resetChannels();
  sendChannelsData(); // Prevent randon values being sent to the FC
  delay(500);

  //****  MSP communication
  Serial2.begin(MSP_BAUDRATE, SERIAL_8N1, MSP_RXPIN, MSP_TXPIN);
  msp.begin(Serial2);

  //**** All ready
#ifdef USE_SERIAL_OUTPUT
  Serial.println("The device started.. Waiting connections");
#endif

  //**** Task to control loops
  xTaskCreatePinnedToCore(
      mainLoop,   /* Task function. */
      "mainLoop", /* name of task. */
      10000,      /* Stack size of task */
      NULL,       /* parameter of the task */
      1,          /* priority of the task */
      &mainTask,  /* Task handle to keep track of created task */
      0);         /* pin task to core 0 */
  delay(500);

  // create a task that will be executed in the Task2code() function, with
  // priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
      mspLoop,   /* Task function. */
      "mspLoop", /* name of task. */
      10000,     /* Stack size of task */
      NULL,      /* parameter of the task */
      1,         /* priority of the task */
      &mspTask,  /* Task handle to keep track of created task */
      1);        /* pin task to core 1 */
  delay(500);
}

//**** Send channels data to the FC
void sendChannelsData() {
  sbus_tx.ch(channels);
  sbus_tx.Write();
}

// Call back to check if the bluetooth was disconnected
void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    bt_status = BT_CONNECTED;
    Serial.println("FCOM (Bluetooth) is connected");
  }
  if (event == ESP_SPP_CLOSE_EVT) {
    bt_status = BT_DISCONNECTED;
    Serial.println("FCOM (Bluetooth) is disconnected");
  }
}

//**** Init bluetooth
void bluetooth_init() {
  if (!SerialBT.begin(AIRCRAFT_NAME)) {
#ifdef USE_SERIAL_OUTPUT
    Serial.println("Bluetooth device can not be initialized");
#endif
  } else {
#ifdef USE_SERIAL_OUTPUT
    Serial.println("Bluetooth device started");
#endif
    SerialBT.register_callback(callback);
  }
  // Set all data lo low values
  initRCBTData();
}

// Copy SBus data to channel, this is mainly for visualize the data
void sBUSToChannel() {
  for (int i = 0; i < bfs::SbusRx::NUM_CH(); i++) {
    channels[i] = sbus_data[i];
  }
}

int32_t getBaroAltitude() {
  msp_altitude_t alt;
  if (msp.request(MSP_ALTITUDE, &alt, sizeof(alt))) {
    return alt.estimatedActualPosition;
  }
  return MSP_INT32_ERROR;
}

// Check is the aircraft is ready to flight
// Check list:
// - Bluetooth connection
// - Radio connection (not in failsafe)
// - TODO:
// - MSP connection
// - AIRCRAFT ready (GPS fixed, all devices working properlly)
void checkConnectionStatus() {
  if (system_status == READY_TO_FLIGHT) {
    if ((bt_status == BT_DISCONNECTED || bt_status == BT_ERROR) &&
        (rc_status == RC_DISCONNECTED || rc_status == RC_FAILSAFE)) {
      // This is a failsafe mode
      if (system_status != FAILSAFE) {
        Serial.println(
            "Failsafe mode enabled.. Drone lost BT and RC connections!!");
      }
      system_status = FAILSAFE;
    }
  }
  if (bt_status == BT_CONNECTED && rc_status == RC_CONNECTED) {
    if (system_status != READY_TO_FLIGHT) {
      system_status = READY_TO_FLIGHT;
      Serial.println("Devices connected.. Drone is ready to flight.");
    }
  }
}

// Check flight mode selection based on the MODE channel from the transmitter
// (manual) Flight mode can be change from the transmitter
void checkFlightMode(int value) {
  // Flight mode status
  if (value < MID_SBUS_VALUE && flight_mode != MANUAL_MODE) {
    flight_mode = MANUAL_MODE;
    Serial.println("Manual mode enabled");
  } else if (value > MID_SBUS_VALUE && flight_mode != AUTONOMOUS_MODE) {
    flight_mode = AUTONOMOUS_MODE;
    Serial.println("Autonomous mode enabled");
  }
}

//**** Update drone status from FC
void updateDroneTelem() {
  msp_sonar_altitude_t sonar_alt;
  msp_analog_t adata;
  msp_raw_imu_t imu_data;
  msp_altitude_t baro_alt;
  // Update data only when is correct

  if (msp.request(MSP_ALTITUDE, &baro_alt, sizeof(baro_alt))) {
    altitude =  baro_alt.estimatedActualPosition;
  }
  // if (msp.request(MSP_SONAR_ALTITUDE, &sonar_alt, sizeof(sonar_alt))) {
  //   altitude = sonar_alt.altitude; // int32_t
  // }
  // if (msp.request(MSP_ANALOG, &adata, sizeof(adata))){
  //   drone_vbat = adata.vbat;
  //   drone_mah = adata.mAhDrawn;
  // }
  // if (msp.request(MSP_RAW_IMU, &imu_data, sizeof(imu_data))){
  //   //Gyro data
  //   drone_gyro[0]=imu_data.gyro[0];
  //   drone_gyro[1]=imu_data.gyro[1];
  //   drone_gyro[2]=imu_data.gyro[2];
  //   //Acc
  //   drone_acc[0]=imu_data.acc[0];
  //   drone_acc[1]=imu_data.acc[1];
  //   drone_acc[2]=imu_data.acc[2];
  //   //Mag
  //   drone_mag[0]=imu_data.mag[0];
  //   drone_mag[1]=imu_data.mag[2];
  //   drone_mag[2]=imu_data.mag[2];
  // }




  // if (msp.request(MSP_ANALOG, &adata, sizeof(adata))) {
  //   voltage = adata.vbat;
  //   rssi = adata.rssi;
  // }
}

//**** Init BT data properly
void initRCBTData() {
  // Min throtle
  bt_data[THROTTLE] = MIN_SBUS_VALUE;
  // All in the middle
  bt_data[PITCH] = MID_SBUS_VALUE;
  bt_data[ROLL] = MID_SBUS_VALUE;
  bt_data[YAW] = MID_SBUS_VALUE;
  // Disarmed
  bt_data[AUX1] = MIN_SBUS_VALUE;
  // Autonomous mode ON
  bt_data[AUX2] = MIN_SBUS_VALUE;
  // All the rest in low position
  bt_data[AUX3] = MAX_SBUS_VALUE;
  // bt_data[BEEP]=MIN_SBUS_VALUE;
}

// Shoe channel values on serial USB out
void showChannelValues() {

  // Serial.print(" T:");
  // Serial.println(channels[THROTTLE]);
  // Serial.print(" P:");
  // Serial.println(channels[PITCH]);
  // Serial.print(" R:");
  // Serial.println(channels[ROLL]);
  // Serial.print(" Y:");
  // Serial.println(channels[YAW]);
  // Serial.print(" A1:");
  // Serial.println(channels[AUX1]);
  // Serial.print(" A2:");
  // Serial.println(channels[AUX2]);
  // Serial.print(" A3(*):");
  // Serial.println(channels[AUX3]);
  /*SerialBT.print(" Rssi:");
  SerialBT.print(rssi);*/

  //SerialBT.print(" Baro alt:");
  SerialBT.println(getBaroAltitude());

  // //SerialBT.print(" Volt:");
  // SerialBT.println(drone_vbat);

  // //SerialBT.print("GyroX: ");
  // SerialBT.println(drone_gyro[0]);

  // //SerialBT.print("GyroY: ");
  // SerialBT.println(drone_gyro[1]);

  // //SerialBT.print("GyroZ: ");
  // SerialBT.println(drone_gyro[2]);

  // //SerialBT.print("AccX: ");
  // SerialBT.println(drone_acc[0]);

  // //SerialBT.print("AccY: ");
  // SerialBT.println(drone_acc[1]);

  // //SerialBT.print("AccZ: ");
  // SerialBT.println(drone_acc[2]); 
}

// Copy BT frame data to channels
void copyBTData(std::array<int16_t, BT_DATA_FRAME_SIZE> buff) {
  // Copy channel data
  for (int i = 0; i < NUM_CHANNELS; i++) {
    channels[i] = buff[i];
  }
  for (int i = NUM_CHANNELS; i < bfs::SbusRx::NUM_CH(); i++) {
    channels[i] = MIN_SBUS_VALUE;
  }
  // Copy rest of the data
  // TODO:
  // Aircraft status, GPS data, altitude data, velocity data, sensor data
}

// Decode data recceived by Bluetooth
int decodeBT() {
  unsigned int b1 = 0;
  unsigned int b2 = 0;
  int val, crc;
  int pk_index = 0;

  // A frame with BT data is comming, we read the channel data from the frame
  while (pk_index <
         BT_DATA_FRAME_SIZE) { // Capture the frame data including the CRC
    if (SerialBT.available()) {
      b1 = SerialBT.read();
      b2 = SerialBT.read();
      val = b2 * 256 + b1;
      if (pk_index < NUM_CHANNELS) // Capture the channel data from the frame
        bt_data[pk_index] = val;
      if (pk_index >= BT_DATA_FRAME_SIZE - 1) { // Last byte on frame is the CRC
        crc = val;
      }
      pk_index = pk_index + 1;
    }
  }
  pk_index = 0;
  return crc;
}

// Compute the CRC from the received frame
int computeCRC(std::array<int16_t, BT_DATA_FRAME_SIZE> buff) {
  int sum1 = 0;
  int sum2 = 0;
  for (int i = 0; i < BT_DATA_FRAME_SIZE - 1; i++) {
    sum1 = (sum1 + buff[i]) % 255;
    sum2 = (sum2 + sum1) % 255;
  }
  return sum1 * 256 + sum2;
}

// Update LED status
void updateLEDStatus() {
  if (system_status != READY_TO_FLIGHT) {
    blinkLED(STARTING_BLINK);
  } else {
    blinkLED(RUNNING_BLINK);
  }
}

//**** Reset channels to minimum value
void resetChannels() {
  for (int i = 0; i < bfs::SbusRx::NUM_CH(); i++) {
    channels[i] = MIN_SBUS_VALUE;
  }
}

// Control the drone
void droneControl() {

  unsigned int b1 = 0;
  unsigned int b2 = 0;
  int val, crc;

  // Update the LED status
  updateLEDStatus();

  showChannelValues();

  // If all devices are connected
  if (system_status == READY_TO_FLIGHT) {

    // Always SBUS is reading from RC commands
    if (sbus_rx.Read() && rc_status == RC_CONNECTED) {
      // Grab the received data
      sbus_data = sbus_rx.ch();
      if (sbus_rx.lost_frame() == false) {
        if (flight_mode ==
            MANUAL_MODE) { // When is in manual mode, all the channels are send
                           // directly to the FC
          sbus_tx.ch(sbus_data);
          // Write the data to the FC
          sbus_tx.Write();
          sBUSToChannel();
          Serial.print("[RC commands] ");
          showChannelValues();
          Serial.println("");
        }
        // Check if flight mode has been changed by RC commands switch
        checkFlightMode(
            sbus_data[AUX3]); // AUX 3 is used to take control of the drone,
                              // this channel is only used by the transmitter
      }
    }

    // Bluetooth reading
    if (flight_mode == AUTONOMOUS_MODE && SerialBT.available()) {

      // Bluettoh decode
      b1 = SerialBT.read();
      b2 = SerialBT.read();
      val = b2 * 256 + b1;
      // This is to prevent getting more frames
      if (bt_status == BT_CONNECTED) {
        if (val == START_BT) {
          if (decodeBT() == computeCRC(bt_data)) {
            copyBTData(bt_data);
            Serial.print("[BT commands] ");
            showChannelValues();
            Serial.println("");
            // Send commands to the FC
            sbus_tx.ch(channels);
            // Write the data to the FC
            sbus_tx.Write();
          }
        }
      }
    }
  } else {
    // Check if the RC data is receiving and it is in not failsafe mode
    if (sbus_rx.Read()) {
      if (sbus_rx.failsafe() != 1) {
        if (rc_status != RC_CONNECTED) {
          rc_status = RC_CONNECTED;
          Serial.println("Transmitter is connected");
        }
      }
    }
    // Bluettoh connection lost is controlled by a callback
  }
  checkConnectionStatus();
}

//**** Telemetry loop
void mspLoop(void *pvParameters) {
  // Update telemetry info
  for (;;) {
    // Get telemetry data
    updateDroneTelem();
    vTaskDelay(10);
  }
}

//**** Main loop
void mainLoop(void *pvParameters) {
  for (;;) {
    droneControl();
    vTaskDelay(10);
  }
}

//**** Not used
void loop() {
  // This delay is important to avoid blocking
  vTaskDelay(10);
}