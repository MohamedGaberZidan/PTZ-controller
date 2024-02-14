/******
 * The firmware is developed to control PTZ motors using TCP commands over WIFI 
 * Hardware used 
 * ESP32 
 * AS5600 Encoder 
 * TMC2208
 * NEMA14 stepper motor
 * Ethernet cable color code
 * Cable                   Pin 
 * Orange                  Tmc2208 EN        
 * White with orange       Tmc2208 VDD or VIO
 * Brown                   AS5600 SDA
 * White with brown        AS5600 SCL
 * Green                   Tmc2208 VM
 * White with green        Tmc2208 DIR
 * Blue                    Tmc2208 STEP
 * White with blue         Common ground 
 * ESP32
 * SCL                     pin22
 * SDA                     pin21
 * White with blue         GND
*/
#include <AS5600.h>
#include <FastAccelStepper.h>
#include <WiFi.h>
#include "esp_task_wdt.h"
#include <SPI.h>
#include <Wire.h>
#include <nvs_flash.h>
#define step1_pinSTEP 26
#define step1_pinDIR  27
#define step2_pinSTEP 12
#define step2_pinDIR  14
#define step3_pinSTEP 25    
#define step3_pinDIR  33

#define StepD 13                              // Pin GPio 13 Activate Deactivate steppers
#define StepFOC 32                            //Focus motor enable pin for Stepper3
#define StepZOOM 32                            //Focus motor enable pin for Stepper3




FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper1 = NULL;
FastAccelStepper *stepper2 = NULL;
FastAccelStepper *stepper3 = NULL;



int Tiltposition;
int Panposition;
int Zoomposition;


unsigned long previousMillis = 0;
const long debouncedelay = 100;
float panspeed_current = 0;
float tiltspeed_current = 0;
float focusspeed_current = 0;
float zoomspeed_current = 0;


float speedfactor = 1;
float accelerationfactor = 1;
int pan_is_moving;
int tilt_is_moving;
int Zoo_is_moving;


//Encoders initialization
AS5600 Z_encoder;
AS5600 P_encoder;
AS5600 T_encoder;

// Replace with your network credentials
const char* ssid = "Your_SSID";
const char* password = "Your_PASSWORD";

// Static IP configuration
IPAddress staticIP(192, 168, 1, 100); // ESP32 static IP
IPAddress gateway(192, 168, 1, 1);    // IP Address of your network gateway (router)
IPAddress subnet(255, 255, 255, 0);   // Subnet mask
IPAddress primaryDNS(192, 168, 1, 1); // Primary DNS (optional)
IPAddress secondaryDNS(0, 0, 0, 0);   // Secondary DNS (optional)

int position = 1;
void setup(){


  //steppers init
  engine.init();
  stepper1 = engine.stepperConnectToPin(step1_pinSTEP);
  if (stepper1) {
  stepper1->setDirectionPin(step1_pinDIR);
  stepper1->setEnablePin(StepD);
  stepper1->setAutoEnable(false);
  }
  stepper2 = engine.stepperConnectToPin(step2_pinSTEP);
  if (stepper2) {
  stepper2->setDirectionPin(step2_pinDIR);
  stepper2->setEnablePin(StepD);
  stepper2->setAutoEnable(false);
  }
  stepper3 = engine.stepperConnectToPin(step3_pinSTEP);
  if (stepper3) {
  stepper3->setDirectionPin(step3_pinDIR);
  stepper3->setEnablePin(StepFOC);
  stepper3->setAutoEnable(false);
  }

  // init serial
  Serial.begin(115200);
  // Hall sensor input pin init
  pinMode(Hall_Pan, INPUT_PULLUP);
  pinMode(Hall_Tilt, INPUT_PULLUP);

  digitalWrite(StepD, LOW);                                     //Stepper Driver Activation
  stepper1->setCurrentPosition(0);                              //Set all steppers to position 0
  stepper2->setCurrentPosition(0);
  stepper3->setCurrentPosition(0);


  //*************************************Setup a core to run Encoder****************************
  xTaskCreatePinnedToCore(coreas1signments, "Core_1", 10000, NULL, 2, &C1, 0);

  delay(100);
  disableCore1WDT();
  delay(2000);


  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  // Configuring static IP
  if(!WiFi.config(staticIP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("Failed to configure Static IP");
  } else {
    Serial.println("Static IP configured!");
  }

  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());  // Print the ESP32 IP address to Serial Monitor


  digitalWrite(StepD, LOW);                                     //Stepper Driver Activation
}//end setup

void loop(){
  stepper1->setSpeedInHz(3000);
  stepper1->setAcceleration(1000);
   stepper1->moveTo(position);  
   while (stepper1->isRunning() ) {
        delay(10);
        print("moving");
      }

  position+=5;


}
void coreas1signments( void * pvParameters ) {
  for (;;) {
    Start_Z_Encoder();
    Start_T_Encoder();
    Start_P_Encoder();
    vTaskDelay(10);
  };
}

//*****Zoom Encoder
void Start_Z_Encoder() {
  TCA9548A(2);
  if (Z_ResetEncoder == 1) {
    //    Z_encoder.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
    //    Z_encoder.resetCumulativePosition(0);
    Z_encoder.setZero();
    //Z_encoder.resetPosition();
    Z_lastOutput = 0;
    Z_S_position = 0;
    Z_E_position = 0;
    Z_revolutions = 0;
    Z_ResetEncoder = 0;

  }
  //Z_output = Z_encoder.getCumulativePosition() ;
  Z_output = Z_encoder.getPosition();           // get the raw value of the encoder

  if ((Z_lastOutput - Z_output) > 2047 ) {       // check if a full rotation has been made
    Z_revolutions++;
  }
  if ((Z_lastOutput - Z_output) < -2047 ) {
    Z_revolutions--;
  }
  Z_E_position = Z_revolutions * 4096 + Z_output;   // calculate the position the the encoder is at based off of the number of revolutions

  //Serial.println("Encoder E_Position=");
  //Serial.println(E_position);

  Z_lastOutput = Z_output;                      // save the last raw value for the next loop
  Z_E_outputPos = Z_E_position;

  Z_S_position = ((Z_E_position / 2.56));       //Ajust encoder to stepper values the number of steps eqiv
  Z_S_position = (Z_S_position * 2);            //Ajust encoder to stepper values the number of steps eqiv
  //Serial.println(Z_S_position);
}


//*******Tilt Encoder
void Start_T_Encoder() {
  TCA9548A(1);
  if (T_ResetEncoder == 1) {
    //    T_encoder.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
    //    T_encoder.resetCumulativePosition(0);
    T_encoder.setZero();
    //T_encoder.resetPosition();
    T_lastOutput = 0;
    T_S_position = 0;
    T_E_position = 0;
    T_revolutions = 0;
    T_ResetEncoder = 0;

  }
  //T_output = T_encoder.getCumulativePosition() ;
  T_output = T_encoder.getPosition();           // get the raw value of the encoder

  if ((T_lastOutput - T_output) > 2047 ) {       // check if a full rotation has been made
    T_revolutions++;
  }
  if ((T_lastOutput - T_output) < -2047 ) {
    T_revolutions--;
  }
  T_E_position = T_revolutions * 4096 + T_output;   // calculate the position the the encoder is at based off of the number of revolutions

  //Serial.println("Encoder E_Position=");
  //Serial.println(E_position);

  T_lastOutput = T_output;                      // save the last raw value for the next loop
  T_E_outputPos = T_E_position;

  T_S_position = ((T_E_position / 2.56));       //Ajust encoder to stepper values the number of steps eqiv


  //Serial.println(P_S_position);
  T_S_position = (T_S_position * 5.18);            //Ajust for gear ratio 5.18:1 which is to fast for the encoder on the!
  //******This is required if the motor is geared and the encoder is NOT on the back of the motor. (The AS5600 canot keep up with the fast moving motor) *****************

  if (T_output < 0) {                               //Reverse the values for the encoder position
    T_S_position = abs(T_S_position);
  } else {
    T_S_position = T_S_position - (T_S_position * 2);

  }
  //***************************************************************************************************************

}

//*******Pan Encoder
void Start_P_Encoder() {
  TCA9548A(3);
  if (P_ResetEncoder == 1) {
    //    P_encoder.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
    //    P_encoder.resetCumulativePosition(0);
    P_encoder.setZero();
    //P_encoder.resetPosition();
    P_lastOutput = 0;
    P_S_position = 0;
    P_E_position = 0;
    P_revolutions = 0;
    P_ResetEncoder = 0;

  }
  //P_output = P_encoder.getCumulativePosition() ;
  //P_output = P_encoder.rawAngle() ;
  P_output = P_encoder.getPosition();           // get the raw value of the encoder

  if ((P_lastOutput - P_output) > 2047 ) {       // check if a full rotation has been made
    P_revolutions++;
  }
  if ((P_lastOutput - P_output) < -2047 ) {
    P_revolutions--;
  }
  P_E_position = P_revolutions * 4096 + P_output;   // calculate the position the the encoder is at based off of the number of revolutions

  //Serial.println("Encoder E_Position=");
  //Serial.println(E_position);

  P_lastOutput = P_output;                      // save the last raw value for the next loop
  P_E_outputPos = P_E_position;

  P_S_position = ((P_E_position / 2.56));       //Ajust encoder to stepper values the number of steps eqiv
  // P_S_position = (P_S_position * 2);            //Ajust encoder to stepper values the number of steps eqiv
  //Serial.println(P_S_position);
  // if (P_output < 0) {                               //Reverse the values for the encoder position
  //    P_S_position = abs(P_S_position);
  //  } else {
  //    P_S_position = P_S_position - (P_S_position * 2);
  //
  //  }
}