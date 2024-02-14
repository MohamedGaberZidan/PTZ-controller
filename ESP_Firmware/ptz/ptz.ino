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


TaskHandle_t C1;






//Encoders initialization
AS5600 Z_encoder;
AS5600 P_encoder;
AS5600 T_encoder;

// Replace with your network credentials
const char* ssid = "SPECTRO";
const char* password = "123456789";

// Static IP configuration
IPAddress staticIP(192, 168, 1, 100); // ESP32 static IP
IPAddress gateway(192, 168, 1, 1);    // IP Address of your network gateway (router)
IPAddress subnet(255, 255, 255, 0);   // Subnet mask
IPAddress primaryDNS(192, 168, 1, 1); // Primary DNS (optional)
IPAddress secondaryDNS(0, 0, 0, 0);   // Secondary DNS (optional)

int position = 1000;
// Select I2C BUS
void TCA9548A(uint8_t bus) {
  Wire.beginTransmission(0x70);  // TCA9548A address
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
  //Serial.print(bus);
}

//Variables
//int T_Rev = 0;
int VLM;                                      //Used by VISCA to set limits
long BD = 1000;                                 //Bounce / Return delay
int ZTest;
int FTest;
int Pan_Stop;
int Tilt_Stop;
int Forward_P_Out = 3700;                   //Pan & Tilt limits
int Forward_P_In = -3700;
int Forward_T_Out = 7000;
int Forward_T_In = -3500;

int Tiltposition;
int Panposition;
int Focusposition;
int Zoomposition;


int Snd;
int clrK;                                     //Clear all keys 1 or 0
int stopM_active;
int OLED_ID;
int lastP1;
int lastP2;
int lastP3;
int lastP4;
int lastP5;
int lastP6;
int lastP7;
int lastP8;
int lastP9;

int lastInP;
int lastOutP;

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
int focus_is_moving;
int Zoo_is_moving;

String DBFirmwareVersion = "frm V:6.03";      //Current firmware version

//*******************************************************************************************************************
//Change these valus depending on which pan tilt head you havebuilt over the top or balanced
int Tilt_J_Speed = 150;                       //Set this to 150 for the Balanced PT 200 for the Over the Top PT
int Pan_J_Speed = 4000;                        //Set this to 4000  for both the Balanced PT and Over the Top PT
//********************************************************************************************************************
//****************************PTZ Variables*************************************//
int Joy_Pan_Speed;                           //PTZ variables
int Joy_Pan_Accel;
int Joy_Tilt_Speed;
int Joy_Tilt_Accel = 1000;
int Joy_Focus_Speed;
int Joy_Focus_Accel;
int Joy_Zoo_Speed;
int Joy_Zoo_Accel;
int Focus_Stop;
int Zoo_Stop;

int Rec;                                      //Record request 0 -1
int lastRecState;                             //Last Record button state for camera
int PTZ_Cam = 0;                              //Current PTZ camera defaulted to 0
int PTS;                                      //Keeps track of the last selected PTZ camera
int PTZ_ID = 5;                               //PTZ Hardwae ID defaulted to 5 cannot be used by PTZ control
int lastPTZ_ID = PTZ_ID;                      //Stores the last known PTZ_ID
int PTZ_ID_Nex;                               //incomming ID request from nection control
int PTZ_Pose;                                 //Curent PTZ Pose key
int PTZ_SaveP;                                //PTZ save position comand true or false
int lastPTZ_Pose;                             //Store last PTZ_position for future save too
int usejoy;                                   //Joystick button state


int LM;                                       //Focus/Zoom limits set
int lastLM;
int limitSetend;
int cam1_F_In;                                //Cam1 Focus/zoom inpoint limit
int cam1_F_Out;                               //Cam1 Focus/zoom inpoint limit
int cam2_F_In;                                //Cam2 Focus/zoom inpoint limit
int cam2_F_Out;                               //Cam2 Focus/zoom inpoint limit
int cam3_F_In;                                //Cam3 Focus/zoom inpoint limit
int cam3_F_Out;                               //Cam3 Focus/zoom inpoint limit

int cam_F_In;                                //Actual value used dependant on ID
int cam_F_Out;                                //Actual value used dependant on ID
int cam_Z_In;                                //Actual value used dependant on ID
int cam_Z_Out;                               //Actual value used dependant on ID

int Z_Stop;                                   //Used to prevent the focus motor crashing into the end over and over
int B_Stop;                                   //Used to prevent the focus motor crashing into the end over and over
int T_position;                               //Holding variable for PTZ positions
int P_position;
int F_position;
int Z_position;

int stopM_play;                               //Stopmotion Trigger SM
int stopM_cancel;                             //Stopmmotion cancel SC
int SM;
int SC;
int stopM_frames;                             //The Total number of frames required for stopmotion
int SMC;                                    //stopmotion counter
int But;

int P1_T = 10;                        //PTZ Tilt positions
int P2_T = 10;
int P3_T = 10;
int P4_T = 10;
int P5_T = 10;
int P6_T = 10;
int P7_T = 10;
int P8_T = 10;
int P9_T = 10;
int P10_T = 10;
int P11_T = 10;
int P12_T = 10;
int P13_T = 10;
int P14_T = 10;
int P15_T = 10;
int P16_T = 10;

int P1_P = 10;                          //PTZ Pan positions
int P2_P = 10;
int P3_P = 10;
int P4_P = 10;
int P5_P = 10;
int P6_P = 10;
int P7_P = 10;
int P8_P = 10;
int P9_P = 10;
int P10_P = 10;
int P11_P = 10;
int P12_P = 10;
int P13_P = 10;
int P14_P = 10;
int P15_P = 10;
int P16_P = 10;


int P1_F = 10;                          //PTZ Focus positions
int P2_F = 10;
int P3_F = 10;
int P4_F = 10;
int P5_F = 10;
int P6_F = 10;
int P7_F = 10;
int P8_F = 10;
int P9_F = 10;
int P10_F = 10;
int P11_F = 10;
int P12_F = 10;
int P13_F = 10;
int P14_F = 10;
int P15_F = 10;
int P16_F = 10;

int P1_Z = 10;                          //PTZ Zoom positions
int P2_Z = 10;
int P3_Z = 10;
int P4_Z = 10;
int P5_Z = 10;
int P6_Z = 10;
int P7_Z = 10;
int P8_Z = 10;
int P9_Z = 10;
int P10_Z = 10;
int P11_Z = 10;
int P12_Z = 10;
int P13_Z = 10;
int P14_Z = 10;
int P15_Z = 10;
int P16_Z = 10;

long F_revolutions = 0;                         // number of revolutions the encoder has made
long Z_revolutions = 0;                         // number of revolutions the encoder has made
long P_revolutions = 0;                         // number of revolutions the encoder has made
long T_revolutions = 0;                         // number of revolutions the encoder has made

long F_E_position = 0;                        // the calculated value the encoder is at
long Z_E_position = 0;
long P_E_position = 0;
long T_E_position = 0;

long F_output;                                // raw value from AS5600
long Z_output;
long P_output;
long T_output;

long F_lastOutput;                              // last output from AS5600
long Z_lastOutput;
long P_lastOutput;
long T_lastOutput;



long F_E_outputPos = 0;                         // Ajustment value
long Z_E_outputPos = 0;
long P_E_outputPos = 0;
long T_E_outputPos = 0;


long F_E_lastOutput;                            // last output from AS5600
long Z_E_lastOutput;
long P_E_lastOutput;
long T_E_lastOutput;

long F_S_position = 0;                          // Number of stepper steps at 16 microsteps/step interpolated by the driver to 256 microsteps/step (confusing!)
long Z_S_position = 0;
long P_S_position = 0;
long T_S_position = 0;

long F_E_Trim = 0;                              // A Trim value for the encoder effectively setting its 0 position to match Step 0 position
long Z_E_Trim = 0;
long P_E_Trim = 0;
long T_E_Trim = 0;

long F_E_Current = 0;
long Z_E_Current = 0;
long P_E_Current = 0;
long T_E_Current = 0;


long F_E_Turn = 0;
long Z_E_Turn = 0;
long P_E_Turn = 0;
long T_E_Turn = 0;

long F_E_outputTurn = 0;
long Z_E_outputTurn = 0;
long P_E_outputTurn = 0;
long T_E_outputTurn = 0;

long F_E_outputHold = 32728;
long Z_E_outputHold = 32728;
long P_E_outputHold = 32728;
long T_E_outputHold = 32728;

long F_loopcount = 0;
long Z_loopcount = 0;
long P_loopcount = 0;
long T_loopcount = 0;

long  F_S_lastPosition = 0;
long  Z_S_lastPosition = 0;
long  P_S_lastPosition = 0;
long  T_S_lastPosition = 0;

float factor;
int Mount = 0;                                 //Is the Mount function active 1 true 0 false

long tilt;
long pan;

long TiltJoySpeed;
long PanJoySpeed;
long tilt_AVG;
long pan_AVG;


long PANin_position = 0;                      // variable to hold IN positions for slider
long TLTin_position = 0;
long FOCin_position = 0;
long ZMin_position = 0;

long PANout_position = 0;                     // variable to hold OUT position for slider
long TLTout_position = 0;
long FOCout_position = 0;
long ZMout_position = 0;

long PANtravel_dist = 0;                        //Distance to travel
long TLTtravel_dist = 0;
long FOCtravel_dist = 0;
long ZOOMtravel_dist = 0;

float PANstep_speed = 0;                        // default travel speed between IN and OUT points
float TLTstep_speed = 0;
float FOCstep_speed = 0;
float ZOOMstep_speed = 0;

int PANfps_step_dist = 0;
int TLTfps_step_dist = 0;
int FOCfps_step_dist = 0;
int Zoo_step_dist = 0;

int TLTease_Value = 0;                      // Variable to hold actual Ease value used for acceleration calculation
int PANease_Value = 0;
int FOCease_Value = 0;
int ZOOMease_Value = 0;

int  TLTTlps_step_dist;
int  PANTlps_step_dist;
int  FOCTlps_step_dist;
int  ZOOMTlps_step_dist;

long Max_Steps = 0;                         //Variable used to set the max number of steps for the given leangth of rail
long set_speed = 0;
long in_position = 0;                       // Not Used by Pan Tilt but required to keep everything the same incoming and outgoing
long out_position = 0;                      // Not Used by Pan Tilt but required to keep everything the same incoming and outgoing


int Crono_time = 10;
int crono_hours = 0;
int crono_minutes = 0;
int crono_seconds = 0;
int ElapsedTime;
int MoveTime;

int move_left = 0;                        // variable to detect move left on Nextion LCD
int move_right = 0;                       // variable to confirm move right on Nextion LCD
int start_cycle = 0;                      // variable to confirm start of travel between IN and OUT points
long Home_position = 0;                   // Variable to st home position
int ease_InOut = 0;                       // Variable to collect Ease_InOut from nextion screen value range 1-3
int LastMove;                             // Last key set
int ease_time = 0;                        // Extra time allowed for ease in out calculation
int move_finished = 1;                    // Used to check if move is completed
long initial_homing = 0;                  // Used to Home Stepper at startup
int nextion_Fnc = 0;
int Bounce = 0;
int Tps = 0;
int Tlps_countdown = 0;
int Tlps_step_dist = 0;
int TpsD = 2000;
int Buttonset = 0;
int timelapse_projected = 0;
int getEncoder = 0;
int InP = 0;
int OutP = 0;
int TiltPanComand = 0;
int playbuttoncounter = 1;
String dataTx;
String wifimessage;
int PanTiltPlay;
long PanTiltINpoint;
long PanTiltOUTpoint;

int Nextion_Fnc;
int Nextion_play;
int But_Com;
int TpsM;

int F_ResetEncoder = 1;
int Z_ResetEncoder = 1;
int P_ResetEncoder = 1;
int T_ResetEncoder = 1;

int incomingEz;
int incomingBo;
int incomingCro;
int incomingFnc;
int incomingTps;
int incomingTpsD;
int incomingPlay;
long incomingIn;
long incomingOut;
int incomingBut;
int incommingInP;
int incommingOutP;
int incommingTpsM;

//Sequencer
int k1A;                                           //Actual interpolated Accel values First Ramp
int k2A;
int k3A;
int k4A;
int k5A;
int k6A;

int k1B;                                        //Actual interpolated Accel values second ramp
int k2B;
int k3B;
int k4B;
int k5B;
int k6B;



int a1 = 3;                                          //Nextion values for Accel 1-3 one for eack key
int a2 = 3;
int a3 = 3;
int a4 = 3;
int a5 = 3;
int a6 = 3;

int P1;                                            //Nextion button state
int P2;
int P3;
int P4;
int P5;
int P6;

int s1 = 50;                                       //Nextion speed as a percentage
int s2 = 50;
int s3 = 50;
int s4 = 50;
int s5 = 50;
int s6 = 50;

int s1_speed;                                      //Actual interpolated speeds
int s2_speed;
int s3_speed;
int s4_speed;
int s5_speed;
int s6_speed;

int k1AH;
int k2AH;
int k3AH;
int k4AH;
int k5AH;
int k6AH;

int Tlt_k1_position;
int Tlt_k2_position;
int Tlt_k3_position;
int Tlt_k4_position;
int Tlt_k5_position;
int Tlt_k6_position;

int Pan_k1_position;
int Pan_k2_position;
int Pan_k3_position;
int Pan_k4_position;
int Pan_k5_position;
int Pan_k6_position;

int Foc_k1_position;
int Foc_k2_position;
int Foc_k3_position;
int Foc_k4_position;
int Foc_k5_position;
int Foc_k6_position;

int Zoo_k1_position;
int Zoo_k2_position;
int Zoo_k3_position;
int Zoo_k4_position;
int Zoo_k5_position;
int Zoo_k6_position;

int BounceReturn = 1;                               //position memory for bounce 'H' for hold
int s1H;
int s2H;
int s3H;
int s4H;
int s5H;
int s6H;

int a1H;                                         //Nextion values for Accel 1-3
int a2H;
int a3H;
int a4H;
int a5H;
int a6H;

int Tlt_k1_positionH;
int Tlt_k2_positionH;
int Tlt_k3_positionH;
int Tlt_k4_positionH;
int Tlt_k5_positionH;
int Tlt_k6_positionH;

int Pan_k1_positionH;
int Pan_k2_positionH;
int Pan_k3_positionH;
int Pan_k4_positionH;
int Pan_k5_positionH;
int Pan_k6_positionH;

int Foc_k1_positionH;
int Foc_k2_positionH;
int Foc_k3_positionH;
int Foc_k4_positionH;
int Foc_k5_positionH;
int Foc_k6_positionH;

int Zoo_k1_positionH;
int Zoo_k2_positionH;
int Zoo_k3_positionH;
int Zoo_k4_positionH;
int Zoo_k5_positionH;
int Zoo_k6_positionH;


int Stp_1_active;
int Stp_2_active;
int Stp_3_active;
int Stp_4_active;
int Stp_active = 1;

int BounceActive;
float Time2Ramp2 = 0;
int Seq_Time = 5;
int Key_Crono_time;
int Tlt_TravelDist;
int Pan_TravelDist;
int Foc_TravelDist;
int Zoo_TravelDist;

int Last_Tlt_accel;
int Last_Tlt_speed;
int Last_Pan_accel;
int Last_Pan_speed;
int Last_Foc_accel;
int Last_Foc_speed;
int Last_Zoo_accel;
int Last_Zoo_speed;

int Base_Tlt_accel = 0;
int Base_Pan_accel = 0;
int Base_Foc_accel = 0;
int Base_Zoo_accel = 0;

int k1_k2_Dir;
int k2_k3_Dir;
int k3_k4_Dir;
int k4_k5_Dir;
int k5_k6_Dir;

int Tlt_move1_Dest;
int Tlt_move2_Dest;
int Tlt_move3_Dest;
int Tlt_move4_Dest;
int Tlt_move5_Dest;

int Pan_move1_Dest;
int Pan_move2_Dest;
int Pan_move3_Dest;
int Pan_move4_Dest;
int Pan_move5_Dest;

int Foc_move1_Dest;
int Foc_move2_Dest;
int Foc_move3_Dest;
int Foc_move4_Dest;
int Foc_move5_Dest;

int Zoo_move1_Dest;
int Zoo_move2_Dest;
int Zoo_move3_Dest;
int Zoo_move4_Dest;
int Zoo_move5_Dest;


int Tlt_move2_Rst;
int Tlt_move3_Rst;
int Tlt_move4_Rst;
int Tlt_move5_Rst;

int Pan_move2_Rst;
int Pan_move3_Rst;
int Pan_move4_Rst;
int Pan_move5_Rst;

int Foc_move2_Rst;
int Foc_move3_Rst;
int Foc_move4_Rst;
int Foc_move5_Rst;

int Zoo_move2_Rst;
int Zoo_move3_Rst;
int Zoo_move4_Rst;
int Zoo_move5_Rst;



int Ramp_1_Dist;
//int Ramp_1_Time;
int Ramp_2_Dist;
//int Ramp_2_Time;

int KeyDist;
int KeyA;
int KeyB;

int Accel;
int PastTime;
int ActiveMove;
int SEQmin = 4000;                                   //Min slowest speed for sequencer larger number slower speed
int SEQmoves = 2;
int Tlt_Last_key;
int Pan_Last_key;
int Foc_Last_key;
int Zoo_Last_key;
int mess ;
int Sld;                                             //Is the Slider  present 1 or 0
int TT;                                              //Is the turntable present 1 or 0
int JB;                                              //Is the Jib present 1 or 0
int PT = 1;                                          //Is the Pan Tilt present 1 or 0
int SEQ = 0;


//*****************VISCA variables

int ViscaComand;
int number;
int IPR;
int IP1 = 192;
int IP2 = 168;
int IP3 = 0;
int IP4 = 130;
int IPGW = 1;        //IP gatway 4th position
long UDP = 1259;
int LastViscaComand;
int VPanSpeed;
int VTiltSpeed;
int VFocusSpeed;
int VZoomSpeed;
int VPoseNumber;
int VPoseSpeed = 22 ;
long PA = 0; //pan angle  = value=angle x 100
long TA = 0; //Tilt angle
long ZA = 0;  //Zoom angle
long FA = 0;  //Focus angle

String PanPPPPP = "05C26"; //Left100
String TiltTTTT = "1BA5"; //Up30
int TLY = 1;
int Tally;

const byte bufferSize = 32;
char serialBuffer[bufferSize];
byte bufferIndex = 0;
char EOL = '\n';
bool hasData = false;
const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data

String strs[4];
int StringCount = 0;
boolean newData = false;
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
//  pinMode(Hall_Pan, INPUT_PULLUP);
//  pinMode(Hall_Tilt, INPUT_PULLUP);

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
        Serial.print("moving");
      }

  position+=1000;


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
