#include <KerbalSimpit.h>
#include <ShiftIn.h>

bool joystickDebug = false;
bool debug = false;
int cameraSensitivity = 100;

//Shift Register Declarations

//SHIFT OUT A (7 registers for output)
const int SHIFT_OUT_A_DATA = 5;
const int SHIFT_OUT_A_CLOCK = 6;
const int SHIFT_OUT_A_LATCH = 7;
bool shiftOutA[56];
int outputA;

//SHIFT OUT B (4 registers for output)
const int SHIFT_OUT_B_DATA = 2;
const int SHIFT_OUT_B_CLOCK = 3;
const int SHIFT_OUT_B_LATCH = 4;
bool shiftOutB[32];
int outputB;

//SHIFT IN A (8 registers)
const int SHIFT_IN_A_ENABLE = 10;
const int SHIFT_IN_A_DATA = 8;
const int SHIFT_IN_A_CLOCK = 9;
const int SHIFT_IN_A_LATCH = 11;

bool sasButtons[10];
bool sasButtonOld[10];
bool sasButtonChange[10];
bool changeState = false;
bool sasToggleState;
bool rcsToggleState;

byte sasCurrentMode;

timewarpMessage tw_msg;

//LED Definitions
bool abortButtonLED = false;
bool stageButtonLED = false;
bool lightsLED = false;
bool gearLED = false;
bool brakesLED = false;
bool CAG01_LED = false;
bool CAG02_LED = false;

//Button state bools
bool stageArmed;
bool abortArmed;
bool camRot;

const long blinkDelay = 500; //delay blink by 500 milliseconds
long abortSavedTime = 0;
long stageSavedTime = 0;

/* Summary of ShiftIn Button Assignments

[0] SAS Stability Assist
[1] SAS Maneuver
[2] SAS Prograde
[3] SAS Retrograde
[4] SAS Normal
[5] SAS Anti-Normal
[6] SAS Radial In
[7] SAS Radial Out
[8] SAS Target
[9] SAS Anti-Target
[10] Fuel Mode = Overall Total (TRUE) or Stage Total (FALSE)
[11] -
[12] Escape key for Pause
[13] F5 key for Quick Save
[14] F9 key for Quick Load
[15] Alt+F4 for Quit
[16] Abort Armed
[17] Abort Button
[18] Stage Armed
[19] Stage Button
[20] SAS Toggle
[21] RCS Toggle
[22] Rocket Mode vs Plane Mode (Plane Mode = HIGH ; Rocket Mode = LOW)
[23] Camera Rotation
[24] Timewarp Stop
[25] Timewarp to Next Burn (-10 seconds)
[26] Timewarp Down
[27] Timewarp Up
[28] Map
[29] View
[30] (Empty)
[31] (Empty)
[32] -
[33] -
[34] -
[35] -
[36] -
[37] -
[38] -
[39] -
[40] -
[41] -
[42] -
[43] -
[44] -
[45] -
[46] -
[47] -
[48] Lights
[49] Gear
[50] Brakes
[51] -
[52] Solar (CAG02)
[53] AG3
[54] AG4
[55] AG5
[56] AG6
[57] AG7
[58] AG8
[59] AG9
[60] AG10
[61] AG11
[62] AG12
[63] Ladder (CAG01)

*/
ShiftIn<8> shiftInA;

//0 : 0%
//1 : 25%
//2 : 50%
//3 : 75%
//4 : 100%
bool throtLED[5];

//Initialize all fuel percentages and LEDs

float sfPercent = 100;
float sfTotalOG = 0;
//0 : Empty
//1 : 10%
//2 : 20%
//3 : 30%
//4 : 40%
//5 : 50%
//6 : 60%
//7 : 70%
//8 : 80%
//9 : 90%
//10 : 100%
bool sfLEDS[11];

float lfPercent = 100;
float lfTotalOG = 0;
//0 : Empty
//1 : 10%
//2 : 20%
//3 : 30%
//4 : 40%
//5 : 50%
//6 : 60%
//7 : 70%
//8 : 80%
//9 : 90%
//10 : 100%
bool lfLEDS[11];

float oxPercent = 100;
float oxTotalOG = 0;
//0 : Empty
//1 : 10%
//2 : 20%
//3 : 30%
//4 : 40%
//5 : 50%
//6 : 60%
//7 : 70%
//8 : 80%
//9 : 90%
//10 : 100%
bool oxLEDS[11];

float mpPercent = 100;
float mpTotalOG = 0;
//0 : Empty
//1 : 10%
//2 : 20%
//3 : 30%
//4 : 40%
//5 : 50%
//6 : 60%
//7 : 70%
//8 : 80%
//9 : 90%
//10 : 100%
bool mpLEDS[11];

float ecPercent = 100;
float ecTotalOG = 0;
//0 : Empty
//1 : 10%
//2 : 20%
//3 : 30%
//4 : 40%
//5 : 50%
//6 : 60%
//7 : 70%
//8 : 80%
//9 : 90%
//10 : 100%
bool ecLEDS[11];


// 0 = Stability Assist
// 1 = Maneuver
// 2 = Prograde
// 3 = Retrograde
// 4 = Normal
// 5 = Anti-Normal
// 6 = Radial Out
// 7 = Radial In
// 8 = Target
// 9 = Anti-Target
bool sasModeLEDS[10];


bool rocketMode;
bool planeMode;
const int throttle = A0; //throttle
const int rX = A1; //rotation x-axis
const int rY = A2; //rotation y-axis
const int rZ = A3; //rotation z-axis
const int tX = A4; //translation x-axis
const int tY = A5; //translation y-axis
const int tZ = A6; //translation z-axis
int deadzoneCenter_rX = 518;
int deadzoneCenter_rY = 540;
int deadzoneCenter_rZ = 511;
int deadzoneCenter_tX = 501;
int deadzoneCenter_tY = 538;
int deadzoneCenter_tZ = 511;
int deadzone = 25;
int16_t pitch = 0; //y
int16_t roll = 0; //z
int16_t yaw = 0; //x
int16_t throt = 0; //throttle

//Store the current action status, as recevied by simpit.
byte currentActionStatus = 0;
byte myLF_TOT = 0;
byte myLF_STAGE = 0;


// Declare a KerbalSimpit object that will communicate using the "Serial" device.
KerbalSimpit mySimpit(Serial);

float LF_TOT_T = 0.0;
float LF_TOT_A = 0.0;
float percentLF_TOT = 0.0;
float LF_STAGE_T = 0.0;
float LF_STAGE_A = 0.0;
float percentLF_STAGE = 0.0;
bool TOTvSTAGE = 0;

bool totalFuel = 0;
bool stageFuel = 0;

void setup() {
  // Open the serial connection.
  Serial.begin(115200);
  shiftInA.begin(SHIFT_IN_A_LATCH,SHIFT_IN_A_ENABLE,SHIFT_IN_A_DATA,SHIFT_IN_A_CLOCK);
  // StageButton.setDebounceTime(debounceTime);

  // Set up the build in LED, and turn it on.
  pinMode(LED_BUILTIN, OUTPUT);
  //pinMode(ProgradePin, OUTPUT);
  pinMode(throttle,INPUT); //rotation x-axis
  pinMode(rX,INPUT); //rotation x-axis
  pinMode(rY,INPUT); //rotation y-axis
  pinMode(rZ,INPUT); //rotation z-axis
  pinMode(tX,INPUT); //translation x-axis
  pinMode(tY,INPUT); //translation y-axis
  pinMode(tZ,INPUT); //translation z-axis
  digitalWrite(LED_BUILTIN, HIGH);

  //Shift register pins
  pinMode(SHIFT_OUT_A_DATA, OUTPUT);
  pinMode(SHIFT_OUT_A_CLOCK, OUTPUT);
  pinMode(SHIFT_OUT_A_LATCH, OUTPUT);

  pinMode(SHIFT_OUT_B_DATA, OUTPUT);
  pinMode(SHIFT_OUT_B_CLOCK, OUTPUT);
  pinMode(SHIFT_OUT_B_LATCH, OUTPUT);

if(!joystickDebug && !debug){
  while (!mySimpit.init()) {
   delay(100);
  }
}
  // Turn off the built-in LED to indicate handshaking is complete.
  digitalWrite(LED_BUILTIN, LOW);
  // Display a message in KSP to indicate handshaking is complete.
  mySimpit.printToKSP("Connected", PRINT_TO_SCREEN);
  // Sets our callback function. The KerbalSimpit library will
  // call this function every time a packet is received.
  mySimpit.inboundHandler(messageHandler);
  // Send a message to the plugin registering for the Action status channel.
  // The plugin will now regularly send Action status  messages while the
  // flight scene is active in-game.
  mySimpit.registerChannel(ACTIONSTATUS_MESSAGE);
  mySimpit.registerChannel(SAS_MODE_INFO_MESSAGE);
  mySimpit.registerChannel(LF_MESSAGE);
  mySimpit.registerChannel(LF_STAGE_MESSAGE);
  mySimpit.registerChannel(SF_MESSAGE);
  mySimpit.registerChannel(SF_STAGE_MESSAGE);
  mySimpit.registerChannel(OX_MESSAGE);
  mySimpit.registerChannel(OX_STAGE_MESSAGE);
  mySimpit.registerChannel(MONO_MESSAGE);
  mySimpit.registerChannel(ELECTRIC_MESSAGE);
  mySimpit.registerChannel(ROTATION_MESSAGE);
  mySimpit.registerChannel(TRANSLATION_MESSAGE);
  mySimpit.registerChannel(THROTTLE_MESSAGE);
  mySimpit.registerChannel(SAS_MODE_MESSAGE);
  mySimpit.registerChannel(TIMEWARP_MESSAGE);
  mySimpit.registerChannel(CAMERA_CONTROL_MODE);
  mySimpit.registerChannel(CAMERA_ROTATION_MESSAGE);
  mySimpit.registerChannel(CAGSTATUS_MESSAGE);

//Initialize bools for various LED states
  shiftInA.getCurrent();
  totalFuel = shiftInA.state(10);
  stageFuel = !shiftInA.state(10);
  abortArmed = shiftInA.state(16);
  stageArmed = shiftInA.state(18);
  sasToggleState = shiftInA.state(20);
  rcsToggleState = shiftInA.state(21);
  planeMode = shiftInA.state(22);
  rocketMode = !shiftInA.state(22);
  camRot = shiftInA.state(23);
  lightsLED = shiftInA.state(48);
  gearLED = shiftInA.state(49);
  brakesLED = shiftInA.state(50);
  CAG01_LED = shiftInA.state(63);
  CAG02_LED = shiftInA.state(52);


  

}

void loop() {

  // Check for new serial messages.
  mySimpit.update();

analogReference(EXTERNAL);
int throtRAW = analogRead(throttle);


int analogInput = analogRead(rX);
analogInput = analogRead(rX); 

if(rocketMode){

  if((deadzoneCenter_rX - deadzone) < analogInput && analogInput < (deadzoneCenter_rX + deadzone)){
    yaw = 0;
  } else {
    yaw = constrain(map(analogInput,0,1023,INT16_MIN,INT16_MAX),INT16_MIN,INT16_MAX); //x
  }

} else if(planeMode){

  if((deadzoneCenter_rX - deadzone) < analogInput && analogInput < (deadzoneCenter_rX + deadzone)){
    roll = 0;
  } else {
    roll = constrain(map(analogInput,0,1023,INT16_MIN,INT16_MAX),INT16_MIN,INT16_MAX); //x
  }


}


analogInput = analogRead(rY);
analogInput = analogRead(rY);

if(rocketMode){
  if((deadzoneCenter_rY - deadzone) < analogInput && analogInput < (deadzoneCenter_rY + deadzone)){
    pitch = 0;
  } else {
    pitch = constrain(map(analogInput,0,1023,INT16_MIN,INT16_MAX),INT16_MIN,INT16_MAX); //y
  }
} else if(planeMode){
  if((deadzoneCenter_rY - deadzone) < analogInput && analogInput < (deadzoneCenter_rY + deadzone)){
    pitch = 0;
  } else {
    pitch = constrain(map(analogInput,0,1023,INT16_MIN+1,INT16_MAX-1),INT16_MIN,INT16_MAX); //y
    pitch *= -1; //invert
  }
}

analogInput = analogRead(rZ);
analogInput = analogRead(rZ);

if(rocketMode){

  if((deadzoneCenter_rZ - deadzone) < analogInput && analogInput < (deadzoneCenter_rZ + deadzone)){
    roll = 0;
  } else {
    roll = constrain(map(analogInput,0,1023,INT16_MIN,INT16_MAX),INT16_MIN,INT16_MAX);; //z
  }

} else if(planeMode){

  if((deadzoneCenter_rZ - deadzone) < analogInput && analogInput < (deadzoneCenter_rZ + deadzone)){
    yaw = 0;
  } else {
    yaw = constrain(map(analogInput,0,1023,INT16_MIN,INT16_MAX),INT16_MIN,INT16_MAX);; //z
  }

}

  if(!joystickDebug){
    if(camRot){
        cameraRotationMessage myCamRot;
        myCamRot.mask = 13; //send pitch, (not roll), yaw, and zoom all at once

        //Optional roll control (using left joystick for zoom)
        // myCamRot.mask = 7;

        myCamRot.cameraPitch = pitch/cameraSensitivity; //y
        if(rocketMode){
          myCamRot.cameraYaw = yaw/cameraSensitivity; //x if rocket, z if plane

          //Optional roll control (using left joystick for zoom)
          // myCamRot.cameraRoll = roll;

          myCamRot.cameraZoom = -roll/cameraSensitivity;

        } else if(planeMode){
          myCamRot.cameraYaw = roll/cameraSensitivity;

          //Optional roll control (using left joystick for zoom)
          // myCamRot.cameraRoll = yaw;

          myCamRot.cameraZoom = -yaw/cameraSensitivity;

        }
      
      mySimpit.send(CAMERA_ROTATION_MESSAGE, myCamRot);

    } else {
      rotationMessage myRot;
      myRot.mask = 7; //send pitch, yaw, and roll all at once

      myRot.pitch = pitch; //y
      myRot.yaw = yaw; //x if rocket, z if plane
      myRot.roll = roll; //z if rocket, x if plane
      mySimpit.send(ROTATION_MESSAGE, myRot);
    }
  
  }


if(rocketMode && !planeMode){

  analogInput = analogRead(tX);
  analogInput = analogRead(tX);
  int translationX = 0;
    if((deadzoneCenter_tX - deadzone) < analogInput && analogInput < (deadzoneCenter_tX + deadzone)){
      translationX = 0;
    } else {
      translationX = constrain(map(analogInput,0,1023,INT16_MIN,INT16_MAX),INT16_MIN,INT16_MAX); //x
    }

  analogInput = analogRead(tY);
  analogInput = analogRead(tY);
  int translationY = 0;
    if((deadzoneCenter_tY - deadzone) < analogInput && analogInput < (deadzoneCenter_tY + deadzone)){
      translationY = 0;
    } else {
      translationY = constrain(map(analogInput,0,1023,INT16_MIN,INT16_MAX),INT16_MIN,INT16_MAX); //y
    }

  analogInput = analogRead(tZ);
  analogInput = analogRead(tZ);
  int translationZ = 0;
    if((deadzoneCenter_tZ - deadzone) < analogInput && analogInput < (deadzoneCenter_tZ + deadzone)){
      translationZ = 0;
    } else {
      translationZ = constrain(map(analogInput,0,1023,INT16_MIN,INT16_MAX),INT16_MIN,INT16_MAX);; //z
    }

      translationMessage translation_msg;
      translation_msg.setX(translationX);
      translation_msg.setY(translationY);
      translation_msg.setZ(translationZ);
      
      if(!joystickDebug){
        if(camRot){
          //Optional zoom control using left joystick

          // cameraRotationMessage myCamRot;
          // myCamRot.mask = 8;
          // myCamRot.cameraZoom = translationZ/cameraSensitivity;
          // mySimpit.send(CAMERA_ROTATION_MESSAGE,myCamRot);
        } else {
          mySimpit.send(TRANSLATION_MESSAGE, translation_msg);
        }
      
      }
}

if(joystickDebug){
  Serial.print("tX =");
  analogRead(tX);
    Serial.println(String(analogRead(tX)));
    delay(50);
  Serial.print("tY ="); 
  analogRead(tY);
    Serial.println(String(analogRead(tY)));
        delay(50);
  Serial.print("tZ ="); 
  analogRead(tZ);
  Serial.println(String(analogRead(tZ)));
      delay(50);
    Serial.print("rX =");
    analogRead(rX);
    Serial.println(String(analogRead(rX)));
        delay(50);
  Serial.print("rY ="); 
  analogRead(rY);
    Serial.println(String(analogRead(rY)));
        delay(50);
  Serial.print("rZ ="); 
  analogRead(rZ);
  Serial.println(String(analogRead(rZ)));
      delay(50);
  delay(1500);
}

  if(throtRAW < 75){
    throtRAW = 0;
  }

  throt = map(throtRAW,0,1023,0,INT16_MAX);

  throttleMessage throttleMsg;

  throttleMsg.throttle = throt;

  mySimpit.send(THROTTLE_MESSAGE, throttleMsg);

throtLED[0] = 1;
if (throtRAW > 255){throtLED[1] = 1;}else{throtLED[1] = 0;}
if (throtRAW > 512){throtLED[2] = 1;}else{throtLED[2] = 0;}
if (throtRAW > 765){throtLED[3] = 1;}else{throtLED[3] = 0;}
if (throtRAW > 1020){throtLED[4] = 1;}else{throtLED[4] = 0;}

if(shiftInA.update()){
  
  if(shiftInA.pressed(0)){
    mySimpit.setSASMode(AP_STABILITYASSIST);
    // sasButtons[0] = 0;
  }

  if(shiftInA.pressed(1)){
    mySimpit.setSASMode(AP_MANEUVER);
    // sasButtons[1] = 0;
  }

  if(shiftInA.pressed(2)){
    mySimpit.setSASMode(AP_PROGRADE);
    // sasButtons[2] = 0;
  }

  if(shiftInA.pressed(3)){
    mySimpit.setSASMode(AP_RETROGRADE);
    // sasButtons[3] = 0;
  }

  if(shiftInA.pressed(4)){
    mySimpit.setSASMode(AP_NORMAL);
    // sasButtons[4] = 0;
  }

  if(shiftInA.pressed(5)){
    mySimpit.setSASMode(AP_ANTINORMAL);
    // sasButtons[5] = 0;
  }

  if(shiftInA.pressed(6)){
    mySimpit.setSASMode(AP_RADIALIN);
    // sasButtons[6] = 0;
  }

  if(shiftInA.pressed(7)){
    mySimpit.setSASMode(AP_RADIALOUT);
    // sasButtons[7] = 0;
  }

  if(shiftInA.pressed(8)){
    mySimpit.setSASMode(AP_TARGET);
    // sasButtons[8] = 0;
  }

  if(shiftInA.pressed(9)){
    mySimpit.setSASMode(AP_ANTITARGET);
    // sasButtons[9] = 0;
  }

  if(shiftInA.state(10) && shiftInA.pressed(10)){
    totalFuel = true;
    stageFuel = false;
  }

  if(shiftInA.released(10) && !shiftInA.state(10)){
    totalFuel = false;
    stageFuel = true;
  }


  //Pin 11

  if(shiftInA.pressed(12)){
    keyboardEmulatorMessage keyMsg(0x1B);
    mySimpit.send(KEYBOARD_EMULATOR,keyMsg);
    // Pause ESC = 0x1B
  }

  if(shiftInA.pressed(13)){
    keyboardEmulatorMessage keyMsg(0x74);
    mySimpit.send(KEYBOARD_EMULATOR,keyMsg);
    // Save F5 = 0x74
  }

  if(shiftInA.pressed(14)){
    keyboardEmulatorMessage keyMsg(0x78);
    keyMsg.modifier = KEY_DOWN_MOD;
    mySimpit.send(KEYBOARD_EMULATOR,keyMsg);
    delay(1000); //press and hold for 1 second
    keyMsg.modifier = KEY_UP_MOD;
    mySimpit.send(KEYBOARD_EMULATOR,keyMsg);
    // Load F9 = 0x78
  }

  if(shiftInA.pressed(15)){
    keyboardEmulatorMessage keyMsg(0x73,ALT_MOD);
    mySimpit.send(KEYBOARD_EMULATOR,keyMsg);
    // Quit ALT+F4 = 0x73
  }

  if(shiftInA.state(16) && shiftInA.pressed(16)){
    abortArmed = true;
    abortButtonLED = true;
    abortSavedTime = millis();
  }

  if(shiftInA.released(16) && !shiftInA.state(16)){
    abortArmed = false;
    abortButtonLED = false;
  }

  if(shiftInA.pressed(17) && abortArmed){
    mySimpit.activateAction(ABORT_ACTION);
  }

  if(shiftInA.state(18) && shiftInA.pressed(18)){
    stageArmed = true;
    stageButtonLED = true;
    stageSavedTime = millis();
  }

  if(shiftInA.released(18) && !shiftInA.state(18)){
    stageArmed = false;
    stageButtonLED = false;
  }

  if(shiftInA.pressed(19) && stageArmed){
    mySimpit.activateAction(STAGE_ACTION);
  }

  if(shiftInA.state(20) && shiftInA.pressed(20)){
    sasToggleState = true;
    mySimpit.activateAction(SAS_ACTION);
  }

  if(shiftInA.released(20) && !shiftInA.state(20)){
    sasToggleState = false;
    mySimpit.deactivateAction(SAS_ACTION);
  }  

  if(shiftInA.state(21) && shiftInA.pressed(21)){
    rcsToggleState = true;
    mySimpit.activateAction(RCS_ACTION);
  }

  if(shiftInA.released(21) && !shiftInA.state(21)){
    rcsToggleState = false;
    mySimpit.deactivateAction(RCS_ACTION);
  }  

  if(shiftInA.state(22) && shiftInA.pressed(22)){
    planeMode = true;
    rocketMode = false;
  }

  if(shiftInA.released(22) && !shiftInA.state(22)){
    rocketMode = true;
    planeMode = false;
  }

  if(shiftInA.state(23) && shiftInA.pressed(23)){
    camRot = true;
  }

  if(shiftInA.released(23) && !shiftInA.state(23)){
    camRot = false;
  }

  if(shiftInA.pressed(24)){
    tw_msg.command = TIMEWARP_CANCEL_AUTOWARP;
    mySimpit.send(TIMEWARP_MESSAGE,tw_msg);
  } 

  if(shiftInA.pressed(25)){
    timewarpToMessage twTo_msg(TIMEWARP_TO_NEXT_BURN,-10);
    mySimpit.send(TIMEWARP_MESSAGE,twTo_msg);
  } 

  if(shiftInA.pressed(26)){
    tw_msg.command = TIMEWARP_DOWN;
    mySimpit.send(TIMEWARP_MESSAGE,tw_msg);
  }

  if(shiftInA.pressed(27)){
    tw_msg.command = TIMEWARP_UP;
    mySimpit.send(TIMEWARP_MESSAGE,tw_msg);
  }

  if(shiftInA.pressed(28)){
    keyboardEmulatorMessage keyMsg(0x4D);
    mySimpit.send(KEYBOARD_EMULATOR,keyMsg);
    // M = 0x4D
  } 

  if(shiftInA.pressed(29)){
    keyboardEmulatorMessage keyMsg(0x56);
    mySimpit.send(KEYBOARD_EMULATOR,keyMsg);
    // V = 0x56
  } 

  if(shiftInA.pressed(30)){
    //keyboardEmulatorMessage keyMsg(0x56);
    //mySimpit.send(KEYBOARD_EMULATOR,keyMsg);
    // V = 0x56
  } 

  if(shiftInA.pressed(31)){
    //keyboardEmulatorMessage keyMsg(0x56);
    //mySimpit.send(KEYBOARD_EMULATOR,keyMsg);
    // V = 0x56
  } 

  if(shiftInA.state(48) && shiftInA.pressed(48)){
    lightsLED = true;
    mySimpit.activateAction(LIGHT_ACTION);
  }

  if(shiftInA.released(48) && !shiftInA.state(48)){
    lightsLED = false;
    mySimpit.deactivateAction(LIGHT_ACTION);
  }  

  if(shiftInA.state(49) && shiftInA.pressed(49)){
    gearLED = true;
    mySimpit.activateAction(GEAR_ACTION);
  }

  if(shiftInA.released(49) && !shiftInA.state(49)){
    gearLED = false;
    mySimpit.deactivateAction(GEAR_ACTION);
  }  

  if(shiftInA.state(50) && shiftInA.pressed(50)){
    brakesLED = true;
    mySimpit.activateAction(BRAKES_ACTION);
  }

  if(shiftInA.released(50) && !shiftInA.state(50)){
    brakesLED = false;
    mySimpit.deactivateAction(BRAKES_ACTION);
  }  

  if(shiftInA.state(63) && shiftInA.pressed(63)){
    CAG01_LED = true;
    mySimpit.activateCAG(1);
  }

  if(shiftInA.released(63) && !shiftInA.state(63)){
    CAG01_LED = false;
    mySimpit.deactivateCAG(1);
  }  

  if(shiftInA.state(52) && shiftInA.pressed(52)){
    CAG02_LED = true;
    mySimpit.activateCAG(2);
  }

  if(shiftInA.released(52) && !shiftInA.state(52)){
    CAG02_LED = false;
    mySimpit.deactivateCAG(2);
  }  

  if(shiftInA.pressed(53)){
    mySimpit.toggleCAG(3);
  }  

  if(shiftInA.pressed(54)){
    mySimpit.toggleCAG(4);
  } 

  if(shiftInA.pressed(55)){
    mySimpit.toggleCAG(5);
  } 

  if(shiftInA.pressed(56)){
    mySimpit.toggleCAG(6);
  } 

  if(shiftInA.pressed(57)){
    mySimpit.toggleCAG(7);
  } 

  if(shiftInA.pressed(58)){
    mySimpit.toggleCAG(8);
  } 

  if(shiftInA.pressed(59)){
    mySimpit.toggleCAG(9);
  } 

  if(shiftInA.pressed(60)){
    mySimpit.toggleCAG(10);
  } 

  if(shiftInA.pressed(61)){
    mySimpit.toggleCAG(11);
  } 

  if(shiftInA.pressed(62)){
    mySimpit.toggleCAG(12);
  }   

}

//Stage & Abort LED Blink
if(stageArmed && millis() - stageSavedTime >= blinkDelay){
  stageButtonLED = !stageButtonLED;
  stageSavedTime = millis();
}
if(abortArmed && millis() - abortSavedTime >= blinkDelay){
  abortButtonLED = !abortButtonLED;
  abortSavedTime = millis();
}

setOutputValues();
sendShiftOut(shiftOutA,SHIFT_OUT_A_DATA,SHIFT_OUT_A_LATCH,SHIFT_OUT_A_CLOCK);
sendShiftOut(shiftOutB,SHIFT_OUT_B_DATA,SHIFT_OUT_B_LATCH,SHIFT_OUT_B_CLOCK);    // sendShiftOut(testArray,SHIFT_OUT_A_DATA,SHIFT_OUT_A_LATCH,SHIFT_OUT_A_CLOCK);

}

void messageHandler(byte messageType, byte msg[], byte msgSize) {
  switch(messageType) {
  case ACTIONSTATUS_MESSAGE:
    // Checking if the message is the size we expect is a very basic
    // way to confirm if the message was received properly.
    if (msgSize == 1) {
      currentActionStatus = msg[0];

      //Let the LED_BUILIN match the current SAS state
      if(currentActionStatus & SAS_ACTION){
        digitalWrite(LED_BUILTIN, HIGH);
      } else {
        digitalWrite(LED_BUILTIN, LOW);
      }
    }
    break;
  case SAS_MODE_INFO_MESSAGE:
 
    if(msgSize == sizeof(SASInfoMessage)){
      
      SASInfoMessage sasMode;
      sasMode = parseMessage<SASInfoMessage>(msg);
      sasCurrentMode = sasMode.currentSASMode;

      sasModeLEDS[0] = (sasCurrentMode == AP_STABILITYASSIST);
      sasModeLEDS[1] = (sasCurrentMode == AP_MANEUVER);
      sasModeLEDS[2] = (sasCurrentMode == AP_PROGRADE);
      sasModeLEDS[3] = (sasCurrentMode == AP_RETROGRADE);
      sasModeLEDS[4] = (sasCurrentMode == AP_NORMAL);
      sasModeLEDS[5] = (sasCurrentMode == AP_ANTINORMAL);
      sasModeLEDS[6] = (sasCurrentMode == AP_RADIALIN);
      sasModeLEDS[7] = (sasCurrentMode == AP_RADIALOUT);
      sasModeLEDS[8] = (sasCurrentMode == AP_TARGET);
      sasModeLEDS[9] = (sasCurrentMode == AP_ANTITARGET);

    }
     
  break;
  
  case SF_MESSAGE:
        if (msgSize == sizeof(resourceMessage))
          {
            resourceMessage sf;
            sf = parseMessage<resourceMessage>(msg);

            if(sfTotalOG == 0){
              sfTotalOG = sf.total;
            }

            if(totalFuel == 1){
              sfPercent = 100 * sf.available / sfTotalOG;

            sfLEDS[10] = (sfPercent > 90.0); //blue 100% LED
            sfLEDS[9] = (sfPercent > 80.0);
            sfLEDS[8] = (sfPercent > 70.0);
            sfLEDS[7] = (sfPercent > 60.0);
            sfLEDS[6] = (sfPercent > 50.0);
            sfLEDS[5] = (sfPercent > 40.0);
            sfLEDS[4] = (sfPercent > 30.0);
            sfLEDS[3] = (sfPercent > 20.0);
            sfLEDS[2] = (sfPercent > 10.0);
            sfLEDS[1] = (sfPercent > 0.0); //last red LED
            sfLEDS[0] = ((sfTotalOG == 0) || (sfPercent < 0.000001)); //extra "empty" LED


            } else {
              break;
            }
            
            
            
              
        

          }
        break;

    case SF_STAGE_MESSAGE:
        if (msgSize == sizeof(resourceMessage))
        {
            resourceMessage sfStage;
            sfStage = parseMessage<resourceMessage>(msg);

            if(stageFuel == 1){
              sfPercent = 100 * sfStage.available / sfStage.total;

            sfLEDS[10] = (sfPercent > 90.0); //blue 100% LED
            sfLEDS[9] = (sfPercent > 80.0);
            sfLEDS[8] = (sfPercent > 70.0);
            sfLEDS[7] = (sfPercent > 60.0);
            sfLEDS[6] = (sfPercent > 50.0);
            sfLEDS[5] = (sfPercent > 40.0);
            sfLEDS[4] = (sfPercent > 30.0);
            sfLEDS[3] = (sfPercent > 20.0);
            sfLEDS[2] = (sfPercent > 10.0);
            sfLEDS[1] = (sfPercent > 0.0); //last red LED
            sfLEDS[0] = ((sfStage.total == 0) || (sfPercent < 0.000001)); //extra "empty" LED


            } else {
              break;
            }

            



        }
        break;


  case LF_MESSAGE:
            if (msgSize == sizeof(resourceMessage))
        {

           


            resourceMessage lf;
            lf = parseMessage<resourceMessage>(msg);

            if(lfTotalOG == 0){
              lfTotalOG = lf.total;
            }

            if(totalFuel == 1){
              lfPercent = 100 * lf.available / lfTotalOG;

                lfLEDS[10] = (lfPercent > 90.0); //blue 100% LED
                lfLEDS[9] = (lfPercent > 80.0);
                lfLEDS[8] = (lfPercent > 70.0);
                lfLEDS[7] = (lfPercent > 60.0);
                lfLEDS[6] = (lfPercent > 50.0);
                lfLEDS[5] = (lfPercent > 40.0);
                lfLEDS[4] = (lfPercent > 30.0);
                lfLEDS[3] = (lfPercent > 20.0);
                lfLEDS[2] = (lfPercent > 10.0);
                lfLEDS[1] = (lfPercent > 0.0); //last red LED
                lfLEDS[0] = ((lfTotalOG == 0) || (lfPercent < 0.000001)); //extra "empty" LED


            } else {
              break;
            }

            
            
              
        }

  break;
case LF_STAGE_MESSAGE:
      if (msgSize == sizeof(resourceMessage))
        {
            resourceMessage lfStage;
            lfStage = parseMessage<resourceMessage>(msg);

            if(stageFuel == 1){
              lfPercent = 100 * lfStage.available / lfStage.total;

            lfLEDS[10] = (lfPercent > 90.0); //blue 100% LED
            lfLEDS[9] = (lfPercent > 80.0);
            lfLEDS[8] = (lfPercent > 70.0);
            lfLEDS[7] = (lfPercent > 60.0);
            lfLEDS[6] = (lfPercent > 50.0);
            lfLEDS[5] = (lfPercent > 40.0);
            lfLEDS[4] = (lfPercent > 30.0);
            lfLEDS[3] = (lfPercent > 20.0);
            lfLEDS[2] = (lfPercent > 10.0);
            lfLEDS[1] = (lfPercent > 0.0); //last red LED
            lfLEDS[0] = ((lfStage.total == 0) || (lfPercent < 0.000001)); //extra "empty" LED

            } else {
              break;
            }

            



        }

  break;
case OX_MESSAGE:
  if (msgSize == sizeof(resourceMessage))
          {
            resourceMessage ox;
            ox = parseMessage<resourceMessage>(msg);

            if(oxTotalOG == 0){
              oxTotalOG = ox.total;
            }

            if(totalFuel == 1){
              oxPercent = 100 * ox.available / oxTotalOG;

            oxLEDS[10] = (oxPercent > 90.0); //blue 100% LED
            oxLEDS[9] = (oxPercent > 80.0);
            oxLEDS[8] = (oxPercent > 70.0);
            oxLEDS[7] = (oxPercent > 60.0);
            oxLEDS[6] = (oxPercent > 50.0);
            oxLEDS[5] = (oxPercent > 40.0);
            oxLEDS[4] = (oxPercent > 30.0);
            oxLEDS[3] = (oxPercent > 20.0);
            oxLEDS[2] = (oxPercent > 10.0);
            oxLEDS[1] = (oxPercent > 0.0); //last red LED
            oxLEDS[0] = ((oxTotalOG == 0) || (oxPercent < 0.000001)); //extra "empty" LED


            } else {
              break;
            }


            
            
            
              
        

          }
break;

case OX_STAGE_MESSAGE:

        if (msgSize == sizeof(resourceMessage))
        {
            resourceMessage oxStage;
            oxStage = parseMessage<resourceMessage>(msg);

            if(stageFuel == 1){
              oxPercent = 100 * oxStage.available / oxStage.total;

            oxLEDS[10] = (oxPercent > 90.0); //blue 100% LED
            oxLEDS[9] = (oxPercent > 80.0);
            oxLEDS[8] = (oxPercent > 70.0);
            oxLEDS[7] = (oxPercent > 60.0);
            oxLEDS[6] = (oxPercent > 50.0);
            oxLEDS[5] = (oxPercent > 40.0);
            oxLEDS[4] = (oxPercent > 30.0);
            oxLEDS[3] = (oxPercent > 20.0);
            oxLEDS[2] = (oxPercent > 10.0);
            oxLEDS[1] = (oxPercent > 0.0); //last red LED
            oxLEDS[0] = ((oxStage.total == 0) || (oxPercent < 0.000001)); //extra "empty" LED
              

            } else {
              break;
            }

            



        }
break;

case MONO_MESSAGE:
  if (msgSize == sizeof(resourceMessage))
          {
            resourceMessage mp;
            mp = parseMessage<resourceMessage>(msg);

            if(mpTotalOG == 0){
              mpTotalOG = mp.total;
            }


            mpPercent = 100 * mp.available / mpTotalOG;
            
            mpLEDS[10] = (mpPercent > 90.0); //blue 100% LED
            mpLEDS[9] = (mpPercent > 80.0);
            mpLEDS[8] = (mpPercent > 70.0);
            mpLEDS[7] = (mpPercent > 60.0);
            mpLEDS[6] = (mpPercent > 50.0);
            mpLEDS[5] = (mpPercent > 40.0);
            mpLEDS[4] = (mpPercent > 30.0);
            mpLEDS[3] = (mpPercent > 20.0);
            mpLEDS[2] = (mpPercent > 10.0);
            mpLEDS[1] = (mpPercent > 0.0); //last red LED
            mpLEDS[0] = ((mpTotalOG == 0) || (mpPercent < 0.000001)); //extra "empty" LED
              
        

          }
break;

case ELECTRIC_MESSAGE:
  if (msgSize == sizeof(resourceMessage))
          {
            resourceMessage ec;
            ec = parseMessage<resourceMessage>(msg);

            if(ecTotalOG == 0){
              ecTotalOG = ec.total;
            }


            ecPercent = 100 * ec.available / ecTotalOG;
            
            ecLEDS[10] = (ecPercent > 90.0); //blue 100% LED
            ecLEDS[9] = (ecPercent > 80.0);
            ecLEDS[8] = (ecPercent > 70.0);
            ecLEDS[7] = (ecPercent > 60.0);
            ecLEDS[6] = (ecPercent > 50.0);
            ecLEDS[5] = (ecPercent > 40.0);
            ecLEDS[4] = (ecPercent > 30.0);
            ecLEDS[3] = (ecPercent > 20.0);
            ecLEDS[2] = (ecPercent > 10.0);
            ecLEDS[1] = (ecPercent > 0.0); //last red LED
            ecLEDS[0] = ((ecTotalOG == 0) || (ecPercent < 0.000001)); //extra "empty" LED
              
        

          }
break;

  
  }
}

void sendShiftOut(bool pins[], int dataPin, int latchPin, int clockPin)
{
    // // Define outputA (4bytes)
    // uint32_t outputA = 0;
    // // Define outputB (4bytes)
    // uint32_t outputB = 0;
    // // For each pin/bit
    // for (int pin = 0; pin < 64; pin++)
    // {
    //     // First 4 bytes
    //     if (pin < 31 && pins[pin] == 1)
    //     {
    //         // Set the value for THIS pin/bit to 1
    //         bitSet(outputA, pin);
    //     }
    //     // Last 4 bytes
    //     else if (pins[pin] == 1)
    //     {
    //         // Set the value for THIS pin/bit to 1
    //         bitSet(outputB, pin);
    //     }
    // }
    // // Break down the bytes 4-2bytes
    // uint16_t b0_1 = (outputA);
    // uint16_t b2_3 = (outputA >> 16);
    // uint16_t b4_5 = (outputB);
    // uint16_t b6_7 = (outputB >> 16);
    // // Break down the bytes 2-1bytes
    // byte b0 = lowByte(b0_1);
    // byte b1 = highByte(b0_1);
    // byte b2 = lowByte(b2_3);
    // byte b3 = highByte(b2_3);
    // byte b4 = lowByte(b4_5);
    // byte b5 = highByte(b4_5);
    // byte b6 = lowByte(b6_7);
    // byte b7 = highByte(b6_7);
    
    // mySimpit.printToKSP(String((int)b7));
    // mySimpit.printToKSP(String((int)b6));
    // mySimpit.printToKSP(String((int)b5));
    // mySimpit.printToKSP(String((int)b4));
    // mySimpit.printToKSP(String((int)b3));
    // mySimpit.printToKSP(String((int)b2));
    // mySimpit.printToKSP(String((int)b1));
    // mySimpit.printToKSP(String((int)b0));

    unsigned long long int outputvalue = 0;
// 18446744073709551615
  

    for (int j = 0; j < 64; j++){
      if(pins[j] == 1){
        ((outputvalue) |= (1ULL << (j)));
      }
    }

  


    // Disable
    // digitalWrite(dataPin, LOW);
    // digitalWrite(clockPin, LOW);
    // digitalWrite(latchPin, LOW);
    // delay(50);
    digitalWrite(latchPin, LOW);
    byte byteOut[8] = {};
    // mySimpit.printToKSP("New Loop");
    for (int i = 7; i > -1; i--){
      byteOut[i] = outputvalue >> (8*i);
      // mySimpit.printToKSP(String((int)byteOut[i]));
      shiftOut(dataPin,clockPin,MSBFIRST,byteOut[i]);
    }
    digitalWrite(latchPin, HIGH);
    digitalWrite(latchPin, LOW);
    

    // Shift the values into the register starting from the last register
    // shiftOut(dataPin, clockPin, MSBFIRST, b7);
    // shiftOut(dataPin, clockPin, MSBFIRST, b6);
    // shiftOut(dataPin, clockPin, MSBFIRST, b5);
    // shiftOut(dataPin, clockPin, MSBFIRST, b4);
    // shiftOut(dataPin, clockPin, MSBFIRST, b3);
    // shiftOut(dataPin, clockPin, MSBFIRST, b2);
    // shiftOut(dataPin, clockPin, MSBFIRST, b1);
    // shiftOut(dataPin, clockPin, MSBFIRST, b0);
    // Enable
    
}

void setOutputValues()
{
    // Shift register out A
    shiftOutA[0] = ecLEDS[10]; // A:0
    shiftOutA[1] = ecLEDS[9]; // A:1
    shiftOutA[2] = ecLEDS[8]; // A:2
    shiftOutA[3] = ecLEDS[7]; // A:3
    shiftOutA[4] = ecLEDS[6]; // A:4
    shiftOutA[5] = ecLEDS[5]; // A:5
    shiftOutA[6] = ecLEDS[4]; // A:6
    shiftOutA[7] = ecLEDS[3]; // A:7
    shiftOutA[8] = ecLEDS[2]; // B:0
    shiftOutA[9] = ecLEDS[1]; // B:1
    shiftOutA[10] = mpLEDS[10]; // B:2
    shiftOutA[11] = mpLEDS[9]; // B:3
    shiftOutA[12] = mpLEDS[8]; // B:4
    shiftOutA[13] = mpLEDS[7]; // B:5
    shiftOutA[14] = mpLEDS[6]; // B:6
    shiftOutA[15] = mpLEDS[5]; // B:7
    shiftOutA[16] = mpLEDS[4]; // C:0
    shiftOutA[17] = mpLEDS[3]; // C:1
    shiftOutA[18] = mpLEDS[2]; // C:2
    shiftOutA[19] = mpLEDS[1]; // C:3      
    shiftOutA[20] = oxLEDS[10]; // C:4      
    shiftOutA[21] = oxLEDS[9]; // C:5      
    shiftOutA[22] = oxLEDS[8]; // C:6      
    shiftOutA[23] = oxLEDS[7]; // C:7      
    shiftOutA[24] = oxLEDS[6]; // D:0      
    shiftOutA[25] = oxLEDS[5]; // D:1      
    shiftOutA[26] = oxLEDS[4]; // D:2      
    shiftOutA[27] = oxLEDS[3]; // D:3      
    shiftOutA[28] = oxLEDS[2]; // D:4      
    shiftOutA[29] = oxLEDS[1]; // D:5      
    shiftOutA[30] = lfLEDS[10]; // D:6      
    shiftOutA[31] = lfLEDS[9]; // D:7      
    shiftOutA[32] = lfLEDS[8]; // E:0      
    shiftOutA[33] = lfLEDS[7]; // E:1      
    shiftOutA[34] = lfLEDS[6]; // E:2      
    shiftOutA[35] = lfLEDS[5]; // E:3      
    shiftOutA[36] = lfLEDS[4]; // E:4      
    shiftOutA[37] = lfLEDS[3]; // E:5      
    shiftOutA[38] = lfLEDS[2]; // E:6      
    shiftOutA[39] = lfLEDS[1]; // E:7      
    shiftOutA[40] = sfLEDS[10]; // F:0      
    shiftOutA[41] = sfLEDS[9]; // F:1      
    shiftOutA[42] = sfLEDS[8]; // F:2      
    shiftOutA[43] = sfLEDS[7]; // F:3      
    shiftOutA[44] = sfLEDS[6]; // F:4      
    shiftOutA[45] = sfLEDS[5]; // F:5      
    shiftOutA[46] = sfLEDS[4]; // F:6      
    shiftOutA[47] = sfLEDS[3]; // F:7      
    shiftOutA[48] = sfLEDS[2]; // G:0      
    shiftOutA[49] = sfLEDS[1]; // G:1      
    shiftOutA[50] = ecLEDS[0]; // G:2      
    shiftOutA[51] = mpLEDS[0]; // G:3      
    shiftOutA[52] = oxLEDS[0]; // G:4      
    shiftOutA[53] = lfLEDS[0]; // G:5      
    shiftOutA[54] = sfLEDS[0]; // G:6      
    shiftOutA[55] = 0; // G:7      
    // shiftOutA[56] = 0; // H:0      
    // shiftOutA[57] = 0; // H:1      
    // shiftOutA[58] = 0; // H:2      
    // shiftOutA[59] = 0; // H:3      
    // shiftOutA[60] = 0; // H:4      
    // shiftOutA[61] = 0; // H:5      
    // shiftOutA[62] = 0; // H:6
    // shiftOutA[63] = 0; // H:7

    

    // Shift register out B
    shiftOutB[0] = sasModeLEDS[1]; // A:0
    shiftOutB[1] = sasModeLEDS[0]; // A:1
    shiftOutB[2] = sasModeLEDS[3]; // A:2
    shiftOutB[3] = sasModeLEDS[2]; // A:3
    shiftOutB[4] = sasModeLEDS[5]; // A:4
    shiftOutB[5] = sasModeLEDS[4]; // A:5
    shiftOutB[6] = sasModeLEDS[7]; // A:6
    shiftOutB[7] = sasModeLEDS[6]; // A:7
    shiftOutB[8] = sasModeLEDS[9]; // B:0
    shiftOutB[9] = sasModeLEDS[8]; // B:1
    shiftOutB[10] = throtLED[0]; // B:2
    shiftOutB[11] = throtLED[1]; // B:3
    shiftOutB[12] = throtLED[2]; // B:4
    shiftOutB[13] = throtLED[3]; // B:5
    shiftOutB[14] = throtLED[4]; // B:6
    shiftOutB[15] = stageButtonLED; // B:7
    shiftOutB[16] = abortButtonLED; // C:0
    shiftOutB[17] = sasToggleState; // C:1
    shiftOutB[18] = !sasToggleState; // C:2
    shiftOutB[19] = rcsToggleState; // C:3      
    shiftOutB[20] = !rcsToggleState; // C:4      
    shiftOutB[21] = rocketMode; // C:5      
    shiftOutB[22] = planeMode; // C:6      
    shiftOutB[23] = camRot; // C:7      
    shiftOutB[24] = !camRot; // D:0      
    shiftOutB[25] = lightsLED; // D:1      
    shiftOutB[26] = gearLED; // D:2      
    shiftOutB[27] = brakesLED; // D:3      
    shiftOutB[28] = CAG01_LED; // D:4      
    shiftOutB[29] = CAG02_LED; // D:5      
    shiftOutB[30] = totalFuel; // D:6      
    shiftOutB[31] = stageFuel; // D:7      
    // shiftOutB[32] = 0; // E:0      
    // shiftOutB[33] = 0; // E:1      
    // shiftOutB[34] = 0; // E:2      
    // shiftOutB[35] = 0; // E:3      
    // shiftOutB[36] = 0; // E:4      
    // shiftOutB[37] = 0; // E:5      
    // shiftOutB[38] = 0; // E:6      
    // shiftOutB[39] = 0; // E:7      
    // shiftOutB[40] = 0; // F:0      
    // shiftOutB[41] = 0; // F:1      
    // shiftOutB[42] = 0; // F:2      
    // shiftOutB[43] = 0; // F:3      
    // shiftOutB[44] = 0; // F:4      
    // shiftOutB[45] = 0; // F:5      
    // shiftOutB[46] = 0; // F:6      
    // shiftOutB[47] = 0; // F:7      
    // shiftOutB[48] = 0; // G:0      
    // shiftOutB[49] = 0; // G:1      
    // shiftOutB[50] = 0; // G:2      
    // shiftOutB[51] = 0; // G:3      
    // shiftOutB[52] = 0; // G:4      
    // shiftOutB[53] = 0; // G:5      
    // shiftOutB[54] = 0; // G:6      
    // shiftOutB[55] = 0; // G:7      
    // shiftOutB[56] = 0; // H:0      
    // shiftOutB[57] = 0; // H:1      
    // shiftOutB[58] = 0; // H:2      
    // shiftOutB[59] = 0; // H:3      
    // shiftOutB[60] = 0; // H:4      
    // shiftOutB[61] = 0; // H:5      
    // shiftOutB[62] = 0; // H:6
    // shiftOutB[63] = 0; // H:7
    
}

// void setInputValues()
// {
//     // Shift registers in A
//     sasButtons[0] = shiftInA.state(0); // A:0
//     sasButtons[1] = shiftInA.state(1); // A:1
//     sasButtons[2] = shiftInA.state(2); // A:2
//     sasButtons[3] = shiftInA.state(3); // A:3
//     sasButtons[4] = shiftInA.state(4); // A:4
//     sasButtons[5] = shiftInA.state(5); // A:5
//     sasButtons[6] = shiftInA.state(6); // A:6
//     sasButtons[7] = shiftInA.state(7); // A:7
//     sasButtons[8] = shiftInA.state(8); // B:0
//     sasButtons[9] = shiftInA.state(9); // B:1
//     // infoModes[10] = (bool)shiftInA[10]; // B:2
//     // infoModes[11] = (bool)shiftInA[11]; // B:3
//     // dirModes[0] = (bool)shiftInA[12]; // B:4
//     // dirModes[1] = (bool)shiftInA[13]; // B:5
//     // dirModes[2] = (bool)shiftInA[14]; // B:6
//     // dirModes[3] = (bool)shiftInA[15]; // B:7
//     // dirModes[4] = (bool)shiftInA[16]; // C:0
//     // dirModes[5] = (bool)shiftInA[17]; // C:1
//     // dirModes[6] = (bool)shiftInA[18]; // C:2
//     // dirModes[7] = (bool)shiftInA[19]; // C:3
//     // dirModes[8] = (bool)shiftInA[20]; // C:4
//     // dirModes[9] = (bool)shiftInA[21]; // C:5
//     // dirModes[10] = (bool)shiftInA[22]; // C:6
//     // dirModes[11] = (bool)shiftInA[23]; // C:7
//     // warnButtons[0] = (bool)shiftInA[24]; // D:0
//     // warnButtons[1] = (bool)shiftInA[25]; // D:1
//     // warnButtons[2] = (bool)shiftInA[26]; // D:2
//     // warnButtons[3] = (bool)shiftInA[27]; // D:3
//     // warnButtons[4] = (bool)shiftInA[28]; // D:4
//     // warnButtons[5] = (bool)shiftInA[29]; // D:5
//     // warnButtons[6] = (bool)shiftInA[30]; // D:6
//     // warnButtons[7] = (bool)shiftInA[31]; // D:7
//     // warnButtons[8] = (bool)shiftInA[32]; // E:0
//     // warnButtons[9] = (bool)shiftInA[33]; // E:1
//     // agButtons[0] = (bool)shiftInA[34]; // E:2
//     // agButtons[1] = (bool)shiftInA[35]; // E:3
//     // agButtons[2] = (bool)shiftInA[36]; // E:4
//     // agButtons[3] = (bool)shiftInA[37]; // E:5
//     // agButtons[4] = (bool)shiftInA[38]; // E:6
//     // agButtons[5] = (bool)shiftInA[39]; // E:7
//     // agButtons[6] = (bool)shiftInA[40]; // F:0
//     // agButtons[7] = (bool)shiftInA[41]; // F:1
//     // agButtons[8] = (bool)shiftInA[42]; // F:2
//     // agButtons[9] = (bool)shiftInA[43]; // F:3
//     // dockingModeSwitch = (bool)shiftInA[44]; // F:4
//     // percisionModeSwitch = (bool)shiftInA[45]; // F:5
//     // gearSwitch = (bool)shiftInA[46]; // F:6
//     // lightsSwitch = (bool)shiftInA[47]; // F:7
//     // brakeSwitch = (bool)shiftInA[48]; // G:0
//     // sasSwitch = (bool)shiftInA[49]; // G:1
//     // rcsSwitch = (bool)shiftInA[50]; // G:2
//     // throttleLockSwitch = (bool)shiftInA[51]; // G:3
//     // setTrimTranslationButton = (bool)shiftInA[52]; // G:4
//     // resetTrimTranslationButton = (bool)shiftInA[53]; // G:5
//     // setTrimRotationButton = (bool)shiftInA[54]; // G:6
//     // resetTrimRotationButton = (bool)shiftInA[55]; // G:7
//     // sasButtons[0] = (bool)shiftInA[56]; // H:0
//     // sasButtons[1] = (bool)shiftInA[57]; // H:1
//     // sasButtons[2] = (bool)shiftInA[58]; // H:2
//     // sasButtons[3] = (bool)shiftInA[59]; // H:3
//     // sasButtons[4] = (bool)shiftInA[60]; // H:4
//     // sasButtons[5] = (bool)shiftInA[61]; // H:5
//     // sasButtons[6] = (bool)shiftInA[62]; // H:6
//     // sasButtons[7] = (bool)shiftInA[63]; // H:7
//     // // Shift registers in B
//     // sasButtons[8] = (bool)shiftInB[0]; // A:0
//     // sasButtons[9] = (bool)shiftInB[1]; // A:1
//     // warpLockSwitch = (bool)shiftInB[2]; // A:2
//     // pauseButton = (bool)shiftInB[3]; // A:3
//     // cancelWarpButton = (bool)shiftInB[4]; // A:4
//     // enablePhysWarpSwitch = (bool)shiftInB[5]; // A:5
//     // decreaseWarpButton = (bool)shiftInB[6]; // A:6
//     // increaseWarpButton = (bool)shiftInB[7]; // A:7
//     // extraButton2 = (bool)shiftInB[8]; // B:0
//     // cycleFocusButton = (bool)shiftInB[9]; // B:1
//     // hideUIButton = (bool)shiftInB[10]; // B:2
//     // screenshotButton = (bool)shiftInB[11]; // B:3
//     // mapFlightSwitch = (bool)shiftInB[12]; // B:4
//     // extIvaSwitch = (bool)shiftInB[13]; // B:5
//     // cycleCamModeButton = (bool)shiftInB[14]; // B:6
//     // resetCamButton = (bool)shiftInB[15]; // B:7
//     // // More values (non-shift register)

// }