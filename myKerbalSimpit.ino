/* KerbalSimpitActionSwitch
   A demonstration of using two switches to command both SAS and RCS.
   In this example, the KSP state is monitored to update the SAS and RCS
   to match the switch state. This ensure that when using the keyboard or when
   switching vessels, the game will still match the switch positions.
   In addition, the LED_BUILTIN matches the SAS state.

*/
#include <KerbalSimpit.h>
#include <ShiftIn.h>
// #include "ezButton.h"
// #include "ShiftOutX.h"
// #include "ShiftPinNo.h"

//const int SAS_SWITCH_PIN = 3; // the pin used for controlling SAS.
//const int RCS_SWITCH_PIN = 4; // the pin used for controlling RCS.
//const int ProgradePin = 5; //sas status on pin 5
// int debounceTime = 50;

// ezButton StageButton(6);

// //SAS Actions
// ezButton stabButton(7);
// ezButton manButton(8);
// ezButton progradeButton(9);
// ezButton retrogradeButton(10);
// ezButton normalButton(11);
// ezButton antinormalButton(12);
// ezButton radialoutButton(14);
// ezButton radialinButton(15);
// ezButton targetButton(16);
// ezButton antitargetButton(17);

//Shift Register Declarations
  //SHIFT OUT A (8 registers for output)
const int SHIFT_OUT_A_DATA = 2;
const int SHIFT_OUT_A_CLOCK = 3;
const int SHIFT_OUT_A_LATCH = 4;
bool shiftOutA[64];
int outputA;

//SHIFT OUT B (3 registers for output)
const int SHIFT_OUT_B_DATA = 5;
const int SHIFT_OUT_B_CLOCK = 6;
const int SHIFT_OUT_B_LATCH = 7;
bool shiftOutB[64];
int outputB;

//SHIFT IN A (4 registers)
const int SHIFT_IN_A_ENABLE = 8;
const int SHIFT_IN_A_DATA = 9;
const int SHIFT_IN_A_CLOCK = 10;
const int SHIFT_IN_A_LATCH = 11;
// bool shiftInA[16];

//SHIFT OUT C (2 registers for output)
const int SHIFT_OUT_C_DATA = 14;
const int SHIFT_OUT_C_LATCH = 15;
const int SHIFT_OUT_C_CLOCK = 16;
bool shiftOutC[64];
int outputC;


bool sasButtons[10];
bool sasButtonOld[10];
bool sasButtonChange[10];
bool changeState = false;

byte sasCurrentMode;

ShiftIn<2> shiftInA;

// 0:Stability Assist
// 1:Maneuver
// 2:Prograde
// 3:Retrograde
// 4:Normal
// 5:Anti-Normal
// 6:Radial Out
// 7:Radial In
// 8:Target
// 9:Anti-Target
bool throtLED[10];

float sfPercent = 100;
float sfTotalOG = 0;
bool sfLEDS[11];

float lfPercent = 100;
float lfTotalOG = 0;
bool lfLEDS[11];

float oxPercent = 100;
float oxTotalOG = 0;
bool oxLEDS[11];

float mpPercent = 100;
float mpTotalOG = 0;
bool mpLEDS[11];

float ecPercent = 100;
float ecTotalOG = 0;
bool ecLEDS[11];

bool sasModeLEDS[10];
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


// int testArray[64];

const int throttle = A0; //throttle
const int rX = A1; //rotation x-axis
const int rY = A2; //rotation y-axis
int deadzoneCenter_rX = 550;
int deadzoneCenter_rY = 562;
int deadzoneCenter_rZ = 550;
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

  // //SAS buttons
  // stabButton.setDebounceTime(debounceTime);
  // manButton.setDebounceTime(debounceTime);
  // progradeButton.setDebounceTime(debounceTime);
  // retrogradeButton.setDebounceTime(debounceTime);
  // normalButton.setDebounceTime(debounceTime);
  // antinormalButton.setDebounceTime(debounceTime);
  // radialoutButton.setDebounceTime(debounceTime);
  // radialinButton.setDebounceTime(debounceTime);
  // targetButton.setDebounceTime(debounceTime);
  // antitargetButton.setDebounceTime(debounceTime);


  // Set up the build in LED, and turn it on.
  pinMode(LED_BUILTIN, OUTPUT);
  //pinMode(ProgradePin, OUTPUT);
  pinMode(throttle,INPUT); //rotation x-axis
  pinMode(rX,INPUT); //rotation x-axis
  pinMode(rY,INPUT); //rotation y-axis
  digitalWrite(LED_BUILTIN, HIGH);

  //Shift register pins
  pinMode(SHIFT_OUT_A_DATA, OUTPUT);
  pinMode(SHIFT_OUT_A_CLOCK, OUTPUT);
  pinMode(SHIFT_OUT_A_LATCH, OUTPUT);

  pinMode(SHIFT_OUT_B_DATA, OUTPUT);
  pinMode(SHIFT_OUT_B_CLOCK, OUTPUT);
  pinMode(SHIFT_OUT_B_LATCH, OUTPUT);

  pinMode(SHIFT_OUT_C_DATA, OUTPUT);
  pinMode(SHIFT_OUT_C_CLOCK, OUTPUT);
  pinMode(SHIFT_OUT_C_LATCH, OUTPUT);

  // pinMode(SHIFT_IN_A_DATA, INPUT);
  // pinMode(SHIFT_IN_A_CLOCK, OUTPUT);
  // pinMode(SHIFT_IN_A_LATCH, OUTPUT);
  // pinMode(SHIFT_IN_A_ENABLE, OUTPUT);
  // digitalWrite(SHIFT_IN_A_LATCH, HIGH);



  // Set up the two switches with builtin pullup.
  //pinMode(SAS_SWITCH_PIN, INPUT_PULLUP);
  //pinMode(RCS_SWITCH_PIN, INPUT_PULLUP);

  // This loop continually attempts to handshake with the plugin.
  // It will keep retrying until it gets a successful handshake.

  while (!mySimpit.init()) {
   delay(100);
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

  // int regsA = 5;

  // for (int j = 0; j < 64; ++j){
  //   testArray[j] = round(pow(2,j))-1;
  //   }

// bool testArray[64] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
// int testArray[64] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


  // for (int i = 0; i < 64; i++){ 
    
  //   testArray[i] = 1;
    
    // digitalWrite(SHIFT_OUT_A_LATCH, HIGH);
    // sendShiftOut(testArray,SHIFT_OUT_A_DATA,SHIFT_OUT_A_LATCH,SHIFT_OUT_A_CLOCK);
    //shiftOut_X(SHIFT_OUT_A_DATA,SHIFT_OUT_A_CLOCK,MSBFIRST,5,testArray[i]);
    // shiftOut(SHIFT_OUT_A_DATA,SHIFT_OUT_A_CLOCK,MSBFIRST,testArray[i]>>64);
    // shiftOut(SHIFT_OUT_A_DATA,SHIFT_OUT_A_CLOCK,MSBFIRST,testArray[i]>>56);
    // shiftOut(SHIFT_OUT_A_DATA,SHIFT_OUT_A_CLOCK,MSBFIRST,testArray[i]>>48);
    // shiftOut(SHIFT_OUT_A_DATA,SHIFT_OUT_A_CLOCK,MSBFIRST,testArray[i]>>40);
    // shiftOut(SHIFT_OUT_A_DATA,SHIFT_OUT_A_CLOCK,MSBFIRST,testArray[i]>>32);
    // shiftOut(SHIFT_OUT_A_DATA,SHIFT_OUT_A_CLOCK,MSBFIRST,testArray[i]>>24);
    // shiftOut(SHIFT_OUT_A_DATA,SHIFT_OUT_A_CLOCK,MSBFIRST,testArray[i]>>16);
    // shiftOut(SHIFT_OUT_A_DATA,SHIFT_OUT_A_CLOCK,MSBFIRST,testArray[i]>>8);
    // shiftOut(SHIFT_OUT_A_DATA,SHIFT_OUT_A_CLOCK,MSBFIRST,testArray[i]);
    // digitalWrite(SHIFT_OUT_A_LATCH,LOW);
    // delay(5000);
  // }

  shiftInA.getCurrent();
  totalFuel = shiftInA.state(10);
  stageFuel = shiftInA.state(11);
  mySimpit.printToKSP("Initialize Total vs Stage");
  mySimpit.printToKSP(String(totalFuel));
  mySimpit.printToKSP(String(stageFuel));
  
  //TOTvSTAGE = digitalRead(SAS_SWITCH_PIN);
}

void loop() {

  // //Initialize Buttons
  // StageButton.loop();

  // //SAS Modes
  // stabButton.loop();
  // manButton.loop();
  // progradeButton.loop();
  // retrogradeButton.loop();
  // normalButton.loop();
  // antinormalButton.loop();
  // radialoutButton.loop();
  // radialinButton.loop();
  // targetButton.loop();
  // antitargetButton.loop();
  
  // Get the SAS switch state
  //TOTvSTAGE = digitalRead(SAS_SWITCH_PIN);
  
  // Check for new serial messages.
  mySimpit.update();

 uint16_t throtRAW = analogRead(throttle);
 uint16_t rXraw = analogRead(rX);
 uint16_t rYraw = analogRead(rY);
 
  if((deadzoneCenter_rX - deadzone) < rXraw && rXraw < (deadzoneCenter_rX + deadzone)){
    yaw = 0;
  } else {
    yaw = map(rXraw,0,1023,-32767,32767); //x
  }

  if((deadzoneCenter_rY - deadzone) < rYraw && rYraw < (deadzoneCenter_rY + deadzone)){
    pitch = 0;
  } else {
    pitch = map(rYraw,0,1023,-32767,32767); //y
  }
  
  yaw *= -1; //invert x axis

  rotationMessage myRot;
  myRot.mask = 7; //send pitch, yaw, and roll all at once

  myRot.pitch = pitch; //y
  myRot.yaw = yaw; //x
  myRot.roll = roll; //z

/*
  Serial.println("X =");
    Serial.println(yaw);
    Serial.println(rXraw);
  Serial.println("Y ="); 
    Serial.println(pitch);
    Serial.println(rYraw);
  delay(1500);
*/
  
  mySimpit.send(ROTATION_MESSAGE, myRot);
  if(throtRAW < 75){
    throtRAW = 0;
  }

  throt = map(throtRAW,0,1023,0,32767);

  throttleMessage throttleMsg;

  throttleMsg.throttle = throt;

  mySimpit.send(THROTTLE_MESSAGE, throttleMsg);

  for (int i = 0; i < 10; i++){
    if(throt > i*3276){
      throtLED[i] = 1;
    } else {
      throtLED[i] = 0;
    }
  }

//  if(StageButton.isPressed()){
//   mySimpit.activateAction(STAGE_ACTION);
//  }

// //SAS Mode Buttons
//  if(stabButton.isPressed()){
//    mySimpit.setSASMode(AP_STABILITYASSIST);
//  }
//   if(manButton.isPressed()){
//    mySimpit.setSASMode(AP_MANEUVER);
//  }
//   if(progradeButton.isPressed()){
//    mySimpit.setSASMode(AP_PROGRADE);
//  }
//   if(retrogradeButton.isPressed()){
//    mySimpit.setSASMode(AP_RETROGRADE);
//  }
//   if(normalButton.isPressed()){
//    mySimpit.setSASMode(AP_NORMAL);
//  }
//   if(antinormalButton.isPressed()){
//    mySimpit.setSASMode(AP_ANTINORMAL);
//  }
//    if(radialoutButton.isPressed()){
//    mySimpit.setSASMode(AP_RADIALIN);
//  }
//   if(radialinButton.isPressed()){
//    mySimpit.setSASMode(AP_RADIALOUT);
//  }
//   if(targetButton.isPressed()){
//     mySimpit.setSASMode(AP_TARGET);
//  }
//    if(antitargetButton.isPressed()){
//     mySimpit.setSASMode(AP_ANTITARGET);
//  }


/*
  // Update the SAS to match the state, only if a change is needed to avoid
  // spamming commands.
  if(sas_switch_state && !(currentActionStatus & SAS_ACTION)){
    mySimpit.printToKSP("Activate SAS!");
    mySimpit.activateAction(SAS_ACTION);
  }
  if(!sas_switch_state && (currentActionStatus & SAS_ACTION)){
    mySimpit.printToKSP("Deactivate SAS!");
    mySimpit.deactivateAction(SAS_ACTION);
  }
*/
/*

Template for toggle inputs

  // Get the SAS switch state
  bool **state = digitalRead(**PIN);

  // Update the SAS to match the state, only if a change is needed to avoid
  // spamming commands.
  if(sas_switch_state && !(currentActionStatus & SAS_ACTION)){
    mySimpit.printToKSP("Activate SAS!");
    mySimpit.activateAction(SAS_ACTION);
  }
  if(!sas_switch_state && (currentActionStatus & SAS_ACTION)){
    mySimpit.printToKSP("Deactivate SAS!");
    mySimpit.deactivateAction(SAS_ACTION);
  }

*/

/*
  // Get the RCS switch state
  bool rcs_switch_state = digitalRead(RCS_SWITCH_PIN);

  // Update the RCS to match the state, only if a change is needed to avoid
  // spamming commands.
  if(rcs_switch_state && !(currentActionStatus & RCS_ACTION)){
    mySimpit.printToKSP("Activate RCS!");
    mySimpit.activateAction(RCS_ACTION);
  }
  if(!rcs_switch_state && (currentActionStatus & RCS_ACTION)){
    mySimpit.printToKSP("Deactivate RCS!");
    mySimpit.deactivateAction(RCS_ACTION);
  }*/

  // getShiftIn(SHIFT_IN_A_ENABLE,SHIFT_IN_A_DATA,SHIFT_IN_A_CLOCK,SHIFT_IN_A_LATCH);
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

if(shiftInA.pressed(10)){
  mySimpit.printToKSP("Total Fuel Mode Activated!");
  totalFuel = 1;
  stageFuel = 0;
}

if(shiftInA.pressed(11)){
  mySimpit.printToKSP("Stage Fuel Mode Activated!");
  stageFuel = 1;
  totalFuel = 0;
}

if((!shiftInA.state(10)) && (!shiftInA.state(11))){
  totalFuel = 1;
  stageFuel = 0;
}

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

}
  // setInputValues();
  // checkSASButtons();





// 0:Stability Assist
// 1:Maneuver
// 2:Prograde
// 3:Retrograde
// 4:Normal
// 5:Anti-Normal
// 6:Radial Out
// 7:Radial In
// 8:Target
// 9:Anti-Target

// //  && (currentActionStatus & SAS_ACTION)
// if((sasButtonChange[0] == true) && (sasCurrentMode != AP_STABILITYASSIST)){
//   mySimpit.setSASMode(AP_STABILITYASSIST);
//   // sasButtons[0] = 0;
// }

// if((sasButtonChange[1] == true) && (sasCurrentMode != AP_MANEUVER)){
//   mySimpit.setSASMode(AP_MANEUVER);
//   // sasButtons[1] = 0;
// }

// if((sasButtonChange[2] == true) && (sasCurrentMode != AP_PROGRADE)){
//   mySimpit.setSASMode(AP_PROGRADE);
//   // sasButtons[2] = 0;
// }

// if((sasButtonChange[3] == true) && (sasCurrentMode != AP_RETROGRADE)){
//   mySimpit.setSASMode(AP_RETROGRADE);
//   // sasButtons[3] = 0;
// }

// if((sasButtonChange[4] == true) && (sasCurrentMode != AP_NORMAL)){
//   mySimpit.setSASMode(AP_NORMAL);
//   // sasButtons[4] = 0;
// }

// if((sasButtonChange[5] == true) && (sasCurrentMode != AP_ANTINORMAL)){
//   mySimpit.setSASMode(AP_ANTINORMAL);
//   // sasButtons[5] = 0;
// }

// if((sasButtonChange[6] == true) && (sasCurrentMode != AP_RADIALIN)){
//   mySimpit.setSASMode(AP_RADIALIN);
//   // sasButtons[6] = 0;
// }

// if((sasButtonChange[7] == true) && (sasCurrentMode != AP_RADIALOUT)){
//   mySimpit.setSASMode(AP_RADIALOUT);
//   // sasButtons[7] = 0;
// }

// if((sasButtonChange[8] == true) && (sasCurrentMode != AP_TARGET)){
//   mySimpit.setSASMode(AP_TARGET);
//   // sasButtons[8] = 0;
// }

// if((sasButtonChange[9] == true) && (sasCurrentMode != AP_ANTITARGET)){
//   mySimpit.setSASMode(AP_ANTITARGET);
//   // sasButtons[9] = 0;
// }

// //without checking current mode

// if((sasButtonChange[0] == true)){
//   mySimpit.setSASMode(AP_STABILITYASSIST);
//   // sasButtons[0] = 0;
// }

// if((sasButtonChange[1] == true)){
//   mySimpit.setSASMode(AP_MANEUVER);
//   // sasButtons[1] = 0;
// }

// if((sasButtonChange[2] == true)){
//   mySimpit.setSASMode(AP_PROGRADE);
//   // sasButtons[2] = 0;
// }

// if((sasButtonChange[3] == true)){
//   mySimpit.setSASMode(AP_RETROGRADE);
//   // sasButtons[3] = 0;
// }

// if((sasButtonChange[4] == true)){
//   mySimpit.setSASMode(AP_NORMAL);
//   // sasButtons[4] = 0;
// }

// if((sasButtonChange[5] == true)){
//   mySimpit.setSASMode(AP_ANTINORMAL);
//   // sasButtons[5] = 0;
// }

// if((sasButtonChange[6] == true)){
//   mySimpit.setSASMode(AP_RADIALIN);
//   // sasButtons[6] = 0;
// }

// if((sasButtonChange[7] == true)){
//   mySimpit.setSASMode(AP_RADIALOUT);
//   // sasButtons[7] = 0;
// }

// if((sasButtonChange[8] == true)){
//   mySimpit.setSASMode(AP_TARGET);
//   // sasButtons[8] = 0;
// }

// if((sasButtonChange[9] == true)){
//   mySimpit.setSASMode(AP_ANTITARGET);
//   // sasButtons[9] = 0;
// }

    


  setOutputValues();
  sendShiftOut(shiftOutA,SHIFT_OUT_A_DATA,SHIFT_OUT_A_LATCH,SHIFT_OUT_A_CLOCK);
  // setOutputValues();
  sendShiftOut(shiftOutB,SHIFT_OUT_B_DATA,SHIFT_OUT_B_LATCH,SHIFT_OUT_B_CLOCK);

  sendShiftOut(shiftOutC,SHIFT_OUT_C_DATA,SHIFT_OUT_C_LATCH,SHIFT_OUT_C_CLOCK);
  // outputA = 0;
  // int i = 0;
  // for (int i = 0; i < sizeof(shiftOutA)/sizeof(shiftOutA[0]); ++i){

  //   int binaryAdd = shiftOutA[i]*round(pow(2,i));

  //   outputA += binaryAdd;
  // }
/*
  String myString = "ShiftOut = ";
  for (int j = 0; j < sizeof(shiftOutA)/sizeof(shiftOutA[0]); ++j){
      myString += String((int)shiftOutA[j]);
  }
*/
  //mySimpit.printToKSP(String(outputA));
  //mySimpit.printToKSP(myString);
  // digitalWrite(SHIFT_OUT_A_LATCH, HIGH);
  // shiftOut(SHIFT_OUT_A_DATA,SHIFT_OUT_A_CLOCK,MSBFIRST,outputA>>64);
  // shiftOut(SHIFT_OUT_A_DATA,SHIFT_OUT_A_CLOCK,MSBFIRST,outputA>>56);
  // shiftOut(SHIFT_OUT_A_DATA,SHIFT_OUT_A_CLOCK,MSBFIRST,outputA>>48);
  // shiftOut(SHIFT_OUT_A_DATA,SHIFT_OUT_A_CLOCK,MSBFIRST,outputA>>40);
  // shiftOut(SHIFT_OUT_A_DATA,SHIFT_OUT_A_CLOCK,MSBFIRST,outputA>>32);
  // shiftOut(SHIFT_OUT_A_DATA,SHIFT_OUT_A_CLOCK,MSBFIRST,outputA>>24);
  // shiftOut(SHIFT_OUT_A_DATA,SHIFT_OUT_A_CLOCK,MSBFIRST,outputA>>16);
  // shiftOut(SHIFT_OUT_A_DATA,SHIFT_OUT_A_CLOCK,MSBFIRST,outputA>>8);
  // shiftOut(SHIFT_OUT_A_DATA,SHIFT_OUT_A_CLOCK,MSBFIRST,outputA);
  // digitalWrite(SHIFT_OUT_A_LATCH,LOW);

  // bool testArray[64] = {0,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
// int testArray[64] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


  // for (int i = 0; i < 64; i++){ 
    
  //   testArray[i] = 1;
    
    // digitalWrite(SHIFT_OUT_A_LATCH, HIGH);
    // sendShiftOut(testArray,SHIFT_OUT_A_DATA,SHIFT_OUT_A_LATCH,SHIFT_OUT_A_CLOCK);
    //shiftOut_X(SHIFT_OUT_A_DATA,SHIFT_OUT_A_CLOCK,MSBFIRST,5,testArray[i]);
    // shiftOut(SHIFT_OUT_A_DATA,SHIFT_OUT_A_CLOCK,MSBFIRST,testArray[i]>>64);
    // shiftOut(SHIFT_OUT_A_DATA,SHIFT_OUT_A_CLOCK,MSBFIRST,testArray[i]>>56);
    // shiftOut(SHIFT_OUT_A_DATA,SHIFT_OUT_A_CLOCK,MSBFIRST,testArray[i]>>48);
    // shiftOut(SHIFT_OUT_A_DATA,SHIFT_OUT_A_CLOCK,MSBFIRST,testArray[i]>>40);
    // shiftOut(SHIFT_OUT_A_DATA,SHIFT_OUT_A_CLOCK,MSBFIRST,testArray[i]>>32);
    // shiftOut(SHIFT_OUT_A_DATA,SHIFT_OUT_A_CLOCK,MSBFIRST,testArray[i]>>24);
    // shiftOut(SHIFT_OUT_A_DATA,SHIFT_OUT_A_CLOCK,MSBFIRST,testArray[i]>>16);
    // shiftOut(SHIFT_OUT_A_DATA,SHIFT_OUT_A_CLOCK,MSBFIRST,testArray[i]>>8);
    // shiftOut(SHIFT_OUT_A_DATA,SHIFT_OUT_A_CLOCK,MSBFIRST,testArray[i]);
    // digitalWrite(SHIFT_OUT_A_LATCH,LOW);
    // delay(5000);

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
   
   
   /* 
    if(TOTvSTAGE){
      break;
    } else {
  if(msgSize == sizeof(resourceMessage)) {
   
    resourceMessage myLF_TOT;
    myLF_TOT = parseResource(msg);

    //initialize total Liquid Fuel
    if(LF_TOT_T == 0){
      LF_TOT_T = myLF_TOT.total;
    }
    
    //continuously update available Liquid Fuel
    LF_TOT_A = myLF_TOT.available;

    //avoid divide by 0 if there's no fuel available
    if(LF_TOT_A == 0){
      //mySimpit.printToKSP("LF Available = " + String(LF_TOT_A,3));
      digitalWrite(ProgradePin, HIGH);
    } else {
       
    //calculate percent from Total
    percentLF_TOT = 100* LF_TOT_A / LF_TOT_T;
    
      if(percentLF_TOT < 50) {
          mySimpit.printToKSP("TOTALLF Calculation performed");
          mySimpit.printToKSP("Total: " + String(LF_TOT_T,3));
          mySimpit.printToKSP("Available: " + String(LF_TOT_A,3));
          mySimpit.printToKSP("Percent: " + String(percentLF_TOT,3));
          digitalWrite(ProgradePin, HIGH);
      } else {
          digitalWrite(ProgradePin, LOW);
          mySimpit.printToKSP("TOTALLF Calculation performed");
          mySimpit.printToKSP("Total: " + String(LF_TOT_T,3));
          mySimpit.printToKSP("Available: " + String(LF_TOT_A,3));
          mySimpit.printToKSP("Percent: " + String(percentLF_TOT,3));
      }
          
    }
  }
 }
  */
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
   /*  
     
      if(TOTvSTAGE){
      
  if(msgSize == sizeof(resourceMessage)) {
    resourceMessage myLF_STAGE;
    myLF_STAGE = parseResource(msg);

    //initialize total Liquid Fuel
    

   LF_STAGE_T = myLF_STAGE.total;
    

    //continuously update available Liquid Fuel
    LF_STAGE_A = myLF_STAGE.available;

    //avoid divide by 0 if there's no fuel available
    if(LF_STAGE_A == 0){
      //mySimpit.printToKSP("LF Available = " + String(LF_STAGE_A,3));
      digitalWrite(ProgradePin, HIGH);
    } else {
       
    //calculate percent from Total
    percentLF_STAGE = 100* LF_STAGE_A / LF_STAGE_T;
    
      if(percentLF_STAGE < 50) {
          mySimpit.printToKSP("STAGELF Calculation performed");
          mySimpit.printToKSP("Total: " + String(LF_STAGE_T,3));
          mySimpit.printToKSP("Available: " + String(LF_STAGE_A,3));
          mySimpit.printToKSP("Percent: " + String(percentLF_STAGE,3));
          digitalWrite(ProgradePin, HIGH);
      } else {
          digitalWrite(ProgradePin, LOW);
          mySimpit.printToKSP("STAGELF Calculation performed");
          mySimpit.printToKSP("Total: " + String(LF_STAGE_T,3));
          mySimpit.printToKSP("Available: " + String(LF_STAGE_A,3));
          mySimpit.printToKSP("Percent: " + String(percentLF_STAGE,3));
      }
          
    }
 }
      } else {
        break;
      }
  */
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
  // if (msgSize == sizeof(resourceMessage))
  //         {
  //           resourceMessage sf;
  //           sf = parseMessage<resourceMessage>(msg);

  //           if(sfTotalOG == 0){
  //             sfTotalOG = sf.total;
  //           }


  //           sfPercent = 100 * sf.available / sfTotalOG;
            
  //           sfLEDS[10] = (sfPercent > 90.0); //blue 100% LED
  //           sfLEDS[9] = (sfPercent > 80.0);
  //           sfLEDS[8] = (sfPercent > 70.0);
  //           sfLEDS[7] = (sfPercent > 60.0);
  //           sfLEDS[6] = (sfPercent > 50.0);
  //           sfLEDS[5] = (sfPercent > 40.0);
  //           sfLEDS[4] = (sfPercent > 30.0);
  //           sfLEDS[3] = (sfPercent > 20.0);
  //           sfLEDS[2] = (sfPercent > 10.0);
  //           sfLEDS[1] = (sfPercent > 0.0); //last red LED
  //           sfLEDS[0] = ((sfTotalOG == 0) || (sfPercent < 0.000001)); //extra "empty" LED
              
        

  //         }
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

/*void checkLFTOT() {

    float LF_TOT_T = myLF_TOT.total;
    float LF_TOT_A = myLF_TOT.available;
    float percentLF_TOT = LF_TOT_A / LF_TOT_T;
    
  if(percentLF_TOT < 0.50) {
          mySimpit.printToKSP("Calculation performed");
    mySimpit.printToKSP("Total: " + String(LF_TOT_T,3));
    mySimpit.printToKSP("Available: " + String(LF_TOT_A,3));
    mySimpit.printToKSP("Percent: " + String(percentLF_TOT,3));
    digitalWrite(ProgradePin, HIGH);
    } else {
      digitalWrite(ProgradePin, LOW);
          mySimpit.printToKSP("Calculation performed");
    mySimpit.printToKSP("Total: " + String(LF_TOT_T,3));
    mySimpit.printToKSP("Available: " + String(LF_TOT_A,3));
    mySimpit.printToKSP("Percent: " + String(percentLF_TOT,3));
    }
    
}*/

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
    shiftOutA[0] = sfLEDS[0]; // A:0
    shiftOutA[1] = sfLEDS[1]; // A:1
    shiftOutA[2] = sfLEDS[2]; // A:2
    shiftOutA[3] = sfLEDS[3]; // A:3
    shiftOutA[4] = sfLEDS[4]; // A:4
    shiftOutA[5] = sfLEDS[5]; // A:5
    shiftOutA[6] = sfLEDS[6]; // A:6
    shiftOutA[7] = sfLEDS[7]; // A:7
    shiftOutA[8] = sfLEDS[8]; // B:0
    shiftOutA[9] = sfLEDS[9]; // B:1
    shiftOutA[10] = sfLEDS[10]; // B:2
    shiftOutA[11] = lfLEDS[0]; // B:3
    shiftOutA[12] = lfLEDS[1]; // B:4
    shiftOutA[13] = lfLEDS[2]; // B:5
    shiftOutA[14] = lfLEDS[3]; // B:6
    shiftOutA[15] = lfLEDS[4]; // B:7
    shiftOutA[16] = lfLEDS[5]; // C:0
    shiftOutA[17] = lfLEDS[6]; // C:1
    shiftOutA[18] = lfLEDS[7]; // C:2
    shiftOutA[19] = lfLEDS[8]; // C:3      
    shiftOutA[20] = lfLEDS[9]; // C:4      
    shiftOutA[21] = lfLEDS[10]; // C:5      
    shiftOutA[22] = oxLEDS[0]; // C:6      
    shiftOutA[23] = oxLEDS[1]; // C:7      
    shiftOutA[24] = oxLEDS[2]; // D:0      
    shiftOutA[25] = oxLEDS[3]; // D:1      
    shiftOutA[26] = oxLEDS[4]; // D:2      
    shiftOutA[27] = oxLEDS[5]; // D:3      
    shiftOutA[28] = oxLEDS[6]; // D:4      
    shiftOutA[29] = oxLEDS[7]; // D:5      
    shiftOutA[30] = oxLEDS[8]; // D:6      
    shiftOutA[31] = oxLEDS[9]; // D:7      
    shiftOutA[32] = oxLEDS[10]; // E:0      
    shiftOutA[33] = mpLEDS[0]; // E:1      
    shiftOutA[34] = mpLEDS[1]; // E:2      
    shiftOutA[35] = mpLEDS[2]; // E:3      
    shiftOutA[36] = mpLEDS[3]; // E:4      
    shiftOutA[37] = mpLEDS[4]; // E:5      
    shiftOutA[38] = mpLEDS[5]; // E:6      
    shiftOutA[39] = mpLEDS[6]; // E:7      
    shiftOutA[40] = mpLEDS[7]; // F:0      
    shiftOutA[41] = mpLEDS[8]; // F:1      
    shiftOutA[42] = mpLEDS[9]; // F:2      
    shiftOutA[43] = mpLEDS[10]; // F:3      
    shiftOutA[44] = ecLEDS[0]; // F:4      
    shiftOutA[45] = ecLEDS[1]; // F:5      
    shiftOutA[46] = ecLEDS[2]; // F:6      
    shiftOutA[47] = ecLEDS[3]; // F:7      
    shiftOutA[48] = ecLEDS[4]; // G:0      
    shiftOutA[49] = ecLEDS[5]; // G:1      
    shiftOutA[50] = ecLEDS[6]; // G:2      
    shiftOutA[51] = ecLEDS[7]; // G:3      
    shiftOutA[52] = ecLEDS[8]; // G:4      
    shiftOutA[53] = ecLEDS[9]; // G:5      
    shiftOutA[54] = ecLEDS[10]; // G:6      
    shiftOutA[55] = 0; // G:7      
    shiftOutA[56] = 0; // H:0      
    shiftOutA[57] = 0; // H:1      
    shiftOutA[58] = 0; // H:2      
    shiftOutA[59] = 0; // H:3      
    shiftOutA[60] = 0; // H:4      
    shiftOutA[61] = 0; // H:5      
    shiftOutA[62] = 0; // H:6
    shiftOutA[63] = 0; // H:7

    

    // Shift register out B
    shiftOutB[0] = sasModeLEDS[0]; // A:0
    shiftOutB[1] = sasModeLEDS[1]; // A:1
    shiftOutB[2] = sasModeLEDS[2]; // A:2
    shiftOutB[3] = sasModeLEDS[3]; // A:3
    shiftOutB[4] = sasModeLEDS[4]; // A:4
    shiftOutB[5] = sasModeLEDS[5]; // A:5
    shiftOutB[6] = sasModeLEDS[6]; // A:6
    shiftOutB[7] = sasModeLEDS[7]; // A:7
    shiftOutB[8] = sasModeLEDS[8]; // B:0
    shiftOutB[9] = sasModeLEDS[9]; // B:1
    shiftOutB[10] = 0; // B:2
    shiftOutB[11] = 0; // B:3
    shiftOutB[12] = 0; // B:4
    shiftOutB[13] = 0; // B:5
    shiftOutB[14] = 0; // B:6
    shiftOutB[15] = 0; // B:7
    shiftOutB[16] = 0; // C:0
    shiftOutB[17] = 0; // C:1
    shiftOutB[18] = 0; // C:2
    shiftOutB[19] = 0; // C:3      
    shiftOutB[20] = 0; // C:4      
    shiftOutB[21] = 0; // C:5      
    shiftOutB[22] = 0; // C:6      
    shiftOutB[23] = 0; // C:7      
    shiftOutB[24] = 0; // D:0      
    shiftOutB[25] = 0; // D:1      
    shiftOutB[26] = 0; // D:2      
    shiftOutB[27] = 0; // D:3      
    shiftOutB[28] = 0; // D:4      
    shiftOutB[29] = 0; // D:5      
    shiftOutB[30] = 0; // D:6      
    shiftOutB[31] = 0; // D:7      
    shiftOutB[32] = 0; // E:0      
    shiftOutB[33] = 0; // E:1      
    shiftOutB[34] = 0; // E:2      
    shiftOutB[35] = 0; // E:3      
    shiftOutB[36] = 0; // E:4      
    shiftOutB[37] = 0; // E:5      
    shiftOutB[38] = 0; // E:6      
    shiftOutB[39] = 0; // E:7      
    shiftOutB[40] = 0; // F:0      
    shiftOutB[41] = 0; // F:1      
    shiftOutB[42] = 0; // F:2      
    shiftOutB[43] = 0; // F:3      
    shiftOutB[44] = 0; // F:4      
    shiftOutB[45] = 0; // F:5      
    shiftOutB[46] = 0; // F:6      
    shiftOutB[47] = 0; // F:7      
    shiftOutB[48] = 0; // G:0      
    shiftOutB[49] = 0; // G:1      
    shiftOutB[50] = 0; // G:2      
    shiftOutB[51] = 0; // G:3      
    shiftOutB[52] = 0; // G:4      
    shiftOutB[53] = 0; // G:5      
    shiftOutB[54] = 0; // G:6      
    shiftOutB[55] = 0; // G:7      
    shiftOutB[56] = 0; // H:0      
    shiftOutB[57] = 0; // H:1      
    shiftOutB[58] = 0; // H:2      
    shiftOutB[59] = 0; // H:3      
    shiftOutB[60] = 0; // H:4      
    shiftOutB[61] = 0; // H:5      
    shiftOutB[62] = 0; // H:6
    shiftOutB[63] = 0; // H:7
    
    // Shift register out C
    shiftOutC[0] = throtLED[0]; // A:0
    shiftOutC[1] = throtLED[1]; // A:1
    shiftOutC[2] = throtLED[2]; // A:2
    shiftOutC[3] = throtLED[3]; // A:3
    shiftOutC[4] = throtLED[4]; // A:4
    shiftOutC[5] = throtLED[5]; // A:5
    shiftOutC[6] = throtLED[6]; // A:6
    shiftOutC[7] = throtLED[7]; // A:7
    shiftOutC[8] = throtLED[8]; // B:0
    shiftOutC[9] = throtLED[9]; // B:1
    shiftOutC[10] = 0; // B:2
    shiftOutC[11] = 0; // B:3
    shiftOutC[12] = 0; // B:4
    shiftOutC[13] = 0; // B:5
    shiftOutC[14] = 0; // B:6
    shiftOutC[15] = 0; // B:7
    shiftOutC[16] = 0; // C:0
    shiftOutC[17] = 0; // C:1
    shiftOutC[18] = 0; // C:2
    shiftOutC[19] = 0; // C:3      
    shiftOutC[20] = 0; // C:4      
    shiftOutC[21] = 0; // C:5      
    shiftOutC[22] = 0; // C:6      
    shiftOutC[23] = 0; // C:7      
    shiftOutC[24] = 0; // D:0      
    shiftOutC[25] = 0; // D:1      
    shiftOutC[26] = 0; // D:2      
    shiftOutC[27] = 0; // D:3      
    shiftOutC[28] = 0; // D:4      
    shiftOutC[29] = 0; // D:5      
    shiftOutC[30] = 0; // D:6      
    shiftOutC[31] = 0; // D:7      
    shiftOutC[32] = 0; // E:0      
    shiftOutC[33] = 0; // E:1      
    shiftOutC[34] = 0; // E:2      
    shiftOutC[35] = 0; // E:3      
    shiftOutC[36] = 0; // E:4      
    shiftOutC[37] = 0; // E:5      
    shiftOutC[38] = 0; // E:6      
    shiftOutC[39] = 0; // E:7      
    shiftOutC[40] = 0; // F:0      
    shiftOutC[41] = 0; // F:1      
    shiftOutC[42] = 0; // F:2      
    shiftOutC[43] = 0; // F:3      
    shiftOutC[44] = 0; // F:4      
    shiftOutC[45] = 0; // F:5      
    shiftOutC[46] = 0; // F:6      
    shiftOutC[47] = 0; // F:7      
    shiftOutC[48] = 0; // G:0      
    shiftOutC[49] = 0; // G:1      
    shiftOutC[50] = 0; // G:2      
    shiftOutC[51] = 0; // G:3      
    shiftOutC[52] = 0; // G:4      
    shiftOutC[53] = 0; // G:5      
    shiftOutC[54] = 0; // G:6      
    shiftOutC[55] = 0; // G:7      
    shiftOutC[56] = 0; // H:0      
    shiftOutC[57] = 0; // H:1      
    shiftOutC[58] = 0; // H:2      
    shiftOutC[59] = 0; // H:3      
    shiftOutC[60] = 0; // H:4      
    shiftOutC[61] = 0; // H:5      
    shiftOutC[62] = 0; // H:6
    shiftOutC[63] = 0; // H:7
    
}

void setInputValues()
{
    // Shift registers in A
    sasButtons[0] = shiftInA.state(0); // A:0
    sasButtons[1] = shiftInA.state(1); // A:1
    sasButtons[2] = shiftInA.state(2); // A:2
    sasButtons[3] = shiftInA.state(3); // A:3
    sasButtons[4] = shiftInA.state(4); // A:4
    sasButtons[5] = shiftInA.state(5); // A:5
    sasButtons[6] = shiftInA.state(6); // A:6
    sasButtons[7] = shiftInA.state(7); // A:7
    sasButtons[8] = shiftInA.state(8); // B:0
    sasButtons[9] = shiftInA.state(9); // B:1
    // infoModes[10] = (bool)shiftInA[10]; // B:2
    // infoModes[11] = (bool)shiftInA[11]; // B:3
    // dirModes[0] = (bool)shiftInA[12]; // B:4
    // dirModes[1] = (bool)shiftInA[13]; // B:5
    // dirModes[2] = (bool)shiftInA[14]; // B:6
    // dirModes[3] = (bool)shiftInA[15]; // B:7
    // dirModes[4] = (bool)shiftInA[16]; // C:0
    // dirModes[5] = (bool)shiftInA[17]; // C:1
    // dirModes[6] = (bool)shiftInA[18]; // C:2
    // dirModes[7] = (bool)shiftInA[19]; // C:3
    // dirModes[8] = (bool)shiftInA[20]; // C:4
    // dirModes[9] = (bool)shiftInA[21]; // C:5
    // dirModes[10] = (bool)shiftInA[22]; // C:6
    // dirModes[11] = (bool)shiftInA[23]; // C:7
    // warnButtons[0] = (bool)shiftInA[24]; // D:0
    // warnButtons[1] = (bool)shiftInA[25]; // D:1
    // warnButtons[2] = (bool)shiftInA[26]; // D:2
    // warnButtons[3] = (bool)shiftInA[27]; // D:3
    // warnButtons[4] = (bool)shiftInA[28]; // D:4
    // warnButtons[5] = (bool)shiftInA[29]; // D:5
    // warnButtons[6] = (bool)shiftInA[30]; // D:6
    // warnButtons[7] = (bool)shiftInA[31]; // D:7
    // warnButtons[8] = (bool)shiftInA[32]; // E:0
    // warnButtons[9] = (bool)shiftInA[33]; // E:1
    // agButtons[0] = (bool)shiftInA[34]; // E:2
    // agButtons[1] = (bool)shiftInA[35]; // E:3
    // agButtons[2] = (bool)shiftInA[36]; // E:4
    // agButtons[3] = (bool)shiftInA[37]; // E:5
    // agButtons[4] = (bool)shiftInA[38]; // E:6
    // agButtons[5] = (bool)shiftInA[39]; // E:7
    // agButtons[6] = (bool)shiftInA[40]; // F:0
    // agButtons[7] = (bool)shiftInA[41]; // F:1
    // agButtons[8] = (bool)shiftInA[42]; // F:2
    // agButtons[9] = (bool)shiftInA[43]; // F:3
    // dockingModeSwitch = (bool)shiftInA[44]; // F:4
    // percisionModeSwitch = (bool)shiftInA[45]; // F:5
    // gearSwitch = (bool)shiftInA[46]; // F:6
    // lightsSwitch = (bool)shiftInA[47]; // F:7
    // brakeSwitch = (bool)shiftInA[48]; // G:0
    // sasSwitch = (bool)shiftInA[49]; // G:1
    // rcsSwitch = (bool)shiftInA[50]; // G:2
    // throttleLockSwitch = (bool)shiftInA[51]; // G:3
    // setTrimTranslationButton = (bool)shiftInA[52]; // G:4
    // resetTrimTranslationButton = (bool)shiftInA[53]; // G:5
    // setTrimRotationButton = (bool)shiftInA[54]; // G:6
    // resetTrimRotationButton = (bool)shiftInA[55]; // G:7
    // sasButtons[0] = (bool)shiftInA[56]; // H:0
    // sasButtons[1] = (bool)shiftInA[57]; // H:1
    // sasButtons[2] = (bool)shiftInA[58]; // H:2
    // sasButtons[3] = (bool)shiftInA[59]; // H:3
    // sasButtons[4] = (bool)shiftInA[60]; // H:4
    // sasButtons[5] = (bool)shiftInA[61]; // H:5
    // sasButtons[6] = (bool)shiftInA[62]; // H:6
    // sasButtons[7] = (bool)shiftInA[63]; // H:7
    // // Shift registers in B
    // sasButtons[8] = (bool)shiftInB[0]; // A:0
    // sasButtons[9] = (bool)shiftInB[1]; // A:1
    // warpLockSwitch = (bool)shiftInB[2]; // A:2
    // pauseButton = (bool)shiftInB[3]; // A:3
    // cancelWarpButton = (bool)shiftInB[4]; // A:4
    // enablePhysWarpSwitch = (bool)shiftInB[5]; // A:5
    // decreaseWarpButton = (bool)shiftInB[6]; // A:6
    // increaseWarpButton = (bool)shiftInB[7]; // A:7
    // extraButton2 = (bool)shiftInB[8]; // B:0
    // cycleFocusButton = (bool)shiftInB[9]; // B:1
    // hideUIButton = (bool)shiftInB[10]; // B:2
    // screenshotButton = (bool)shiftInB[11]; // B:3
    // mapFlightSwitch = (bool)shiftInB[12]; // B:4
    // extIvaSwitch = (bool)shiftInB[13]; // B:5
    // cycleCamModeButton = (bool)shiftInB[14]; // B:6
    // resetCamButton = (bool)shiftInB[15]; // B:7
    // // More values (non-shift register)

}

// void getShiftIn(int enableA, int dataA, int clockA, int latchA)
// {
    
//     // Pulse to A
//     digitalWrite(clockA, HIGH);
//     digitalWrite(latchA, LOW);
//     delayMicroseconds(5);
//     digitalWrite(latchA, HIGH);
//     delayMicroseconds(5);
//     byte inputA[4];
//     // Get input A data
    
//     digitalWrite(enableA, LOW);
//     inputA[0] = shiftIn(dataA, clockA, MSBFIRST);
//     inputA[1] = shiftIn(dataA, clockA, MSBFIRST);
//     inputA[2] = shiftIn(dataA, clockA, MSBFIRST);
//     inputA[3] = shiftIn(dataA, clockA, MSBFIRST);
//     mySimpit.printToKSP("InputA : 0 thru 3");
//     mySimpit.printToKSP(String((int)inputA[0]));
//     mySimpit.printToKSP(String((int)inputA[1]));
//     mySimpit.printToKSP(String((int)inputA[2]));
//     mySimpit.printToKSP(String((int)inputA[3]));
//     // inputA[4] = shiftIn(dataA, clockA, MSBFIRST);
//     // inputA[5] = shiftIn(dataA, clockA, MSBFIRST);
//     // inputA[6] = shiftIn(dataA, clockA, MSBFIRST);
//     // inputA[7] = shiftIn(dataA, clockA, MSBFIRST);
//     digitalWrite(enableA, HIGH);
    
//     for (int i = 0; i < 16; i++)
//     {
//         if (i < 8){
//             if (1 == bitRead(inputA[0], i)){
//               shiftInA[i] = 1;
//             }
                
//         // } else if (i < 16){
//         //     if (1 == bitRead(inputA[1], i - 8)){
//         //       shiftInA[i] = 1;
//         //     }
                
//         // } else if (i < 24){
//         //     if (1 == bitRead(inputA[2], i - 16)){
//         //       shiftInA[i] = 1;
//         //     }

//         } else {
//             if (1 == bitRead(inputA[1], i - 8)){
//               shiftInA[i] = 1;
//             }

//         }
            
        
            
        
            
                
//         // else if (i < 32)
//         //     if (1 == bitRead(inputA[3], i - 24))
//         //         shiftInA[i] = 1;
//         // else if (i < 40)
//         //     if (1 == bitRead(inputA[4], i - 32))
//         //         shiftInA[i] = 1;
//         // else if (i < 48)
//         //     if (1 == bitRead(inputA[5], i - 40))
//         //         shiftInA[i] = 1;
//         // else if (i < 56)
//         //     if (1 == bitRead(inputA[6], i - 48))
//         //         shiftInA[i] = 1;
        
            
                
//     }
   
// }

// void checkSASButtons()
// {   
//     // if(changeState == false){
//     //   for (int i = 0; i < 10; i++){
//     //     sasButtonOld[i] = sasButtons[i];
//     //     sasButtonChange[i] = 0;
//     //   }
//     //   changeState = true;
//     // }

//     for (int i = 0; i < 10; i++){
//       if((sasButtons[i] != sasButtonOld[i]) && sasButtons[i]){
//         sasButtonChange[i] = true;
//         sasButtonOld[i] = sasButtons[i];
//       } else {
//         sasButtonChange[i] = false;
//         sasButtonOld[i] = sasButtons[i];
//       }
//     }
    
//     // sasButtonChange[0] = !(sasButtonOld[0] == sasButtons[0]);
//     // sasButtonChange[1] = !(sasButtonOld[1] == sasButtons[1]);
//     // sasButtonChange[2] = !(sasButtonOld[2] == sasButtons[2]);
//     // sasButtonChange[3] = !(sasButtonOld[3] == sasButtons[3]);
//     // sasButtonChange[4] = !(sasButtonOld[4] == sasButtons[4]);
//     // sasButtonChange[5] = !(sasButtonOld[5] == sasButtons[5]);
//     // sasButtonChange[6] = !(sasButtonOld[6] == sasButtons[6]);
//     // sasButtonChange[7] = !(sasButtonOld[7] == sasButtons[7]);
//     // sasButtonChange[8] = !(sasButtonOld[8] == sasButtons[8]);
//     // sasButtonChange[9] = !(sasButtonOld[9] == sasButtons[9]);
    
// }
