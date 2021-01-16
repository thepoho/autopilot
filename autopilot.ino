// TODO
// Alt Sync
// HDG Sync
  // Need to use on/off events for heading in order to not capture the current hdg
// Turn hdg mode off when nav mode pressed
// Fix up vspeed display
// fix up speed set circular get

#include "TM1637_6D.h"
#include "TM1637.h"

// https://github.com/buxtronix/arduino/tree/master/libraries/Rotary
#include "poho_rotary.h"

#define FAST_SPIN_DIFF 350
#define FAST_SPIN_SAMPLES 8

#define TOGGLE_INTERVAL 50

#define NumberOfInputs 24
#define NumberOfOutputs 16 // If this changes, need to update the doShiftOut() method

#define altSegDIO 49
#define altSegCLK 48

#define hdgSegDIO 47
#define hdgSegCLK 46

#define flcSegDIO 45
#define flcSegCLK 44

#define vsSegDIO 43
#define vsSegCLK 42

#define ShiftOutClockPin 38
#define ShiftOutLatchPin 39
#define ShiftOutDataPin  41

#define ShiftInPLoadPin       35
#define ShiftInClockEnablePin 37
#define ShiftInClockPin       36
#define ShiftInDataPin        40

#define pi 3.14159

PohoRotary *rotaryAlt;
PohoRotary *rotaryHdg;
PohoRotary *rotaryFlc;
PohoRotary *rotaryVs;

TM1637_6D altDisplay(altSegCLK,altSegDIO);
TM1637    hdgDisplay(hdgSegCLK,hdgSegDIO);
TM1637    flcDisplay(flcSegCLK,flcSegDIO);
TM1637_6D vsDisplay(vsSegCLK, vsSegDIO);

enum outputIndex  {
  out_nav  = 0,
  out_appr = 1,
  out_yd   = 2,
  out_ap   = 3,
  out_flc  = 4,
  out_vs   = 5,
  out_hdg  = 6,
  out_alt  = 7,

  out_vnav = 15,


  out_unused0 = 9,
  out_unused1 = 10,
  out_unused2 = 11,
  out_unused3 = 12,
  out_unused4 = 13,
  out_unused5 = 14,
  out_unused6 = 8
};

enum inputIndex{
  in_alt           = 7,
  in_altRotatePush = 6,
  in_altRotate1    = 4,
  in_altRotate2    = 5,
  in_hdg           = 3,
  in_hdgRotatePush = 2,
  in_hdgRotate1    = 0,
  in_hdgRotate2    = 1,
  in_flc           = 15,
  in_flcRotatePush = 14,
  in_flcRotate1    = 12,
  in_flcRotate2    = 13,
  in_vs            = 11,
  in_vsRotatePush  = 10,
  in_vsRotate1     = 8,
  in_vsRotate2     = 9,
  in_na0,
  in_na1,
  in_na2,
  in_na3,
  in_appr          = 20,
  in_ap            = 21,
  in_nav           = 22,
  in_yd            = 23
};

bool outputValues[NumberOfOutputs];

int inputValues[NumberOfInputs];
int oldInputValues[NumberOfInputs];

unsigned long lastSend[NumberOfInputs];

#define REQUEST_SOME_INTERVAL 3000
unsigned long lastRequestSome = 0;
unsigned long nextSendResetTime = 0;

int hdgState, hdgLastState;
int altState, altLastState;
int flcState, flcLastState;
int vsState, vsLastState;

long altBug = 0;
int  hdgBug = 1;
int  flcBug = 1;
int  vsBug  = 0;

unsigned long altBugLastSent = 0;

bool altSyncing = false;
bool hdgSyncing = false;

bool altDirty = false;
bool hdgDirty = false;
bool flcDirty = false;
bool vsDirty  = false;

unsigned long hdgChanges[FAST_SPIN_SAMPLES] = {0};
unsigned long altChanges[FAST_SPIN_SAMPLES] = {0};
unsigned long flcChanges[FAST_SPIN_SAMPLES] = {0};
unsigned long  vsChanges[FAST_SPIN_SAMPLES] = {0};

unsigned long lastSystemSendMillis = 0;
enum
{
  MsgId_Ok        = 0,
  MsgId_Sim       = 1,
  MsgId_DebugMsg  = 2,
  MsgId_GetAll    = 3,
  MessageId_Reset = 4,
};

bool syncingHdg = false;

bool okToSend = false;

void(* resetFunc) (void) = 0;

void setup() {
  setupShiftOut();
  setupShiftIn();

  rotaryAlt = new PohoRotary();
  rotaryHdg = new PohoRotary();
  rotaryFlc = new PohoRotary();
  rotaryVs  = new PohoRotary();

  altDisplay.init();
  altDisplay.set(BRIGHT_TYPICAL);//BRIGHT_TYPICAL = 2,BRIGHT_DARKEST = 0,BRIGHTEST = 7;

  hdgDisplay.begin();
  hdgDisplay.setBrightness(2);

  flcDisplay.begin();
  flcDisplay.setBrightness(2);

  vsDisplay.init();
  vsDisplay.set(BRIGHT_TYPICAL);

  for(int i =0; i < NumberOfOutputs; i++){
    outputValues[i] = 0;
  }

  Serial.begin(115200);

  displayAlt();
  displayHdg();
  displayFlc();
  displayVs();
}

void setupComms(){
  //8 button pushes
  SendString(MsgId_Sim,"RGK;AP_MASTER");
  SendString(MsgId_Sim,"RGK;YAW_DAMPER_TOGGLE");
  SendString(MsgId_Sim,"RGK;AP_NAV1_HOLD");
  SendString(MsgId_Sim,"RGK;AP_APR_HOLD");

  SendString(MsgId_Sim,"RGK;AP_ALT_HOLD"); //AP_PANEL_ALTITUDE_HOLD
  SendString(MsgId_Sim,"RGK;AP_HDG_HOLD"); //AP_PANEL_HEADING_HOLD
  SendString(MsgId_Sim,"RGK;AP_PANEL_SPEED_HOLD_TOGGLE"); //AP_PANEL_SPEED_HOLD
  SendString(MsgId_Sim,"RGK;AP_PANEL_VS_HOLD"); //AP_PANEL_SPEED_HOLD

  //Events from the sim. We usually set the first one in each group
  SendString(MsgId_Sim,"RGK;AP_ALT_VAR_SET_ENGLISH");
  SendString(MsgId_Sim,"RGK;AP_ALT_VAR_INC");
  SendString(MsgId_Sim,"RGK;AP_ALT_VAR_DEC");

  SendString(MsgId_Sim,"RGK;AP_SPD_VAR_SET");
  SendString(MsgId_Sim,"RGK;AP_SPD_VAR_INC");
  SendString(MsgId_Sim,"RGK;AP_SPD_VAR_DEC");

  SendString(MsgId_Sim,"RGK;AP_VS_VAR_SET_ENGLISH");
  SendString(MsgId_Sim,"RGK;AP_VS_VAR_INC");
  SendString(MsgId_Sim,"RGK;AP_VS_VAR_DEC");

  SendString(MsgId_Sim,"RGK;HEADING_BUG_SET");
  SendString(MsgId_Sim,"RGK;HEADING_BUG_INC");
  SendString(MsgId_Sim,"RGK;HEADING_BUG_DEC");


  // // TODO  encoder pushes

  // // // Things to get
  SendString(MsgId_Sim,"RGO;AUTOPILOT ALTITUDE LOCK;bool;DOUBLE");
  SendString(MsgId_Sim,"RGO;AUTOPILOT ALTITUDE LOCK VAR;feet;DOUBLE");

  SendString(MsgId_Sim,"RGO;AUTOPILOT HEADING LOCK;bool;DOUBLE");
  SendString(MsgId_Sim,"RGO;AUTOPILOT HEADING LOCK DIR;degrees;DOUBLE");

  SendString(MsgId_Sim,"RGO;AUTOPILOT AIRSPEED HOLD;bool;DOUBLE");
  SendString(MsgId_Sim,"RGO;AUTOPILOT AIRSPEED HOLD VAR;knots;DOUBLE");

  SendString(MsgId_Sim,"RGO;AUTOPILOT VERTICAL HOLD;bool;DOUBLE");
  SendString(MsgId_Sim,"RGO;AUTOPILOT VERTICAL HOLD VAR;feet/minute;DOUBLE");

  SendString(MsgId_Sim,"RGO;AUTOPILOT MASTER;bool;DOUBLE");
  SendString(MsgId_Sim,"RGO;AUTOPILOT YAW DAMPER;bool;DOUBLE");
  SendString(MsgId_Sim,"RGO;AUTOPILOT APPROACH HOLD;bool;DOUBLE");
  SendString(MsgId_Sim,"RGO;AUTOPILOT NAV1 LOCK;bool;DOUBLE");

  SendString(MsgId_Sim,"RGO;PLANE ALTITUDE;feet;DOUBLE");
  SendString(MsgId_Sim,"RGO;HEADING INDICATOR;radians;DOUBLE");
}

void setupShiftOut(){
  pinMode(ShiftOutClockPin, OUTPUT);
  pinMode(ShiftOutLatchPin, OUTPUT);
  pinMode(ShiftOutDataPin,  OUTPUT);
  pinMode(13,  OUTPUT);
}

void setupShiftIn(){
  pinMode(ShiftInPLoadPin,       OUTPUT);
  pinMode(ShiftInClockEnablePin, OUTPUT);
  pinMode(ShiftInClockPin,       OUTPUT);
  pinMode(ShiftInDataPin,        INPUT);

  digitalWrite(ShiftInClockPin, LOW);
  digitalWrite(ShiftInPLoadPin, HIGH);
}

//http://www.cplusplus.com/forum/beginner/197890/#msg949227
long roundNearestHundred(long d){
  long d_i = d;
  return ((d_i % 100) < 50) ? d_i - (d_i % 100) : d_i + (100 - (d_i % 100));
}

void rotateAlt() {
  unsigned char result = rotaryAlt->process(inputValues[in_altRotate1],inputValues[in_altRotate2]);

  int step = 100;

  if (result == DIR_CW || result == DIR_CCW){
    int fss = FAST_SPIN_SAMPLES-1;
    for(int i=0; i<fss; i++){
      altChanges[i] = altChanges[i+1];
    }
    altChanges[fss] = millis();

    if((altChanges[fss] - altChanges[0]) < FAST_SPIN_DIFF){
      step = 1000;
    }

    if (result == DIR_CW) {
      altBug += step;
      displayAlt();
      altDirty = true;
    } else if (result == DIR_CCW) {
      altBug -= step;
      displayAlt();
      altDirty = true;
    }
  }
}

void displayAlt(){
  String num = "";
  // Serial.println(altBug);
  // SendDebugMessage(String(altBug));
  if(altBug <= -10000)
    num = ("" +String(altBug));
  else if(altBug <= -1000)
    num = (" " +String(altBug));
  else if(altBug < 0)
    num = ("  " +String(altBug));
  else if(altBug == 0)
    num = ("   00" +String(altBug));
  else if(altBug < 1000)
    num = ("   "+String(altBug));
  else if(altBug < 10000)
    num = ("  "+String(altBug));
  else if(altBug < 100000)
    num = (" "+String(altBug));
  else
    num = (String(altBug));

  int8_t ListDisp[6];
  for(int i=0; i < 6; i++){
    if(num.charAt(i) == ' ')
      ListDisp[5-i] = 10; //blank
    else
      ListDisp[5-i] = num.charAt(i) - '0';
  }
  int8_t ListDispPoint[6] = {POINT_OFF,POINT_OFF,POINT_OFF,POINT_OFF,POINT_OFF,POINT_OFF};

  altDisplay.display(ListDisp, ListDispPoint);
}

void sendAlt(){
  altBugLastSent = millis();
  String str = "SET;AP_ALT_VAR_SET_ENGLISH;"+String(altBug);
  SendString(MsgId_Sim, str);
}

void rotateHdg() {
  unsigned char result = rotaryHdg->process(inputValues[in_hdgRotate1],inputValues[in_hdgRotate2]);
  int step = 1;

  bool hdgChanged = false;

  if (result == DIR_CW || result == DIR_CCW){
    int fss = FAST_SPIN_SAMPLES-1;
    for(int i=0; i<fss; i++){
      hdgChanges[i] = hdgChanges[i+1];
    }
    hdgChanges[fss] = millis();

    if((hdgChanges[fss] - hdgChanges[0]) < FAST_SPIN_DIFF){
      step = 10;
    }

    if (result == DIR_CW) {
      hdgBug += step;
      hdgChanged = true;
    } else if (result == DIR_CCW) {
      hdgBug -= step;
      hdgChanged = true;
    }
    if(hdgChanged){
      if(hdgBug <= 0)
        hdgBug = 360;
      if(hdgBug > 360)
        hdgBug = 1;
      hdgDirty = true;
      displayHdg();
    }
  }
}

void displayHdg(){
  String num = "";
  if(hdgBug < 10)
    num = " 00" + String(hdgBug);
  else if(hdgBug < 100)
    num = " 0" + String(hdgBug);
  else
    num = " " + String(hdgBug);

  hdgDisplay.display(num);
}

void sendHdg(){
  String str = "SET;HEADING_BUG_SET;"+String(hdgBug);
  SendString(MsgId_Sim, str);
}

void rotateFlc() {
  unsigned char result = rotaryFlc->process(inputValues[in_flcRotate1],inputValues[in_flcRotate2]);
  int step = 1;

  bool flcChanged = false;

  if (result == DIR_CW || result == DIR_CCW){
    int fss = FAST_SPIN_SAMPLES-1;
    for(int i=0; i<fss; i++){
      flcChanges[i] = flcChanges[i+1];
    }
    flcChanges[fss] = millis();

    if((flcChanges[fss] - flcChanges[0]) < FAST_SPIN_DIFF){
      step = 10;
    }

    if (result == DIR_CW) {
      flcBug += step;
      flcChanged = true;
    } else if (result == DIR_CCW) {
      flcBug -= step;
      flcChanged = true;
    }
    if(flcChanged){
      flcDirty = true;
      displayFlc();
    }
  }
}

void displayFlc(){
  String num = "";
  if(flcBug < 10)
    num = " 00" + String(flcBug);
  else if(flcBug < 100)
    num = " 0" + String(flcBug);
  else
    num = " " + String(flcBug);

  flcDisplay.display(num);
}

void sendFlc(){
  String str = "SET;AP_SPD_VAR_SET;"+String(flcBug);
  SendString(MsgId_Sim, str);
}


void rotateVs() {
  unsigned char result = rotaryVs->process(inputValues[in_vsRotate1],inputValues[in_vsRotate2]);
  int step = 100;

  if (result == DIR_CW || result == DIR_CCW){
    int fss = FAST_SPIN_SAMPLES-1;
    for(int i=0; i<fss; i++){
      vsChanges[i] = vsChanges[i+1];
    }
    vsChanges[fss] = millis();

    if (result == DIR_CW) {
      vsBug += step;
      vsDirty = true;
      displayVs();
    } else if (result == DIR_CCW) {
      vsBug -= step;
      vsDirty = true;
      displayVs();
    }
  }
}

void displayVs(){
  String num = "";

  if(vsBug <= -10000)
    num = ("" +String(vsBug));
  else if(vsBug <= -1000)
    num = (" " +String(vsBug));
  else if(vsBug < 0)
    num = ("  " +String(vsBug));
  else if(vsBug == 0)
    num = ("   00" +String(vsBug));
  else if(vsBug < 1000)
    num = ("   "+String(vsBug));
  else if(vsBug < 10000)
    num = ("  "+String(vsBug));
  else if(vsBug < 100000)
    num = (" "+String(vsBug));
  else
    num = (String(vsBug));

  int8_t ListDisp[6];
  for(int i=0; i < 6; i++){
    if(num.charAt(i) == ' ')
      ListDisp[5-i] = 10; //blank
    else
      ListDisp[5-i] = num.charAt(i) - '0';
  }
  int8_t ListDispPoint[6] = {POINT_OFF,POINT_OFF,POINT_OFF,POINT_OFF,POINT_OFF,POINT_OFF};

  vsDisplay.display(ListDisp, ListDispPoint);
}

void sendVs(){
  String str = "SET;AP_VS_VAR_SET_ENGLISH;"+String(vsBug);
  SendString(MsgId_Sim, str);
}

char tmp = 0;

void doShiftOut(){
  int output = 0;
  for(int i=0; i < NumberOfOutputs; i++){
    if(outputValues[i]){
      output |= (1 << i);
    }
  }
  digitalWrite(ShiftOutLatchPin, LOW);
  shiftOut(ShiftOutDataPin, ShiftOutClockPin, MSBFIRST, output >> 8);
  shiftOut(ShiftOutDataPin, ShiftOutClockPin, MSBFIRST, output);
  // shiftOut(ShiftOutDataPin, ShiftOutClockPin, MSBFIRST, output);
  digitalWrite(ShiftOutLatchPin, HIGH);
}

bool doShiftIn()
{
  digitalWrite(ShiftInClockEnablePin, HIGH);
  digitalWrite(ShiftInPLoadPin, LOW);
  delayMicroseconds(5);
  digitalWrite(ShiftInPLoadPin, HIGH);
  digitalWrite(ShiftInClockEnablePin, LOW);

  /* Loop to read each bit value from the serial out line
  * of the SN74HC165N.
  */
  bool dirty = false;
  for(int i = 0; i < NumberOfInputs; i++)
  {
    oldInputValues[i] = inputValues[i];
    int tmp = digitalRead(ShiftInDataPin);
    if(tmp != inputValues[i]){
      dirty = true;
    }

    // Inputs actually are sitting high and toggle low when pressed.
    // reverse this around for less confusion when checking elsewhere
    if(tmp == 0)
      inputValues[i] = 1;
    else
      inputValues[i] = 0;

    digitalWrite(ShiftInClockPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(ShiftInClockPin, LOW);
  }
  return dirty;
}

bool buttonWasPressed(inputIndex idx){
  if(lastSend[idx] < (millis() - TOGGLE_INTERVAL)){
    if(inputValues[idx] == 1 && oldInputValues[idx] == 0){
      lastSend[idx] = millis();
      return true;
    }
  }
  return false;
}

void checkApMaster(){
  if(buttonWasPressed(in_ap)){
    SendString(MsgId_Sim, "SET;AP_MASTER;0");
    SendString(MsgId_Sim, "GET;AUTOPILOT MASTER");
  }
}

void checkYawDamper(){
  if(buttonWasPressed(in_yd)){
    SendString(MsgId_Sim, "SET;YAW_DAMPER_TOGGLE;0");
    SendString(MsgId_Sim, "GET;AUTOPILOT YAW DAMPER");
  }
}

void checkNavHold(){
  if(buttonWasPressed(in_nav)){
    SendString(MsgId_Sim, "SET;AP_NAV1_HOLD;0");;
    SendString(MsgId_Sim, "GET;AUTOPILOT NAV1 LOCK");
  }
}

void checkApprHold(){
  if(buttonWasPressed(in_appr)){
    SendString(MsgId_Sim, "SET;AP_APR_HOLD;0");
    SendString(MsgId_Sim, "GET;AUTOPILOT APPROACH HOLD");
  }
}

void checkAlt(){
  if(buttonWasPressed(in_alt)){
    SendString(MsgId_Sim, "SET;AP_ALT_HOLD;0");
    SendString(MsgId_Sim, "GET;AUTOPILOT ALTITUDE LOCK");
  }
}

void checkHdg(){
  if(buttonWasPressed(in_hdg)){
    SendString(MsgId_Sim, "SET;AP_HDG_HOLD;0");
    SendString(MsgId_Sim, "GET;AUTOPILOT HEADING LOCK");
  }
}

void checkFlc(){
  if(buttonWasPressed(in_flc)){
    SendString(MsgId_Sim, "SET;AP_PANEL_SPEED_HOLD_TOGGLE;0");
    SendString(MsgId_Sim, "GET;AUTOPILOT AIRSPEED HOLD");
  }
}

void checkVs(){
  if(buttonWasPressed(in_vs)){
    SendString(MsgId_Sim, "SET;AP_PANEL_VS_HOLD;0");
    SendString(MsgId_Sim, "GET;AUTOPILOT VERTICAL HOLD");
    sendVs();
  }
}

void checkForReqAll(){
  if(inputValues[in_vsRotatePush] == 1){
    requestAll();
    lastSystemSendMillis = millis();
  }
}

void checkForHdgSync(){
  if(buttonWasPressed(in_hdgRotatePush) == 1){
    // TODO: Get the heading, set the heading
    // SendString(MsgId_Sim, "GET;AUTOPILOT HEADING LOCK DIR");
  }
}

void checkForAltSync(){
  if(buttonWasPressed(in_altRotatePush) == 1){
    altSyncing = true;
    SendString(MsgId_Sim, "GET;PLANE ALTITUDE");
  }
}

void loop() {
  delay(1);

  for(int i =0; i < NumberOfOutputs; i++){
    outputValues[i] = 0;
  }
  if(hdgBug < NumberOfOutputs){
    outputValues[hdgBug] = 1;
  }else{
    outputValues[0] = 1;
  }

  doShiftOut();
  bool inputDirty = doShiftIn();

  if(inputDirty){
    rotateAlt();
    rotateHdg();
    rotateFlc();
    rotateVs();

    checkApMaster();
    checkYawDamper();
    checkNavHold();
    checkApprHold();
    checkAlt();
    checkHdg();
    checkFlc();
    checkVs();

    checkForHdgSync();
    checkForAltSync();

    if(millis() > (lastSystemSendMillis + REQUEST_SOME_INTERVAL)){
      checkForReqAll();
    }

    if(outputValues[out_vs] || outputValues[out_flc]){
      if(millis() > lastRequestSome + REQUEST_SOME_INTERVAL){
        requestSome();
      }
    }
  }

  if(altDirty){
    altDirty = false;
    sendAlt();
  }
  if(hdgDirty){
    hdgDirty = false;
    sendHdg();
  }
  if(flcDirty){
    flcDirty = false;
    sendFlc();
  }
  if(vsDirty){
    vsDirty = false;
    sendVs();
  }

  if(okToSend){
    readFromSerial();
  }else{
    if(millis() > nextSendResetTime){
      SendString(MessageId_Reset, ";;");
      readFromSerial();
      nextSendResetTime = millis() + 1000;
    }
  }
}

String byteToString(byte var){
  String ret = "";
  for(int i=0; i < 8; i++){
    if(var & (1 << 8-i)){
      ret = ret + "1";
    }else{
      ret = ret + "0";
    }
  }
  return ret;
}

long roundAlt(long in){
  int mod = in % 100;
  long out = in / 100;
  out = out * 100;
  if(mod > 50){
    out += 100;
  }
  return(out);
}

void readFromSerial(){
  if (Serial.available() > 0){
    byte msgLen[2];
    Serial.readBytes(msgLen, 2);
    int len = msgLen[1] << 8 | msgLen[0];

    byte buff[len];

    Serial.readBytes(buff,len);

    int msgId = buff[1] << 8 | buff[0];

    switch(msgId)
    {
      case MsgId_Ok:
        okToSend = true;
        setupComms();
        requestAll();
        break;
      case MsgId_Sim:
        char *str = (char*)&buff[2];
        char *ptr = NULL;
        char *strings[5];
        int index = 0;
        ptr = strtok(str, ";");  // takes a list of delimiters
        while(ptr != NULL)
        {
          strings[index] = ptr;
          index++;
          ptr = strtok(NULL, ";");  // takes a list of delimiters
        }

        if(strcmp("SET", strings[0]) == 0){
          if(strcmp("AUTOPILOT MASTER", strings[1]) == 0){
            outputValues[out_ap] = setOutputValueFromSerial(strings[2]);
          }else if(strcmp("AUTOPILOT YAW DAMPER", strings[1]) == 0){
            outputValues[out_yd] = setOutputValueFromSerial(strings[2]);
          }else if(strcmp("AUTOPILOT APPROACH HOLD", strings[1]) == 0){
            outputValues[out_appr] = setOutputValueFromSerial(strings[2]);
          }else if(strcmp("AUTOPILOT NAV1 LOCK", strings[1]) == 0){
            outputValues[out_nav] = setOutputValueFromSerial(strings[2]);
          }else if(strcmp("AUTOPILOT ALTITUDE LOCK", strings[1]) == 0){
            outputValues[out_alt] = setOutputValueFromSerial(strings[2]);
            SendString(MsgId_Sim, "GET;AUTOPILOT VERTICAL HOLD VAR");
          }else if(strcmp("AUTOPILOT HEADING LOCK", strings[1]) == 0){
            outputValues[out_hdg] = setOutputValueFromSerial(strings[2]);
          }else if(strcmp("AUTOPILOT AIRSPEED HOLD", strings[1]) == 0){
            outputValues[out_flc] = setOutputValueFromSerial(strings[2]);
          }else if(strcmp("AUTOPILOT VERTICAL HOLD", strings[1]) == 0){
            outputValues[out_vs] = setOutputValueFromSerial(strings[2]);
          }else if(strcmp("AUTOPILOT ALTITUDE LOCK VAR", strings[1]) == 0){
            if(altBugLastSent + 100 < millis()){
              altBug = (atol(strings[2])); //use atol (l for long)
              displayAlt();
            }
          }else if(strcmp("AUTOPILOT HEADING LOCK DIR", strings[1]) == 0){
            hdgBug = atoi(strings[2]);
            displayHdg();
          }else if(strcmp("AUTOPILOT AIRSPEED HOLD VAR", strings[1]) == 0){
            flcBug = atoi(strings[2]);
            displayFlc();
          }else if(strcmp("AUTOPILOT VERTICAL HOLD VAR", strings[1]) == 0){
            vsBug = atoi(strings[2]);
            displayVs();
          }else if(strcmp("PLANE ALTITUDE", strings[1]) == 0){
            if(altSyncing){
              altSyncing = false;
              altBug = roundAlt(atol(strings[2]));
              displayAlt();
              sendAlt();
            }
          }else{
            // An event
            if(strcmp("AP_MASTER", strings[1]) == 0){
              SendString(MsgId_Sim, "GET;AUTOPILOT MASTER");

            }else if(strcmp("YAW_DAMPER_TOGGLE", strings[1]) == 0){
              SendString(MsgId_Sim, "GET;AUTOPILOT YAW DAMPER");

            }else if(strcmp("AP_APR_HOLD", strings[1]) == 0){
              SendString(MsgId_Sim, "GET;AUTOPILOT APPROACH HOLD");
              SendString(MsgId_Sim, "GET;AUTOPILOT HEADING LOCK");
              SendString(MsgId_Sim, "GET;AUTOPILOT NAV1 LOCK");

            }else if(strcmp("AP_NAV1_HOLD", strings[1]) == 0){
              SendString(MsgId_Sim, "GET;AUTOPILOT HEADING LOCK");
              SendString(MsgId_Sim, "GET;AUTOPILOT HEADING LOCK DIR");
              SendString(MsgId_Sim, "GET;AUTOPILOT NAV1 LOCK");
              SendString(MsgId_Sim, "GET;AUTOPILOT APPROACH HOLD");

            }else if(strcmp("AP_HDG_HOLD", strings[1]) == 0){
              SendString(MsgId_Sim, "GET;AUTOPILOT HEADING LOCK");
              SendString(MsgId_Sim, "GET;AUTOPILOT NAV1 LOCK");
              SendString(MsgId_Sim, "GET;AUTOPILOT APPROACH HOLD");
              SendString(MsgId_Sim, "GET;AUTOPILOT HEADING LOCK DIR");

            }else if(strcmp("AP_PANEL_VS_HOLD", strings[1]) == 0){
              SendString(MsgId_Sim, "GET;AUTOPILOT VERTICAL HOLD");
              SendString(MsgId_Sim, "GET;AUTOPILOT ALTITUDE LOCK");

            }else if(strcmp("AP_ALT_HOLD", strings[1]) == 0){
              SendString(MsgId_Sim, "GET;AUTOPILOT VERTICAL HOLD");
              SendString(MsgId_Sim, "GET;AUTOPILOT ALTITUDE LOCK");
              SendString(MsgId_Sim, "GET;AUTOPILOT ALTITUDE LOCK VAR");

            }else if(strcmp("AP_ALT_VAR_INC", strings[1]) == 0 ||
                     strcmp("AP_ALT_VAR_DEC", strings[1]) == 0 ||
                     strcmp("AP_ALT_VAR_SET_ENGLISH", strings[1]) == 0){
              SendString(MsgId_Sim, "GET;AUTOPILOT ALTITUDE LOCK VAR");

            }else if(strcmp("AP_SPD_VAR_INC", strings[1]) == 0 ||
                     strcmp("AP_SPD_VAR_DEC", strings[1]) == 0 ||
                     strcmp("AP_SPD_VAR_SET", strings[1]) == 0){
              SendString(MsgId_Sim, "GET;AUTOPILOT AIRSPEED HOLD VAR");

            }else if(strcmp("AP_VS_VAR_INC", strings[1]) == 0 ||
                     strcmp("AP_VS_VAR_DEC", strings[1]) == 0 ||
                     strcmp("AP_VS_VAR_SET", strings[1]) == 0){
              SendString(MsgId_Sim, "GET;AUTOPILOT VERTICAL HOLD VAR");

            }else if(strcmp("HEADING_BUG_VAR_INC", strings[1]) == 0 ||
                     strcmp("HEADING_BUG_VAR_DEC", strings[1]) == 0 ||
                     strcmp("HEADING_BUG_SET", strings[1]) == 0){
              SendString(MsgId_Sim, "GET;AUTOPILOT HEADING LOCK DIR");
            }
          }
        }
      // Gap coz of switch above
    }
  }
}

void requestAll(){
  SendString(MsgId_GetAll, ";;");
}

void requestSome(){
  lastRequestSome = millis();
  SendString(MsgId_Sim, "GET;AUTOPILOT VERTICAL HOLD");
  SendString(MsgId_Sim, "GET;AUTOPILOT ALTITUDE LOCK");
}

int setOutputValueFromSerial(char *tmp){
  if(strcmp(tmp, "1.00") == 0 || strcmp(tmp, "1") == 0){
    return(1);
  }
  return(0);
}

void SendString(short msgId,String theString)
{
  // delay(100);
  short len     = theString.length();
  short msgLen  = len + 2;
  byte syncByte = '#';

  if(!okToSend && msgId != MessageId_Reset){
    //Don't try to send unless we have comms.
    //Except if we are trying to send a reset msg
    return;
  }

  // Write sync bytes
  Serial.write(syncByte);
  Serial.write(syncByte);

  // Write total size including message id and payload length
  Serial.write((msgLen & 0xFF));
  Serial.write((msgLen>>8 & 0xFF));

  // Write payload message id
  Serial.write((msgId & 0xFF));
  Serial.write((msgId>>8 & 0xFF));

  // Write the payload message
  Serial.println(theString);
}

void SendDebugMessage(String str){
  SendString(MsgId_DebugMsg,str);
}
