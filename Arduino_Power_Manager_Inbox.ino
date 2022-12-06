//Power Manager V1.0

#include <EEPROM.h>
#include "pins_arduino.h"

byte EepromAlreadyConfigure = 199;         // (1 = OK, 255 = new unit)
byte EepromBatterytoMainController = 200;  // (1 = Battery #1, 2 = Battery #2)
byte EepromManualtoMainController = 201;   //(0 = Autorun, 1 = Override)
byte EepromBatterytoWinch = 202;           //(1 = Battery #1, 2 = Battery #2)
byte EepromManualtoWinch = 203;            // (0 = Autorun, 1 = Override)
byte EepromBatterytoTurbine = 204;         //(1 = Battery #1, 2 = Battery #2)
byte EepromManualtoTurbine = 205;          // (0 = Autorun, 1 = Override)
byte EepromTurbineBrakeActivated = 206;    //(0 = Released, 1 = Brake On)

//Relay State
int BatterytoMainController = 1;
int BatterytoTurbine = 1;
int BatterytoWinch = 1;

//Manual Control
boolean ManualtoMainController = 0;
boolean ManualtoWinch = 0;
boolean ManualtoTurbine = 0;
boolean TurbineBrakeActivated = 0;

//Starting
boolean bootUp = 1;



//Analog input

const int WaterDetection = A0;
const int Turbine = A1;
const int PowerManager = A2;
const int Winch = A3;
const int SolarAnalog = A4;
const int AccPower = A5;
const int Resetpin = A6;

int WaterDetectionPercent = 0;


//Digital (0-5V) output

//int SerialMainRx = 0;
//int SerialMainTx = 1;
int Batt_1toMainController = 2;
//int SerialSolarTx = 3;
int Batt_2toMainController = 4;
int SerialTxRx = 5;
//int SerialSolarRx = 6;
int Batt_1toTurbine = 7;
int Batt_2toTurbine = 8;
int Batt_1toWinch = 9;
int Batt_2toWinch = 10;
int TurbineBrake = 11;
int TurbineRelease = 12;
int SerialSolar = A7;


//Variables

//Data entries variables
const byte numChars = 32;
char receivedChars[numChars];  // an array to store the received data
boolean newData = false;

//Timed sequence
unsigned long nextAverageData = 0; //to compare with millis()
unsigned long humidityTimer = 0; //to compare with millis()
boolean closeRelay = false;
unsigned long currentMillis = 0;
int timeBetweenAutoCollect = 30; //sec


//Configurable Menu
boolean needsEntry = false;
boolean allowsReset = false;
boolean manualControl = false;
byte manualControlStep = 0;
boolean autoControl = false;
byte autoControlStep = 0;


//new if Power Data from PV Solar
boolean newVBatt1 = 0;
boolean newABatt1 = 0;
boolean newVBatt2 = 0;
boolean newABatt2 = 0;
boolean newVSolar = 0;
boolean newASolar = 0;
boolean newAMain = 0;

//String to send to Solar Regulator for data
byte Solar[] = { 0xC0, 0xF0, 0x01, 0x00, 0x00, 0xE4, 0x62, 0xC0, 0xFF };
byte Batt1[] = { 0xC0, 0xF0, 0x02, 0x02, 0x00, 0x84, 0x93, 0xC0, 0xFF };
byte Batt2[] = { 0xC0, 0xF0, 0x02, 0x82, 0x00, 0x44, 0xF2, 0xC0, 0xFF };


//Voltage, Amperage and Relay
String WinchState = "not working";
float VBatt1 = 0;
float ABatt1 = 0;
float VBatt2 = 0;
float ABatt2 = 0;
float VSolar = 0;
float ASolar = 0;
float AWinch = 0;
float ATurbine = 0;
float APowerManager = 0;

//To send DV, averaged data
float VBatt1a = 0;
float ABatt1a = 0;
float VBatt2a = 0;
float ABatt2a = 0;
float VSolara = 0;
float ASolara = 0;
float AWincha = 0;
float ATurbinea = 0;
float APowerManagera = 0;


//Tab lenght can be bigger than lenghtOfTab, and must be a constant so the lenghtOfTab really define the average lenght
byte lenghtOfTab = 3;
int averageTab = 0;
boolean longAverage = 0;
float VBatt1Tab[60];
float ABatt1Tab[60];
float VBatt2Tab[60];
float ABatt2Tab[60];
float VSolarTab[60];
float ASolarTab[60];
float ATurbineTab[60];
float APowerManagerTab[60];
float AWinchTab[60];


//Watch, Warning, Alarm and Error

int HumidityFlag = 0; //00 = NoError, 01 = NotPresent, 02 = Over40%, 03 = Over75%, 04 = Over90%



//Scale and Offset for Current Measurement
float PowerManagerScale = 0.0120;
float PowerManagerOffset = -6.4579;
float TurbineScale = 0.01483;
float TurbineOffset = -7.59;
float WinchScale = 0.04648;
float WinchOffset = -23.745;
int navg = 15;

///Démarrage
///
///
///
///

void setup() {
  //réattribuer les pins
  pinMode(Batt_1toMainController, OUTPUT);  //pin2
  pinMode(Batt_2toMainController, OUTPUT);  //pin3
  pinMode(SerialTxRx, OUTPUT);              //pin6
  pinMode(Batt_1toTurbine, OUTPUT);         //pin7
  pinMode(Batt_2toTurbine, OUTPUT);         //pin8
  pinMode(Batt_1toWinch, OUTPUT);           //pin9
  pinMode(Batt_2toWinch, OUTPUT);           //pin10
  pinMode(TurbineBrake, OUTPUT);            //pin11
  pinMode(TurbineRelease, OUTPUT);          //pin12
  pinMode(AccPower, OUTPUT);                //pinA5

  pinMode(SerialSolar, OUTPUT);             //pinA7

  //Reset must have 5v, otherwise is resetting
  pinMode(Resetpin, OUTPUT);                //pinA6
  digitalWrite(Resetpin, HIGH);


  Serial1.begin(19200);
  digitalWrite(SerialTxRx, HIGH);
  Serial1.println();
  Serial1.println();
  Serial1.println("Powering Arduino");
  Serial1.flush();

  //Eeprom will be 255 if never uploaded with this code
  if (EEPROM.read(EepromAlreadyConfigure) == 255) {
    EEPROM.write(EepromBatterytoMainController, BatterytoMainController);
    EEPROM.write(EepromManualtoMainController, ManualtoMainController);
    EEPROM.write(EepromBatterytoWinch, BatterytoWinch);
    EEPROM.write(EepromManualtoWinch, ManualtoWinch);
    EEPROM.write(EepromBatterytoTurbine, BatterytoTurbine);
    EEPROM.write(EepromManualtoTurbine, ManualtoTurbine);
    EEPROM.write(EepromTurbineBrakeActivated, TurbineBrakeActivated);
    EEPROM.write(EepromAlreadyConfigure, 1);
    Serial1.println("System set to Default configuration");
    Serial1.flush();
  } else {
    BatterytoMainController = EEPROM.read(EepromBatterytoMainController);
    ManualtoMainController = EEPROM.read(EepromManualtoMainController);
    BatterytoWinch = EEPROM.read(EepromBatterytoWinch);
    ManualtoWinch = EEPROM.read(EepromManualtoWinch);
    BatterytoTurbine = EEPROM.read(EepromBatterytoTurbine);
    ManualtoTurbine = EEPROM.read(EepromManualtoTurbine);
    TurbineBrakeActivated = EEPROM.read(EepromTurbineBrakeActivated);
    Serial1.println("Previous configuration loaded");
    Serial1.flush();
  }

  //if the Power Manager is set to work on manual on which parameter
  if (ManualtoMainController == true || ManualtoWinch == true || ManualtoTurbine == true) {
    Serial1.println();
    Serial1.println("!!!! Warning !!!!");
    Serial1.println();
    if (ManualtoMainController == true) { Serial1.println((String) "The Main Controller is only powered by Battery #" + BatterytoMainController); }
    if (ManualtoWinch == true) { Serial1.println((String) "The Winch in only powered by Battery #" + BatterytoWinch); }
    if (ManualtoTurbine == true) { Serial1.println((String) "The Turbine in only charging Battery #" + BatterytoTurbine); }
    Serial1.println();
    Serial1.println("End of Warning");
    Serial1.println();
    Serial1.flush();
  } else {
    Serial1.println("Power Manager is in Automated Control");
    Serial1.flush();
  }

  //Collecting data before starting to see if it's need to switch some relays
  autoCollectData();
  bootUp = 0;
  Serial1.println("   <Arduino is ready>");
  Serial1.print(">");
  Serial1.flush();
  digitalWrite(SerialTxRx, LOW);
}


///
///
///
///
/// Fin du démarrage

void batt_1toMain(){
  digitalWrite(Batt_1toMainController, HIGH);
  delay(60);
  digitalWrite(Batt_1toMainController, LOW);
  if(EEPROM.read(EepromBatterytoMainController) != 1){
    EEPROM.write(EepromBatterytoMainController, 1);
  }
  BatterytoMainController = 1;
}
void batt_2toMain(){
  digitalWrite(Batt_2toMainController, HIGH);
  delay(60);
  digitalWrite(Batt_2toMainController, LOW);
  if(EEPROM.read(EepromBatterytoMainController) != 2){
    EEPROM.write(EepromBatterytoMainController, 2);
  }
  BatterytoMainController = 2;
}
void batt_1toWinch(){
  digitalWrite(Batt_1toWinch, HIGH);
  delay(60);
  digitalWrite(Batt_1toWinch, LOW);
  if(EEPROM.read(EepromBatterytoWinch) != 1){
    EEPROM.write(EepromBatterytoWinch, 1);
  }
  BatterytoWinch = 1;
}
void batt_2toWinch(){
  digitalWrite(Batt_2toWinch, HIGH);
  delay(60);
  digitalWrite(Batt_2toWinch, LOW);
  if(EEPROM.read(EepromBatterytoWinch) != 2){
    EEPROM.write(EepromBatterytoWinch, 2);
  }
  BatterytoWinch = 2;
}
void batt_1toTurbine(){
  digitalWrite(Batt_1toTurbine, HIGH);
  delay(60);
  digitalWrite(Batt_1toTurbine, LOW);
  if(EEPROM.read(EepromBatterytoTurbine) != 1){
    EEPROM.write(EepromBatterytoTurbine, 1);
  }
  BatterytoTurbine = 1;
}
void batt_2toTurbine(){
  digitalWrite(Batt_2toTurbine, HIGH);
  delay(60);
  digitalWrite(Batt_2toTurbine, LOW);
  if(EEPROM.read(EepromBatterytoTurbine) != 2){
    EEPROM.write(EepromBatterytoTurbine, 2);
  }
  BatterytoTurbine = 2;
}

void collectRelativeHumidity(){
  WaterDetectionPercent = ((analogRead(WaterDetection)) - 1023) * -1;
  WaterDetectionPercent = WaterDetectionPercent / 8;
  if (WaterDetectionPercent > 100){WaterDetectionPercent = 100;}
  digitalWrite(AccPower, LOW);
  
  //Pour savoir si le sensor donne 0% pendant 24h (savoir s'il fonctionne)
  if (HumidityFlag == 0 && humidityTimer == 0){
    humidityTimer = millis();
  }
  else if (HumidityFlag == 0 && humidityTimer != 0){
    currentMillis = millis();
    if (currentMillis - humidityTimer > (1000 * 60 * 60 * 24)){
      HumidityFlag = 1;
      humidityTimer = 0;
    }
  }
  else if (WaterDetectionPercent != 0 && (humidityTimer != 0 || HumidityFlag == 1)){
    HumidityFlag = 0;
    humidityTimer = 0;
  }
  
  //donc si WaterDetectionPercent = 0, peut avoir Flag=1
  if (WaterDetectionPercent > 0 && WaterDetectionPercent < 40){HumidityFlag = 0;}
  if (WaterDetectionPercent >= 40 && WaterDetectionPercent < 75){HumidityFlag = 2;}
  if (WaterDetectionPercent >= 75 && WaterDetectionPercent < 90){HumidityFlag = 3;}
  if (WaterDetectionPercent >= 90){HumidityFlag = 4;}
  Serial1.println("");
  Serial1.flush();
}

void headerPowerData(){
  Serial1.println();
  Serial1.println("Collecting voltage and amperage informations");
  Serial1.flush();
}

void collectPowerData() {
  Serial3.begin(9600);
  digitalWrite(AccPower, HIGH);
  delay(20);
  //Collecting information from Solar
  for (int t=0; t<3; t++){
    byte PowerResult[25];
    int i = 0;
    digitalWrite(SerialSolar, HIGH);
    Serial3.write(Solar, sizeof(Solar));
    Serial3.flush();
    digitalWrite(SerialSolar, LOW);
    delay(40);

    while(Serial3.available()){
      int incomingByte = Serial3.read();
      PowerResult[i] = incomingByte;
      //Serial1.println(PowerResult[i],HEX);
      if (PowerResult[i] == 0x41 && PowerResult[i-5] == 0x0D && PowerResult[i-7] == 0x01){
        VSolar = ((PowerResult[i-1] * 0.625) + 81) / 10;
        if (VSolar > 17){
          VSolar = ((PowerResult[i-1] * 1.25) + 0.4) / 10;
        }
        //Serial1.println((String) "VSolar : " + VSolar);
        newVSolar = 1;
      }
      if ((PowerResult[i] == 0x3E || PowerResult[i] == 0x3F || PowerResult[i] == 0x40 || PowerResult[i] == 0x41 || PowerResult[i] == 0x42)  && PowerResult[i-9] == 0x0D && PowerResult[i-11] == 0X01){
        //Serial1.println(PowerResult[i-1]+((PowerResult[i]-62)*255));
        ASolar = 0.0000000478*pow(PowerResult[i-1]+((PowerResult[i]-62)*255),3)-0.000027414*pow(PowerResult[i-1]+((PowerResult[i]-62)*255),2)+0.0056868293*(PowerResult[i-1]+((PowerResult[i]-62)*255))+0.45752933;
        if (ASolar < 1){
          //Serial1.println((String)"ASolar from PVSolar : " + ASolar);
          ASolar = analogRead(A4);
          ASolar = (ASolar * 0.0147) - 7.4632;
          //Serial1.println((String)"ASolar from ACS712 = "+ ASolar);
          //Serial1.flush();
        }
        newASolar = 1;
      }
      i++;
    }
    Serial1.print(".");
    if(newVSolar == 1 && newASolar == 1){break;}
  }

  //Collecting information from Battery 1
  for (int t=0; t<3; t++){
    byte PowerResult[25];
    int i = 0;
    digitalWrite(SerialSolar, HIGH);
    Serial3.write(Batt1, sizeof(Batt1));
    Serial3.flush();
    digitalWrite(SerialSolar, LOW);
    delay(40);
    while(Serial3.available()){
      int incomingByte = Serial3.read();
      PowerResult[i] = incomingByte;
      if (PowerResult[i] == 0x41 && PowerResult[i-7] == 0x0F && PowerResult[i-8] == 0x02){
        VBatt1 = ((PowerResult[i-1] * 0.625) + 81) / 10;
        newVBatt1 = 1;
      }
      if ((PowerResult[i] == 0x3E || PowerResult[i] == 0x3F || PowerResult[i] == 0x40 || PowerResult[i] == 0x41 || PowerResult[i] == 0x42)  && PowerResult[i-11] == 0x0F && PowerResult[i-12] == 0X02){
        ABatt1 = 0.1391 * exp((PowerResult[i-1]+((PowerResult[i]-62)*255)) * 0.0053);
        if (ABatt1 < 0.8){
          ABatt1 = (ASolar * 1.2903) - 0.1723;
          if(ABatt1 < 0){ABatt1 = 0;}
        }
        //Serial1.println((String)"ABatt1 : " + ABatt1);
        newABatt1 = 1;
      }
      i++;
    }
    Serial1.print(".");
    if (newVBatt1 == 1 && newABatt1 == 1){break;}
  }

  //Collecting information from Battery 2
  for (int t=0; t<3; t++){
    byte PowerResult[25];
    int i = 0;
    digitalWrite(SerialSolar, HIGH);
    Serial3.write(Batt2, sizeof(Batt1));
    Serial3.flush();
    digitalWrite(SerialSolar, LOW);
    delay(40);
    while(Serial3.available()){
      int incomingByte = Serial3.read();
      PowerResult[i] = incomingByte;
      //Serial1.println(PowerResult[i],HEX);
      if (PowerResult[i] == 0x41 && PowerResult[i-7] == 0x0F && PowerResult[i-8] == 0x82){
        //Serial1.println(PowerResult[i-1],HEX);
        //Serial1.println(PowerResult[i-1]);
        VBatt2 = ((PowerResult[i-1] * 0.625) + 81) / 10;
        //Serial1.println((String) "VBatt2 : " + VBatt2);
        newVBatt2 = 1;
      }
      if ((PowerResult[i] == 0x3E || PowerResult[i] == 0x3F || PowerResult[i] == 0x40 || PowerResult[i] == 0x41 || PowerResult[i] == 0x42)  && PowerResult[i-11] == 0x0F && PowerResult[i-12] == 0x82){
        ABatt2 = 0.1391 * exp((PowerResult[i-1]+((PowerResult[i]-62)*255)) * 0.0053);
        if (ABatt2 < 0.8){
          ABatt2 = (ASolar * 1.2903) - 0.1723;
          if(ABatt2 < 0){ABatt2 = 0;}
        }
        newABatt2 = 1;
      }
      i++;
    }
    Serial1.print(".");
    if (newVBatt2 == 1 && newABatt2 == 1){break;}
  }

  //Put values to 0 if no new values
  if (newVBatt1 == 1){newVBatt1 = 0;} else {VBatt1 = 0; newVBatt1 = 0;}
  if (newABatt1 == 1){newABatt1 = 0;} else {ABatt1 = 0; newABatt1 = 0;}
  if (newVBatt2 == 1){newVBatt2 = 0;} else {VBatt2 = 0; newVBatt2 = 0;}
  if (newABatt2 == 1){newABatt2 = 0;} else {ABatt2 = 0; newABatt2 = 0;}
  if (newVSolar == 1){newVSolar = 0;} else {VSolar = 0; newVSolar = 0;}
  if (newASolar == 1){newASolar = 0;} else {ASolar = 0; newASolar = 0;}
  
  if(VBatt1 != 0 && VBatt2 == 0){
    if (EEPROM.read(EepromManualtoMainController) == 0){batt_1toMain();}    
    if (EEPROM.read(EepromManualtoWinch) == 0){batt_1toWinch();}
    if (EEPROM.read(EepromManualtoTurbine) == 0){batt_1toTurbine();}
  }

  if(VBatt1 == 0 && VBatt2 != 0){
    if (EEPROM.read(EepromManualtoMainController) == 0){batt_2toMain();}
    if (EEPROM.read(EepromManualtoWinch) == 0){batt_2toWinch();}
    if (EEPROM.read(EepromManualtoTurbine) == 0){batt_2toTurbine();}
  }

  //Power sends to the Winch
  for (int t=0; t<1; t++){
    float sum = 0;
    int k = 0;
    for (int j = 0; j < navg; j++){
      AWinch = analogRead(Winch);
      if (AWinch > 509 && AWinch < 513){ //Replace 0 values with noise around 511 (0A)
        k++;
        }
      //Serial1.println(AWinch);
      sum += AWinch;
    }
    //Serial1.println(sum / navg);
    //Serial1.println(sum);
    Serial1.print(".");
    if (k < (navg/3)){AWinch = WinchScale * (sum / navg) + WinchOffset;}
    else {AWinch = 0;}
  
  }

  //Collecting information from Turbine
  for (int t = 0; t<1; t++){
    int k = 0;
    float sum = 0;

    for (int j = 0; j < navg; j++){
      ATurbine = analogRead(Turbine);
      if (ATurbine > 509 && ATurbine < 513){ //Replace 0 values with noise around 511 (0A)
        k++;
      }
      //Serial1.println(ATurbine);
      //Serial1.flush();
      sum += ATurbine;
      //Serial1.println(sum);
    }
    //Serial1.println(sum / navg);
    Serial1.print(".");
    if (k < (navg/3)) { ATurbine = (TurbineScale * (sum / navg)) + TurbineOffset;} //si 1/3 des valeurs sont 0, écrire 0
    else{ATurbine = 0;}
    //Serial1.println((String)"ATurbine = "+ ATurbine);
    //Serial1.flush();
  }

  //Power drawn by Power Manager and Main Controller
  for (int t = 0; t<1; t++){
    int k = 0;
    float sum = 0;

    for (int j = 0; j < navg; j++){
      APowerManager = analogRead(PowerManager);
      //Serial1.println(APowerManager);
      //Serial1.flush();
      sum += APowerManager;
      //Serial1.println(sum);
    }
    //Serial1.println(sum/navg);
    if (k < (navg/3)){APowerManager = (PowerManagerScale * (sum / navg) + PowerManagerOffset) - 0.045;}
    else {APowerManager = 0;}
    //Serial1.println((String)"APowerManager = "+ APowerManager);
      //Serial1.flush();
  }
  Serial3.end();
}

void showPowerData() {
  Serial1.println();
  Serial1.println();
  Serial1.print("Solar = ");Serial1.print(VSolar,1);Serial1.print("V, producing ");Serial1.print(ASolar,1);Serial1.println("A");
  Serial1.print("Batt1 = ");Serial1.print(VBatt1,1);Serial1.print("V, charging ");Serial1.print(ABatt1,1);Serial1.println("A from Solar");
  Serial1.print("Batt2 = ");Serial1.print(VBatt2,1);Serial1.print("V, charging ");Serial1.print(ABatt2,1);Serial1.println("A from Solar");
  if(TurbineBrakeActivated == 1){
    Serial1.println("The Turbine's Break is Activated");
  }
  else {
  Serial1.println((String) "Batt #" + BatterytoTurbine + " is being charged by the Turbie at " + ATurbine + "Ah.");
  }

  Serial1.println();
  Serial1.println((String) "Batt #" + BatterytoMainController + " is powering " + APowerManager + "A to the Buoy");

  
  Serial1.println((String) "Batt #" + BatterytoWinch + " is powering " + AWinch + "A to the Winch");
  
  Serial1.println();
  Serial1.flush();
}


void loop() {
  recvWithEndMarker();
  autoCollectData();
  showNewData();
}

float averageData(float x[], float y){
  x[averageTab] = y;
  float sumTab = 0;
  if (longAverage == 0){
      //Serial1.println("I'm here");
      for (int a = 0; a <= averageTab; a++){
        //Serial1.println(x[a]);
        sumTab += x[a];
        }
        y = sumTab / (averageTab + 1);
        //Serial1.println(y);
    }
    else {
      //Serial1.println("I'm there");
      for (int a = 0; a < lenghtOfTab; a++){
        //Serial1.println(x[a]);
        sumTab += x[a];
      }
      y = sumTab / (lenghtOfTab);
      //Serial1.println(y);
    }
  return y;
}

void autoCollectData(){
  currentMillis = millis();
  if(currentMillis >= nextAverageData){
    if(bootUp == 0){
      digitalWrite(SerialTxRx, HIGH);
      Serial1.println("");
      Serial1.print("Wait");
    }
    collectPowerData();
    collectRelativeHumidity();
    
    VBatt1a = averageData(VBatt1Tab, VBatt1);
    ABatt1a = averageData(ABatt1Tab, ABatt1);
    VBatt2a = averageData(VBatt2Tab, VBatt2);
    ABatt2a = averageData(ABatt2Tab, ABatt2);    
    VSolara = averageData(VSolarTab, VSolar);    
    ASolara = averageData(ASolarTab, ASolar);    
    ATurbinea = averageData(ATurbineTab, ATurbine);
    APowerManagera = averageData(APowerManagerTab, APowerManager);
    AWincha = averageData(AWinchTab, AWinch);
    
    if (averageTab == lenghtOfTab - 1){
      averageTab = 0;
      longAverage = 1;
    }
    else{averageTab++;}
    
    nextAverageData = currentMillis + (timeBetweenAutoCollect*1000);

    if(bootUp == 0){
      Serial1.println();
      Serial1.print(">");
      Serial1.flush();
      digitalWrite(SerialTxRx, LOW);
    }

  }
}


void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\r';
  char rc;

  while (Serial1.available() > 0 && newData == false) {

    rc = Serial1.read();

    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      digitalWrite(SerialTxRx, HIGH);
      Serial1.print(rc);
      Serial1.flush();
      digitalWrite(SerialTxRx, LOW);
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    } else {
      receivedChars[ndx] = '\0';  // terminate the string
      ndx = 0;
      newData = true;
    }
  }
}

void showNewData() {
  if (newData == true) {
    digitalWrite(SerialTxRx, HIGH);
    Serial1.println();
    if (needsEntry == false) {
      if (strcmp(receivedChars, "a") == 0) {
        delay(1000);
        float sum = 0;

        for (int j = 0; j < 20; j++){
          APowerManager = analogRead(PowerManager);
          //Serial1.println(APowerManager);
          //Serial1.flush();
          sum += APowerManager;
          //Serial1.println(sum);
          
        }
        Serial1.println(APowerManager);
        APowerManager = 0.0121*(sum / 20) - 6.5442;
        Serial1.println((String)"APowerManager = "+ APowerManager);
        Serial1.flush();
      }
    
      //Turbine Menu
      else if (strcmp(receivedChars, "TB") == 0 || strcmp(receivedChars, "tb") == 0) {
        digitalWrite(TurbineBrake, HIGH);
        delay(60);
        digitalWrite(TurbineBrake, LOW);
        TurbineBrakeActivated = 1;
        Serial1.println("Remote Brake Activated");
      }

      else if (strcmp(receivedChars, "TR") == 0 || strcmp(receivedChars, "tr") == 0) {
        digitalWrite(TurbineRelease, HIGH);
        delay(60);
        digitalWrite(TurbineRelease, LOW);
        TurbineBrakeActivated = 0;
        Serial1.println("Remote Brake Released");
      }

      //Show Power Data
      else if (strcmp(receivedChars, "DS") == 0 || strcmp(receivedChars, "ds") == 0) {
        headerPowerData();
        collectPowerData();
        showPowerData();
        collectRelativeHumidity();
        Serial1.println((String) "Relative Humidity : " + WaterDetectionPercent + "%");
        Serial1.println();
        Serial1.flush();
      }

      else if (strcmp(receivedChars, "DV") == 0 || strcmp(receivedChars, "dv") == 0){
        Serial1.print((String)VBatt1a + ", " + ABatt1a + ", " + VBatt2a + ", " + ABatt2a + ", ");
        Serial1.println((String)VSolara + ", " + ASolara + ", " + APowerManagera + ", " + ATurbinea + ", " + AWincha + ", " + WaterDetectionPercent);
        Serial1.println();
        Serial1.flush();
      }

      // Help Menu
      else if (strcmp(receivedChars, "H") == 0 || strcmp(receivedChars, "h") == 0 || strcmp(receivedChars, "?") == 0) {
        Serial1.println();
        Serial1.println();
        Serial1.println("### This is the Power Manager's Menu ###");
        Serial1.println();
        Serial1.println("Management Options :");
        Serial1.println("MC to Manually Control batteries' function");
        Serial1.println("AC to return to Automated Control");
        Serial1.println();
        Serial1.println("Turbine Options :");
        Serial1.println("T1 to charge Battery #1");
        Serial1.println("T2 to charge Battery #2");
        Serial1.println("TB to brake the turbine");
        Serial1.println("TR to release the turbine's brake");
        Serial1.println("");
        Serial1.println("Other Options :");
        Serial1.println("DS to Display Status");
        Serial1.println("DV to Display Average Data");
        Serial1.println("H for Help");
        Serial1.println("W for Water detection sensor");
        Serial1.println("End");
        Serial1.println();
      }


      // Water Detect
      else if (strcmp(receivedChars, "W") == 0 || strcmp(receivedChars, "w") == 0) {
        digitalWrite(AccPower, HIGH);
        delay(40);
        collectRelativeHumidity();
        Serial1.println((String) "Relative Humidity : " + WaterDetectionPercent + "%");
        Serial1.println("");
      }

      // Reset
      else if (strcmp(receivedChars, "r") == 0 || strcmp(receivedChars, "R") == 0) {
        Serial1.println();
        Serial1.println("Do you want to reset the Power Manager ?  Y/N");
        Serial1.flush();
        needsEntry = true;
        allowsReset = true;
      }
      //Manual Control
      else if (strcmp(receivedChars, "MC") == 0 || strcmp(receivedChars, "mc") == 0) {
        Serial1.println();
        Serial1.println("Do you want to manually choose batteries functions ? Y/N ");
        Serial1.flush();
        needsEntry = true;
        manualControl = true;
      }
      //Automatic Control
      else if (strcmp(receivedChars, "AC") == 0 || strcmp(receivedChars, "ac") == 0) {
        Serial1.println();
        Serial1.println("Do you want the Power Manager to go in Automated Control ? Y/N");
        Serial1.flush();
        needsEntry = true;
        autoControl = true;
      }
      //Others
      else if (strcmp(receivedChars, "") == 0) {
      } else {
        Serial1.println("!Invalid Command, press h for help");
      }
      newData = false;
      Serial1.print(">");
      Serial1.flush();
      digitalWrite(SerialTxRx, LOW);
    }


    //Submenu
    else {

      if (allowsReset == true) {
        if (strcmp(receivedChars, "y") == 0 || strcmp(receivedChars, "Y") == 0) {
          Serial1.println("Reset confirmed");
          Serial1.flush();
          needsEntry = false;
          allowsReset = false;
          digitalWrite(Resetpin, LOW);
        } else {
          Serial1.println("Reset aborded");
          Serial1.flush();
          needsEntry = false;
          allowsReset = false;
        }
      }

      //Submenu of Automated Control
      else if (autoControl == true) {
        if (autoControlStep == 0 && (strcmp(receivedChars, "y") == 0 || strcmp(receivedChars, "Y") == 0)) {
          headerPowerData();
          collectPowerData();
          showPowerData();
          Serial1.println("Do you really want to put the Power Manager to Automated Control ? Y/N");
          Serial1.flush();
          autoControlStep = 1;
        } else if (autoControlStep == 1 && (strcmp(receivedChars, "y") == 0 || strcmp(receivedChars, "Y") == 0)) {
          ManualtoMainController = 0;
          if (EEPROM.read(EepromManualtoMainController) != 0){
            EEPROM.write(EepromManualtoMainController, 0);
          }
          ManualtoWinch = 0;
          if(EEPROM.read(EepromManualtoWinch) != 0){
            EEPROM.write(EepromManualtoWinch, ManualtoWinch);
          }
          ManualtoTurbine = 0;
          if (EEPROM.read(EepromManualtoTurbine) != 0){
            EEPROM.write(EepromManualtoTurbine, 0);
          }
          Serial1.println("The Power Manager is now set to Automated Control");
          Serial1.println();
          Serial1.flush();
          needsEntry = false;
          autoControl = false;
          autoControlStep = 0;
        }

        else {
          Serial1.println("Automated Control aborded");
          Serial1.flush();
          needsEntry = false;
          autoControl = false;
          autoControlStep = 0;
        }
      }

      //Submenu of Manual Control

      else if (manualControl == true) {
        if (manualControlStep == 0 && (strcmp(receivedChars, "y") == 0 || strcmp(receivedChars, "Y") == 0)) {
          headerPowerData();
          collectPowerData();
          showPowerData();
          Serial1.println("Choose one of the following :");
          Serial1.println("(1) Only use one battery pack for everything");
          Serial1.println("(2) Only use one battery pack for the Main Controller");
          Serial1.println("(3) Only use one battery pack for the Winch");
          Serial1.println("(4) Only charge one battery pack with the Turbine");
          Serial1.println();
          Serial1.flush();
          manualControlStep = 1;
        }

        //Submenu of Manual Control, choice (1)

        else if (manualControlStep == 1 && strcmp(receivedChars, "1") == 0) {
          Serial1.println();
          Serial1.println("Which battery must be used for everything ? 1 or 2");
          Serial1.flush();
          manualControlStep = 11;
        }

        else if (manualControlStep == 11 && strcmp(receivedChars, "1") == 0) {
          Serial1.println();
          Serial1.println("Do you really want to switch everything to Battery #1 ? Y/N");
          Serial1.flush();
          manualControlStep = 111;
        }

        else if (manualControlStep == 111 && (strcmp(receivedChars, "y") == 0 || strcmp(receivedChars, "Y") == 0)) {
          Serial1.println();

          batt_1toMain();
          if (EEPROM.read(EepromManualtoMainController) != 1){
            EEPROM.write(EepromManualtoMainController, 1);
          }

          batt_1toWinch();
          if (EEPROM.read(EepromManualtoWinch) != 1){
            EEPROM.write(EepromManualtoWinch, 1);
          }

          batt_1toTurbine();
          if (EEPROM.read(EepromManualtoTurbine) != 1){
            EEPROM.write(EepromManualtoTurbine, 1);
          }    

          Serial1.println("All powers switchs to Battery #1");
          Serial1.println();
          Serial1.flush();
          needsEntry = false;
          manualControl = false;
          manualControlStep = 0;
        }

        else if (manualControlStep == 11 && strcmp(receivedChars, "2") == 0) {
          Serial1.println();
          Serial1.println("Do you really want to switch everything to Battery #2 ? Y/N");
          Serial1.flush();
          manualControlStep = 112;
        }

        else if (manualControlStep == 112 && (strcmp(receivedChars, "y") == 0 || strcmp(receivedChars, "Y") == 0)) {
          Serial1.println();

          batt_2toMain();
          if (EEPROM.read(EepromManualtoMainController) != 1){
            EEPROM.write(EepromManualtoMainController, 1);
          }

          batt_2toWinch();
          if (EEPROM.read(EepromManualtoWinch) != 1){
            EEPROM.write(EepromManualtoWinch, 1);
          }

          batt_2toTurbine();
          if (EEPROM.read(EepromManualtoTurbine) != 1){
            EEPROM.write(EepromManualtoTurbine, 1);
          }    
          Serial1.println("All powers switchs to Battery #2");
          Serial1.println();
          Serial1.flush();
          needsEntry = false;
          manualControl = false;
          manualControlStep = 0;
        }

        //Submenu for Manual control, choice (2)

        else if (manualControlStep == 1 && strcmp(receivedChars, "2") == 0) {
          Serial1.println();
          Serial1.println("Which battery must power the Main Controller ? 1 or 2");
          Serial1.flush();
          manualControlStep = 12;

        } else if (manualControlStep == 12 && strcmp(receivedChars, "1") == 0) {
          Serial1.println();
          Serial1.println("Do you really want to power the Main Controller with Battery #1 ? Y/N");
          Serial1.flush();
          manualControlStep = 121;
        }

        else if (manualControlStep == 121 && (strcmp(receivedChars, "y") == 0 || strcmp(receivedChars, "Y") == 0)) {
          Serial1.println();
          batt_1toMain();
          if (EEPROM.read(EepromManualtoMainController) != 1){
            EEPROM.write(EepromManualtoMainController, 1);
          }
          Serial1.println("The Main Controller is now powered by Battery #1");
          Serial1.println();
          Serial1.flush();
          needsEntry = false;
          manualControl = false;
          manualControlStep = 0;
        }

        else if (manualControlStep == 12 && strcmp(receivedChars, "2") == 0) {
          Serial1.println();
          Serial1.println("Do you really want to power the Main Controller with Battery #2 ? Y/N");
          Serial1.flush();
          manualControlStep = 122;
        }

        else if (manualControlStep == 122 && (strcmp(receivedChars, "y") == 0 || strcmp(receivedChars, "Y") == 0)) {
          Serial1.println();
          batt_2toMain();
          if (EEPROM.read(EepromManualtoMainController) != 1){
            EEPROM.write(EepromManualtoMainController, 1);
          }
          Serial1.println("The Main Controller is now powered by Battery #2");
          Serial1.println();
          Serial1.flush();
          needsEntry = false;
          manualControl = false;
          manualControlStep = 0;
        }

        //Submenu for Manual control, choice (3)

        else if (manualControlStep == 1 && strcmp(receivedChars, "3") == 0) {
          Serial1.println();
          Serial1.println("Which battery must power the Winch ? 1 or 2");
          Serial1.flush();
          manualControlStep = 13;

        } else if (manualControlStep == 13 && strcmp(receivedChars, "1") == 0) {
          Serial1.println();
          Serial1.println("Do you really want to power the Winch with Battery #1 ? Y/N");
          Serial1.flush();
          manualControlStep = 131;
        }

        else if (manualControlStep == 131 && (strcmp(receivedChars, "y") == 0 || strcmp(receivedChars, "Y") == 0)) {
          Serial1.println();
          batt_1toWinch();
          if (EEPROM.read(EepromManualtoWinch) != 1){
            EEPROM.write(EepromManualtoWinch, 1);
          }
          Serial1.println("The Winch is now powered by Battery #1");
          Serial1.println();
          Serial1.flush();
          needsEntry = false;
          manualControl = false;
          manualControlStep = 0;
        }

        else if (manualControlStep == 13 && strcmp(receivedChars, "2") == 0) {
          Serial1.println();
          Serial1.println("Do you really want to power the Winch with Battery #2 ? Y/N");
          Serial1.flush();
          manualControlStep = 132;
        }

        else if (manualControlStep == 132 && (strcmp(receivedChars, "y") == 0 || strcmp(receivedChars, "Y") == 0)) {
          Serial1.println();
          batt_2toWinch();
          if (EEPROM.read(EepromManualtoWinch) != 1){
            EEPROM.write(EepromManualtoWinch, 1);
          }
          Serial1.println("The Winch is now powered by Battery #2");
          Serial1.println();
          Serial1.flush();
          needsEntry = false;
          manualControl = false;
          manualControlStep = 0;
        }

        //Submenu for Manual control, choice (4)

        else if (manualControlStep == 1 && strcmp(receivedChars, "4") == 0) {
          Serial1.println();
          Serial1.println("Which battery must be charged by the Turbine ? 1 or 2");
          Serial1.flush();
          manualControlStep = 14;

        } else if (manualControlStep == 14 && strcmp(receivedChars, "1") == 0) {
          Serial1.println();
          Serial1.println("Do you really want the Turbine to only charge Battery #1 ? Y/N");
          Serial1.flush();
          manualControlStep = 141;
        }

        else if (manualControlStep == 141 && (strcmp(receivedChars, "y") == 0 || strcmp(receivedChars, "Y") == 0)) {
          Serial1.println();
          batt_1toTurbine();
          if (EEPROM.read(EepromManualtoTurbine) != 1){
            EEPROM.write(EepromManualtoTurbine, 1);
          }
          Serial1.println("The Turbine is now only charging Battery #1");
          Serial1.println();
          Serial1.flush();
          needsEntry = false;
          manualControl = false;
          manualControlStep = 0;
        }

        else if (manualControlStep == 14 && strcmp(receivedChars, "2") == 0) {
          Serial1.println();
          Serial1.println("Do you really want the Turbine to only charge Battery #2 ? Y/N");
          ;
          Serial1.flush();
          manualControlStep = 142;
        }

        else if (manualControlStep == 142 && (strcmp(receivedChars, "y") == 0 || strcmp(receivedChars, "Y") == 0)) {
          Serial1.println();
          batt_2toTurbine();
          if (EEPROM.read(EepromManualtoTurbine) != 1){
            EEPROM.write(EepromManualtoTurbine, 1);
          }
          Serial1.println("The Turbine is now only charging Battery #2");
          Serial1.println();
          Serial1.flush();
          needsEntry = false;
          manualControl = false;
          manualControlStep = 0;
        }

        else {
          Serial1.println("Manual control aborded");
          Serial1.println();
          Serial1.flush();
          needsEntry = false;
          manualControl = false;
          manualControlStep = 0;
        }
      }

      newData = false;
      Serial1.print(">");
      Serial1.flush();
      digitalWrite(SerialTxRx, LOW);
    }
  }
}