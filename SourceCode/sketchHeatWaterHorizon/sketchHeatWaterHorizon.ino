#include <OneWire.h>
#include <DallasTemperature.h>
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define ONE_WIRE_BUS 3                             // Data wire is plugged into digital pin 3 on the Arduino
#define NumberOfDevices 10                         // Set maximum number of devices in order to dimension 
#define NameMaxCharLength 7                        // extra char for string terminator
// Array holding all Device Address arrays.
//----------------------------------------------------------------------------------------------------------------

//SensorMapping
#define T0 0
#define CC 1
#define CH 2
#define BC 3
#define BH 4
#define T5 5
#define T6 6
#define T7 7
#define T8 8
#define T9 9

//ROMAddresses
#define T0ROM 0x28,0xFF,0x84,0x63,0x90,0x15,0x01,0xE1
#define T1ROM 0x28,0xFF,0x1E,0x29,0x90,0x15,0x04,0x2F
#define T2ROM 0x28,0xFF,0x4c,0x77,0x90,0x15,0x01,0xCB
#define T3ROM 0x28,0xFF,0x85,0x19,0x90,0x15,0x04,0xA0
#define T4ROM 0x28,0xFF,0xA8,0x5E,0x90,0x15,0x01,0xED
#define T5ROM 0x28,0xFF,0xAE,0x1A,0x90,0x15,0x04,0xA0
#define T6ROM 0x28,0xFF,0x45,0x18,0x90,0x15,0x04,0x0D
#define T7ROM 0x28,0xFF,0xD4,0x84,0x90,0x15,0x01,0xBB
#define T8ROM 0x28,0xFF,0x83,0x65,0x90,0x15,0x01,0xB9
#define T9ROM 0x28,0xFF,0x50,0x63,0x90,0x15,0x01,0xA0

//----------------------------------------------------------------------------------------------------------------
#define PinOutputLED 13                            // Optics
#define PinOutputFET 7                             // To MOSFET-Gate
#define NilTemperature -127
#define ValidTemperatureDifference 5
#define ActionCollectorDiffTempON 5
#define ActionCollectorDiffTempOFF 10
#define ActionCollectorToHot 40
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
OneWire oneWire(ONE_WIRE_BUS);                     // Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature sensors(&oneWire);               // Pass our oneWire reference to Dallas Temperature.
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte allAddress [NumberOfDevices][8];              // Device Addresses are 8-element byte arrays.
// we need one for each of our DS18B20 sensors.

byte totalDevices;                                 // Declare variable to store number of One Wire devices
// that are actually discovered.

char allNames[NumberOfDevices][NameMaxCharLength] = {"T0", "BC", "BH", "CC", "CH", "T5", "T6", "T7", "T8", "T9"};
byte ROMMapping[NumberOfDevices] = {T0, BC, BH, CC, CH, T5, T6, T7, T8, T9};

int USBPlugged = 0;                                 // me: 0 = external Power, 1 = USB
int loopCount = 0;
struct SensorRecord {
  //  char location[6];
  byte address[8];
  bool active = LOW;
  float temperature[2] = {NilTemperature, NilTemperature};
};
typedef struct SensorRecord SensorData;
SensorData Sensors[NumberOfDevices];
bool PumpON = LOW;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void printLine(char sign[1]) {
  for (byte i = 0; i < 70; i++) {
    Serial.print(sign);
  }
}
//----------------------------------------------------------------------------------------------------------------
void mapSensors() {
  byte allROMs[NumberOfDevices][8] = {T0ROM, T1ROM, T2ROM, T3ROM, T4ROM, T5ROM, T6ROM, T7ROM, T8ROM, T9ROM};
  Serial.println("mapSensors()");
  for (byte d = 0; d < NumberOfDevices; d++) {
    //each device, starting with dd = "BC"
    byte r = ROMMapping[d];
    Serial.print(allNames[d]);
    Serial.print(" ");
    Serial.print("T");
    Serial.print(r);
    Serial.print("ROM ");
    for (byte ab = 0; ab < 8; ab++) {
      Sensors[d].address[ab] = allROMs[r][ab];
      Serial.print("0x");
      Serial.print(Sensors[d]. address[ab], HEX);
      Serial.print(" ");
    }
    Serial.println("");
  }
  Serial.println("");
}
//----------------------------------------------------------------------------------------------------------------
bool byteArrayMatch(byte byte1[8], byte byte2[8]) {
  byte goil = 0;
  for (byte i = 0; i < 8; i++) {
    if (byte1[i] == byte2[i]) {
      goil++;
    }
  }
  if (goil == 8) {
    return HIGH;
  }
  else {
    return LOW;
  }
}
//----------------------------------------------------------------------------------------------------------------
void setSensorStatus() {
  //  Serial.print("setSensorStatus()");
  //  Serial.println("");
  for (byte d = 0; d < NumberOfDevices; d++) {
    byte tempByteArray[8];
    for (byte ab = 0; ab < 8; ab++) {
      tempByteArray[ab] = Sensors[d].address[ab];
    }
    //    Serial.print(allNames[d]);
    int hit = -1;
    byte a = 0;
    while (a < NumberOfDevices) {
      if (byteArrayMatch(allAddress[a], tempByteArray)) {
        hit = a;
        Sensors[d].active = HIGH;
        //        Serial.print(" = ");
        //        Serial.print("Device ");
        //        Serial.print(a);
        //        Serial.print(" active");
        //        Serial.println("");
        break;
      }
      else {
        Sensors[d].active = LOW;
      }
      a++;
    }
    if (hit == -1) {
      //      Serial.print(" not found");
      //      Serial.println("");
    }
  }
  //  Serial.println("");
}
//----------------------------------------------------------------------------------------------------------------
void getDataFromDevices() {
  Serial.println("getDataFromDevices()");
  sensors.requestTemperatures();                // Initiate  temperature request to all devices
  for (byte d = 0; d < NumberOfDevices; d++) {
    //Write to SensorsArray
    Sensors[d].temperature[1] = Sensors[d].temperature[0];
    if (Sensors[d].active) {
      Sensors[d].temperature[0] = sensors.getTempC(Sensors[d].address);
      Serial.print(allNames[d]);
      Serial.print(" -> ");
      Serial.print(Sensors[d].temperature[0]);
      Serial.print("C\r\n");
    }
    else {
      Sensors[d].temperature[0] = NilTemperature;
    }
  }
  Serial.println("");
}
//----------------------------------------------------------------------------------------------------------------
void printAddress(DeviceAddress addr) {
  byte i;
  for ( i = 0; i < 8; i++) {                      // prefix the printout with 0x
    Serial.print("0x");
    if (addr[i] < 16) {
      Serial.print('0');                        // add a leading '0' if required.
    }
    Serial.print(addr[i], HEX);                 // print the actual value in HEX
    Serial.print(" ");
  }
  Serial.println("");
}
//----------------------------------------------------------------------------------------------------------------
byte discoverOneWireDevices() {
  byte j = 0;                                      // search for one wire devices and
  // copy to device address arrays.
  //  Serial.print("discoverOneWireDevices()");
  //  Serial.print("\r\n");
  for (byte i = 0; i < NumberOfDevices; i++) {
    //clearAllAddresses
    for (byte ab = 0; ab < 8; ab++) {
      allAddress[i][ab] = 0xFF;
    }
  }
  while ((j < NumberOfDevices) && (oneWire.search(allAddress[j]))) {
    j++;
  }
  for (byte i = 0; i < j; i++) {
    //    Serial.print("Device ");
    //    Serial.print(i);
    //    Serial.print(": ");
    //    printAddress(allAddress[i]);                  // print address from each device address arry.
  }
  //  Serial.print("\r\n");
  return j                      ;                 // return total number of devices found.
}
//----------------------------------------------------------------------------------------------------------------
/*bool TempValueValid(float TempArray[2]){
  float tempFloat = abs((TempArray[1]-TempArray[0]));
  if (tempFloat < ValidTemperatureDifference){
    return HIGH;
  }
  else {
    return LOW;
  }
  }
*/
//----------------------------------------------------------------------------------------------------------------
float checkedTempValue(float TempArray[2]) {
  float DiffTemp = (TempArray[1] - TempArray[0]);
  float tempFloat = abs(DiffTemp);
  if (tempFloat < ValidTemperatureDifference) {
    return ((TempArray[1] + TempArray[0]) / 2);
  }
  else {
    return NilTemperature;
  }
}
//----------------------------------------------------------------------------------------------------------------
byte arrayPos(byte definition) {
  byte i = 0;
  byte thePosition = NumberOfDevices + 1;
  while (i < NumberOfDevices) {
    //find position of definition in sensor array
    if (definition ==  ROMMapping[i]) {
      thePosition = i;
      break;
    }
    i++;
  }
  if (thePosition == NumberOfDevices + 1) {
    Serial.println ("arrayPos: Can't find definition in ROMMappingArray");
  }
  return thePosition;
}
//----------------------------------------------------------------------------------------------------------------
bool PumpRequestCollectorDiff() {
  Serial.println("PumpRequestCollectorDiff()");
  float TempCC = checkedTempValue(Sensors[arrayPos(CC)].temperature);
  float TempCH = checkedTempValue(Sensors[arrayPos(CH)].temperature);
  float DiffTemp = TempCH - TempCC;
  bool result = LOW;
  if (TempCC != NilTemperature && TempCH != NilTemperature) {
    Serial.print("TempCH-TempCC= ");
    Serial.print(TempCH);
    Serial.print(" - ");
    Serial.print(TempCC);
    Serial.print(" = ");
    Serial.print(DiffTemp);
    if (PumpON == LOW) {
      Serial.print(" > ONDiffT(");
      Serial.print(ActionCollectorDiffTempON);
      Serial.print(")?");
      if (DiffTemp > ActionCollectorDiffTempON) {
        result = HIGH;
        Serial.print(" -> PumpRequest");
      }
    }
    else {
      Serial.print(" < OFFDiffT(");
      Serial.print(ActionCollectorDiffTempOFF);
      Serial.print(")?");
      if (DiffTemp < ActionCollectorDiffTempOFF) {
        result = LOW;
        Serial.print(" -> PumpOFFRequest");
      }
      else {
        //keep ON
        result = HIGH;
      }
    }
  }
  else {
    Serial.print("TempCC(");
    Serial.print(TempCC);
    Serial.print(") and/or TempCH(");
    Serial.print(TempCH);
    Serial.print(") invalid");
    result = LOW;
  }
  Serial.print(" -> ");
  Serial.println(result);
  Serial.println("");
  return result;
}
//----------------------------------------------------------------------------------------------------------------
/*bool PumpRequestCollectorToHot() {
  Serial.print("PumpRequestCollectorToHot()");
  Serial.println("");
  float TempCC = checkedTempValue(Sensors[arrayPos(CC)].temperature);
  float TempCH = checkedTempValue(Sensors[arrayPos(CH)].temperature);
  bool result = LOW;
    Serial.print(TempCH);
    Serial.print(" || ");
    Serial.print(TempCC);
    Serial.print(" > ");
    Serial.print(ActionCollectorToHot);
    Serial.print("?");
  if (TempCC != NilTemperature && TempCH != NilTemperature){
    if (TempCC > ActionCollectorToHot || TempCH > ActionCollectorToHot){
      result = HIGH;
    }
    else {
      result = LOW;
    }
  }
  Serial.print(" -> ");
  Serial.print(result);
  Serial.println("");
  Serial.println("");
  return result;
  }
*/
//----------------------------------------------------------------------------------------------------------------
bool PumpRequestSensorToHot(byte mappingName, byte switchTemp) {
  Serial.println("PumpRequestSensorToHot()");
  float Temp = checkedTempValue(Sensors[arrayPos(mappingName)].temperature);
  bool result = LOW;
  Serial.print(allNames[arrayPos(mappingName)]);
  Serial.print(" (");
  Serial.print(Temp);
  Serial.print(") > ");
  Serial.print(switchTemp);
  Serial.print("?");
  if (Temp != NilTemperature) {
    if (Temp > switchTemp) {
      result = HIGH;
    }
    else {
      result = LOW;
    }
  }
  else {
    Serial.print(" invalid");
  }
  Serial.print(" -> ");
  Serial.println(result);
  Serial.println("");
  return result;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  pinMode(PinOutputLED, OUTPUT);
  pinMode(PinOutputFET, OUTPUT);
  //  digitalWrite(PinOutputFET, HIGH);  //LOWActive -> set LOW to MOSFET-GATE for Setup
  digitalWrite(PinOutputFET, LOW);  //LOWActive -> set LOW to MOSFET-GATE for Setup
  Serial.begin(38400);
  sensors.begin();
  delay(1000);
  printLine("-");
  Serial.println("");
  mapSensors();
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  printLine(".");
  Serial.println(loopCount);
  totalDevices = discoverOneWireDevices();         // get addresses of our one wire devices into allAddress array
  for (byte i = 0; i < totalDevices; i++) {
    sensors.setResolution(allAddress[i], 10);      // and set the a to d conversion resolution of each.
  }
  setSensorStatus();
  getDataFromDevices();
  if (PumpRequestSensorToHot(CC, ActionCollectorToHot) || PumpRequestSensorToHot(CH, ActionCollectorToHot) || PumpRequestCollectorDiff()) {
    PumpON = HIGH;
  }
  else {
    PumpON = LOW;
  }
  if (PumpON) {
    digitalWrite(PinOutputLED, HIGH);
    //    digitalWrite(PinOutputFET, LOW);    //LOWActive -> set HIGH to MOSFET-GATE
    digitalWrite(PinOutputFET, HIGH);    //LOWActive -> set LOW to MOSFET-GATE
    Serial.println("Pump -> ON");
  }
  else {
    digitalWrite(PinOutputLED, LOW);
    //    digitalWrite(PinOutputFET, HIGH);    //LOWActive -> set LOW to MOSFET-GATE
    digitalWrite(PinOutputFET, LOW);    //LOWActive -> set LOW to MOSFET-GATE
    Serial.println("Pump -> OFF");
  }
  loopCount++;
  delay(1000);
}


