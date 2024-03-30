
/*
    Temperaturregelung für den Z-Apfapparat 0.5
    http://dan.drown.org/stm32duino/package_STM32duino_index.json
    https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
    Runs on STM32F411CEU6 mit der 1.9er umgebung
    Kompilieren mit Enable Serial (no generic Serial) und USB generic Serial

    done:
    - kein Kühlwasserdurchfluss: Lüfter on!
    - PID

    to do:
    - weg mit dem pwm zeug
    

   



*/

#include "communication.h"    // definiert die ganzen I2C sachen und die Variablen die vom Master geändert werden können
#include "variablen.h"        // (Global-)Variablen, die hardcoded werden
#include "dps-control.h"      // ModBus RTU Kommunikation mit den DPS 5020
#include "tempsensor.h"       // MAX31865 Sensoren
#include "hardware.h"         // Pinbelegung und dergleichen


// Bibliotheken
#include <SPI.h>

// #include <Wire.h>
#include <Wire.h>   //Nimmt die Wire Slave Bib für STM32Duino
#include <QuickPID.h>  //PID Bib

//Variablen zum pidden
float Setpoint, Input, Output;
//Define the aggressive and conservative and POn Tuning Parameters

//Specify the links
QuickPID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd,  /* OPTIONS */
               myPID.pMode::pOnError,                   /* pOnError, pOnMeas, pOnErrorMeas */
               myPID.dMode::dOnError,                    /* dOnError, dOnMeas */
               myPID.iAwMode::iAwCondition,             /* iAwCondition, iAwClamp, iAwOff */
               myPID.Action::reverse);                   /* direct, reverse */

#include "check10s.h"  //Das checkt er alle 10s (Lüfter, Voltage...)



void setup() {
  // BUILTIN LED
  pinMode(statusLed, OUTPUT);
  digitalWrite(statusLed, LOW);   // turn the LED on (HIGH is the voltage level)

  // USB Serial für debug
  Serial.begin(115200);

  // Hardware SPI1 initialisieren
  SPI.setMISO(PA6);
  SPI.setMOSI(PA7);
  SPI.setSCLK(PA5);
  SPI.begin();

  // Max31865 Init
  auslauf.begin(MAX31865_3WIRE);
  block.begin(MAX31865_3WIRE);
  checkMAXfaults();  // im Setup das reicht zum checken ob die Sensoren noch funzen

  //i2C
  Wire.setClock(400000);
  Wire.setSDA(SDApin);
  Wire.setSCL(SCLpin);
  Wire.begin(tempi2c);         // join i2c bus
  Wire.onRequest(requestEvent); // register event
  Wire.onReceive(receiveEvent); // register event

  // Lüfter, Pumpen
  pinMode(PUMPE_KRAFT_PIN, OUTPUT); // invertiert!
  pinMode(PUMPE_NORMAL_PIN, OUTPUT);
  pinMode(LUEFTER_PIN, OUTPUT);

  digitalWrite(PUMPE_KRAFT_PIN, LOW); // -> AN!
  digitalWrite(PUMPE_NORMAL_PIN, HIGH);
  digitalWrite(LUEFTER_PIN, HIGH);
  

  // Setup PWM für Pumpen und Lüfter
  /*** hinfort
  luefterTim->setMode(luefterChannel, TIMER_OUTPUT_COMPARE_PWM1, luefterPin);
  luefterTim->setOverflow(20000, HERTZ_FORMAT); // In Hertz, wahlweise in DoppelHerz /2
  luefterTim->setCaptureCompare(luefterChannel, 100, PERCENT_COMPARE_FORMAT); // mal vollgas beim starten!
  luefterTim->resume();

  pumpeKraftTim->setMode(pumpeKraftChannel, TIMER_OUTPUT_COMPARE_PWM1, pumpeKraftPin);
  pumpeKraftTim->setOverflow(20000, HERTZ_FORMAT);
  pumpeKraftTim->setCaptureCompare(pumpeKraftChannel, 0, PERCENT_COMPARE_FORMAT); // mal vollgas beim starten, invertiert natürlich (latürnich!)
  pumpeKraftTim->resume();

  pumpeNormalTim->setMode(pumpeNormalChannel, TIMER_OUTPUT_COMPARE_PWM1, pumpeNormalPin);
  pumpeNormalTim->setOverflow(20000, HERTZ_FORMAT);
  pumpeNormalTim->setCaptureCompare(pumpeNormalChannel, 100, PERCENT_COMPARE_FORMAT); // mal vollgas beim starten!
  pumpeNormalTim->resume();
  ***/
  
  // Inputpin für Flowsensor Kühlwasser
  pinMode(flowSensor, INPUT);
  digitalWrite(flowSensor, LOW); //setz den Flowmeter Sensorpin auf LOW
  attachInterrupt(flowSensor, pulseCounter, RISING); // Interrupt bei steigender Flanke

  // ModBus + DPS hochfahren
  Serial1.begin(DPS_baud);  // HardwareSerial
  Serial2.begin(DPS_baud);
  dps1.begin(1, Serial1);   // communicate with Modbus slave ID 1 over Serial Ports 1&2
  dps2.begin(1, Serial2);
  delay(500);
  //dps1.writeSingleRegister(6, 1); //lock device
  //dps2.writeSingleRegister(6, 1); //lock device
  dpsPower(1);
  for (byte x = 0; x < 6; x++) {
    dpsBacklight(x); // mach mal das Display Hell
  }
  dpsControl(); // Setze Voltage und Current aus Globalvariablen (setvoltage, setcurrent)
  dpsPower(1); // schalt ein die Dinger


  //luefterTim->setCaptureCompare(luefterChannel, 0, PERCENT_COMPARE_FORMAT); // Lüfter ausschalten, normal brauchts den ja nicht
  digitalWrite(LUEFTER_PIN, LOW);
  //pumpeKraftTim->setCaptureCompare(pumpeKraftChannel, 100, PERCENT_COMPARE_FORMAT); // KraftPumpe ausschalten, brauchts normal auch nicht
  digitalWrite(PUMPE_KRAFT_PIN, HIGH); // aus

  //PID
  myPID.SetOutputLimits(float(minCurrent) / 100, float(maxCurrent) / 100);
  myPID.SetSampleTimeUs(100000);
  myPID.SetTunings(consKp, consKi, consKd);
  myPID.SetMode(myPID.Control::automatic);


  flowOldMillis = millis();
  oldMillis = millis();

  digitalWrite(statusLed, HIGH);  // LED aus wenn Setup fertig

  //I2C
}


// the loop function runs over and over again forever
void loop() {
  //erstma Flow checken, Voltage und so weiter
  if ((millis() - flowOldMillis > 10000)&& (nachtBool == false)) {   // alle 10s mal nachkucken
    check10();
    flowOldMillis = millis();
  }

  // Rest checken
  if ((millis() - oldMillis >= steuerZeit) && (nachtBool == false)) {
    oldMillis = millis();

    //Temperaturen holen und konvertieren fürs PID
    auslaufTemp = auslauf.temperature(RNOMINAL, RREF) * 100;   //nur interessant für den Zapfer
    Input = block.temperature(RNOMINAL, RREF); //  Hier gleich als float speichern
    blockTemp = Input * 100; //  Hier mal 100 für Ausgabe über I2C an den Master
    Setpoint = float(zielTemperatur) / 100;

    //Kucken ob wir conservativ oder aggressiv regeln
    float gap = abs(Setpoint - Input); // Wie weit sind wir weg vom Ziel
    if (gap < float(unterschiedAggPid) || lowPower || kurzBevorZapfEndeBool || !beginZapfBool) { //geringer Unterschied Ziel/Set oder wir ham keinen strom/oder wir sind kurz vorm Zapfende
      myPID.SetTunings(consKp, consKi, consKd);
    }
    else if (gap > float(unterschiedAggPid) || beginZapfBool || ebiModeBool) {
      //Der Temperaturunterschied ist größer oder wir beginnen den Zapfvorgang, dann stellen wir auf aggressiv, oder Ebi zapft
      myPID.SetTunings(aggKp, aggKi, aggKd);
    }
    //sollte klappen.

    myPID.SetOutputLimits(float(minCurrent) / 100, float(maxCurrent) / 100);
    

    setCurrent = Output * 100;
    dpsCurrentControl();

    //PowerPumpe. Die soll laufen wenn wir massiv Strom wegheizen
    if (power > 2400 || setCurrent > 500) { // 90 W oder 5A(*2 DPSn)
      //map(value, fromLow, fromHigh, toLow, toHigh) Hier die Prozentwerte der Pumpe machen
      //int pumpeKraftProzent = map(setCurrent, highCurrent, 500, 0, 40); // 0-500W zu 0-40 Pumpe PWM, achtung: 0 wäre an, 100 wäre aus. 
      //pumpeKraftTim->setCaptureCompare(pumpeKraftChannel, 0, PERCENT_COMPARE_FORMAT); 
      digitalWrite(PUMPE_KRAFT_PIN, LOW);
    }
    else {
      //pumpeKraftTim->setCaptureCompare(pumpeKraftChannel, 100, PERCENT_COMPARE_FORMAT); // KraftPumpe ausschalten
      digitalWrite(PUMPE_KRAFT_PIN, HIGH);
    }


    //Frostwächter
    if (Input < 0.5 && dpsIstAn) {  // ab 0.5°C soll er ausschalten
      dpsPower(0); //ausschalten, dann heizt er gleich zurück
    }
    else {
      if (!lowPower || !veryLowPower) {
        dpsPower(1); //einschalten
      }
    }

    
      //Serial Monitor
      Serial.println();
      Serial.print(F("Setpoint:"));  Serial.print(Setpoint); Serial.print(",");
      Serial.print(F("Maxcurrent:"));  Serial.print(float(maxCurrent)/100); Serial.print(",");
      Serial.print(F("Input:"));  Serial.print(Input); Serial.print(",");
      Serial.print(F("Output:"));  Serial.print(Output); Serial.print(",");
      Serial.print(F("Pterm:"));  Serial.print(myPID.GetPterm()); Serial.print(",");
      Serial.print(F("Iterm:"));  Serial.print(myPID.GetIterm()); Serial.print(",");
      Serial.print(F("Dterm:"));  Serial.print(myPID.GetDterm()); Serial.print(",");
      Serial.print(F("Control:"));  Serial.print(myPID.GetMode()); Serial.print(",");
      Serial.print(F("Action:"));  Serial.print(myPID.GetDirection()); Serial.print(",");
      Serial.print(F("Pmode:"));  Serial.print(myPID.GetPmode()); Serial.print(",");
      Serial.print(F("Dmode:"));  Serial.print(myPID.GetDmode()); Serial.print(",");
      Serial.print(F("AwMode:"));  Serial.println(myPID.GetAwMode());
    

  }
myPID.Compute();   // sollte immer einmal im loop laufen

} // void loop



void pulseCounter()   //Hier den Flowmeter eins hochzählen, Interruptroutine
{
  pulseCount++;
}


// I2C Kommunikation mit To Do.

void receiveEvent(int howMany) {
  byte x = 0;
  while (1 < Wire.available()) { // loop through all but the last
    digitalWrite(statusLed, LOW);
    recieveByte[x] = Wire.read(); // receive byte
    x++;
    digitalWrite(statusLed, HIGH);
  }
  recieveByte[x] = Wire.read();    // receive byte



  if (recieveByte[0] == transmitBlockTemp) {
    sendeByte[0] = highByte(blockTemp);
    sendeByte[1] = lowByte(blockTemp);
  }

  if (recieveByte[0] == transmitAuslaufTemp) {
    sendeByte[0] = highByte(auslaufTemp);
    sendeByte[1] = lowByte(auslaufTemp);
  }

  if (recieveByte[0] > 0x41) {   //check die andern Sachen nur, wenns größer ist, spart Zeit
    if (recieveByte[0] == transmitPower) {
      sendeByte[0] = highByte(power);
      sendeByte[1] = lowByte(power);
    }
    if (recieveByte[0] == transmitInVoltage) {
      sendeByte[0] = highByte(inVoltage);
      sendeByte[1] = lowByte(inVoltage);
    }
    if (recieveByte[0] == transmitKuehlFlow) {
      sendeByte[0] = highByte(kuehlFlow);
      sendeByte[1] = lowByte(kuehlFlow);
    }
  }

  if (recieveByte[0] > 0x59) {   //check die andern Sachen nur, wenns größer ist, spart Zeit, kommt nur beim Set von Sachen vor
    if (recieveByte[0] == setHighTemperatur) {
      unsigned int recieveInt = (recieveByte[1] << 8) + recieveByte[2];
      highTemperatur = recieveInt;
    }
    if (recieveByte[0] == setMidTemperatur) {
      unsigned int recieveInt = (recieveByte[1] << 8) + recieveByte[2];
      midTemperatur = recieveInt;
    }
    if (recieveByte[0] == setLowTemperatur) {
      unsigned int recieveInt = (recieveByte[1] << 8) + recieveByte[2];
      lowTemperatur = recieveInt;
    }
    if (recieveByte[0] == setMinCurrent) {
      unsigned int recieveInt = (recieveByte[1] << 8) + recieveByte[2];
      minCurrent = recieveInt;
    }
    if (recieveByte[0] == setLowCurrent) {
      unsigned int recieveInt = (recieveByte[1] << 8) + recieveByte[2];
      lowCurrent = recieveInt;
    }
    if (recieveByte[0] == setMidCurrent) {
      unsigned int recieveInt = (recieveByte[1] << 8) + recieveByte[2];
      midCurrent = recieveInt;
    }
    if (recieveByte[0] == setHighCurrent) {
      unsigned int recieveInt = (recieveByte[1] << 8) + recieveByte[2];
      highCurrent = recieveInt;
    }
    if (recieveByte[0] == setNormVoltage) {
      unsigned int recieveInt = (recieveByte[1] << 8) + recieveByte[2];
      normVoltage = recieveInt;
    }
    if (recieveByte[0] == setMaxVoltage) {
      unsigned int recieveInt = (recieveByte[1] << 8) + recieveByte[2];
      maxVoltage = recieveInt;
    }
    if (recieveByte[0] == setLowBatteryVoltage) {
      unsigned int recieveInt = (recieveByte[1] << 8) + recieveByte[2];
      lowBatteryVoltage = recieveInt;
    }
    if (recieveByte[0] == setMidBatteryVoltage) {
      unsigned int recieveInt = (recieveByte[1] << 8) + recieveByte[2];
      midBatteryVoltage = recieveInt;
    }
    if (recieveByte[0] == setHighBatteryVoltage) {
      unsigned int recieveInt = (recieveByte[1] << 8) + recieveByte[2];
      highBatteryVoltage = recieveInt;
    }
    if (recieveByte[0] == setWasserTemp) {
      unsigned int recieveInt = (recieveByte[1] << 8) + recieveByte[2];
      wasserTemp = recieveInt;
    }
    if (recieveByte[0] == setEinlaufTemp) {
      unsigned int recieveInt = (recieveByte[1] << 8) + recieveByte[2];
      einlaufTemp = recieveInt;
    }
    if (recieveByte[0] == setConsKp) {
      unsigned int recieveInt = (recieveByte[1] << 8) + recieveByte[2];
      consKp = float(recieveInt) / 100;
      myPID.SetTunings(consKp, consKi, consKd);
    }
    if (recieveByte[0] == setConsKi) {
      unsigned int recieveInt = (recieveByte[1] << 8) + recieveByte[2];
      consKi = float(recieveInt) / 100;
      myPID.SetTunings(consKp, consKi, consKd);
    }
    if (recieveByte[0] == setConsKd) {
      unsigned int recieveInt = (recieveByte[1] << 8) + recieveByte[2];
      consKd = float(recieveInt) / 100;
      myPID.SetTunings(consKp, consKi, consKd);
    }
    if (recieveByte[0] == setAggKp) {
      unsigned int recieveInt = (recieveByte[1] << 8) + recieveByte[2];
      aggKp = float(recieveInt) / 100;
    }
    if (recieveByte[0] == setAggKi) {
      unsigned int recieveInt = (recieveByte[1] << 8) + recieveByte[2];
      aggKi = float(recieveInt) / 100;
    }
    if (recieveByte[0] == setAggKd) {
      unsigned int recieveInt = (recieveByte[1] << 8) + recieveByte[2];
      aggKd = float(recieveInt) / 100;
    }
    if (recieveByte[0] == setUnterschiedAggPid) {
      unsigned int recieveInt = (recieveByte[1] << 8) + recieveByte[2];
      unterschiedAggPid = recieveInt;
    }
    if (recieveByte[0] == setSteuerZeit) {
      unsigned int recieveInt = (recieveByte[1] << 8) + recieveByte[2];
      steuerZeit = recieveInt;
    }


    if (recieveByte[0] == ebiMode) {
      //mir ist so egal was der da macht
      digitalWrite(statusLed, LOW);
      unsigned int recieveInt = (recieveByte[1] << 8) + recieveByte[2];
      if (recieveInt = 1) {
        ebiModeBool = true;
      }
      else {
        ebiModeBool = false;
      }
      digitalWrite(statusLed, HIGH);
    }

    if (recieveByte[0] == beginZapf) {
      //Da könnte man auf die aggressiven Werte umschwenken
      digitalWrite(statusLed, LOW);
      beginZapfBool = true;
      digitalWrite(statusLed, HIGH);
    }

    if (recieveByte[0] == endZapf) {
      //Da weiß man dasses aus ist.
      digitalWrite(statusLed, LOW);
      kurzBevorZapfEndeBool = false;
      beginZapfBool = false;
      digitalWrite(statusLed, HIGH);
    }

    if (recieveByte[0] == kurzBevorZapfEnde) {
      //Da könnte man wohl auf die konservativen Werte umschwenken
      digitalWrite(statusLed, LOW);
      beginZapfBool = false;
      kurzBevorZapfEndeBool = true;
      digitalWrite(statusLed, HIGH);
    }


    if (recieveByte[0] == lowEnergy) {
      //Macht der doch selber... aber die lowPower setzen wir mal auf true
      digitalWrite(statusLed, LOW);
      lowPower = true;
      digitalWrite(statusLed, HIGH);
    }

    if (recieveByte[0] == wachAuf) {
      nachtBool = false;
      setCurrent = minCurrent;
      setVoltage = normVoltage;
      dpsControl();
      dpsPower(1); //mal einschalten
      dpsBacklight(5); //mal Licht an
      //luefterTim->setCaptureCompare(luefterChannel, 100, PERCENT_COMPARE_FORMAT); // mal vollgas beim starten!
      //luefterTim->resume();
      digitalWrite(LUEFTER_PIN, HIGH);
      delay(1000);
      //pumpeKraftTim->setCaptureCompare(pumpeKraftChannel, 0, PERCENT_COMPARE_FORMAT); // mal vollgas beim starten, invertiert natürlich (latürnich!)
      //pumpeKraftTim->resume();
      digitalWrite(PUMPE_KRAFT_PIN, LOW);
      delay(1000);
      //pumpeNormalTim->setCaptureCompare(pumpeNormalChannel, 100, PERCENT_COMPARE_FORMAT); // mal vollgas beim starten!
      //pumpeNormalTim->resume();
      digitalWrite(PUMPE_NORMAL_PIN, HIGH);
    }

    if (recieveByte[0] == zapfenStreich) {
      nachtBool = true;
      setCurrent = 0;
      setVoltage = 0;
      dpsControl();
      dpsBacklight(0); // Licht aus
      delay(1000);
      dpsPower(0);
      digitalWrite(LUEFTER_PIN, LOW);
      delay(1000);
      digitalWrite(PUMPE_KRAFT_PIN, HIGH);
      delay(1000);
      digitalWrite(PUMPE_NORMAL_PIN, LOW);
      
    }
  }

} //void recieveEvent


// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent()
{
  Wire.write(sendeByte[0]); // Schick dem Master die Daten, Du Lappen
  Wire.write(sendeByte[1]); // Schick dem Master die anderen Daten
}
