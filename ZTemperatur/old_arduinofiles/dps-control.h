
//Serial für DPS5020 / Modbus
// So schaut auch der Stecker aus (v.o.n.u)
#define rxDPS2 PA3 // Kabel blau -> DPS2_TX (DPSkabel schwarz)
#define txDPS2 PA2 //Kabel gelb -> DPS2_RX (DPSkabel gelb)
#define rxDPS1 PA10  //Kabel grün -> DPS1_TX (DPSkabel gelb)
#define txDPS1 PA9  //Kabel rot -> DPS1_RX (DPSkabel schwarz)
#define DPS_baud 19200 //Speed of DPS (beim Einschalten V Taste gedrückt halten beim DPS5020)


unsigned int power1 = 0;   //aktuelle P out DPS1
unsigned int power2 = 0;   //aktuelle P out DPS2

unsigned int uOut = 0;  //aktuelle U out
unsigned int iOut = 0;  //aktuelle I out

bool dps1present = true;
bool dps2present = true;

#include <ModbusMaster.h>
ModbusMaster dps1;
ModbusMaster dps2;

HardwareSerial Serial1(rxDPS1, txDPS1);
HardwareSerial Serial2(rxDPS2, txDPS2);

void dpsCurrentControl() {   // hier wird nur Strom an die DPS5020 geschickt
  if (dps1present) {
    dps1.writeSingleRegister(1, setCurrent); //aktuellen strom setzen
  }
  if (dps2present) {
    dps2.writeSingleRegister(1, setCurrent); //aktuelle current setzen
  }
}


void dpsControl() {   // hier wird Spannung und Strom an die DPS5020 geschickt
  if (dps1present) {
    dps1.writeSingleRegister(0, setVoltage); //aktuelle Voltage setzen
    dps1.writeSingleRegister(1, setCurrent); //aktuellen strom setzen
  }

  if (dps2present) {
    dps2.writeSingleRegister(0, setVoltage); //aktuelle voltage setzen
    dps2.writeSingleRegister(1, setCurrent); //aktuellen strom setzen
  }
}

void dpsEingangsSpannung() {
  //Hier Inputvoltage auslesen, ein DPS reicht, ist ja die gleiche inputvoltage
  int result = dps1.readHoldingRegisters(2, 4); // vier rgister auslesen ab register 2
  if (result == dps1.ku8MBSuccess)   // only do something with data if read is successful
  {
    dps1present = true; // jetzt wissen wir dass die da ist
    uOut = dps1.getResponseBuffer(0); // get voltage from response buffer
    iOut = dps1.getResponseBuffer(1); // get current from response buffer
    power1  = dps1.getResponseBuffer(2); // get Power from DPS1
    inVoltage =  dps1.getResponseBuffer(3); // get involtage from response buffer
  }
  else {
    dps1present = false; // ist wohl zum Wirt gegangen
  }
  
  // Hier Stromverbrauch in W von 2. DPS auslesen
  result = dps2.readHoldingRegisters(2, 4); // vier rgister auslesen ab register 2
  if (result == dps2.ku8MBSuccess)   // only do something with data if read is successful
  {
    dps2present = true; // jetzt wissen wir dass die zweite auch da ist
    power2 = dps2.getResponseBuffer(2); // get Power from DPS2
  }
  else {
    dps2present = false; // ist wohl zum Wirt gegangen
  }
  
  
  power = power1 + power2;
}

void dpsPower(byte einaus) {
  dps1.writeSingleRegister(6, 1); //lock device
  dps2.writeSingleRegister(6, 1); //lock device
  dps1.writeSingleRegister(0x09, einaus); //set power on/off !
  dps2.writeSingleRegister(0x09, einaus); //set power on/off !
  
  if (einaus == 1) {
    dpsIstAn = true;
  }
  if (einaus == 0) {
    dpsIstAn = false;
  }
}

void dpsBacklight(byte brightness) {
  if (dps1present) {
    dps1.writeSingleRegister(0x0A, brightness); // setze die Brightness
  }

  if (dps2present) {
    dps2.writeSingleRegister(0x0A, brightness); // setze die Brightness
  }

}

/*  REGISTER RuiDeng DPS5020
     Modbus RTU

   Function   Description       Number of   Decimal   UNIT  Read/Write    Register
                                bytes       places                        address
   U-SET      Voltage setting   2           2         V     R/W           0000H
   I-SET      Current setting   2           2         A     R/W           0001H
   UOUT       Output voltage    2           2         V     R             0002H
              display value
   IOUT       Output current    2           2         A     R             0003H
              display value
   POWER      Output power      2           1 or 2    W     R             0004H
              display value
   UIN        Input voltage     2           2         V     R             0005H
              display value
   LOCK       Key lock          2           0         -     R/W           0006H
   PROTECT    ProtectionStatus  2           0         -     R             0007H
   CV/CC      Status CV/CC      2           0         -     R             0008H
   ONOFF      Switch output     2           0         -     R/W           0009H
   B_LED      Backlight bright  2           0         -     R/W           000AH
   MODEL      Product Model     2           0         -     R             000BH
   VERSON     Firmware Version  2           0         -     R             000CH


   Die anderen Sachen stellt man lieber vorher ein, direkt am Gerät
*/
