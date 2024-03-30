// aus Communication.h

unsigned int blockTemp;                // #define transmitBlockTemp       0x40  // Data send:   blockTemp in °C*10
unsigned int auslaufTemp;              // #define transmitAuslaufTemp     0x41  // Data send:   hahnTemp in °C*10
unsigned int power=0;                  // #define transmitPower           0x42  // Data send:   Leistung in W (power1+power2)    
unsigned int inVoltage = 0;            // #define transmitInVoltage       0x43  // Data send:   inVoltage in V*100   
unsigned int kuehlFlow = 0;            // #define transmitKuehlFlow       0x44  // Data send:   Durchfluss Kühlwasser (extern) pro 10000ms

unsigned int highTemperatur = 300;     // #define setHighTemperatur       0x60  // Data get: Zieltemperatur Block * 100 (2°C)
unsigned int midTemperatur = 500;      // #define setMidTemperatur        0x61  // Data get: Normale Temperatur in °C * 100 (6°C)
unsigned int lowTemperatur = 800;      // #define setLowTemperatur        0x62  // Data get: Energiespar Temperatur * 100 (9°C)
unsigned int minCurrent = 20;          // #define setMinCurrent           0x63  // Data get: Current in mA / 10 (11 = 0,11 A), Untere Regelgrenze
unsigned int lowCurrent = 100;          // #define setLowCurrent           0x64  // Data get: current in mA / 10, Obere Regelgrenze bei wenig Strom
unsigned int midCurrent = 700;         // #define setMidCurrent           0x65  // Data get: Current in mA / 10, Obere Regelgrenze bei normalem Strom
unsigned int highCurrent = 950;       // #define setHighCurrent          0x66  // Data get: Current in mA / 10, Obere Regelgrenze bei gutem Strom 

unsigned int normVoltage = 850;           //#define setNormVoltage          0x68  // Data get: norm Voltage * 100, passt normal, mehr als 9V macht wenig Sinn bei den Peltierelementen
unsigned int maxVoltage = 1000;           //#define setMaxVoltage           0x69  // Data get: max Voltage * 100, das wäre dann eigentlich die Batteriespannung
unsigned int lowBatteryVoltage = 1140;  //#define setLowBatteryVoltage    0x6A  // 11V Eingangsspannung 
unsigned int midBatteryVoltage = 1200;  //#define setMidBatteryVoltage    0x6B  // 12V Eingangsspannung
unsigned int highBatteryVoltage = 1280; //#define setHighBatteryVoltage   0x6C  // 13V Eingangsspannung

unsigned int wasserTemp;                //#define setWasserTemp           0x6D  // Data get: kühlwasserTemp in °C*100 vom DS18B20 Sensor vom Master: Fühler neben Peltier
unsigned int einlaufTemp;               //#define setEinlaufTemp          0x6E  // Data get: Biertemperatur in °C*100 vom DS18B20 Sensor vom Master: Bierzulauf       

float consKp = 0.8;                       //#define setConsKp               0x70  // Data get: konservativer Kp
float consKi = 0.05;                    //#define setConsKi               0x71  // Data get: konservativer Ki
float consKd = 0.05;                    //#define setConsKd               0x72  // Data get: konservativer Kd
float aggKp = 1.5;                        //#define setAggKp                0x73  // Data get: aggressiver Kp
float aggKi = 0.2;                      //#define setAggKi                0x74  // Data get: aggressiver Ki
float aggKd = 0.5;                        //#define setAggKd                0x75  // Data get: aggressiver Kd
unsigned int unterschiedAggPid = 12;    //#define setUnterschiedAggPid    0x75  // mal soundso grad nehmen ab wann der aggressiv regelt
unsigned int steuerZeit = 500;          //#define setSteuerZeit           0x76  // alle sekunde mal nachjustieren

bool ebiModeBool = false;               //#define ebiMode               0xF9      //    1 an, 0 aus       Temperatur auf 2°C, Hahn auf, Zapfmusik
bool beginZapfBool = false;             // #define beginZapf             0xFA      //    Beginn das Zapfprogramm -> PID auf aggressiv
bool endZapfBool = false;               //#define endZapf               0xFB      //    Data send : milliliter    
bool kurzBevorZapfEndeBool= false;      //#define kurzBevorZapfEnde     0xFC      //    sagt das wir kurz vor Ende sind → Valve schließen -> PID auf konservativ
bool nachtBool = false;


// I2C Kommunikation
byte recieveByte[3]; // Das will der Master von uns
byte sendeByte[2]; //Bytearray zum senden über I2C

// Steuerungskram
unsigned int zielTemperatur = highTemperatur;
unsigned int setVoltage = normVoltage; //mal gaaanz klein beginnen
unsigned int setCurrent = minCurrent;
unsigned int maxCurrent = midCurrent; //Obere Regelgrenze auf mittlere stellen
bool dpsIstAn = false;
bool lowPower = false; 
bool veryLowPower = false;
byte inVoltageState = 1; // 0: very low Voltage, 1: low Voltage, 2: mid Voltage, 3: High Voltage

unsigned int pulseCount = 0; // für Flowsensor, hier int weil wir das nicht so oft kontrollieren wollen
unsigned int flowOldMillis = 0;
unsigned int oldMillis = 0;
