//PINS
#define PUMPE_KRAFT_PIN PB0   //Kontrolled rot -> MOSFET-Treiber invertiert diesen Ausgang!
#define PUMPE_NORMAL_PIN PB1  //Kontrolled gelb
#define LUEFTER_PIN PA15     //Kontrolled grün
#define statusLed PC13      // Eingebaute LED
#define flowSensor PB14     // Flowsensor für Kühlwasser
#define SDApin PB7
#define SCLpin PB6


// HardwareTimer für PWM an Lüfter und Pumpenpins
  /***
  // Lüfter 
  TIM_TypeDef *InstanceLuefter = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(luefterPin), PinMap_PWM);
  uint32_t luefterChannel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(luefterPin), PinMap_PWM));
  HardwareTimer *luefterTim = new HardwareTimer(InstanceLuefter);

  // PumpeKraft
  TIM_TypeDef *InstanceKraft = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pumpeKraftPin), PinMap_PWM);
  uint32_t pumpeKraftChannel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pumpeKraftPin), PinMap_PWM));
  HardwareTimer *pumpeKraftTim = new HardwareTimer(InstanceKraft);

  // PumpeNormal
  TIM_TypeDef *InstanceNormal = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pumpeNormalPin), PinMap_PWM);
  uint32_t pumpeNormalChannel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pumpeNormalPin), PinMap_PWM));
  HardwareTimer *pumpeNormalTim = new HardwareTimer(InstanceNormal);

  ***/
