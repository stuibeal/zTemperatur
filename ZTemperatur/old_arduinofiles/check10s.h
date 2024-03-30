void check10() {
  //Check ob da Kühlwasser läuft und Lüfter einschalten wenn nicht
  kuehlFlow = pulseCount;
  pulseCount = 0;

  flowOldMillis = millis(); // das speichern wir mal, damit wir was haben wenn der MEGA frägt

  dpsEingangsSpannung(); //Erstmal werte aktualisieren

  //Lüfterkontrolle
  if (kuehlFlow < 10) {  //wenn praktisch nix durchlafft
    // # map(value, fromLow, fromHigh, toLow, toHigh) Hier die Prozentwerte Lüfter nach aktueller Power machen... nicht exakt, aber besser als nix, Wasser ist eh träg
    //int luefterProzent = map(power, 0, 50000, 20, 100); // 0-500W zu 20-100% Lüfter
    //luefterTim->setCaptureCompare(luefterChannel, luefterProzent, PERCENT_COMPARE_FORMAT); // Lüfter einschalten, 20% reicht normal (duty cycle ist eher 40% bei dem Setup, bassd aber weil braucht ja saft am anfang)
    digitalWrite(LUEFTER_PIN, HIGH);
  
  }
  else {
    //luefterTim->setCaptureCompare(luefterChannel, 0, PERCENT_COMPARE_FORMAT); // Lüfter ausschalten
    digitalWrite(LUEFTER_PIN, LOW);
  }

  //Check Batteriespannung und wenn dann

  if (inVoltage > highBatteryVoltage) {
    if (inVoltageState != 3) {
      inVoltageState = 3;
      Serial.print("High Voltage: ");
      Serial.print(float(inVoltage) / 100);
      Serial.println(" V");// machwas
      if (lowPower || veryLowPower) {
        lowPower = false;
        veryLowPower = false;
        dpsBacklight(5);
        dpsPower(1);
      }
      maxCurrent = highCurrent;
      setVoltage = maxVoltage;
      zielTemperatur = highTemperatur;

      dpsControl();
      myPID.SetOutputLimits(float(minCurrent) / 100, float(maxCurrent) / 100);
    }
  }
  else if (inVoltage > midBatteryVoltage) {
    if (inVoltageState != 2) {
      inVoltageState = 2;

      Serial.print("Mid Voltage: ");
      Serial.print(float(inVoltage) / 100);
      Serial.println(" V");

      if (lowPower || veryLowPower) {
        lowPower = false;
        veryLowPower = false;
        dpsBacklight(5);
        dpsPower(1);
      }
      maxCurrent = midCurrent;
      setVoltage = normVoltage;


      zielTemperatur = midTemperatur;
      setVoltage = normVoltage;
      dpsControl();
      myPID.SetOutputLimits(float(minCurrent) / 100, float(maxCurrent) / 100);
    }
  }

  else if (inVoltage > lowBatteryVoltage) {
    if (inVoltageState != 1) {
      inVoltageState = 1;

      Serial.print("Low Voltage: ");
      Serial.print(float(inVoltage) / 100);
      Serial.println(" V");

      if (veryLowPower) {
        veryLowPower = false;
        dpsBacklight(1);
        dpsPower(1);
      }
      lowPower = true;

      maxCurrent = lowCurrent;
      zielTemperatur = lowTemperatur;
      setVoltage = normVoltage;
      dpsControl();
      dpsBacklight(0);
      myPID.SetOutputLimits(float(minCurrent) / 100, float(maxCurrent) / 100);
    }
  }

  else if (inVoltage < lowBatteryVoltage) {
    if (inVoltageState != 0) {
      inVoltageState = 0;

      Serial.print("VERY Low Voltage: ");
      Serial.print(float(inVoltage) / 100);
      Serial.println(" V");
      // keine Kühlung, alles auf low
      dpsBacklight(0);
      dpsPower(0);
      lowPower = true;
      veryLowPower = true;
    }
  }
} //void check10
