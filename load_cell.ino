/**
 * Force and load cell-specific code and helpers. HX711 chip.
 */

void loadSetup() {
  Serial.println();
  Serial.println("Starting-up loadcell HX711...");

  LoadCell.begin();
  InternalFS.begin();

  file.open(CALIBRATIONS_FILENAME, Adafruit_LittleFS_Namespace::FILE_O_READ);
  if (file) {
    uint32_t readlen;
    Serial.printf("Calibrations found.\n");
    readlen = file.read((char*)&nvram_settings, sizeof(nvram_settings));
    file.close();
  } else {
    Serial.printf("No calibrations found!\n");
  }

  if (nvram_settings.load_multiplier == 0) {
    nvram_settings.load_multiplier = LOAD_MULT_DEFAULT;
    nvram_settings.load_offset     = LOAD_OFFSET_DEFAULT;
  }

  LoadCell.setCalFactor(nvram_settings.load_multiplier);
  LoadCell.setTareOffset(nvram_settings.load_offset);

  Serial.printf("Load offset set to: %ld\n",   nvram_settings.load_offset);
  Serial.printf("Load multiplier set to: %f\n", nvram_settings.load_multiplier);

  attachInterrupt(digitalPinToInterrupt(HX711_dout), dataReadyISR, FALLING);
}

// Interrupt routine — called on HX711 DOUT falling edge
void dataReadyISR() {
  if (LoadCell.update()) {
    newLoadDataReady++;
  }
}

/**
 * Get the current force from the load cell.
 * Returns the latest smoothed reading in Newtons.
 */
float getAvgForce() {
  static float currentData = 0;

  if (newLoadDataReady) {
    noInterrupts();
    currentData = abs(LoadCell.getData());
    newLoadDataReady = 0;
    interrupts();
  }

  return currentData;
}
