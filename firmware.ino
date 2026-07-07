// =========================================================================
//   HX711 SOIL CORE WATER-LOSS TRACKER + AUTO WATERING (fully hands-off)
// =========================================================================
// Diagnosis from the test logs:
//   * RAW median readings are ACCURATE (~325 g, +/-3 g). Sensor is fine.
//   * The visible "wandering" was EMA LAG, not sensor noise -> EMA removed.
//   * Slow 0 -> -8 g creep is HX711 zero-drift (thermal) -> handled by
//     warm-up + smart re-tare, NOT by filtering.
//   * Phantom pump firing was wrong thresholds vs an empty/tared scale.
//
// Operating mode chosen: WATER-LOSS mode with the CORE ON.
//   - Power on with the core already sitting on the cell.
//   - After an automatic warm-up, it auto-tares -> reads ~0 g at "full".
//   - As water evaporates the reading goes NEGATIVE (e.g. -30 g = 30 g lost).
//   - Pump ON when loss reaches WATER_ON_LOSS; OFF once refilled to ~baseline.
//
// Drift handling (weeks-long): the zero is re-tared automatically ONLY right
//   after a completed watering cycle (core is back at its full reference),
//   which cancels accumulated electronic drift WITHOUT erasing the real
//   water-loss signal. A long-interval safety re-tare also runs if the core
//   has been stable and full for a long time.
//
// Everything is automatic: power on with the core on and walk away.
// (Serial 't' = force re-tare, 'p' = 1s pump self-test -- optional overrides.)
// =========================================================================

#include <WiFi.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <WiFiClientSecure.h>
#include "HX711.h"

// =========================================================================
// 1. EDUROAM WI-FI & GITHUB OTA SETTINGS
// =========================================================================
const char* WIFI_SSID = "eduroam";
#define EAP_IDENTITY "emurciabotache@ufl.edu" // Must be full @ufl.edu email
#define EAP_PASSWORD "VDMCLiebch.en1" 


const String CURRENT_FIRMWARE_VERSION = "1.0"; // Change this before uploading new OTA versions
const char* GITHUB_VERSION_URL  = "https://raw.githubusercontent.com/Emurciab/ESP32_Salinity/main/version.txt";
const char* GITHUB_FIRMWARE_URL = "https://raw.githubusercontent.com/Emurciab/ESP32_Salinity/main/firmware.bin";

// --- Pin configuration ---
// GPIO39 is input-only, no pull resistor, ADC-noise-prone. OK for DOUT, but
// move to 16/17/25/26 if you ever see erratic spikes.
const int LOADCELL_DOUT_PIN = 39;  // DT
const int LOADCELL_SCK_PIN  = 4;   // SCK
const int PUMP_PIN          = 26;  // relay / MOSFET (leave unconnected for now)
const bool PUMP_ACTIVE_LOW  = false;

HX711 scale;

// --- Calibration ---
float calibration_factor = 380.85;

// --- Reading parameters ---
const int   SAMPLES_PER_READ = 21;   // odd -> clean median
const float DEADBAND_GRAMS   = 1.0;  // display updates only on change > this

// --- Warm-up (thermal drift settles before we set the zero) ---
// Set to ~30 min for real deployment. Lower (e.g. 60000) while bench testing.
const unsigned long WARMUP_MS = 30UL * 60UL * 1000UL;  // 30 minutes

// --- Watering logic (WATER-LOSS mode: reading is grams relative to "full") ---
// Water when the core has LOST this many grams (reading <= -WATER_ON_LOSS).
const float WATER_ON_LOSS  = 30.0;   // start watering after 30 g of loss
// Consider "refilled" once the reading climbs back to near baseline.
const float WATER_OFF_LEVEL = -5.0;  // stop when within 5 g of full

// --- Smart re-tare / drift correction ---
// After a watering cycle finishes, the core is "full": re-zero to cancel drift.
const bool  RETARE_AFTER_WATERING = true;
// Safety re-tare if the reading has stayed near baseline (full & stable) for
// this long (catches drift even if watering hasn't happened in a while).
const unsigned long SAFETY_RETARE_MS = 24UL * 60UL * 60UL * 1000UL; // 24 h
const float STABLE_BAND_GRAMS = 3.0;  // "near baseline" window for the timer

// --- Pump safety ---
const unsigned long PUMP_MAX_RUN_MS  = 15000;
const unsigned long PUMP_COOLDOWN_MS = 60000;

// --- State ---
float medianWeight    = 0.0;
float displayedWeight = 0.0;
bool  isFirstReading  = true;

bool          pumpRunning = false;
unsigned long pumpStartMs = 0;
unsigned long pumpStopMs  = 0;

unsigned long lastStableRetareCheck = 0;  // start of current "stable near-full" period

// =========================================================================
float get_median_weight(int num_samples) {
  float readings[num_samples];
  for (int i = 0; i < num_samples; i++) readings[i] = scale.get_units(1);
  for (int i = 0; i < num_samples - 1; i++)
    for (int j = 0; j < num_samples - i - 1; j++)
      if (readings[j] > readings[j + 1]) {
        float t = readings[j]; readings[j] = readings[j + 1]; readings[j + 1] = t;
      }
  if (num_samples % 2 == 1) return readings[num_samples / 2];
  return (readings[num_samples / 2 - 1] + readings[num_samples / 2]) / 2.0;
}

void setPump(bool on) {
  digitalWrite(PUMP_PIN, (PUMP_ACTIVE_LOW ? !on : on) ? HIGH : LOW);
  pumpRunning = on;
}

// Re-zero so the CURRENT load (core at "full") reads ~0. Cancels drift.
void retareToFull(const char* reason) {
  Serial.print("[RE-TARE] "); Serial.println(reason);
  scale.set_scale();
  scale.tare(30);
  scale.set_scale(calibration_factor);
  displayedWeight = 0.0;
  lastStableRetareCheck = millis();
}

// =========================================================================
// NETWORK & OTA FUNCTIONS
// =========================================================================
void connectToEduroam() {
  Serial.print("Connecting to UF Eduroam ");
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  
  // WPA2 Enterprise connection standard for modern ESP32 cores
  WiFi.begin(WIFI_SSID, WPA2_AUTH_PEAP, EAP_IDENTITY, EAP_IDENTITY, EAP_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[SUCCESS] Connected to Eduroam!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n[FAILED] Could not connect to Eduroam. Continuing offline...");
  }
}

void checkForUpdates() {
  if (WiFi.status() != WL_CONNECTED) return;
  
  Serial.println("\nChecking GitHub for firmware updates...");
  
  WiFiClientSecure client;
  client.setInsecure(); // Bypass SSL certificate verification for simplicity

  HTTPClient http;
  http.begin(client, GITHUB_VERSION_URL);
  
  int httpCode = http.GET();
  if (httpCode == HTTP_CODE_OK) {
    String onlineVersion = http.getString();
    onlineVersion.trim(); // Remove any accidental newlines or spaces
    
    Serial.println("Current Local Version: " + CURRENT_FIRMWARE_VERSION);
    Serial.println("Online GitHub Version: " + onlineVersion);

    if (onlineVersion != CURRENT_FIRMWARE_VERSION) {
      Serial.println(">>> NEW VERSION FOUND! Downloading and flashing...");
      
      t_httpUpdate_return ret = httpUpdate.update(client, GITHUB_FIRMWARE_URL);
      
      switch (ret) {
        case HTTP_UPDATE_FAILED:
          Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
          break;
        case HTTP_UPDATE_NO_UPDATES:
          Serial.println("HTTP_UPDATE_NO_UPDATES");
          break;
        case HTTP_UPDATE_OK:
          Serial.println("HTTP_UPDATE_OK - Rebooting!");
          break;
      }
    } else {
      Serial.println(">>> Firmware is up to date. Proceeding to normal operation.");
    }
  } else {
    Serial.printf("[FAILED] Could not read version.txt. HTTP Code: %d\n", httpCode);
  }
  http.end();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PUMP_PIN, OUTPUT);
  setPump(false);

  Serial.println("=================================================");
  Serial.println("   SOIL CORE WATER-LOSS + AUTO WATERING          ");
  Serial.println("=================================================");
  
  // --- Connect to Wi-Fi & Check OTA Updates ---
  connectToEduroam();
  checkForUpdates();
  Serial.println("=================================================\n");

  Serial.println("Place the CORE on the cell now (at its FULL/watered weight).");

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  int attempts = 0;
  while (!scale.is_ready() && attempts < 20) { delay(200); Serial.print("."); attempts++; }
  Serial.println(scale.is_ready() ? "\nHX711 ready." : "\nWARNING: HX711 not responding!");

  // --- Automatic warm-up so the zero is set AFTER thermal drift settles ---
  Serial.print("Warming up for ");
  Serial.print(WARMUP_MS / 60000UL);
  Serial.println(" min before setting zero (drift settling)...");
  unsigned long warmStart = millis();
  while (millis() - warmStart < WARMUP_MS) {
    if (scale.is_ready()) {
      // show live raw so you can watch drift settle; no averaging needed here
      Serial.print("  warmup raw: ");
      Serial.println(scale.get_units(5) / calibration_factor, 1);
    }
    delay(5000);
  }

  // Auto-tare with the core on -> establishes the "full" baseline as 0 g.
  retareToFull("Initial auto-tare (core = full baseline)");

  Serial.println("-------------------------------------------------");
  Serial.print("Water ON at loss >= "); Serial.print(WATER_ON_LOSS);
  Serial.print(" g | OFF when back within "); Serial.print(-WATER_OFF_LEVEL);
  Serial.println(" g of full");
  Serial.println("Optional Serial: 't'=force re-tare  'p'=pump self-test(1s)");
  Serial.println("-------------------------------------------------");
}

void handleSerial() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 't' || c == 'T') retareToFull("Manual re-tare (Serial 't')");
    else if (c == 'p' || c == 'P') {
      Serial.println("Pump self-test 1s...");
      setPump(true); delay(1000); setPump(false);
      Serial.println("Self-test done.");
    }
  }
}

void updateWatering(float w) {
  unsigned long now = millis();

  if (pumpRunning && (now - pumpStartMs > PUMP_MAX_RUN_MS)) {
    setPump(false); pumpStopMs = now;
    Serial.println("[SAFETY] Max run time reached -> OFF (check water tank/flow)");
    return;
  }

  if (pumpRunning) {
    // Refilled back near baseline -> stop, then re-tare to cancel drift.
    if (w >= WATER_OFF_LEVEL) {
      setPump(false); pumpStopMs = now;
      Serial.println("[PUMP] Refilled to baseline -> OFF");
      if (RETARE_AFTER_WATERING) retareToFull("Post-watering (core full again)");
    }
  } else {
    if (now - pumpStopMs < PUMP_COOLDOWN_MS) return;  // let water settle
    if (w <= -WATER_ON_LOSS) {                        // enough loss -> water
      setPump(true); pumpStartMs = now;
      Serial.println("[PUMP] Water loss threshold reached -> ON");
    }
  }
}

// Safety drift correction: if the reading has stayed near baseline (full &
// stable) for SAFETY_RETARE_MS, re-zero to remove slow electronic drift.
void maybeSafetyRetare(float w) {
  unsigned long now = millis();
  bool nearBaseline = (fabs(w) <= STABLE_BAND_GRAMS);
  if (!nearBaseline || pumpRunning) {
    lastStableRetareCheck = now;  // reset the stability timer
    return;
  }
  if (now - lastStableRetareCheck >= SAFETY_RETARE_MS) {
    retareToFull("Safety re-tare (stable near baseline)");
  }
}

void loop() {
  handleSerial();

  medianWeight = get_median_weight(SAMPLES_PER_READ);

  // Stable displayed value (deadband) -- purely cosmetic, decision uses median
  if (isFirstReading) { displayedWeight = medianWeight; isFirstReading = false; }
  else if (fabs(medianWeight - displayedWeight) > DEADBAND_GRAMS)
    displayedWeight = medianWeight;

  updateWatering(medianWeight);
  maybeSafetyRetare(medianWeight);

  Serial.print("Loss(median): "); Serial.print(medianWeight, 1);
  Serial.print(" g | STABLE: ");   Serial.print(displayedWeight, 0);
  Serial.print(" g | Pump: ");     Serial.println(pumpRunning ? "ON" : "OFF");

  delay(2000);  // evaporation is slow; a few seconds is plenty
}