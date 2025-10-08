#include <Arduino.h>
#include <TuyaWifi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

TuyaWifi my_device;

/* Relay states */
unsigned char relay1_state = 0;  // For reporting current relay outputs
unsigned char relay2_state = 0;
unsigned char relay3_state = 0;
unsigned char pairing_led_state = 0; // For pairing indication

/* User-defined normal relay configuration (set via DPID 105 & 106) */
unsigned char normalRelay1 = 1; // default ON
unsigned char normalRelay2 = 0; // default OFF

/* Button DPID states (these are toggled via physical press or app) */
unsigned char buttonDPState1 = 0;
unsigned char buttonDPState2 = 0;
unsigned char buttonDPState3 = 0;
unsigned char buttonDPState4 = 0;

/* Pins */
int key_pin = 4;              // Button for pairing
#define PAIRING_LED 2          // LED for pairing/network indication
#define RELAY1 16              // Relay1 controlled from Tuya App (normal pattern)
#define RELAY2 17              // Relay2 controlled from Tuya App (normal pattern)
#define RELAY3 18              // Relay3 controlled from Tuya App (new)
#define ONE_WIRE_BUS 19        // DS18B20 data pin
#define PRESSURE_PIN 34        // Potentiometer simulating pressure sensor

// New Analog Output Pins (DAC capable pins)
#define ANALOG_OUT1 25   // Downstream
#define ANALOG_OUT2 26   // Downstream1

// Push Button Pins (physical)
#define BUTTON1_PIN 32
#define BUTTON2_PIN 33
#define BUTTON3_PIN 27
#define BUTTON4_PIN 14

/* Tuya Data Points */
#define DPID_BUTTON1   101
#define DPID_BUTTON2   102
#define DPID_BUTTON3   103
#define DPID_BUTTON4   104
#define DPID_SWITCH1   105  // reused for normalRelay1
#define DPID_SWITCH2   106  // reused for normalRelay2
#define DPID_SWITCH3   107
#define DPID_TEMP      108
#define DPID_PRESSURE  109
#define DPID_ANALOG1   110
#define DPID_ANALOG2   111

/* DP array: tell Tuya this device has 11 DPs */
unsigned char dp_array[][2] =
{
  {DPID_BUTTON1,   DP_TYPE_BOOL},
  {DPID_BUTTON2,   DP_TYPE_BOOL},
  {DPID_BUTTON3,   DP_TYPE_BOOL},
  {DPID_BUTTON4,   DP_TYPE_BOOL},
  {DPID_SWITCH1,   DP_TYPE_BOOL},
  {DPID_SWITCH2,   DP_TYPE_BOOL},
  {DPID_SWITCH3,   DP_TYPE_BOOL},
  {DPID_TEMP,      DP_TYPE_VALUE},
  {DPID_PRESSURE,  DP_TYPE_VALUE},
  {DPID_ANALOG1,   DP_TYPE_VALUE},
  {DPID_ANALOG2,   DP_TYPE_VALUE}
};

unsigned char pid[] = {"angdu4ksv2vaidwq"};
unsigned char mcu_ver[] = {"1.0.0"};

/* timing bookkeeping */
unsigned long last_time = 0;
unsigned long last_temp_report = 0;
const unsigned long TEMP_REPORT_INTERVAL = 2000;
unsigned long last_pressure_report = 0;
const unsigned long PRESSURE_REPORT_INTERVAL = 2000;
unsigned long last_button_report = 0;
const unsigned long BUTTON_REPORT_INTERVAL = 1000;

/* DS18B20 Setup */
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

/* Pressure and Temperature Setpoints (modifiable by you in code or later via DP if desired) */
int LP_Setpoint = 30;   // Low pressure threshold
int HP_Setpoint = 120;  // High pressure threshold
int LT_Setpoint = 10;   // Low temperature threshold (°C)
int HT_Setpoint = 80;   // High temperature threshold (°C)

/* OLED Setup */
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/* For physical button edge detection */
int lastPhysicalButtonState1 = HIGH;
int lastPhysicalButtonState2 = HIGH;
int lastPhysicalButtonState3 = HIGH;
int lastPhysicalButtonState4 = HIGH;

/* Forward declarations */
unsigned char dp_process(unsigned char dpid,const unsigned char value[], unsigned short length);
void dp_update_all(void);

/* Helper: convert C to F */
float c_to_f(float c) {
  return c * 9.0 / 5.0 + 32.0;
}

/* Core logic handler:
   - independent flags for pressure/temp/sensor faults
   - determines whether relays should enter fault mode (and be forced)
   - obeys rules:
     * High Pressure  -> triggers relay fault mode
     * Low Pressure   -> only display LP (do NOT change relays)
     * Low Temp       -> triggers relay fault mode
     * High Temp      -> only display HT (do NOT change relays)
     * Sensor Faults  -> only display
     * Button1 (DP state) -> manual fault if ON (toggled via XOR)
*/
void handleSystemLogic(float pressure, float tempC, bool button1_manual)
{
  // Fault flags (independent)
  bool pressureSensorFault = false;
  bool tempSensorFault = false;
  bool pressureHigh = false;
  bool pressureLow = false;
  bool tempHigh = false;
  bool tempLow = false;

  // Validate sensor ranges -> mark SF if out of expected ranges
  if (pressure < 0 || pressure > 150) {
    pressureSensorFault = true;
  } else {
    if (pressure < LP_Setpoint) pressureLow = true;
    if (pressure > HP_Setpoint) pressureHigh = true;
  }

  if (tempC < -100 || tempC > 200) { // sanity wide range for sensor error
    tempSensorFault = true;
  } else {
    if (tempC < LT_Setpoint) tempLow = true;
    if (tempC > HT_Setpoint) tempHigh = true;
  }

  // Relay fault conditions: (Sensor faults only DISPLAY now)
  bool relayFaultCondition = false;
  if (pressureHigh || tempLow || button1_manual) {
    relayFaultCondition = true;
  }

  // Apply relay outputs
  bool onlyDisplayOnlyCondition = false;
  if (!relayFaultCondition && (pressureLow || tempHigh)) {
    onlyDisplayOnlyCondition = true;
  }

  if (relayFaultCondition) {
    // Fault mode (safety): fixed
    digitalWrite(RELAY1, LOW);
    digitalWrite(RELAY2, HIGH);
    relay1_state = 0;
    relay2_state = 1;
  } else if (onlyDisplayOnlyCondition) {
    // Do not change relays: keep existing relay1_state and relay2_state
    digitalWrite(RELAY1, relay1_state ? HIGH : LOW);
    digitalWrite(RELAY2, relay2_state ? HIGH : LOW);
  } else {
    // Normal operation: user-defined normal pattern
    digitalWrite(RELAY1, normalRelay1 ? HIGH : LOW);
    digitalWrite(RELAY2, normalRelay2 ? HIGH : LOW);
    relay1_state = normalRelay1;
    relay2_state = normalRelay2;
  }

  // Prepare indicators
  String pInd = "";
  if (pressureSensorFault) pInd += "SF";
  else {
    if (pressureHigh) pInd += "HP";
    if (pressureLow) {
      if (pInd.length()) pInd += " ";
      pInd += "LP";
    }
  }

  String tInd = "";
  if (tempSensorFault) tInd += "SF";
  else {
    if (tempHigh) tInd += "HT";
    if (tempLow) {
      if (tInd.length()) tInd += " ";
      tInd += "LT";
    }
  }

  // Display
  float tempF = c_to_f(tempC);
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("=== SYSTEM MONITOR ===");

  display.print("Press.: ");
  display.print((int)pressure);
  display.print(" PSI   ");
  if (pInd.length()) display.print(pInd);
  display.println();

  display.print("Temp.:  ");
  display.print((int)tempF);
  display.print(" F   ");
  if (tInd.length()) display.print(tInd);
  display.println();

  display.print("LED1: ");
  display.print(relay1_state ? "ON" : "OFF");
  display.print("    LED2: ");
  display.print(relay2_state ? "ON" : "OFF");
  display.println();
  display.println();
  display.println();

  display.display();

  my_device.mcu_dp_update(DPID_SWITCH1, relay1_state, 1);
  my_device.mcu_dp_update(DPID_SWITCH2, relay2_state, 1);
}


/* Setup and loop (kept largely as original, with button edge detection + toggling) */
void setup()
{
  Serial.begin(9600);

  // Initialize OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("System Booting...");
  display.display();
  delay(1000);

  // Initialize LEDs / Relays
  pinMode(PAIRING_LED, OUTPUT);
  digitalWrite(PAIRING_LED, LOW);

  pinMode(RELAY1, OUTPUT);
  digitalWrite(RELAY1, LOW);

  pinMode(RELAY2, OUTPUT);
  digitalWrite(RELAY2, LOW);

  pinMode(RELAY3, OUTPUT);
  digitalWrite(RELAY3, LOW);

  // Initialize networking key
  pinMode(key_pin, INPUT_PULLUP);

  // Initialize physical push buttons
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(BUTTON3_PIN, INPUT_PULLUP);
  pinMode(BUTTON4_PIN, INPUT_PULLUP);

  // Initialize Tuya
  my_device.init(pid, mcu_ver);
  my_device.set_dp_cmd_total(dp_array, 11);
  my_device.dp_process_func_register(dp_process);
  my_device.dp_update_all_func_register(dp_update_all);

  // Initialize DS18B20
  sensors.begin();

  // ADC pin
  pinMode(PRESSURE_PIN, INPUT);

  last_time = millis();
  last_temp_report = millis();
  last_pressure_report = millis();
  last_button_report = millis();
}

void loop()
{
  my_device.uart_service();

  // WiFi pairing key
  if (digitalRead(key_pin) == LOW) {
    delay(80);
    if (digitalRead(key_pin) == LOW) {
      my_device.mcu_set_wifi_mode(SMART_CONFIG);
    }
  }

  // Pairing LED blink when not connected states
  if ((my_device.mcu_get_wifi_work_state() != WIFI_LOW_POWER) &&
      (my_device.mcu_get_wifi_work_state() != WIFI_CONN_CLOUD) &&
      (my_device.mcu_get_wifi_work_state() != WIFI_SATE_UNKNOW)) {
    if (millis() - last_time >= 500) {
      last_time = millis();
      pairing_led_state = !pairing_led_state;
      digitalWrite(PAIRING_LED, pairing_led_state);
    }
  } else {
    digitalWrite(PAIRING_LED, HIGH);
  }

  // Read sensors
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);
  int rawValue = analogRead(PRESSURE_PIN);
  int pressure = map(rawValue, 0, 4095, 1, 150);

  // Physical button edge detection + toggle (XOR) behavior
  int raw1 = digitalRead(BUTTON1_PIN);
  if (raw1 == LOW && lastPhysicalButtonState1 == HIGH) {
    // Button pressed -> toggle DPID state 1
    buttonDPState1 ^= 1;
    // Report to Tuya
    my_device.mcu_dp_update(DPID_BUTTON1, buttonDPState1, 1);
  }
  lastPhysicalButtonState1 = raw1;

  int raw2 = digitalRead(BUTTON2_PIN);
  if (raw2 == LOW && lastPhysicalButtonState2 == HIGH) {
    buttonDPState2 ^= 1;
    my_device.mcu_dp_update(DPID_BUTTON2, buttonDPState2, 1);
  }
  lastPhysicalButtonState2 = raw2;

  int raw3 = digitalRead(BUTTON3_PIN);
  if (raw3 == LOW && lastPhysicalButtonState3 == HIGH) {
    buttonDPState3 ^= 1;
    my_device.mcu_dp_update(DPID_BUTTON3, buttonDPState3, 1);
  }
  lastPhysicalButtonState3 = raw3;

  int raw4 = digitalRead(BUTTON4_PIN);
  if (raw4 == LOW && lastPhysicalButtonState4 == HIGH) {
    buttonDPState4 ^= 1;
    my_device.mcu_dp_update(DPID_BUTTON4, buttonDPState4, 1);
  }
  lastPhysicalButtonState4 = raw4;

  // Use buttonDPState1 as manual fault flag (if true => manual fault engaged)
  bool manualFault = (buttonDPState1 != 0);

  // Handle system logic (display + relay decisions)
  handleSystemLogic(pressure, tempC, manualFault);

  // Regular DP uploads
  // temperature: convert to int Celsius? original code sent int(tempC) - keep same but we also send temp in C for DP.
  my_device.mcu_dp_update(DPID_TEMP, (int)tempC, 1);
  my_device.mcu_dp_update(DPID_PRESSURE, pressure, 1);

  // Report button DP states periodically (we already update on toggle, but keep dp_update_all for sync)
  // my_device.mcu_dp_update(DPID_BUTTON1, buttonDPState1, 1); // dp_update_all will handle on connect

  delay(10);
}

/**
 * @description: DP download callback function.
 * dp_process will toggle button DP states when DPID_BUTTONx arrives (XOR),
 * and also handle switches and analog DPs.
 */
unsigned char dp_process(unsigned char dpid,const unsigned char value[], unsigned short length)
{
  switch(dpid) {
    // Button DPs: when app presses button DP, toggle local button state (XOR) and report new state back
    case DPID_BUTTON1: {
      // toggle
      buttonDPState1 ^= 1;
      // report updated state
      my_device.mcu_dp_update(DPID_BUTTON1, buttonDPState1, 1);
    } break;

    case DPID_BUTTON2: {
      buttonDPState2 ^= 1;
      my_device.mcu_dp_update(DPID_BUTTON2, buttonDPState2, 1);
    } break;

    case DPID_BUTTON3: {
      buttonDPState3 ^= 1;
      my_device.mcu_dp_update(DPID_BUTTON3, buttonDPState3, 1);
    } break;

    case DPID_BUTTON4: {
      buttonDPState4 ^= 1;
      my_device.mcu_dp_update(DPID_BUTTON4, buttonDPState4, 1);
    } break;

    // DPID_SWITCH1 & DPID_SWITCH2 are used to configure user-defined normal pattern
    case DPID_SWITCH1:
      normalRelay1 = my_device.mcu_get_dp_download_data(dpid, value, length);
      my_device.mcu_dp_update(dpid, normalRelay1, 1);
    break;

    case DPID_SWITCH2:
      normalRelay2 = my_device.mcu_get_dp_download_data(dpid, value, length);
      my_device.mcu_dp_update(dpid, normalRelay2, 1);
    break;

    case DPID_SWITCH3:
      relay3_state = my_device.mcu_get_dp_download_data(dpid, value, length);
      digitalWrite(RELAY3, relay3_state ? HIGH : LOW);
      my_device.mcu_dp_update(dpid, relay3_state, length);
    break;

    case DPID_ANALOG1: {
      int val = my_device.mcu_get_dp_download_data(dpid, value, length);
      int dacVal = map(val, 0, 100, 58, 232);
      dacWrite(ANALOG_OUT1, dacVal);
    } break;

    case DPID_ANALOG2: {
      int val = my_device.mcu_get_dp_download_data(dpid, value, length);
      int dacVal = map(val, 0, 100, 58, 232);
      dacWrite(ANALOG_OUT2, dacVal);
    } break;

    default: break;
  }
  return TY_SUCCESS;
}

/**
 * @description: Upload all DP status (called on connect or user request)
 */
void dp_update_all(void)
{
  my_device.mcu_dp_update(DPID_SWITCH1, normalRelay1, 1);
  my_device.mcu_dp_update(DPID_SWITCH2, normalRelay2, 1);
  my_device.mcu_dp_update(DPID_SWITCH3, relay3_state, 1);

  // report button DP states
  my_device.mcu_dp_update(DPID_BUTTON1, buttonDPState1, 1);
  my_device.mcu_dp_update(DPID_BUTTON2, buttonDPState2, 1);
  my_device.mcu_dp_update(DPID_BUTTON3, buttonDPState3, 1);
  my_device.mcu_dp_update(DPID_BUTTON4, buttonDPState4, 1);

  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);
  if (tempC > -100 && tempC < 125) {
    my_device.mcu_dp_update(DPID_TEMP, (int)tempC, 1);
  }

  int rawValue = analogRead(PRESSURE_PIN);
  int pressure = map(rawValue, 0, 4095, 1, 150);
  my_device.mcu_dp_update(DPID_PRESSURE, pressure, 1);
}
