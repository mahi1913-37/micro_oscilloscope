// --- YOUR ORIGINAL INCLUDES AND SETUP ---
#include "SPI.h"
#include <EEPROM.h>
#include <Adafruit_GFX_AS.h>
#include <Adafruit_ILI9341_STM.h>
#include <STM32ADC.h>
#include <HardwareTimer.h>
#include <math.h>

// Component type enumeration (placed at top to fix compilation error)
enum ComponentType { RESISTOR, CAPACITOR, INDUCTOR, UNKNOWN };

STM32ADC myADC(ADC1);
#define ADC_PIN PA0
#define RLC_PIN PA1
#define LC_PIN PA3
#define maxSamples 4000
#define screenwidth 320
#define screenheight 240

#define TFT_CS     PB1
#define TFT_DC     PB10
#define TFT_RST    PB11

Adafruit_ILI9341_STM tft = Adafruit_ILI9341_STM(TFT_CS, TFT_DC, TFT_RST);

// Button Pins
#define BUTTON_HOLD      PB12
#define BUTTON_UP        PB14
#define BUTTON_DOWN      PB13
#define BUTTON_TIME_DIV  PB15
#define BUTTON_AMP_DIV   PB8
#define BUTTON_RLC       PB9    // New RLC button

int y_offset = 0;
bool hold = false;
int timeDivIndex = 0;
const float timeDivOptions[] = {50, 40, 30, 20, 10};
const float ampDivOptions[] = {0.5, 1.0, 2.0, 5.0};
int ampDivIndex = 1;

float lastAmplitude = 0.0;
float lastFrequency = 0.0;
float lastPosPeak = 0.0;
float lastNegPeak = 0.0;
float lastPeakToPeak = 0.0;

unsigned long lastWaveUpdate = 0;
unsigned long lastButtonCheck = 0;

// RLC Measurement variables
enum Mode { OSCILLOSCOPE, RLC_MODE };
Mode currentMode = OSCILLOSCOPE;

float measuredR = 0.0;
float measuredC = 0.0;
float measuredL = 0.0;

// --- ADDED: Packet Constants ---
#define START_BYTE  0xAA
#define END_BYTE    0x55
#define TYPE_WAVEFORM 0x01
#define TYPE_RLC      0x02

// --- ADDED: Send Packet Function ---
void sendPacket(uint8_t type, uint8_t* data, uint8_t length) {
  uint8_t checksum = START_BYTE ^ type ^ length;
  for (uint8_t i = 0; i < length; i++) checksum ^= data[i];

  Serial1.write(START_BYTE);
  Serial1.write(type);
  Serial1.write(length);
  Serial1.write(data, length);
  Serial1.write(checksum);
  Serial1.write(END_BYTE);
}


// --- CLASS FOR WAVEFORM STORAGE ---
class sam_hand {
  public:
    int samples[screenwidth];
    sam_hand() {
      for (int i = 0; i < screenwidth; i++)
        samples[i] = 0;
    }
    void updateArray(int x, int val) {
      if (x >= 0 && x < screenwidth) samples[x] = val;
    }
    void PrintArray() {
      for (int i = 0; i < screenwidth; i++) {
        Serial1.println(samples[i]);
      }
    }
};
sam_hand sam;

void drawGrid() {
  tft.fillScreen(ILI9341_BLACK);
  for (int x = 0; x < screenwidth; x += 32)
    tft.drawFastVLine(x, 0, screenheight, ILI9341_DARKGREY);
  for (int y = 0; y < screenheight; y += 24)
    tft.drawFastHLine(0, y, screenwidth, ILI9341_DARKGREY);
  tft.drawFastHLine(0, 120, screenwidth, ILI9341_DARKCYAN);
}

void setup() {
  tft.begin();
  tft.setRotation(1);
  drawGrid();

  pinMode(BUTTON_HOLD, INPUT_PULLUP);
  pinMode(BUTTON_UP, INPUT_PULLUP);
  pinMode(BUTTON_DOWN, INPUT_PULLUP);
  pinMode(BUTTON_TIME_DIV, INPUT_PULLUP);
  pinMode(BUTTON_AMP_DIV, INPUT_PULLUP);
  pinMode(BUTTON_RLC, INPUT_PULLUP);

  Serial1.begin(115200);  // <-- Use PA9 (TX) to send data to ESP32

  pinMode(ADC_PIN, INPUT);
}


void loop() {
  if (millis() - lastButtonCheck > 30) {
    handleButtons();
    lastButtonCheck = millis();
  }

  if (currentMode == OSCILLOSCOPE && !hold && millis() - lastWaveUpdate > 50) {
    plotWaveform(timeDivOptions[timeDivIndex]);
    displayMeasurements();
    sendWaveformPacket();  // 🔁 Send waveform to ESP32
    lastWaveUpdate = millis();
  } else if (currentMode == RLC_MODE) {
    measureAndDisplayRLC();
    sendRLCPacket();  // 🔁 Send RLC values to ESP32
    delay(200);
  }
}

void handleButtons() {
  static bool lastHold = HIGH, lastTimeDiv = HIGH;
  static bool lastAmpDiv = HIGH, lastRLC = HIGH;

  // HOLD toggle
  if (digitalRead(BUTTON_HOLD) == LOW && lastHold == HIGH)
    hold = !hold;
  lastHold = digitalRead(BUTTON_HOLD);

  // Time/Div toggle
  if (digitalRead(BUTTON_TIME_DIV) == LOW && lastTimeDiv == HIGH) {
    timeDivIndex = (timeDivIndex + 1) % (sizeof(timeDivOptions) / sizeof(timeDivOptions[0]));
    if (currentMode == OSCILLOSCOPE) drawGrid(); // Redraw grid on time/div change
  }
  lastTimeDiv = digitalRead(BUTTON_TIME_DIV);

  // Amp/Div toggle
  if (digitalRead(BUTTON_AMP_DIV) == LOW && lastAmpDiv == HIGH) {
    ampDivIndex = (ampDivIndex + 1) % (sizeof(ampDivOptions) / sizeof(ampDivOptions[0]));
    if (currentMode == OSCILLOSCOPE) drawGrid(); // Redraw grid on amp/div change
  }
  lastAmpDiv = digitalRead(BUTTON_AMP_DIV);

  // RLC Mode toggle
  if (digitalRead(BUTTON_RLC) == LOW && lastRLC == HIGH) {
    currentMode = (currentMode == OSCILLOSCOPE) ? RLC_MODE : OSCILLOSCOPE;
    tft.fillScreen(ILI9341_BLACK); // Clear screen on mode switch
    if (currentMode == OSCILLOSCOPE) drawGrid();
  }
  lastRLC = digitalRead(BUTTON_RLC);

  // UP/DOWN for vertical shift
  if (digitalRead(BUTTON_UP) == LOW)
    y_offset = max(y_offset - 2, -100);
  if (digitalRead(BUTTON_DOWN) == LOW)
    y_offset = min(y_offset + 2, 100);
}

void plotWaveform(float timeDivFactor) {
  int samples[screenwidth];
  int minVal = 4095, maxVal = 0;
  int prevVal = analogRead(ADC_PIN);
  unsigned long firstCrossTime = 0, secondCrossTime = 0;
  bool crossingDetected = false;

  unsigned long samplingInterval = timeDivFactor;
  unsigned long lastSampleTime = micros();

  for (int x = 0; x < screenwidth;) {
    if ((micros() - lastSampleTime) >= samplingInterval) {
      int val = analogRead(ADC_PIN);
      samples[x] = val;
      sam.updateArray(x, val);

      if (val < minVal) minVal = val;
      if (val > maxVal) maxVal = val;

      if (!crossingDetected && prevVal < 2048 && val >= 2048) {
        firstCrossTime = micros();
        crossingDetected = true;
      } else if (crossingDetected && prevVal < 2048 && val >= 2048 && secondCrossTime == 0) {
        secondCrossTime = micros();
      }

      prevVal = val;
      lastSampleTime += samplingInterval;
      x++;
    }
  }

  // Smoothing
  for (int i = 1; i < screenwidth - 1; i++) {
    samples[i] = (samples[i - 1] + samples[i] + samples[i + 1]) / 3;
  }

  float voltsPerUnit = 3.3 / 4095.0;
  float pixelsPerVolt = 40.0 / ampDivOptions[ampDivIndex];
  int yCenter = 120;  // Center of the screen

  // Clear only the waveform area (leave grid and text)
  tft.fillRect(0, 48, screenwidth, screenheight - 48, ILI9341_BLACK);

  // Track the absolute peak values (positive and negative)
  float posPeak = 0, negPeak = 3.3; // Initialize for 0V to 3.3V signal

  for (int x = 1; x < screenwidth; x++) {
    float v1 = samples[x - 1] * voltsPerUnit;  // Voltage value for sample 1
    float v2 = samples[x] * voltsPerUnit;      // Voltage value for sample 2

    // Calculate absolute peaks (positive and negative)
    posPeak = max(posPeak, v1);  // Track maximum positive voltage
    negPeak = min(negPeak, v1);  // Track minimum (most negative) voltage

    // Map the voltage to y-coordinates (center the waveform around 0V)
    int y1 = yCenter - (int)(v1 * pixelsPerVolt) + y_offset;  // Calculate y-coordinate for sample 1
    int y2 = yCenter - (int)(v2 * pixelsPerVolt) + y_offset;  // Calculate y-coordinate for sample 2

    // Constrain the y-values to ensure they stay within the screen boundaries
    y1 = constrain(y1, 0, 239);
    y2 = constrain(y2, 0, 239);

    // Draw the waveform on the TFT screen
    tft.drawLine(x - 1, y1, x, y2, ILI9341_WHITE);
  }

  // Calculate peak-to-peak voltage (absolute)
  lastPosPeak = posPeak;
  lastNegPeak = negPeak;
  lastPeakToPeak = posPeak - negPeak;  // Calculate peak-to-peak voltage

  // Calculate the amplitude (for display purposes)
  lastAmplitude = (maxVal - minVal) * voltsPerUnit;

  static float stableFreq = 0;
  if (firstCrossTime && secondCrossTime > firstCrossTime) {
    stableFreq = 1e6 / (secondCrossTime - firstCrossTime);  // Frequency calculation
  }
  lastFrequency = stableFreq;
}


void displayMeasurements() {
  // Clear only the text areas
  tft.fillRect(5, 5, 150, 40, ILI9341_BLACK);   // Left-side measurements
  tft.fillRect(160, 5, 150, 40, ILI9341_BLACK); // Right-side measurements

  tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
  tft.setTextSize(1);
  tft.setCursor(5, 5);
  tft.print("Time/div: "); tft.print(timeDivOptions[timeDivIndex]); tft.print(" us");
  tft.setCursor(160, 35);
  tft.print("Amp: "); tft.print(lastAmplitude, 2); tft.print(" V");
  tft.setCursor(5, 20);
  tft.print("Freq: "); tft.print(lastFrequency, 1); tft.print(" Hz");
  tft.setCursor(5, 35);
  tft.print("Amp/Div: "); tft.print(ampDivOptions[ampDivIndex], 2); tft.print(" V/div");
  tft.setCursor(160, 20);
  tft.print("Pos Peak: +"); tft.print(lastPosPeak, 3); tft.print(" V");
  tft.setCursor(160, 5);
  tft.print("Neg Peak: -"); tft.print(lastNegPeak, 3); tft.print(" V");
}

// ComponentType identifyComponent() {
//   const float VREF = 3.3;       // ADC reference voltage
//   const int ADC_MAX = 4095;     // 12-bit ADC
//   const float R_known = 1000.0; // Known resistor value in ohms (1k)

//   // Step 1: Test for Resistor (steady-state voltage divider)
//   analogRead(RLC_PIN); // Discard first sample
//   delay(10);
//   int adcVal = analogRead(RLC_PIN);
//   float Vadc = (adcVal * VREF) / ADC_MAX;
//   if (Vadc > 0.1 && Vadc < VREF - 0.1) { // Valid resistor voltage (0.1V to 3.2V)
//     return RESISTOR;
//   }

//   // Step 2: Test for Capacitor (charging behavior)
//   pinMode(PB7, OUTPUT);
//   digitalWrite(PB7, LOW); // Discharge
//   delay(10);
//   digitalWrite(PB7, HIGH); // Start charging
//   unsigned long tStart = micros();
//   int val1 = analogRead(RLC_PIN);
//   delayMicroseconds(100); // Wait 100 µs
//   int val2 = analogRead(RLC_PIN);
//   digitalWrite(PB7, LOW); // Stop charging
//   float V1 = (val1 * VREF) / ADC_MAX;
//   float V2 = (val2 * VREF) / ADC_MAX;
//   if (V2 > V1 + 0.05 && V2 < VREF - 0.1) { // Voltage rising but not at VREF
//     return CAPACITOR;
//   }

//   // Step 3: Test for Inductor (inductive kick)
//   digitalWrite(PB7, HIGH); // Apply voltage to inductor
//   delayMicroseconds(20);   // Brief pulse to trigger the inductor
//   pinMode(PB7, INPUT);     // Set PB7 to input (float pin to detect kick)
//   delayMicroseconds(5);    // Short delay to allow kick to develop
//   int valKick = analogRead(RLC_PIN);  // Measure voltage spike across inductor
//   float Vkick = (valKick * VREF) / ADC_MAX;  // Convert ADC value to voltage
//   pinMode(PB7, OUTPUT);    // Set PB7 back to output (reset state)
//   digitalWrite(PB7, LOW);  // Reset PB7 to LOW
//   if (Vkick > VREF * 0.5) { // Lower threshold to detect smaller inductors
//     return INDUCTOR;
//   }

//   return UNKNOWN; // No component or invalid response
// }

void measureAndDisplayRLC() {
  // Placeholder estimation — replace with real hardware logic
  const float VREF = 3.3;            // ADC reference voltage
  const int ADC_MAX = 4095;          // 12-bit ADC
  const float R_known = 1000.0;      // Known resistor value in ohms (1k)
  const float C_threshold = 2.0;     // Voltage to stop timing for capacitance
  const float L_sampleTime = 20e-6;  // Time step for initial slope estimation

  tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.setCursor(5, 5);
  tft.print("RLC MODE");

  tft.setTextSize(1);

  // ----------- RESISTANCE -----------
  analogRead(RLC_PIN); delay(10);  // discard first sample
  int adcVal = analogRead(RLC_PIN);
  float Vadc = (adcVal * VREF) / ADC_MAX;

  if (Vadc < VREF && Vadc > 0.1) {
    measuredR = R_known * (Vadc / (VREF - Vadc));
  } else {
    measuredR = 0;
  }

  tft.setCursor(5, 40);
  tft.print("R = ");
  tft.print(measuredR, 1);
  tft.print(" Ohm");

  // ----------- CAPACITANCE -----------
    pinMode(PB7, OUTPUT);
  digitalWrite(PB7, LOW);
  delay(50);  // Longer discharge for high C

  digitalWrite(PB7, HIGH);  // Start charging

  unsigned long tStart = micros();
  float Vcap = 0;
  const unsigned long maxTimeout = 3000000UL;  // 3 seconds timeout

  while (true) {
    // Average 5 samples to reduce noise
    long sum = 0;
    for (int i = 0; i < 5; i++) {
      sum += analogRead(LC_PIN);
      delayMicroseconds(100);
    }
    int avgVal = sum / 5;
    Vcap = (avgVal * VREF) / ADC_MAX;

    if (Vcap >= C_threshold || (micros() - tStart) > maxTimeout) break;
  }

  unsigned long tCharge = micros() - tStart;

  if (Vcap < VREF && Vcap > 0.1) {
    float lnTerm = log(1.0 / (1.0 - (Vcap / VREF)));
    if (lnTerm > 0.00001)  // Avoid division by very small numbers
      measuredC = ((float)tCharge / (R_known * 1e6 * lnTerm)) ;  // in Farads
    else
      measuredC = 0;
  } else {
    measuredC = 0;
  }

  tft.setCursor(5, 60);
  tft.print("C = ");
  tft.print(measuredC * 1e6, 2);
  tft.print(" uF");



  // ----------- INDUCTANCE -----------
  digitalWrite(PB7, LOW); delay(10);  // Discharge inductor magnetic field
  pinMode(PB7, OUTPUT);
  digitalWrite(PB7, HIGH);           // Apply step voltage

  delayMicroseconds(2);              // Let current begin to rise

  int V1 = analogRead(LC_PIN);      // Sample 1
  delayMicroseconds(L_sampleTime * 1e6);
  int V2 = analogRead(LC_PIN);      // Sample 2

  float dV = ((V2 - V1) * VREF) / ADC_MAX;
  float dT = L_sampleTime;           // In seconds

  if (dV > 0.01) {
    measuredL = (R_known * dT * VREF) / dV;  // in Henries
  } else {
    measuredL = 0;
  }

  tft.setCursor(5,80);
  tft.print("L = ");
  tft.print(measuredL * 1e3, 2);
  tft.print(" mH");

  pinMode(PB7, INPUT);  // Float PB7 after measurements
}


void sendWaveformData() {
  sam.PrintArray();
}

void sendWaveformPacket() {
  uint8_t buffer[100];
  for (int i = 0; i < 100; i++) {
    buffer[i] = sam.samples[i] >> 4;  // 12-bit to 8-bit compression
  }
  sendPacket(TYPE_WAVEFORM, buffer, 100);
}

// --- ADDED: Send R, L, C as 3 floats (12 bytes) ---
void sendRLCPacket() {
  uint8_t buffer[12];
  memcpy(&buffer[0], &measuredR, 4);
  memcpy(&buffer[4], &measuredL, 4);
  memcpy(&buffer[8], &measuredC, 4);
  sendPacket(TYPE_RLC, buffer, 12);
}
