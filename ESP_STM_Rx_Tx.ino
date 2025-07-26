#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

const char* ssid = "Mi 11 Lite 5G";
const char* password = "5taek4tadui";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

#define START_BYTE     0xAA
#define END_BYTE       0x55
#define TYPE_WAVEFORM  0x01
#define RX_PIN 4  // Update if needed
#define TX_PIN 5

HardwareSerial SerialFromSTM(2);  // UART2 for STM32

uint8_t data[324];
uint8_t type = 0, length = 0, checksum = 0, dataIndex = 0;
enum ParseState { WAIT_START, READ_TYPE, READ_LENGTH, READ_DATA, READ_CHECKSUM, WAIT_END };
ParseState state = WAIT_START;

// ===== Web UI HTML =====
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Oscilloscope via ESP32</title>
  <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
</head>
<body>
  <h2>Waveform Display</h2>
  <canvas id="waveformChart" width="600" height="250"></canvas>
  <h3 id="frequency">Frequency: Waiting...</h3>
<script>
let ctx = document.getElementById('waveformChart').getContext('2d');
let chart = new Chart(ctx, {
  type: 'line',
  data: {
    labels: Array(320).fill(''),
    datasets: [{
      label: 'Waveform',
      borderColor: 'rgb(0, 200, 255)',
      data: Array(320).fill(0),
      fill: false,
      pointRadius: 0,
      borderWidth: 1,
      tension: 0.1
    }]
  },
  options: {
    animation: false,
    scales: { y: { min: 0, max: 255 } }
  }
});
let freqDisplay = document.getElementById("frequency");
let ws = new WebSocket('ws://' + location.hostname + '/ws');
ws.onmessage = function(event) {
  let msg = JSON.parse(event.data);
  if(msg.type === "waveform"){
    chart.data.datasets[0].data = msg.data;
    chart.update();
    if ('freq' in msg) {
      freqDisplay.innerText = `Frequency: ${msg.freq.toFixed(1)} Hz`;
    }
  }
};
</script>
</body>
</html>
)rawliteral";

// ===== Convert waveform to JSON for browser =====
String createWaveformJson(uint8_t* d, uint8_t len) {
  int dataCount = min((int)len, 320);
  String s = "{\"type\":\"waveform\",\"data\":[";
  for (int i = 0; i < dataCount; i++) {
    s += String(d[i]);
    if (i < dataCount - 1) s += ",";
  }
  s += "]";
  if (len >= 324) {
    float freq;
    memcpy(&freq, &d[320], 4);
    s += ",\"freq\":" + String(freq, 1);
  }
  s += "}";
  return s;
}

void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type,
               void * arg, uint8_t * data, size_t len){
  // Currently unused
}

void setup() {
  Serial.begin(115200);  // For debug
  SerialFromSTM.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected! IP: " + WiFi.localIP().toString());

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });
  server.begin();
}

void loop() {
  while (SerialFromSTM.available()) {
    uint8_t byte = SerialFromSTM.read();

    switch (state) {
      case WAIT_START:
        if (byte == START_BYTE) state = READ_TYPE;
        break;

      case READ_TYPE:
        type = byte;
        state = READ_LENGTH;
        break;

      case READ_LENGTH:
        length = byte;
        dataIndex = 0;
        checksum = START_BYTE ^ type ^ length;
        state = READ_DATA;
        break;

      case READ_DATA:
        data[dataIndex++] = byte;
        checksum ^= byte;
        if (dataIndex == length) state = READ_CHECKSUM;
        break;

      case READ_CHECKSUM:
        if (byte == checksum) state = WAIT_END;
        else state = WAIT_START;
        break;

      case WAIT_END:
        if (byte == END_BYTE && type == TYPE_WAVEFORM) {
          // Send to WebSocket
          ws.textAll(createWaveformJson(data, length));

          // Also print to Serial Monitor
          Serial.println("Waveform:");
          for (int i = 0; i < length - 4; i++) {
            Serial.print(data[i]);
            Serial.print(i < length - 5 ? "," : "\n");
          }

          float freq;
          memcpy(&freq, &data[length - 4], 4);
          Serial.printf("Frequency: %.2f Hz\n", freq);
        }
        state = WAIT_START;
        break;
    }
  }

  ws.cleanupClients();
}
