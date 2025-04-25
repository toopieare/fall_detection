#define MQTT_MAX_PACKET_SIZE 1024  

#include <WiFi.h>
#include <PubSubClient.h>

#define MIC_PIN        34
#define SAMPLE_RATE    8000           
#define CHUNK_SIZE     64            

const char* ssid        = "";
const char* password    = "";
const char* mqtt_server = "test.mosquitto.org";
const int   mqtt_port   = 1883;
const char* mqtt_topic  = "esp32/audio/raw";

WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);

static int16_t chunkBuf[CHUNK_SIZE];

void connectWiFi() {
  Serial.print("WiFi…");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print('.');
  }
  Serial.println(" ✔");
}

void connectMQTT() {
  mqtt.setServer(mqtt_server, mqtt_port);
#if defined(PubSubClient_VERSION)
  mqtt.setBufferSize(MQTT_MAX_PACKET_SIZE);
  Serial.printf("Set MQTT buffer to %d bytes\n", MQTT_MAX_PACKET_SIZE);
#endif

  Serial.print("MQTT…");
  while (!mqtt.connected()) {
    if (mqtt.connect("ESP32_Audio")) {
      Serial.println(" ✔");
    } else {
      Serial.print('.');
      delay(200);
    }
  }
}

void setup() {
  Serial.begin(115200);
  connectWiFi();
  connectMQTT();
}

void loop() {
  if (!mqtt.connected()) {
    connectMQTT();
  }
  mqtt.loop();

  for (int i = 0; i < CHUNK_SIZE; ++i) {
    int raw = analogRead(MIC_PIN);
    chunkBuf[i] = raw - 2048;
    delayMicroseconds(1000000UL / SAMPLE_RATE);
  }

  size_t nbytes = CHUNK_SIZE * sizeof(int16_t);
  mqtt.publish(mqtt_topic, (uint8_t*)chunkBuf, nbytes, false);
}


