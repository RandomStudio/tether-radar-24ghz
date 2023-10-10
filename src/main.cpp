/* **************************************************** 
* @brief 24GHz Microwave Radar Sensor

 * @copyright    [DFRobot](https://www.dfrobot.com), 2016
 * @copyright    GNU Lesser General Public License

* @author [Xiaoyu](Xiaoyu.zhang@dfrobot.com)
* @version  V1.0
* @date  2019-03-11

* GNU Lesser General Public License.
* All above must be included in any redistribution
* ****************************************************/
#include <Arduino.h>
#include <ArduinoMqttClient.h>
#include <ArduinoJson.h> 
#include <SoftwareSerial.h>
#include <WiFi.h>

// #define WLAN_SSID "tether_2.4"
// #define WLAN_PASS "sp_ceB0ss!"
#define WLAN_SSID "Random Guest"
#define WLAN_PASS "beourguest"

#define MQTT_IP "10.112.20.165" 
#define MQTT_PORT 1883
#define MQTT_USER "tether"
#define MQTT_PASS "sp_ceB0ss!"

#define AGENT_ROLE "dfrobotSEN0306"
#define SEND_INTERVAL 16

char col;// For storing the data read from serial port
unsigned char buffer_RTT[8] = {};
int YCTa = 0, YCTb = 0,YCT1 = 0;
SoftwareSerial mySerial(27, 33);

WiFiClient wifiClient;
MqttClient mqtt(wifiClient);

String agentRole = String(AGENT_ROLE);
String outputTopicStatus = agentRole + "/any/status";
String outputTopicDistance = agentRole + "/any/distance";


StaticJsonDocument<32> outputDoc;
std::string outputMessage;

bool sensorOK;
const long interval = SEND_INTERVAL;
unsigned long savedTime = 0;


void connectWiFi() {
  Serial.println("Connecting to WiFi");
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  delay(2000);
  bool ledOn = false;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(LED_PIN, ledOn ? HIGH : LOW);
    ledOn = !ledOn;
  }
  Serial.println("...WiFi connected");
  digitalWrite(LED_PIN, HIGH);
}


void sendStatus(bool sensorOK = true) {
   outputDoc["sensorOK"] = sensorOK;

  outputMessage = ""; // clear the output string

  serializeMsgPack(outputDoc, outputMessage); // serialize the data

  // send: retain = false, QOS 2
  mqtt.beginMessage(outputTopicStatus, false, 2);

  for (int i = 0; i < outputMessage.length(); i++) {
    mqtt.print(outputMessage[i]);
  }

  mqtt.endMessage();
}



// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care of connecting.
void MQTT_connect() {
  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT Broker @ ");
  Serial.print(MQTT_IP);
  Serial.println(" ... ");

  mqtt.setUsernamePassword(MQTT_USER, MQTT_PASS);

  while (!mqtt.connect(MQTT_IP, MQTT_PORT)) {
    Serial.println(mqtt.connectError());
    Serial.println("Retrying MQTT connection in 5 seconds...");
    delay(5000);  // wait 5 seconds
  }

  Serial.println("...MQTT Connected!");

  sendStatus(false);

}

void sendDistance(int d) {
  // JsonArray array = outputDoc.to<JsonArray>();
  
  // outputDoc["quat"] = JsonArray[x,y,z,w];
  outputDoc = JsonInteger(d);

  outputMessage = ""; // clear the output string
  serializeMsgPack(outputDoc, outputMessage); // serialize the data

  // send: retain = false, QOS 0
  mqtt.beginMessage(outputTopicDistance, false, 0);

  for (int i = 0; i < outputMessage.length(); i++) {
    mqtt.print(outputMessage[i]);
  }

  mqtt.endMessage();
}

void updateOutputTopics(String id) {
  outputTopicStatus = agentRole + "/" + id + "/status";
  outputTopicDistance = agentRole + "/" + id + "/distance";
}

void setup() {
  sensorOK = false;
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  mySerial.begin(57600);
  Serial.begin(115200);
  delay(10);
  connectWiFi();

  String id = WiFi.macAddress();
  updateOutputTopics(id);

  delay(1000);
  MQTT_connect();
}



void loop() {

  unsigned long currentTime = millis();


  if (currentTime - savedTime < interval) {
    return;
  }
  
   // call poll() regularly to allow the library to send MQTT keep alives which avoids being disconnected by the broker
  mqtt.poll();

  savedTime = currentTime;

  // check if we're connected, and reconnect if not
  MQTT_connect();

  // Send data only when received data
  if (mySerial.read() == 0xff)
  {

    // First time got a reading; send status OK
    if (!sensorOK) {
      sensorOK = true;
      sendStatus(true);
    }

    // Read the incoming byte.
    for (int j = 0; j < 8; j++)
    {
      col = mySerial.read();
      buffer_RTT[j] = (char)col;
      delay(2);
    }
    mySerial.flush();
    if (buffer_RTT[1] == 0xff)
    {
      if (buffer_RTT[2] == 0xff)
      {
        YCTa = buffer_RTT[3];
        YCTb = buffer_RTT[4];
        YCT1 = (YCTa << 8) + YCTb;
      }
    } // Read the obstacle distance of maximum reflection intensity
    // Serial.println(YCT1); // Output the obstacle distance of maximum reflection intensity
    sendDistance(YCT1);
  }
}