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
#include <SoftwareSerial.h>
#include <WiFi.h>

// #define WLAN_SSID "tether_2.4"
// #define WLAN_PASS "sp_ceB0ss!"
#define WLAN_SSID "Random Guest"
#define WLAN_PASS "beourguest"

char col;// For storing the data read from serial port
unsigned char buffer_RTT[8] = {};
int YCTa = 0, YCTb = 0,YCT1 = 0;
SoftwareSerial mySerial(27, 33);

WiFiClient wifiClient;

void connectWiFi();

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  mySerial.begin(57600);
  Serial.begin(115200);
  delay(10);
  connectWiFi();
}

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
  Serial.println("WiFi connected");
  digitalWrite(LED_PIN, HIGH);
}

void loop() {
  // Send data only when received data
  if (mySerial.read() == 0xff)
  {
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
    Serial.println(YCT1); // Output the obstacle distance of maximum reflection intensity
  }
}