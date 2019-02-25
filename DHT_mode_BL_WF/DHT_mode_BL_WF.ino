#include <Adafruit_Sensor.h>
#include "BluetoothSerial.h"
#include "EEPROM.h"
#define EEPROM_SIZE 50
#include <WiFi.h>
#include <DHT.h>
#include <DHT_U.h>
#define DHTPIN 25
#define DHTTYPE DHT22
#define MODE 35
#define modeBT 27
#define modeWF 26
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;
DHT dht (DHTPIN, DHTTYPE);
int addr = 0;
String dataArray [3];
const char* ssid;
const char* pass;
//const char* host = "http://10.99.254.30";
IPAddress host(10, 99, 254, 30);
boolean bluetoothOn;

void bacaEEPROM() {
  if (!EEPROM.begin(EEPROM_SIZE))
  {
    Serial.println("failed to initialise EEPROM"); delay(1000000);
  }
  int ptr = 0;
  dataArray[ptr] = "";
  for (int i = 0; i < EEPROM_SIZE; i++)
  {
    if (char (EEPROM.read(i)) == ';') {
      ptr++;
      dataArray[ptr] = "";
    }
    else {
      dataArray[ptr] += char (EEPROM.read(i));
    }
  }
  ssid = dataArray[0].c_str();
  pass = dataArray[1].c_str();
  Serial.println(ssid);
  Serial.println(pass);
  Serial.println();
}

void connectWiFi() {
  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, pass);
  unsigned long timeout1 = 10000;
  unsigned long starttimeout = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - starttimeout >= timeout1) break;
  }
  Serial.println(" CONNECTED");
}

void setup() {
  Serial.begin(115200);
  dht.begin();
  pinMode (MODE, INPUT);
  pinMode (modeBT, OUTPUT);
  pinMode (modeWF, OUTPUT);
  bacaEEPROM();
  connectWiFi();
  bluetoothOn = false;
  Serial.println("Ini void Setup");
}

void loop() {
  if (digitalRead (MODE) == LOW) {
    if (bluetoothOn == false) {
      SerialBT.begin("BT-TSAT01");
      bluetoothOn = true;
      Serial.println("BT nyala");
      delay(2000);
    }
    digitalWrite(modeBT, HIGH);
    digitalWrite(modeWF, LOW);
    if (SerialBT.available()) {
      while (SerialBT.available()) {
        char val = (char) SerialBT.read();
        EEPROM.write(addr, val);
        addr++;
      }
      for (int i = addr; i < EEPROM_SIZE; i++)
      {
        EEPROM.write(i, 0);
      }
      addr = 0;
      EEPROM.commit();
      bacaEEPROM();
      connectWiFi();
    }
  }

  else {
    if (bluetoothOn == true) {
      SerialBT.end();
      bluetoothOn = false;
      Serial.println("BT Mati");
      delay(2000);
    }
    digitalWrite(modeBT, LOW);
    digitalWrite(modeWF, HIGH);

    Serial.print("connecting to ");
    Serial.println(host);

    // Use WiFiClient class to create TCP connections
    WiFiClient client;
    const int httpPort = 8000;
    if (!client.connect(host, httpPort)) {
      Serial.println("connection failed");
      return;
    }
    float v = 220;
    float i = 0.28;
    float p = 150;
    float e = 1000;
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    Serial.print(String(h));
    Serial.print(" ");
    Serial.println(String(t));
    // Make a HTTP request:
    client.print("GET /user/data-iot?code=code&voltage=" + String(v) + "&current=" + String(i) + "&power=" + String(p) + "&energy=" + String(e) + "&humidity=" + String(h) + "&temperature=" + String(t));
    client.println(" HTTP/1.1");
    client.println("Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJfaWQiOiI1YzZmYmI2ZjQzMWRiZjEzNTc3MGMyMWIiLCJyb2xlIjoidXNlciIsImlhdCI6MTU1MDgyNjM5N30.f6RfrfhZcczq9ago4BUWcj2ipNfvFWB4s8m4AQ5vypw");
    client.println("Content-Type: application/json");
    client.print("Host: ");
    client.println(host);
    client.println("Connection: close");
    client.println();
    unsigned long timeout2 = millis();
    while (client.available() == 0) {
      if (millis() - timeout2 > 5000) {
        Serial.println(">>> Client Timeout !");
        client.stop();
        return;
      }
    }

    // Read all the lines of the reply from server and print them to Serial
    while (client.available()) {
      Serial.print((char) client.read());
    }
    Serial.println();
    Serial.println("closing connection");
    Serial.println();
    delay(2000);
  }
}






