#include <Servo.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <AESLib.h>

//TEST with Mosquito MQTT broker: 
//cd "\Program Files (x86)\mosquitto"
//mosquitto_sub.exe -h 127.0.0.1 -i arduino_SUB5 -t arduino_1/sensor

//TODO: AES Encryption

#define ARDUINO_CLIENT_ID "arduino_1"                     // Client ID for Arduino pub/sub
#define PUB_ARDUINO "arduino_1/sensor"
//#define PUB_LIGHT "arduino_1/sensor/light"              // MQTT topic for light density
//#define PUB_SERVO1 "arduino_1/pos/servo1"               // MQTT topic for control of pos for servo 1
//#define PUB_SERVO2 "arduino_1/pos/servo2"
#define SUB_POS1 "arduino_1/input/servo1"                           // MQTT topic for INPUT
#define SUB_POS2 "arduino_1/input/servo2"                           // MQTT topic for INPUT
#define PUBLISH_DELAY 3000                                // Publishing delay [ms]
#define MQTTUSER "USER1"          //user for mqtt auth
#define MQTTPASSWORD  "PASS1"     //password for mqtt auth

// Networking details
byte mac[]    = {  0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02 };  // Ethernet shield (W5100) MAC address
IPAddress ip(10, 43, 11, 105);                           // Ethernet shield (W5100) IP address
IPAddress mqtt_server(10, 43, 11, 207);                       // MQTT server IP address
//const char* mqtt_server = "cloud.com";
EthernetClient ethClient;
PubSubClient client(ethClient);

Servo Servo1;
Servo Servo2;

byte plain[] =    "            ";
byte reset[] =    "            ";

//init vars
int debug = 0;
int scan = 0;

long interval = 500;  //5 sec
int servoSpeed = 10;
int scanSpeed = 10;

int lightresitor = A0;    //IO PIN light resistor
int lightvalue = 0;     //lowest: 97, highest: ~1100
int button_left = 7;      //IO PIN Button
int button_left_state = 0;
int button_right = 6;
int button_right_state = 0;
int button1_left = 5;
int button1_left_state = 0;
int button1_right = 4;
int button1_right_state = 0;
unsigned long counter = 0;

long previousMillis = 0;
int light_percentage = 0;
int angle_servo_1 = 90;
int angle_servo_2 = 90;

char tmpBuffer[20];
String allVarString = "";
int mqttAngle1;
int mqttAngle2;

void setup() {
  pinMode(button_left, INPUT);
  pinMode(button_right, INPUT);
  Serial.begin(9600);
  Servo1.attach(8);
  Servo2.attach(9);

  //init pos of servos to 90°
  initServo(90);

  // MTTQ parameters
  client.setServer(mqtt_server, 1883);  //adress and port of mqtt broker
  client.setCallback(callback);
  // Ethernet shield configuration
  Ethernet.begin(mac, ip);
  delay(1500); // Allow hardware to stabilize
  previousMillis = millis();
}

void loop() {

  if (!client.connected())  //if not already, connect to mqtt broker
    reconnect();

  buttonsControl();
  lightSensor();
  showData();
  if (scan)       //start scan
    startScan();

  client.loop();  
}
void mqqtServoControl(int angle, int servoNr) {
  if ((servoNr == 1) && (angle != angle_servo_1)) {
    angle_servo_1 = angle;
    writeServo1(angle_servo_1);
  }
  if ((servoNr == 2) && (angle != angle_servo_2)) {
    angle_servo_2 = angle;
    writeServo2(angle_servo_2);
  }
}

void reconnect() {
  // Loop until reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection to... ");
    Serial.println(mqtt_server);
    // Attempt to connect
    if (client.connect(ARDUINO_CLIENT_ID)) {
      //if (client.connect(ARDUINO_CLIENT_ID, MQTTUSER, MQTTPASSWORD)) {
      Serial.println("connected");
      // (re)subscribe
      client.subscribe(SUB_POS1);
      client.subscribe(SUB_POS2);
    } else {
      Serial.print("Connection failed, state: ");
      Serial.print(client.state());
      Serial.println(", retrying in 5 seconds");
      delay(5000); // Wait 5 seconds before retrying
    }
  }
}
// sub callback function
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("[sub: ");
  Serial.print(topic);
  Serial.print("]: ");
  char message[length + 1] = "";
  for (int i = 0; i < length; i++)
    message[i] = (char)payload[i];
  message[length] = '\0';
  Serial.println(message);

  // SUB_POS topic section
  if (strcmp(topic, SUB_POS1) == 0)
  {
    mqttAngle1 = atoi(message);
    Serial.print("mqttAngle1: ");
    Serial.println(mqttAngle1);
    mqqtServoControl(mqttAngle1, 1);
  }
  if (strcmp(topic, SUB_POS2) == 0)
  {
    mqttAngle2 = atoi(message);
    Serial.print("mqttAngle2: ");
    Serial.println(mqttAngle2);
    mqqtServoControl(mqttAngle2, 2);
  }
}

void publishData(char data[]) {
  client.publish(PUB_ARDUINO, data);
  //client.publish(PUB_LIGHT, dtostrf(light_percentage, 6, 2, tmpBuffer));
  //client.publish(PUB_SERVO1, dtostrf(angle_servo_1, 6, 2, tmpBuffer));
  //client.publish(PUB_SERVO2, dtostrf(angle_servo_2, 6, 2, tmpBuffer));
}

void showData() {
  counter = millis();
  if (counter - previousMillis > interval)  //print out data every intervall[s] to serial interface
  {
    if(debug) {
      Serial.print("Lightvalue: ");
      Serial.println(lightvalue);
      Serial.print("Lightpercentage: ");
      Serial.println(light_percentage);
      Serial.print("angle_servo_1: ");
      Serial.println(angle_servo_1);
      Serial.print("angle_servo_2: ");
      Serial.println(angle_servo_2);
    }
    allVarString = String(light_percentage) + "," + String(angle_servo_1) + "," + String(angle_servo_2);
    Serial.println("allVarString: Light / Servo1 / Servo 2");
    Serial.println(allVarString);
    allVarString.toCharArray(plain, allVarString.length()+1);
    char plain1[] = "";
    allVarString.toCharArray(plain1, allVarString.length()+1);
    if (debug) {
      Serial.println(allVarString.length());
      Serial.println(sizeof(plain)); 
    }
    Serial.println();
    Serial.println();
    Serial.println("AES Start: ");
    uint8_t key[] = {0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5};
    char data[] = "0123456789012345"; //16 chars == 16 bytes
    Serial.println(data);
    Serial.println(sizeof(data));
    aes128_enc_single(key, data);
    Serial.print("encrypted: ");
    
    Serial.println();
    publishData(data);
    String str;
    for (byte i=0; i<16; i++) {
      Serial.write(data[i]&0xFF);
    }
    Serial.println(" HEX:");
    for (byte i=0; i<16; i++) {
      Serial.print(data[i]&0xFF,HEX);
      Serial.print(" ");
    }
    Serial.println();
    Serial.println("Cipher in BIN: ");
    for (byte i=0; i<16; i++) {
      Serial.print(data[i],BIN);
      str = str + String(data[i]&0xFF,HEX);
    }
    Serial.println("Cipher in orig: ");
    for (byte i=0; i<16; i++) {
      Serial.print(data[i]);
    }
    Serial.println();
    Serial.println("String: ");
    Serial.println(str);
    Serial.println(str.length());
    Serial.println();
    aes128_dec_single(key, data);
    Serial.println();
    Serial.print("decrypted: ");
    Serial.println(data);
    if (!scan) {
      publishData(plain);    //if not in scan mode, publish to mqtt broker
    }
    memcpy(plain, reset, sizeof(plain));
    previousMillis = counter;
  }
}
void lightSensor() {
  lightvalue = analogRead(lightresitor);
  light_percentage = map(lightvalue, 95, 1100, 0, 100);
}
void buttonsControl() {

  button_left_state = digitalRead(button_left);
  button_right_state = digitalRead(button_right);
  button1_left_state = digitalRead(button1_left);
  button1_right_state = digitalRead(button1_right);

  if ((button_left_state == HIGH) && (button_right_state == HIGH))
  {
    scan = 1; //start scan
  }
  else {
    if ((button_left_state == HIGH) && (angle_servo_1 < 180))
    {
      //Servo to left
      if (debug) {
        Serial.println("left");
      }
      angle_servo_1 = angle_servo_1 + 1;
      writeServo1(angle_servo_1);
    }
    if ((button_right_state == HIGH) && (angle_servo_1 > 0))
    {
      //Servo to right
      if (debug) {
        Serial.println("right");
      }
      angle_servo_1 = angle_servo_1 - 1;
      writeServo1(angle_servo_1);
    }
  }
  if ((button1_left_state == HIGH) && (button1_right_state == HIGH))
  {
    scan = 1;
  }
  else {
    if ((button1_left_state == HIGH) && (angle_servo_2 < 180))
    {
      //Servo to left
      if (debug) {
        Serial.println("2left");
      }
      angle_servo_2 = angle_servo_2 + 1;
      writeServo2(angle_servo_2);
    }
    if ((button1_right_state == HIGH) && (angle_servo_2 > 0))
    {
      //Servo to right
      if (debug) {
        Serial.println("2right");
      }
      angle_servo_2 = angle_servo_2 - 1;
      writeServo2(angle_servo_2);
    }
  }
}
void writeServo2(int angle) {
  Servo2.write(angle);
  delay(servoSpeed);
}
void writeServo1(int angle) {
  Servo1.write(angle);
  delay(servoSpeed);
}
void startScan() {
  Serial.println("AUTO SCAN START");
  initServo(0);
  delay(500);
  Serial.println(scanSpeed);
  for (angle_servo_2 = 0; angle_servo_2 < 180 - 10; angle_servo_2 = angle_servo_2 + 10) {
    scanSpeed = 10;
    writeServo2(angle_servo_2);
    for (angle_servo_1 = 0; angle_servo_1 > -10 ; angle_servo_1 = angle_servo_1 + scanSpeed) {
      writeServo1(angle_servo_1);
      if (debug) {
        Serial.print("angle_servo_1: ");
        Serial.println(angle_servo_1);
      }
      lightSensor();
      showData();
      publishData(plain);
      delay(servoSpeed * 10);
      if (angle_servo_1 == 180) {
        scanSpeed = -10;
        angle_servo_2 = angle_servo_2 + 10;
        delay(servoSpeed * 5);
        writeServo2(angle_servo_2);
      }
    }
  }
  initServo(90);
}
void initServo(int initAngle) {
  //init
  angle_servo_1 = initAngle;
  angle_servo_2 = initAngle;
  writeServo1(angle_servo_1);
  writeServo2(angle_servo_2);
  scan = 0;
}
