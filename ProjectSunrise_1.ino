#include <Servo.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>

//TEST with Mosquito MQTT broker: 
//cd "\Program Files (x86)\mosquitto"
//mosquitto.exe -v
//mosquitto_sub.exe -h 127.0.0.1 -i arduino_SUB5 -t arduino_1/sensor
//mosquitto_pub.exe -h 10.43.11.207 -i PUB -t arduino_1/input -m "10,30,0"

#define ARDUINO_CLIENT_ID "arduino_1"                     // Client ID for Arduino pub/sub
#define PUB_ARDUINO "arduino_1/sensor"                  
#define SUB_MQTT "arduino_1/input"            // MQTT topic for INPUT
#define PUBLISH_DELAY 3000                                // Publishing delay [ms]
#define MQTTUSER "USER1"          //user for mqtt auth
#define MQTTPASSWORD  "PASS1"     //password for mqtt auth

// Networking details
byte mac[]    = {  0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02 };  // Ethernet shield (W5100) MAC address
IPAddress ip(10, 43, 11, 105);                           // Ethernet shield (W5100) IP address
IPAddress mqtt_server(10, 43, 11, 207);                       // MQTT server IP address of RaspPi
//const char* mqtt_server = "test.mosquitto.org";
EthernetClient ethClient;
PubSubClient client(ethClient);

Servo Servo1;
Servo Servo2;

byte plain[] =    "            ";
byte reset[] =    "            ";
//String PUB_ARDUINO: Lightvalue, Servo1, Servo2 = nnn,nnn,nnn
//Sring SUB_MQTT: Servo1, Servo2, Scanmode(0/1) = nnn,nnn,n

// debug vars
int debug = 0;
int debugscan = 0;
int debugmqtt = 0;
int test = 0;   //1 = disable mqtt connection

//init vars
int scan = 0;
long interval = 10000;  //5 sec
int servoSpeed = 10;
int scanSpeed = 10;
int maxValServo1 = 90; 
int maxValServo2 = 90;
int maxValLight = 0;  
int autoScanCounter = 0; 
long autoScanInterval = 1000000;    //500 sec

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
  Serial.println("Project Sunrise");
}
void loop() {
  if (!client.connected() && !test)  //if not already, connect to mqtt broker
    reconnect();
  buttonsControl();
  lightSensor();
  showData();
  if (scan)       //start scan
    startScan();
  client.loop();  
}
void mqttServoControl(int angle, int servoNr) {
  if ((angle >= 0) && (angle <= 180)) {    
    if ((servoNr == 1) && (angle != angle_servo_1)) {
      angle_servo_1 = angle;
      writeServo1(angle_servo_1);
    }
    if ((servoNr == 2) && (angle != angle_servo_2)) {
      angle_servo_2 = angle;
      writeServo2(angle_servo_2);
    }
  }
  else 
    Serial.println("!!Invalid MQTT Servo Input!!");
}
void reconnect() {
  // Loop until reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection to... ");
    Serial.println(mqtt_server);
    // Attempt to connect
    //if (client.connect(ARDUINO_CLIENT_ID)) {      //without credentials
    if (client.connect(ARDUINO_CLIENT_ID, MQTTUSER, MQTTPASSWORD)) {
      Serial.println("connected");
      // (re)subscribe
      client.subscribe(SUB_MQTT);
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
  if (strcmp(topic, SUB_MQTT) == 0) {
    String part01 = getValue(message,',',0);
    String part02 = getValue(message,',',1);
    String part03 = getValue(message,',',2);
    mqttAngle1 = getValue(message,',',0).toInt();
    mqttAngle2 = getValue(message,',',1).toInt();
    
    
    if (getValue(message,',',2).toInt() == 0 || getValue(message,',',2).toInt() == 1)
      scan = getValue(message,',',2).toInt(); 
    if (!scan) {
      mqttServoControl(mqttAngle1, 1);
      mqttServoControl(mqttAngle2, 2);
    }
    if(debugmqtt) {
      Serial.print("Message: part01: ");
      Serial.println(part01);
      Serial.print("part02: ");
      Serial.println(part02);
      Serial.print("part03: ");
      Serial.println(part03);
      Serial.print("mqttAngle1: ");
      Serial.println(mqttAngle1);
      Serial.print("mqttAngle2: ");
      Serial.println(mqttAngle2);
      Serial.print("ScanMode: ");
      Serial.println(scan);
    } 
  }
}
void publishData(char data[]) {
  client.publish(PUB_ARDUINO, data);
}
void showData() {
  counter = millis();
  if ((counter - previousMillis > interval) && !scan) {  //print out data every intervall[s] to serial interface

    allVarString = String(light_percentage) + "," + String(angle_servo_1) + "," + String(angle_servo_2);
    Serial.println("allVarString: Light / Servo1 / Servo 2");
    Serial.println(allVarString);
    allVarString.toCharArray(plain, allVarString.length()+1);
    Serial.print("Next autoscan in ");
    Serial.print((autoScanInterval - (autoScanCounter*interval))/1000);
    Serial.println(" sec");

    if (!scan) {
      publishData(plain);    //if not in scan mode, publish to mqtt broker
    }
    memcpy(plain, reset, sizeof(plain));  //delete the last char array
    previousMillis = counter;
  
    autoScanCounter++; 
    if((autoScanCounter * interval) > autoScanInterval) { //do autoscan after interval time
      scan = 1;
      Serial.println("Scan because of interval");
      autoScanCounter = 0;  
    }
    if(debug && !scan) {
      Serial.print("Lightvalue: ");
      Serial.println(lightvalue);
      Serial.print("Lightpercentage: ");
      Serial.println(light_percentage);
      Serial.print("angle_servo_1: ");
      Serial.println(angle_servo_1);
      Serial.print("angle_servo_2: ");
      Serial.println(angle_servo_2);
      Serial.print("Scanmode: ");
      Serial.println(scan);
      Serial.print("Scancounter: ");
      Serial.println(autoScanCounter*interval);
      Serial.println("Size of the Strings: MQTT String, Arduino-Char Array"); 
      Serial.println(allVarString.length());
      Serial.println(sizeof(plain)); 
    } 
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

  if ((button_left_state == HIGH) && (button_right_state == HIGH)) {
    scan = 1; //start scan
  }
  else {
    if ((button_left_state == HIGH) && (angle_servo_1 < 180)) {
      //Servo to left
      if (debug) {
        Serial.println("left");
      }
      angle_servo_1 = angle_servo_1 + 1;
      writeServo1(angle_servo_1);
    }
    if ((button_right_state == HIGH) && (angle_servo_1 > 0)) {
      //Servo to right
      if (debug) {
        Serial.println("right");
      }
      angle_servo_1 = angle_servo_1 - 1;
      writeServo1(angle_servo_1);
    }
  }
  if ((button1_left_state == HIGH) && (button1_right_state == HIGH)) {
    scan = 1;
  }
  else {
    if ((button1_left_state == HIGH) && (angle_servo_2 < 180)) {
      //Servo to left
      if (debug) {
        Serial.println("2left");
      }
      angle_servo_2 = angle_servo_2 + 1;
      writeServo2(angle_servo_2);
    }
    if ((button1_right_state == HIGH) && (angle_servo_2 > 0)) {
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
  if ((angle < 0) || (angle > 180))
    Serial.println("!!!Angle 2 invalid!!!");
}
void writeServo1(int angle) {
  Servo1.write(angle);
  delay(servoSpeed);
  if ((angle < 0) || (angle > 180))
    Serial.println("!!!Angle 1 invalid!!!");
}
void startScan() {
  Serial.println("AUTO SCAN START");
  initServo(0);
  delay(500);
  //Serial.println(scanSpeed);
  for (angle_servo_2 = 0; angle_servo_2 <= 180; angle_servo_2 = angle_servo_2 + 10) {
    scanSpeed = 10;
    writeServo2(angle_servo_2);
    delay(servoSpeed * 20);
    if (debugscan) {
      Serial.print("angle_servo_2: ");
      Serial.println(angle_servo_2);
      Serial.println("");
    }
    for (angle_servo_1 = 0; angle_servo_1 > -10 ; angle_servo_1 = angle_servo_1 + scanSpeed) {
      writeServo1(angle_servo_1);
      if (debugscan) {
        Serial.print("angle_servo_1: ");
        Serial.println(angle_servo_1);
      }
      lightSensor();
      showData();
      allVarString = String(light_percentage) + "," + String(angle_servo_1) + "," + String(angle_servo_2);
      allVarString.toCharArray(plain, allVarString.length()+1);
      publishData(plain);
      memcpy(plain, reset, sizeof(plain));  //delete the last char array

      // save max Values
      if (maxValLight < light_percentage) {
        maxValLight = light_percentage; 
        maxValServo1 = angle_servo_1;
        maxValServo2 = angle_servo_2;
      }   
      delay(servoSpeed * 10);
      if (angle_servo_1 >= 180) {
        if (angle_servo_2 >= 180) {
          break; 
        }
        else {
          scanSpeed = -10;
          angle_servo_2 = angle_servo_2 + 10;
          delay(servoSpeed * 50);
          writeServo2(angle_servo_2);
          if (debugscan) {
            Serial.print("angle_servo_2: ");
            Serial.println(angle_servo_2);
            Serial.println("");
          }
        }
      }
    }
  }
  scanSpeed = 10;
  Serial.println("__________________");
  Serial.print("Finished Scan with max values, light: ");
  Serial.println(maxValLight);
  Serial.print("At Servo1 Pos: ");
  Serial.println(maxValServo1);
  Serial.print("At Servo2 Pos: ");
  Serial.println(maxValServo2);
  Serial.println("__________________");
  scan = 0;
  angle_servo_1 = maxValServo1;
  angle_servo_2 = maxValServo2;
  writeServo1(angle_servo_1);
  writeServo2(angle_servo_2);
  autoScanCounter = 0;
}
void initServo(int initAngle) {
  //init
  angle_servo_1 = initAngle;
  angle_servo_2 = initAngle;
  writeServo1(angle_servo_1);
  writeServo2(angle_servo_2);
  //scan = 0;
}
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i <= maxIndex && found <= index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }
  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}
