
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h" 
#include "Adafruit_MQTT_Client.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "stdbool.h"
//#include "String.h";

//nRF24 kütüphaneleri
#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>

#define WiFi_Led D0
#define MQTT_Led D1
#define TX_Topic "NodeMCU_TEST_TX"
#define RX_Topic "NodeMCU_TEST_RX1"

boolean WiFi_Connect = false;
boolean MQTT_Connect = false;

#define WLAN_SSID       "AET_2.4GHz"            //WiFi SSID
#define WLAN_PASS       "16megabit"             //WiFi Password
#define MQTT_SERVER      "192.168.1.147"        //MQTT Static IP Adress
#define MQTT_PORT         1883                  //MQTT Port Adress  
#define MQTT_USERNAME    ""                     //MQTT Server Username
#define MQTT_PASSWORD         ""                //MQTT Server Password

//MQTT Publish (Yazma) ve Subscribe (Okuma) Değişkenleri
WiFiClient client; 
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, MQTT_PORT, MQTT_USERNAME, MQTT_PASSWORD); //MQTT Client nesneni oluştur
Adafruit_MQTT_Publish TX = Adafruit_MQTT_Publish(&mqtt, MQTT_USERNAME TX_Topic);         //MQTT Veri Gönderme Publish Topic Adres
Adafruit_MQTT_Subscribe RX = Adafruit_MQTT_Subscribe(&mqtt, MQTT_USERNAME RX_Topic);     //MQTT Veri Okuma Subscribe Topic Adres

String UART_DATA = "";
boolean completeReading = false;

RF24 nrf24(2, 4); // use this for NodeMCU Amica/AdaFruit Huzzah ESP8266 Feather
// RH_NRF24 nrf24(8, 7); // use this with Arduino UNO/Nano
 
const byte address[6] = "00001";
unsigned long timer = 0;

//DBMS_Sensors Data Struct 
typedef struct{
  byte leftMotorRPM;        //Sol Motor RPM
  byte rightMotorRPM;       //Sağ Motor RPM
  float leftMotorCurrent;   //Sol Motor Akım
  float rightMotorCurrent;  //Sağ Motor Akım
  float batteryCurrent;     //Akü Akım
  byte batteryLevel;        //Akü Seviyesi
  byte agvSpeed;            //AGV Hız cm/s
  byte loadcell;            //Ağırlık Sensörü
  byte ecuTemperature;      //ECU Sıcaklık
  int forwardDistance;    //Ön Mesafe
  byte leftDistance;       //Sol Mesafe
  byte rightDistance;      //Sağ Mesafe
  byte compass;            //Pusula
  byte dir;
//  byte IMU_ax;             //IMU X Ekseninde İvmelenme
//  byte IMU_ay;             //IMU Y Ekseninde İvmelenme
//  byte IMU_az;             //IMU Z Ekseninde İvmelenme
  byte fanState;         //Fan Durumu
  byte carLights;        //AGV Aydınlatma Durumu
  byte task;                //Görev Bilgisi
}DBMS_Sensors;

DBMS_Sensors sensors = {150, 149, 1.5, 1.45, 8.72, 95, 15, 0, 30, 75, 100, 100, 180, 4, 1, 0, 1};
//SOF,150,149,1.5,1.45,8.72,95,15,0,30,75,100,100,180,4,1,0,1,EOF

unsigned long agv_sensors_previousTime = 0;
unsigned long agv_indoor_previousTime = 0;
unsigned long currentTime = 0;
long agv_sensors_sample_time = 5000;
long agv_indoor_sample_time = 1200;

void WiFi_MQTT_Led_Test();
void WiFi_connect();
void MQTT_connect();
void MQTT_Check();
boolean DBMS_Gateway_Transmit();
void DBMS_Sensors_Display();
void Sensor_Layer_Parse(String);
void serialCommunication();
  
void setup()
{
  Serial.begin(115200);
  
  pinMode(WiFi_Led, OUTPUT);
  pinMode(MQTT_Led, OUTPUT);

  delay(1000);
  WiFi_connect();
  mqtt.subscribe(&RX);
  Serial.println("\nConnecting to MQTT | MQTT Server IP: " + String(MQTT_SERVER) + " | MQTT Port: " + String(MQTT_PORT)); 
  MQTT_connect();
  
  //client.subscribe("RX");
  //Serial.println("nRF24L01 Baslatiliyor");
  boolean state = nrf24.begin();
  nrf24.openWritingPipe(address);
  nrf24.setPALevel(RF24_PA_HIGH);
  nrf24.setDataRate(RF24_1MBPS);
  nrf24.stopListening();
  
  if(state == true){
    Serial.println("nRF24-SPI Communication Started.");
  }
  else{
    Serial.println("nRF24-SPI Communication Failled!");
    //while(1);
  }
  Serial.println("nRF24L01 Tranciever Starting...");
  
  agv_sensors_previousTime = agv_indoor_previousTime = millis();
}
 
void loop(){
  MQTT_connect();
  //MQTT_Check();
  WiFi_MQTT_Led_Test();
//  currentTime = millis();
//  if((currentTime - agv_sensors_previousTime) > agv_sensors_sample_time){  //AGV_Sensors Verilerini Gönder
//   DBMS_Sensors_Display();
//   DBMS_Gateway_Transmit();
//   agv_sensors_previousTime = currentTime;
// }
  
  
  serialCommunication();
  if(completeReading == true){
    //UART_DATA
    Sensor_Layer_Parse(UART_DATA);
    UART_DATA = "";
    completeReading = false;
  }
}

void Sensor_Layer_Parse(String sensorLayerData){
  //19 adet veri gelicek + SOF + EOF
  sensorLayerData += ",";
  byte sensorLayerIndex = 0;
  byte sensorLayerDataLen = sensorLayerData.length();
  String sensorLayerStr = "";
  String sensorLayerSplit[19];

  for(byte i = 0; i < sensorLayerDataLen; i++){
    char ch = sensorLayerData[i];
    if(ch != ','){
      sensorLayerStr += ch;
    }
    else{
      sensorLayerSplit[sensorLayerIndex] = sensorLayerStr;
      sensorLayerStr = "";
      sensorLayerIndex++;
    }
  }
  if(sensorLayerSplit[0] == "SOF" && sensorLayerSplit[18] == "EOF"){
    sensors.leftMotorRPM = (byte) sensorLayerSplit[1].toInt();
    sensors.rightMotorRPM = (byte) sensorLayerSplit[2].toInt();
    sensors.leftMotorCurrent = (float) sensorLayerSplit[3].toFloat();
    sensors.rightMotorCurrent = (float) sensorLayerSplit[4].toFloat();
    sensors.batteryCurrent = (float) sensorLayerSplit[5].toFloat();
    sensors.batteryLevel = (byte) sensorLayerSplit[6].toInt();
    sensors.agvSpeed = (byte) sensorLayerSplit[7].toInt();
    sensors.loadcell = (byte) sensorLayerSplit[8].toInt();
    sensors.ecuTemperature = (byte) sensorLayerSplit[9].toInt();
    sensors.forwardDistance = (byte) sensorLayerSplit[10].toInt();
    sensors.leftDistance = (byte) sensorLayerSplit[11].toInt();
    sensors.rightDistance = (byte) sensorLayerSplit[12].toInt();
    sensors.compass = (byte) sensorLayerSplit[13].toInt();
//    sensors.IMU_ax = (byte) sensorLayerSplit[14].toInt();
//    sensors.IMU_ay = (byte) sensorLayerSplit[15].toInt();
//    sensors.IMU_az = (byte) sensorLayerSplit[16].toInt();
    sensors.dir = (byte) sensorLayerSplit[14].toInt();
//    if(sensorLayerSplit[17] == "true")
//    sensors.fanState = true;
//    else if(sensorLayerSplit[17] == "false")
//    sensors.fanState = false;
    sensors.fanState = (byte) sensorLayerSplit[15].toInt();
//    if(sensorLayerSplit[18] == "true")
//    sensors.carLights = true;
//    else if(sensorLayerSplit[18] == "false")
//    sensors.carLights = false;
    sensors.carLights = (byte) sensorLayerSplit[16].toInt();
    sensors.task = (byte) sensorLayerSplit[17].toInt();
    //DBMS_Sensors_Display();
    DBMS_Gateway_Transmit();
  }
  else{
    Serial.println("Hatali Veri!");
  }
//  for(byte i; i < 21; i++){
//    Serial.println(sensorLayerSplit[i]);
//  }
}

boolean DBMS_Gateway_Transmit(){
  boolean transmitState = nrf24.write(&sensors, sizeof(DBMS_Sensors));
  return transmitState;
}

//WiFi Ağına Bağlanıyoruz...
void WiFi_connect() {
  delay(10);
  Serial.println();
  Serial.println("WiFi Connection Starting");
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WLAN_SSID, WLAN_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi Connection Succesful");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  WiFi_Connect = true;
}

// Bağalntı Kesilirse Tekrar Bağlanması için gereken kodlar
void MQTT_connect(){
  int8_t ret;
  //mqtt.connect();
 // Stop if already connected. 
 if (mqtt.connected()) {
   MQTT_Connect = true;
   return; 
 }
 uint8_t retries = 5;
 while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected 
      //Serial.println(mqtt.connectErrorString(ret)); 
      Serial.println("Retrying MQTT connection in 1 seconds..."); 
      mqtt.disconnect();
      MQTT_Connect = false; 
      delay(1000);  // wait 1 seconds 
      retries--; 
      if (retries == 0) { 
        // basically die and wait for WDT to reset me 
        while (1); 
      } 
 } 
}

void MQTT_Check(){
 Adafruit_MQTT_Subscribe *subscription; 
 while ((subscription = mqtt.readSubscription())) {
   if (subscription == &RX) {
     char *message = (char *)RX.lastread;
     String comingValue = String(message);
     Serial.println("MQTT: " + comingValue);

     String str = String(comingValue);
     int str_len = str.length() + 1;
     char txBuffer[str_len];
     str.toCharArray(txBuffer, str_len);
    //TX.publish(txBuffer);
    //delay(50);
     boolean transmitSucces = nrf24.write(&txBuffer, sizeof(txBuffer));
    if(transmitSucces == true){
      Serial.println("Veri gonderildi.");
      TX.publish("Veri gonderildi");
    }
    else{
      Serial.println("Veri Gonderilemedi");
      TX.publish("Veri Gonderilemedi"); 
    }
   }
 }
}

void DBMS_Sensors_Display(){
  Serial.println("\n*********************************************************");
  Serial.println("                       AGV_Sensors\n");
  Serial.println("-----------------------------------------------------------");
  Serial.println("Left Motor Speed    (RPM)     : " + String(sensors.leftMotorRPM));
  Serial.println("Right Motor Speed   (RPM)     : " + String(sensors.rightMotorRPM));
  Serial.println("Left Motor Current  (A)       : " + String(sensors.leftMotorCurrent));
  Serial.println("Right Motor Current (A)       : " + String(sensors.rightMotorCurrent));
  Serial.println("Battery Current     (A)       : " + String(sensors.batteryCurrent));
  Serial.println("Battery Level       (%100)    : " + String(sensors.batteryLevel));
  Serial.println("AGV Speed           (cm/sn)   : " + String(sensors.agvSpeed));
  Serial.println("Loadcell            (gram)    : " + String(sensors.loadcell));
  Serial.println("ECU Temperature     (C)       : " + String(sensors.ecuTemperature));
  Serial.println("Compass             (Degrees) : " + String(sensors.compass));
  Serial.println("AGV Direction                 : " + String(sensors.dir));
  Serial.print("Fan State                       : "); if(sensors.fanState == 1) Serial.println("on"); else if(sensors.fanState == 0) Serial.println("off");
  Serial.print("Light State                     : "); if(sensors.carLights == 1) Serial.println("on"); else if(sensors.carLights == 0) Serial.println("off");
  Serial.println("Task                          : " + String(sensors.task));
}

void serialCommunication(){
  while(Serial.available() > 0){
    char ch = Serial.read();
    if(ch != '\n'){
      UART_DATA += ch;
    }
    else{
      completeReading = true;
      break;
    }
  }
}

void WiFi_MQTT_Led_Test(){
  if(WiFi_Connect == true)
  digitalWrite(WiFi_Led, HIGH);
  else
  digitalWrite(WiFi_Led, LOW);
  if(MQTT_Connect == true)
  digitalWrite(MQTT_Led, HIGH);
  else
  digitalWrite(MQTT_Led, LOW);
}
