                          
/* ------------------------------------------- v SENSITIVE PROPS v ----------------------------------------------------------*/
// Replace with your network credentials
const char* ssid = "*"; //Set your ssid wifi network name
const char* password = "*"; //Set password for ssid wifi network

const char* mqttUser = "*";
const char* mqttPassword = "*";

char* TEAM_COLOR = "BLUE";
char* APPLICATION_NAME = "TBEAM-LOCATOR";

/* ------------------------------------------- ^ SENSITIVE PROPS ^ ----------------------------------------------------------*/

/* ------------------------------------------- v PRINCIPAL PROPS v ----------------------------------------------------------*/

//---------------- WIFI:
unsigned long previousMillis = 0;
unsigned long interval = 30000; //Timeout to reconnect wifi

//---------------- MQTT:
const char* mqttServer = "192.168.1.165";
const int mqttPort = 1883;
String clientMqttId = "ESP32-LILLY-TTGO-T-BEAM-Client-";

//Best pattern: "dt/<application>/<context>/<thing-name>/<dt-type>"
char* PATH_SEPARATOR = "/";
char* TELEMETRY_DT = "GPS-COORDINATES";
char* TELEMETRY_CONTEXT = "V1.1";


//char* MQTT_TOPIC_PUB = TELEMETRY_DT,PATH_SEPARATOR),APPLICATION_NAME),PATH_SEPARATOR),TELEMETRY_CONTEXT),PATH_SEPARATOR),TEAM_COLOR); 
char* MQTT_TOPIC_PUB = "GPS-COORDINATES/TBEAM-LOCATOR/TBEAM/BLUE";//FIX-ME: usar consts
char* MQTT_TOPIC_SUB = "teste/teste";

/* ------------------------------------------- ^ PRINCIPAL PROPS ^ ----------------------------------------------------------*/

/* ------------------------------------------- v GPS DEFINES v ----------------------------------------------------------*/
#define T_BEAM_V10  // AKA Rev1 for board versions T-beam_V1.0 and V1.1 (second board released)

#if defined(T_BEAM_V07)
#define GPS_RX_PIN      12
#define GPS_TX_PIN      15
#elif defined(T_BEAM_V10)
#include <Wire.h>
#include <axp20x.h>
AXP20X_Class axp;
#define I2C_SDA         21
#define I2C_SCL         22
#define GPS_RX_PIN      34
#define GPS_TX_PIN      12
#endif

#include <SparkFun_Ublox_Arduino_Library.h> //http://librarymanager/All#SparkFun_Ublox_GPS
SFE_UBLOX_GPS myGPS;
int state = 0; // steps through states
HardwareSerial SerialGPS(1);

String read_sentence;
float coord[2];

void initGPS(){
  Wire.begin(I2C_SDA, I2C_SCL);

  #if defined(T_BEAM_V10)
    if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
      Serial.println("[GPS ] AXP192 Begin PASS");
    } else {
      Serial.println("[GPS ] AXP192 Begin FAIL");
    }
    axp.setPowerOutPut(AXP192_LDO3, AXP202_ON); // GPS main power
    axp.setPowerOutPut(AXP192_LDO2, AXP202_ON); // provides power to GPS backup battery
    axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
    axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
    axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
    axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON); // enables power to ESP32 on T-beam
    axp.setPowerOutPut(AXP192_DCDC3, AXP202_ON); // I foresee similar benefit for restting T-watch
    // where ESP32 is on DCDC3 but remember to change I2C pins and GPS pins!
  #endif
    SerialGPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    Serial.println("[GPS ] All comms started");
    delay(100);
  
    do {
      if (myGPS.begin(SerialGPS)) {
        Serial.println("[GPS ] Connected to GPS");
        myGPS.setUART1Output(COM_TYPE_NMEA); //Set the UART port to output NMEA only
        myGPS.saveConfiguration(); //Save the current settings to flash and BBR
        Serial.println("[GPS ] GPS serial connected, output set to NMEA");
        myGPS.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_UART1);
        myGPS.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART1);
        myGPS.disableNMEAMessage(UBX_NMEA_GSV, COM_PORT_UART1);
        myGPS.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_UART1);
        myGPS.disableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART1);
        myGPS.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART1);
        myGPS.saveConfiguration(); //Save the current settings to flash and BBR
        Serial.println("[GPS ] Enabled/disabled NMEA sentences");
        break;
      }
      delay(1000);
    } while (1);
}
/* ------------------------------------------- ^ GPS DEFINES ^ ----------------------------------------------------------*/

/* ------------------------------------------- v WIFI DEFINES v ----------------------------------------------------------*/
#include <WiFi.h>

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("[WIFI] Connecting..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.print("Local IP: ");
  Serial.print(WiFi.localIP());
  Serial.print(" - RRSI: ");
  Serial.println(WiFi.RSSI());
}
/* ------------------------------------------- ^ WIFI DEFINES ^ ----------------------------------------------------------*/

/* ------------------------------------------- v MQTT DEFINES v ----------------------------------------------------------*/

#include <PubSubClient.h>

WiFiClient espClient;
PubSubClient clientMqtt(espClient);

String height;
String numSat;

void callback(char* topic, byte* payload, unsigned int length) {

    Serial.print("[MQTT] Receive topic: ");
    Serial.print(topic);

    Serial.print(" - message: ");
    for (int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();
}

void initMQTTClient(){

    clientMqtt.setServer(mqttServer, mqttPort);
    clientMqtt.setCallback(callback);
    
    Serial.print("[MQTT] Connecting");
    clientMqttId += String(random(0xffff), HEX);
    
    while (!clientMqtt.connected()) {
        Serial.print(".");
        if (clientMqtt.connect(clientMqttId.c_str(), mqttUser, mqttPassword )) {
            Serial.println(" Connected!");
        } else {
            Serial.println("failed with state " + clientMqtt.state());
            delay(2000);
        }
    }

    //Serial.print("Tentando enviar a mensagem");
    Serial.println("[MQTT] MQTT_TOPIC_PUB: GPS-COORDINATES/TBEAM-LOCATOR/TBEAM/BLUE/");//FIX-ME: usar consts;
    
    //char* mqttPubTopicInit = strcat(strcat(MQTT_TOPIC_PUB,PATH_SEPARATOR),"init");
    char* mqttPubTopicInit = "GPS-COORDINATES/TBEAM-LOCATOR/TBEAM/BLUE/INIT";//FIX-ME: usar consts;
    //char* mqttPubInitMessage = strcat("Iniciando dispositivo:",TEAM_COLOR);
    char* mqttPubInitMessage = "Iniciando dispositivo:BLUE";//FIX-ME: usar consts;
    
    clientMqtt.publish(mqttPubTopicInit, mqttPubInitMessage);
    clientMqtt.subscribe(MQTT_TOPIC_SUB);
}

/* ------------------------------------------- ^ MQTT DEFINES ^ ----------------------------------------------------------*/

/* ------------------------------------------- v LORA DEFINES v ----------------------------------------------------------*/

#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>  
//#include "SSD1306.h" 
//#include "images.h"

#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISnO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
//#define RST     14   // GPIO14 -- SX1278's RESET
#define RST     23   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)

//433E6 for Asia
//866E6 for Europe
//915E6 for North America
//923E6
#define BAND 915E6

unsigned int counter_lora = 0;

String rssi = "RSSI --";
String packSize = "--";
String packet ;

void initLora(){
  pinMode(16,OUTPUT);
  pinMode(2,OUTPUT);
  
  digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
  delay(50); 
  digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high
  
  Serial.print("[LORA] LoRa Connecting.");
  
  //SPI.begin(SCK,MISO,MOSI,SS);
  LoRa.setPins(SS,RST,DI0);
  
  while (!LoRa.begin(BAND)) {
    Serial.print(".");
    delay(500);
  }

  Serial.print(" Connected!");
  Serial.print(" - rssi:");
  Serial.print(LoRa.rssi());
  Serial.print(" - Available:");
  Serial.print(LoRa.available());
  Serial.println();
  
}

/* ------------------------------------------- ^ LORA DEFINES ^ ----------------------------------------------------------*/


void setup()
{

  Serial.begin(115200);
  while (!Serial);  // Wait for user to open the terminal
  Serial.println("[SERIAL] Connected!");

  Serial.println("--------------------------------------------------------------");
  Serial.println("[SETUP] Start ");
  Serial.println("--------------------------------------------------------------");
  
  initGPS();  

  initWiFi();

  initMQTTClient();

  initLora();

  Serial.println("--------------------------------------------------------------");
  Serial.println("[SETUP] End ");
  Serial.println("--------------------------------------------------------------");
  
}  // endofsetup


/* ------------------------------------------- GPS FUNCTIONS ----------------------------------------------------------*/
String sentence_sep (String input, int index) {
  int finder =  0 ;
  int strIndex [] = { 0 , - 1 };
  int maxIndex = input.length () -  1 ; for ( int i = 0 ; i <= maxIndex && finder <= index; i

      ++ ) {
    if (input.charAt (i) ==  ','  || i == maxIndex) {   // ',' = separator
      finder ++ ;
      strIndex [ 0 ] = strIndex [ 1 ] +  1 ;
      strIndex [ 1 ] = (i == maxIndex) ? i +  1  : i;
    }
  } return finder > index ? input.substring (strIndex [ 0 ], strIndex [
             1 ]) :  "" ;
}

float convert_gps_coord ( float deg_min, String orientation) {
  double gps_min = fmod (( double ) deg_min, 100.0 );
  int gps_deg = deg_min / 100 ;
  double dec_deg = gps_deg + (gps_min / 60 );
  if (orientation == "W" || orientation == "S" ) {
    dec_deg =

      0  - dec_deg;
  } return dec_deg;
}

void get_gps_coord () {
  
    if (SerialGPS.available()) {
    //Serial.write(SerialGPS.read());  // print anything comes in from the GPS
    read_sentence = SerialGPS.readStringUntil ( 13 ); // 13 = return (ASCII) 
    read_sentence.trim ();
    
    if (read_sentence.startsWith ( "$GPGGA" )) {
      String gps_lat = sentence_sep (read_sentence, 2 ); // Latitude in degrees & minutes
      String gps_lon = sentence_sep (read_sentence, 4 ); // Longitude in degrees & minutes
      String gps_sat = sentence_sep (read_sentence, 7 );
      String gps_hgt = sentence_sep (read_sentence, 9 );
      String gps_lat_o = sentence_sep (read_sentence, 3 );  // Orientation (N or S)
      String gps_lon_o = sentence_sep (read_sentence, 5 ); // Orientation (E or W)

      Serial.print ("[GPS ] ");
      Serial.print ( "(h:" );
      Serial.print (gps_hgt);
      Serial.print ( " - sat:" );
      Serial.print (gps_sat); 
      Serial.print ( ") " );
      
      float latitude = convert_gps_coord (gps_lat.toFloat (), gps_lat_o);
      float longitude = convert_gps_coord (gps_lon.toFloat (), gps_lon_o);
      
      Serial.print (latitude, 6 );
      Serial.print ( "," );
      Serial.println (longitude, 6 );

      coord[0]=latitude;
      coord[1]=longitude;
      
      height = gps_hgt;
      numSat = gps_sat;
      
    }

  }
}

String getCoordStrJson(){
   String str = "{\"h\":";
          str.concat(height);
          str.concat(", \"sat\":");
          str.concat(numSat);
          str.concat(", \"lat\":");
          str.concat(coord[0]);
          str.concat(", \"long\":");
          str.concat(coord[1]);
          str.concat("}");

  return str;
}

/* ------------------------------------------- GPS FUNCTIONS ----------------------------------------------------------*/

/* ------------------------------------------- WIFI FUNCTIONS ----------------------------------------------------------*/
void wifi_check_n_reconnect(){

  unsigned long currentMillis = millis();
  
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >=interval)) {
    Serial.print("[WIFI]");
    Serial.print(millis());
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    previousMillis = currentMillis;
  }
}
/* ------------------------------------------- ^ WIFI FUNCTIONS ^ ----------------------------------------------------------*/

/* ------------------------------------------- v MQTT FUNCTIONS v ----------------------------------------------------------*/
void mqtt_check_n_reconnect(){

  unsigned long currentMillis = millis();

    if(!clientMqtt.connected() && (currentMillis - previousMillis >=interval)) {
        Serial.print("[MQTT]");
        Serial.print(millis());
        Serial.print("Reconnecting ");
        if (clientMqtt.connect(clientMqttId.c_str(), mqttUser, mqttPassword )) {
            Serial.println("Connected");
            previousMillis = currentMillis;
        } else {
            Serial.println("failed with state " + clientMqtt.state());
            delay(2000);
        }
    }
  
}

void mqtt_publish_coord(){

  char* TOPIC_TO_PUBLISH = MQTT_TOPIC_SUB;
  
  //TODO: separar em método
  String str = getCoordStrJson();
  char msg[100];
  str.toCharArray(msg,100);

  Serial.print("[MQTT] Publish on Topic ");
  Serial.print(TOPIC_TO_PUBLISH);
  Serial.print(" - message: ");
  Serial.println(msg);
  
  clientMqtt.publish(TOPIC_TO_PUBLISH, msg);//FIX-ME: MQTT_TOPIC_PUB
}

/* ------------------------------------------- ^ MQTT FUNCTIONS ^ ----------------------------------------------------------*/

/* ------------------------------------------- v LORA FUNCTIONS v ----------------------------------------------------------*/

void sender_lora(){

  //TODO: separar em método
  String str = getCoordStrJson();
  char msg[100];
  str.toCharArray(msg,100);

  Serial.print("[LORA] Send pkg_number(" + String(counter_lora) + ")");
  Serial.print(" - message: ");
  Serial.print(msg);

  // send packet
  LoRa.beginPacket();
  //LoRa.print("[LORA] hello ");
  LoRa.write(counter_lora);
  LoRa.print(msg);
  LoRa.endPacket();

  Serial.println();

  counter_lora++;
  blink_led();

}

void receiver_lora(int packetSize) {

  Serial.print("[LORA] Receive (pkg_size:");
  Serial.print(String(packetSize));
  Serial.print(") - available: ");
  Serial.println(LoRa.available());
  
  if (packetSize == 0) return;          // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  String incoming = "";                 // payload of packet

  while (LoRa.available()) {            // can't use readString() in callback, so
    incoming += (char)LoRa.read();      // add bytes one by one
  }

  if (incomingLength != incoming.length()) {   // check length for error
    Serial.println("[LORA] Receive: error: message length does not match length");
    return;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  
  //global defines:------------------------------------------
  //byte localAddress = 0xBB;     // address of this device
  //byte destination = 0xFF;      // destination to send to
  //global defines:------------------------------------------
  
  //if (recipient != localAddress && recipient != 0xFF) {
  //  Serial.println("[LORA] Receive: This message is not for me.");
  //  return;                             // skip rest of function
  //}

  // if message is for this device, or broadcast, print details:
  Serial.println("[LORA] Receive: Received from: 0x" + String(sender, HEX));
  Serial.println("[LORA] Receive: Sent to: 0x" + String(recipient, HEX));
  Serial.println("[LORA] Receive: Message ID: " + String(incomingMsgId));
  Serial.println("[LORA] Receive: Message length: " + String(incomingLength));
  Serial.println("[LORA] Receive: Message: " + incoming);
  Serial.println("[LORA] Receive: RSSI: " + String(LoRa.packetRssi()));
  Serial.println("[LORA] Receive: Snr: " + String(LoRa.packetSnr()));
  Serial.println();
}

/* ------------------------------------------- ^ LORA FUNCTIONS ^ ----------------------------------------------------------*/

void blink_led(){
  digitalWrite(2, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(2, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}

void loop()
{

  Serial.println("--------------------------------------------------------------");
  Serial.println("[LOOP] Start ");
  Serial.println("--------------------------------------------------------------");
  
  wifi_check_n_reconnect();
  mqtt_check_n_reconnect();
  
  get_gps_coord();  

  clientMqtt.loop();

  sender_lora();

  receiver_lora(LoRa.parsePacket());
  
  mqtt_publish_coord();

  Serial.println("--------------------------------------------------------------");
  Serial.println("[LOOP] End ");
  Serial.println("--------------------------------------------------------------");
  
}  // endofloop
