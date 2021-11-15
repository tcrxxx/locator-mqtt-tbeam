
/* ------------------------------------------- v SENSITIVE PROPS v ----------------------------------------------------------*/
// Replace with your network credentials
const char* ssid = "*"; //Set your ssid wifi network name
const char* password = "*"; //Set password for ssid wifi network

const char* mqttUser = "*";
const char* mqttPassword = "*";

/* ------------------------------------------- ^ SENSITIVE PROPS ^ ----------------------------------------------------------*/


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
      Serial.println("AXP192 Begin PASS");
    } else {
      Serial.println("AXP192 Begin FAIL");
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
    Serial.println("All comms started");
    delay(100);
  
    do {
      if (myGPS.begin(SerialGPS)) {
        Serial.println("Connected to GPS");
        myGPS.setUART1Output(COM_TYPE_NMEA); //Set the UART port to output NMEA only
        myGPS.saveConfiguration(); //Save the current settings to flash and BBR
        Serial.println("GPS serial connected, output set to NMEA");
        myGPS.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_UART1);
        myGPS.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART1);
        myGPS.disableNMEAMessage(UBX_NMEA_GSV, COM_PORT_UART1);
        myGPS.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_UART1);
        myGPS.disableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART1);
        myGPS.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART1);
        myGPS.saveConfiguration(); //Save the current settings to flash and BBR
        Serial.println("Enabled/disabled NMEA sentences");
        break;
      }
      delay(1000);
    } while (1);
}
/* ------------------------------------------- ^ GPS DEFINES ^ ----------------------------------------------------------*/

/* ------------------------------------------- v WIFI DEFINES v ----------------------------------------------------------*/
#include <WiFi.h>

unsigned long previousMillis = 0;
unsigned long interval = 30000; //Timeout to reconnect wifi

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.print("Local IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("RRSI: ");
  Serial.println(WiFi.RSSI());
}
/* ------------------------------------------- ^ WIFI DEFINES ^ ----------------------------------------------------------*/

/* ------------------------------------------- v MQTT DEFINES v ----------------------------------------------------------*/

#include <PubSubClient.h>
const char* mqttServer = "192.168.1.165";
const int mqttPort = 1884;
WiFiClient espClient;
PubSubClient clientMqtt(espClient);

void callback(char* topic, byte* payload, unsigned int length) {

    Serial.print("Message arrived in topic: ");
    Serial.println(topic);

    Serial.print("Message:");
    for (int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }

}

void initMQTTClient(){

    clientMqtt.setServer(mqttServer, mqttPort);
    clientMqtt.setCallback(callback);

    while (!clientMqtt.connected()) {
        Serial.println("Connecting to MQTT…");
        String clientId = "ESP32-LILLY-TTGO-T-BEAM-Client-";
        clientId += String(random(0xffff), HEX);
        if (clientMqtt.connect(clientId.c_str(), mqttUser, mqttPassword )) {
            Serial.println("connected");
        } else {
            Serial.println("failed with state " + clientMqtt.state());
            delay(2000);
        }
    }

    Serial.print("Tentando enviar a mensagem");
    //client.publish(“esp∕test”, “Hello from ESP32”);
    //client.subscribe(“esp/test”);
  
}

/* ------------------------------------------- ^ MQTT DEFINES ^ ----------------------------------------------------------*/


void setup()
{

  Serial.begin(115200);
  while (!Serial);  // Wait for user to open the terminal
  Serial.println("Connected to Serial");

  initGPS();  

  initWiFi();

  initMQTTClient();
  
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

      Serial.print ( "[h:" );
      Serial.print (gps_hgt);
      Serial.print ( " - sat:" );
      Serial.print (gps_sat); 
      Serial.print ( "] " );
      
      float latitude = convert_gps_coord (gps_lat.toFloat (), gps_lat_o);
      float longitude = convert_gps_coord (gps_lon.toFloat (), gps_lon_o);
      
      Serial.print (latitude, 6 );
      Serial.print ( "," );
      Serial.println (longitude, 6 );

      coord[0]=latitude;
      coord[1]=longitude;
      
    }

  }
}
/* ------------------------------------------- GPS FUNCTIONS ----------------------------------------------------------*/

/* ------------------------------------------- WIFI FUNCTIONS ----------------------------------------------------------*/
void wifi_check_n_reconnect(){

  unsigned long currentMillis = millis();
  
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >=interval)) {
    Serial.print(millis());
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    previousMillis = currentMillis;
  }
}
/* ------------------------------------------- WIFI FUNCTIONS ----------------------------------------------------------*/

void loop()
{

  wifi_check_n_reconnect();
  
  get_gps_coord();  

}  // endofloop
