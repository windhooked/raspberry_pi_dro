/*

 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DHT.h>
#include <Adafruit_BMP085.h>
#include <FS.h>
#include <TimeLib.h>
#include <NtpClientLib.h>
#include <ArduinoJson.h>

#define ssid      "yourSSID"        // WiFi SSID
#define password  "yourPASSWORD"    // WiFi password
#define DHTTYPE   DHT22             // DHT type (DHT11, DHT22)
#define DHTPIN    D4                // Broche du DHT / DHT Pin
#define HISTORY_FILE "/history.json"
const uint8_t GPIOPIN[4] = {D5,D6,D7,D8};  // Led
int     sizeHist = 84 ;        // Taille historique (7h x 12pts) - History size

const size_t capacity = JSON_OBJECT_SIZE(2) + JSON_OBJECT_SIZE(3) + JSON_OBJECT_SIZE(5);
DynamicJsonBuffer jsonBuffer(capacity);

JsonObject& dro = jsonBuffer.createObject();

JsonObject& disp = dro.createNestedObject("disp");
disp["x"] = 1351824120;
disp["z"] = 1351824120;
disp["s"] = 1351824120;

JsonObject& status = dro.createNestedObject("status");
status["state"] = "ref";
status["mode"] = "lathe";
status["position"] = "abs";
status["tool"] = "T01";
status["base"] = "mm";


const long intervalHist = 1000 * 60 * 5;  // 5 mesures / heure - 5 measures / hours
unsigned long previousMillis = intervalHist;  // Dernier point enregistré dans l'historique - time of last point added

ESP8266WebServer server ( 80 );

StaticJsonBuffer<10000> jsonBuffer;                 // Buffer static contenant le JSON courant - Current JSON static buffer
JsonObject& root = jsonBuffer.createObject();
JsonArray& timestamp = root.createNestedArray("timestamp");
//JsonArray& hist_t = root.createNestedArray("t");
//JsonArray& hist_h = root.createNestedArray("h");
//JsonArray& hist_pa = root.createNestedArray("pa");
//JsonArray& bart = root.createNestedArray("bart");   //  Key histogramm (temp/humidity)
//JsonArray& barh = root.createNestedArray("barh");   //  - Key histogramm (temp/humidity)

char json[10000];                                   // Buffer pour export du JSON - JSON export buffer
/* GPIO endpoint*/
void updateGpio(){

  if (server.hasArg("id")== false || server.hasArg("state")== false ){
     server.send(200, "application/json", "{error: specify GPIO ID and state}");
    return;
  }

  String gpio = server.arg("id");
  String state = server.arg("state");

// int pin = D5;
 if ( gpio == "D5" ) {
      pin = D5;
 } else if ( gpio == "D7" ) {
     pin = D7;
 } else if ( gpio == "D8" ) {
     pin = D8;
 } else {
    server.send(200, "application/json", "{ error: Invalid Pin}");
    return;
 }


  if ( state == "1" ) {
    digitalWrite(pin, HIGH);
  } else if ( state == "0" ) {
    digitalWrite(pin, LOW);
  }

  state = digitalRead(pin);

  String json = "{\"gpio\":\"" + String(gpio) + "\",";
  json += "\"state\":\"" + String(state) ;
  json +=  "\"}";

  server.send(200, "application/json", json);
}
/*
 Return
 disp { k=v, k=v, k=v }, status { state: ref,run,fault; mode: lathe,mill; position: abs,nom,dtg; tool: name;  base: mm,in }

 */
 struct dro {
   float a;
   float b;
   float c;
 } dro_status;

void getDro() {
  String json = "{\"disp\":\"" + String(a)  + "\",";
  json += "\"h\":\"" + String(h) + "\",";
  json += "\"pa\":\"" + String(pa) + "\"}";

  server.send(200, "application/json", json);
  Serial.println("Send measures");
}

void getTools() {
  double temp = root["t"][0];      // Récupère la plus ancienne mesure (temperature) - get oldest record (temperature)
  String json = "[";
  json += "{\"mesure\":\"Température\",\"valeur\":\"" + String(t) + "\",\"unite\":\"°C\",\"glyph\":\"glyphicon-indent-left\",\"precedente\":\"" + String(temp) + "\"},";
  temp = root["h"][0];             // Récupère la plus ancienne mesure (humidite) - get oldest record (humidity)
  json += "{\"mesure\":\"Humidité\",\"valeur\":\"" + String(h) + "\",\"unite\":\"%\",\"glyph\":\"glyphicon-tint\",\"precedente\":\"" + String(temp) + "\"},";
  temp = root["pa"][0];             // Récupère la plus ancienne mesure (pression atmospherique) - get oldest record (Atmospheric Pressure)
  json += "{\"mesure\":\"Pression Atmosphérique\",\"valeur\":\"" + String(pa) + "\",\"unite\":\"mbar\",\"glyph\":\"glyphicon-dashboard\",\"precedente\":\"" + String(temp) + "\"}";
  json += "]";
  server.send(200, "application/json", json);
  Serial.println("Send data tab");
}

void sendHistory(){
  root.printTo(json, sizeof(json));             // Export JSON object as a string
  server.send(200, "application/json", json);   //  Send history data to the web client
  Serial.println("Send History");
}

void loadHistory(){
  File file = SPIFFS.open(HISTORY_FILE, "r");
  if (!file){
    Serial.println("Failed to open history file");
  } else {
    size_t size = file.size();
    if ( size == 0 ) {
      Serial.println("history file empty");
    } else {
      std::unique_ptr<char[]> buf (new char[size]);
      file.readBytes(buf.get(), size);
      JsonObject& root = jsonBuffer.parseObject(buf.get());
      if (!root.success()) {
        Serial.println("Failed to parse json");
      } else {
        Serial.println("History loaded");
        root.prettyPrintTo(Serial);
      }
    }
    file.close();
  }
}

void saveHistory(){
  Serial.println("Save History");
  File historyFile = SPIFFS.open(HISTORY_FILE, "w");
  root.printTo(historyFile); //  Export and save JSON object to SPIFFS area
  historyFile.close();
}

void setup() {

  NTP.onNTPSyncEvent([](NTPSyncEvent_t error) {
    if (error) {
      Serial.print("Time Sync error: ");
      if (error == noResponse)
        Serial.println("NTP server not reachable");
      else if (error == invalidAddress)
        Serial.println("Invalid NTP server address");
      }
    else {
      Serial.print("Got NTP time: ");
      Serial.println(NTP.getTimeDateString(NTP.getLastNTPSync()));
    }
  });
  // Serveur NTP, decalage horaire, heure été - NTP Server, time offset, daylight
  NTP.begin("pool.ntp.org", 0, true);
  NTP.setInterval(60000);
  delay(500);

  for ( int x = 0 ; x < 5 ; x++ ) {
    pinMode(GPIOPIN[x], OUTPUT);
  }

// start reading scale data from Arduino
  Serial.begin ( 115200 );

//  Serial.println("DRO OK");

  WiFi.begin ( ssid, password );
  int connectLoop = 0;
  // Wait for connection
  Serial.println ( "WiFI Connect " );

  while ( WiFi.status() != WL_CONNECTED ) {
    delay ( 500 ); Serial.print ( "." );
    connectLoop++;
    if ( connectLoop > 20 ) {
      ESP.reset();
      delay(1);
    }
  }
  // WiFi connected
  Serial.println ( "" );
  Serial.print ( "Wifi SSID: " ); Serial.println ( ssid );
  Serial.print ( "WiFi IP:  " ); Serial.println ( WiFi.localIP() );

  Serial.print("Mount SPIFFS:  " + ((SPIFFS.begin()) ? "ok" : "failed" ));
 // on success loadHistory()

  delay(50);
// GET dro               // disp { k=v, k=v, k=v }, status { state: ref,run,fault; mode: lathe,mill; position: abs,nom,dtg; tool: name;  base: mm,in }
//  GET tools // return tool list
//  POST tools // add new tool    tool { type: mill, lathe; coords: k=v,k=v  }  // can be dia, length, x, z
//  PUT tools/{id} // update existing  tool { type: mill, lathe; coords: k=v,k=v  }  // can be dia, length, x, z
  server.on("/dro.json", getDro);
  server.on("/tools.json", getTools);
  server.on("/tools", updateTools);
  server.on("/gpio", updateGpio);

  //server.on("/graph_temp.json", sendHistory);

  server.serveStatic("/js", SPIFFS, "/js");
  server.serveStatic("/css", SPIFFS, "/css");
  server.serveStatic("/img", SPIFFS, "/img");
  server.serveStatic("/", SPIFFS, "/index.html");

  server.begin();
  Serial.println ( "HTTP server started" );

  Serial.print("Uptime :");
  Serial.println(NTP.getUptime());
  Serial.print("LastBootTime :");
  Serial.println(NTP.getLastBootTime());
}
void readSerial() {
if (Serial.available() > 0) {
    incomingByte = Serial.read();
  }
}
void loop() {
  // put your main code here, to run repeatedly:
  server.handleClient();

//  t = dht.readTemperature();
//  h = dht.readHumidity();
//  pa = bmp.readPressure() / 100.0F;
//  if ( isnan(t) || isnan(h) ) {
    // Error, no valid value
//  } else {
//    addPtToHist();
//  }
  //delay(5);
}

void addPtToHist(){
  unsigned long currentMillis = millis();

  //Serial.println(currentMillis - previousMillis);
  if ( currentMillis - previousMillis > intervalHist ) {
    long int tps = NTP.getTime();
    previousMillis = currentMillis;
    //Serial.println(NTP.getTime());
    if ( tps > 0 ) {
      timestamp.add(tps);
      hist_t.add(double_with_n_digits(t, 1));
      hist_h.add(double_with_n_digits(h, 1));
      hist_pa.add(double_with_n_digits(pa, 1));

      //root.printTo(Serial);
      if ( hist_t.size() > sizeHist ) {
        //Serial.println("efface anciennes mesures");
        timestamp.removeAt(0);
        hist_t.removeAt(0);
        hist_h.removeAt(0);
        hist_pa.removeAt(0);
      }
      //Serial.print("size hist_t ");Serial.println(hist_t.size());
      calcStat();
      delay(100);
      saveHistory();
      //root.printTo(Serial);
    }
  }
}
