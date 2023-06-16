#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include<Arduino_JSON.h>
#include <utility>

using namespace std;

const char* ssid = "WIFI_SSID";
const char* password = "WIFI_PASSWORD";

String serverName = "http://api.open-notify.org/iss-now.json";

unsigned long lastTime = 0;
unsigned long timerDelay = 5000;

float myLongitude = YOUR_LONGITUDE;
float myLatitude = YOUR_LATITUDE;

const float maxLat = myLatitude + 5;
const float minLat = myLatitude - 5;
const float maxLong = myLongitude + 5;
const float minLong = myLongitude - 5;

const int redLeds[] = {D8, D7, D6, D5, D4, D3, D2};
const int greenLed = D1;
const int buzzer = D0;

void setup() {
  Serial.begin(115200); 
  for(int i=0;i<7;i++){
    pinMode(redLeds[i], OUTPUT);
  }
  pinMode(greenLed, OUTPUT);
  pinMode(buzzer, OUTPUT);
  
  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) {
    digitalWrite(redLeds[0], HIGH);
    delay(250);
    Serial.print(".");
    digitalWrite(redLeds[0], LOW);
    delay(250);
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
}

pair<float, float> getCoordinates(){
  pair<float, float> coordinates;
  //Check WiFi connection status
  if(WiFi.status()== WL_CONNECTED){
    WiFiClient client;
    HTTPClient http;
    
    String serverPath = serverName;
    
    // Your Domain name with URL path or IP address with path
    http.begin(client, serverPath.c_str());

    // Send HTTP GET request
    int httpResponseCode = http.GET();
    if (httpResponseCode>0) {
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
      String payload = http.getString();
      JSONVar myObject = JSON.parse(payload);
      
      const char* long_str = myObject["iss_position"]["longitude"];
      coordinates.first = atof(long_str);
      
      const char* lat_str = myObject["iss_position"]["latitude"];
      coordinates.second = atof(lat_str);
    }
    else {
      Serial.print("Error code: ");
      Serial.println(httpResponseCode);
      coordinates = {-200, -200};
    }
    // Free resources
    http.end();
  }
  else {
    Serial.println("WiFi Disconnected");
  }
  return coordinates;
}

float getDistance(float lon, float lat){
  float distance;
  if(lon == -200){
    return -1;
  }
  if((myLongitude-180) <= lon and lon <= (myLongitude+180)){
    distance = abs(lon - myLongitude);
  }else{
    if(myLongitude>0){
      distance = abs(lon - myLongitude + 360);
    }else{
      distance = abs(-lon + myLongitude + 360);
    }
  }
  return distance;
}

void showStatusOnLed(float distance){
    int ledsToOn;
    if(distance<=25.71){
      ledsToOn = 7;
    }else if(distance<=51.42){
      ledsToOn = 6;
    }else if(distance<=77.14){
      ledsToOn = 5;
    }else if(distance<=102.85){
      ledsToOn = 4;
    }else if(distance<=128.57){
      ledsToOn = 3;
    }else if(distance<=154.28){
      ledsToOn = 2;
    }else if(distance<=180){
      ledsToOn = 1;
    }
    
    for(int i=0;i<ledsToOn;i++){
      digitalWrite(redLeds[i], HIGH);
    }
    if(ledsToOn != 7){
      for(int i=6;i>=ledsToOn;i--){
        digitalWrite(redLeds[i], LOW);
      }
    }

    if(distance < 0){
      for(int i=0;i<7;i++){
        digitalWrite(redLeds[i], LOW);
      }
    }
}

void loop() {
  // Send an HTTP POST request depending on timerDelay
  float longitude, latitude;
  if ((millis() - lastTime) > timerDelay) {
    pair<float, float> coords;
    coords = getCoordinates();
    longitude = coords.first;
    latitude = coords.second;
    float distance = getDistance(longitude, latitude);
    showStatusOnLed(distance);
    lastTime = millis();
  }
  if(minLong<longitude and longitude<maxLong and minLat<latitude and latitude<maxLat){
    digitalWrite(greenLed, HIGH);
    digitalWrite(buzzer, HIGH);
  }else{
    digitalWrite(greenLed, LOW);
    digitalWrite(buzzer, LOW);
  }
}
