#include <Arduino.h>
#include <HttpClient.h>
#include <WiFi.h>
#include <inttypes.h>
#include <stdio.h>


#include "esp_system.h"
//#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

#define PIR_INPUT 27
#define GREEN_PIN 25
// put function declarations here:
const int sampleWindow = 1000;
const int calibWindow = 50;
unsigned int sample;
unsigned int minS;
unsigned int getPeak();

char ssid[50]; // your network SSID (name)
char pass[50]; // your network password (use for WPA, or use
// as key for WEP)
// Name of the server we want to connect to
const char kHostname[] = "3.15.34.160";
// Path to download (this is the bit after the hostname in the URL
// that you want to download
const char kPath[] = "/post";
// Number of milliseconds to wait without receiving any data before we give up
const int kNetworkTimeout = 30 * 1000;
// Number of milliseconds to wait if no data is available before trying again
const int kNetworkDelay = 1000;

const int deviceID = 2;
void init_NVS();
void nvs_access();
void setup_wifi();
void pirDetect();
std::pair<double, bool> getReadings();
WiFiClient wifiClient;
HttpClient httpClient = HttpClient(wifiClient,kHostname,5000);
void setup() {
  Serial.begin(9600);
  pinMode(PIR_INPUT, INPUT);
  // pinMode(GREEN_PIN, OUTPUT);
  // digitalWrite(GREEN_PIN, LOW);
  //init_NVS();
  setup_wifi();
  minS = getPeak();
  for (int i = 0; i < 200; i++){
    sample = getPeak();
    if (sample > minS){
      minS = sample;
    }
  }
  Serial.println("Done calibrating");
  Serial.print("Threshold: ");
  Serial.println(minS);
}

void loop() {
  // unsigned int peakToPeak = getPeak();
  // if (peakToPeak < minS){
  //   Serial.println("Ignore this crap");
  // }
  // else{
  //   Serial.println(peakToPeak);
  // }
  // pirDetect();
  // delay(50);
  double audioSum = 0;
  unsigned int motionSum = 0;
  for(int i = 0 ; i<20;i++){
    std::pair<double, bool> reading = getReadings();
    audioSum+=reading.first;
    motionSum+=reading.second;
  }
  std::string s = "{\"id\" : "+std::to_string(deviceID)+", \"audio\" : "+std::to_string(audioSum)+", \"motion\" : "+std::to_string(motionSum)+"}";
  String contentType = "application/json";
  //String postData = "{\"pee\":\"poo\"}";
  httpClient.post("/post",contentType,s.c_str());

  int statusCode = httpClient.responseStatusCode();
  String response = httpClient.responseBody();
  Serial.print("Status code: ");
  Serial.println(statusCode);
  Serial.print("Response: ");
  Serial.println(response);

}

void pirDetect(){
  int motion = digitalRead(PIR_INPUT);
  if (motion){
    Serial.println("Motion detected");
    digitalWrite(GREEN_PIN, HIGH);
  }
  else {
    Serial.println("No movement");
    digitalWrite(GREEN_PIN, LOW);
  }
}

unsigned int getPeak(){
  unsigned long startMillis = millis();
  unsigned int peakToPeak = 0;
  unsigned int signalMax = 0;
  unsigned int signalMin = 5000;
  while(millis()- startMillis < calibWindow){
    sample = analogRead(39);
      if (sample > signalMax){
        signalMax = sample;
      }
      else if (sample < signalMin){
        signalMin = sample;
      }
  }
  peakToPeak = signalMax - signalMin;
  return peakToPeak;
}

std::pair<double, bool> getReadings(){
  unsigned long startMillis = millis();
  unsigned int peakToPeak = 0;
  bool motionFlag = false;
  unsigned int signalMax = 0;
  unsigned int signalMin = 5000;
  while(millis()- startMillis < sampleWindow){
    sample = analogRead(39);
    int motion = digitalRead(PIR_INPUT);
    if (motion){
      motionFlag = true;
    }
      if (sample > signalMax){
        signalMax = sample;
      }
      else if (sample < signalMin){
        signalMin = sample;
      }
  }
  peakToPeak = signalMax - signalMin;
  if (peakToPeak <= minS){
    peakToPeak = 0;
  }
  double standVal = (1.0*peakToPeak)/4096.0;
  Serial.println(standVal);
  return std::make_pair(standVal, motionFlag);
}

void init_NVS(){
  Serial.begin(9600);
  delay(1000);
  // Initialize NVS
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    // NVS partition was truncated and needs to be erased
    // Retry nvs_flash_init
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);
  Serial.printf("\n");
  Serial.printf("Opening Non-Volatile Storage (NVS) handle... ");
  nvs_handle_t my_handle;
  err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK) {
    Serial.printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  } else {
    Serial.printf("Done\n");
    // Write
    Serial.printf("Updating ssid/pass in NVS ... ");
    char ssid[] = "UCInet Mobile Access";
    char pass[] = "";
    err = nvs_set_str(my_handle, "ssid", ssid);
    err |= nvs_set_str(my_handle, "pass", pass);
    Serial.printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
    // Commit written value.
    // After setting any values, nvs_commit() must be called to ensure changes
    // are written to flash storage. Implementations may write to storage at
    // other times, but this is not guaranteed.
    Serial.printf("Committing updates in NVS ... ");
    err = nvs_commit(my_handle);
    Serial.printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
    // Close
    nvs_close(my_handle);
  }
}

void nvs_access() {
// Initialize NVS
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
  err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    // NVS partition was truncated and needs to be erased
    // Retry nvs_flash_init
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);
  // Open
  Serial.printf("\n");
  Serial.printf("Opening Non-Volatile Storage (NVS) handle... ");
  nvs_handle_t my_handle;
  err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK) {
    Serial.printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  } else {
  Serial.printf("Done\n");
  Serial.printf("Retrieving SSID/PASSWD\n");
  size_t ssid_len;
  size_t pass_len;
  err = nvs_get_str(my_handle, "ssid", ssid, &ssid_len);
  err |= nvs_get_str(my_handle, "pass", pass, &pass_len);
  switch (err) {
    case ESP_OK:
      Serial.printf("Done\n");
      //Serial.printf("SSID = %s\n", ssid);
      //Serial.printf("PASSWD = %s\n", pass);
      break;
    case ESP_ERR_NVS_NOT_FOUND:
      Serial.printf("The value is not initialized yet!\n");
      break;
    default:
    Serial.printf("Error (%s) reading!\n", esp_err_to_name(err));
  }
  }
  // Close
  nvs_close(my_handle);
}
void setup_wifi(){
  Serial.begin(9600);
  delay(1000);
  // Retrieve SSID/PASSWD from flash before anything else
  nvs_access();
  // We start by connecting to a WiFi network
  delay(1000);
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("MAC address: ");
  Serial.println(WiFi.macAddress());
}