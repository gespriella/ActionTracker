//* INCLUDED Libraries
#include <Arduino.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include <JY901.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <arduino-timer.h>
#include <Wifi.h>
#include <HTTPClient.h>
#include <Bounce2.h>
#include <time.h>

//* Recording Button
Bounce2::Button button = Bounce2::Button();
bool startRecoding = false;
#define BUTTON_PIN 5

//* WIFI and REST service
const char* ssid = "hydra";
const char* password = "elbuengus";
String serverName = "https://192.168.1.135:7216/api/SensorReadings";

//* Screen
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET     4
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//* Time and Timers
const char* ntpServer = "pool.ntp.org";
time_t initialEpoch;
auto timerWT901 = timer_create_default();
auto timerGPS = timer_create_default();

//* GPS
HardwareSerial neogps(1);
TinyGPSPlus gps;

//* WT901 PINs and Setup
#define WT901_HZ 10
#define RXD2 16
#define TXD2 17
#define SDA_PIN 21
#define SLC_PIN 22

//* Variables and Structs

bool recordingMode = false;
String activities[6]={"Walking","Abs", "Jumping","Squats","Twists", "Running"};
static volatile byte activityNumber = 5;
String currentActivity = "Inconclusive...";
double latitude = 0, longitude = 0;
int altitude = 0, satellites = 0;
const int arraySize = 50;
struct XYZ {int x; int y; int z;} accelerometer, gyroscope, angle;
struct structSensorReading {
  struct XYZ accelerometer, gyroscope, angle;
  uint64_t timestamp;
  double latitude, longitude;
  int altitude, satellites;
} arraySensorReadings[arraySize];
int arrayIndex = 0;
TaskHandle_t Task2;

//! FUNCTIONS ////////////////////////////////////////////
//! displayActivity: Display the predicted activity in the OLED FUNCTION
void displayActivity(String activity){
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("Current Activity:");
  display.setTextSize(2);
  display.setCursor(10,10);
  display.print(activity);
  display.display();
}
//! displayRecording: Display RECORDING in the OLED FUNCTION
void displayRecording(){
  display.fillScreen(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(10,10);
  display.setTextColor(SSD1306_BLACK);
  display.print("RECORDING");
  display.display();
}
//! taskCore2: FUNCTION
void taskCore2( void * pvParameters ){
  for(;;){
    delay(1);
    if(arrayIndex>=arraySize){
      Serial.println("Getting ready to POST");
      DynamicJsonDocument doc(32768);
      JsonArray arr = doc.to<JsonArray>();
      for(int i=0;i<arrayIndex;i++){//- Format each set of readings into a JSON object inside the array
        JsonObject jsonObject = arr.createNestedObject();
        jsonObject["MillisInSensor"] = arraySensorReadings[i].timestamp;
        jsonObject["AccelX"] = arraySensorReadings[i].accelerometer.x;
        jsonObject["AccelY"] = arraySensorReadings[i].accelerometer.y;
        jsonObject["AccelZ"] = arraySensorReadings[i].accelerometer.z;
        jsonObject["GyroX"] = arraySensorReadings[i].gyroscope.x;
        jsonObject["GyroY"] = arraySensorReadings[i].gyroscope.y;
        jsonObject["GyroZ"] = arraySensorReadings[i].gyroscope.z;
        jsonObject["AngleX"] = arraySensorReadings[i].angle.x;
        jsonObject["AngleY"] = arraySensorReadings[i].angle.y;
        jsonObject["AngleZ"] = arraySensorReadings[i].angle.z;
        jsonObject["GPSLng"] = arraySensorReadings[i].longitude;
        jsonObject["GPSLat"] = arraySensorReadings[i].latitude;
        jsonObject["GPSAltitude"] = arraySensorReadings[i].altitude;
        jsonObject["GPSSatellites"] = arraySensorReadings[i].satellites;
      }
      arrayIndex=0;//- Setting arrayIndex to 0 here allow the other core to continue
      HTTPClient httpClient;
      if (recordingMode){
        httpClient.begin(serverName + "/Multiple");
      } else {
        httpClient.begin("http://192.168.1.135:1080/predict");
      }
      httpClient.addHeader("Content-Type", "application/json");
      String json;
      serializeJson(doc, json);
      Serial.println(httpClient.POST(json));
      if (!recordingMode){
        DynamicJsonDocument docResult(256);
        deserializeJson(docResult, httpClient.getStream());
        activityNumber=docResult["prediction"].as<byte>();
        Serial.println(activityNumber);
      }
      httpClient.end(); 
    }
  }
}
//! readWT901: Reading the WT901 sensor FUNCTION
bool readWT901(void *){
  //- Get sensor readings into global variables
  JY901.GetAcc();
  JY901.GetGyro();
  JY901.GetAngle();
  accelerometer.x = JY901.stcAcc.a[0];
  accelerometer.y = JY901.stcAcc.a[1];
  accelerometer.z = JY901.stcAcc.a[2];
  gyroscope.x = JY901.stcGyro.w[0];
  gyroscope.y = JY901.stcGyro.w[1];
  gyroscope.z = JY901.stcGyro.w[2];
  angle.x = JY901.stcAngle.Angle[0];
  angle.y = JY901.stcAngle.Angle[1];
  angle.z = JY901.stcAngle.Angle[2];
  //- Only fill array if device is "recording"
  if(startRecoding){
    arraySensorReadings[arrayIndex].accelerometer = accelerometer;
    arraySensorReadings[arrayIndex].gyroscope = gyroscope;
    arraySensorReadings[arrayIndex].angle = angle;
    arraySensorReadings[arrayIndex].latitude = latitude;
    arraySensorReadings[arrayIndex].longitude = longitude;
    arraySensorReadings[arrayIndex].altitude = altitude;
    arraySensorReadings[arrayIndex].satellites = satellites;
    arraySensorReadings[arrayIndex].timestamp = ((uint64_t)initialEpoch*1000)+millis();//-Estimate realtime millis
    arrayIndex++;

    //-If array is at limit, wait until other core takes care of passing array to json
    if(arrayIndex>=arraySize){
      ulong initialTime=millis();
      while(arrayIndex>=arraySize) delay(1);
      Serial.print("Core 0 was idle for: "); Serial.println(millis()-initialTime);
    }
  }
  return true;
}
//! readGPS: Reading the GPS NEO 6m sensor FUNCTION
bool readGPS(void *){
  boolean newData = false;
  //- Read GPS serial info until there's no more
  while (neogps.available()){
    if(gps.encode(neogps.read())){
      newData = true;
    }
  }
  //- If there's GPS data, put it in global variables
  if(newData){
    newData=false;
    satellites = gps.satellites.value();
    altitude = (int)gps.altitude.meters();
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    Serial.print("Sats: ");Serial.print(satellites);
    Serial.print(", Alt: ");Serial.print(altitude);
    Serial.print(", Lat: ");Serial.print(latitude, 10);
    Serial.print(", Long: ");Serial.println(longitude, 10);
  }
  return true;
}
//! SETUP FUNCTION ////////////////////////////////////////////
void setup(){
  //- Rec Button Setup
  button.attach(BUTTON_PIN, INPUT_PULLUP);
  button.interval(20);
  button.setPressedState(LOW);
  //- Wifi and serials Setup
  Serial.begin(115200);
  Serial.println(xPortGetCoreID());
  Wire.begin();
  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());

  //- Screen Setup
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) Serial.println(F("SSD1306 allocation failed"));
  displayActivity(currentActivity);
  //- GPS setup
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);
  //- WT901 Setup
  JY901.StartIIC();
  //- Get real-time Epoch 
  configTime(0, 0, ntpServer);
  struct tm timeinfo;
  getLocalTime(&timeinfo);
  time(&initialEpoch);
  Serial.print("Epoch Time: ");
  Serial.println(initialEpoch);
  //-Initialize timed Events
  // delay(2000);
  timerWT901.every(1000 / WT901_HZ, readWT901);
  timerGPS.every(1000, readGPS);
  xTaskCreatePinnedToCore(taskCore2,"Task2",10000,NULL,1,&Task2,0);
}
//! LOOP FUNCTION ////////////////////////////////////////////
void loop(){
  button.update();
  if (button.pressed()) {
    startRecoding=!startRecoding;
  }
  if(startRecoding && recordingMode) {
    displayRecording();
  } else {
    displayActivity(activities[activityNumber]);
  }
  timerWT901.tick();
  timerGPS.tick();
}
