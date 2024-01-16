#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino_JSON.h>
#include <ESP32Servo.h>
#include "SPIFFS.h"

#define _DEBUG_PORT Serial

//Motores:
Servo ESC1;
Servo ESC2;
Servo ESC3;
Servo ESC4;

int potValue1;
int potValue2;
int potValue3;
int potValue4;

int signalOut1 = 15;
int signalOut2 = 2;
int signalOut3 = 4;
int signalOut4 = 5;

int minPWM = 1000;
int maxPWM = 2000;
int pot_max = 180;
int pot_min = 0;

String slider_values = "0,0,0,0";

int esc_impulse_brake = 0;
int MaxImpulseForce = 180;
int MinImpulseForce = 0;
int ESCIB_EN = 1;

const char* input_parameter = "value";

// Replace with your network credentials
const char* ssid = "Guifi";
const char* password = "christian25";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create an Event Source on /events
AsyncEventSource events("/events");

// Json Variable to Hold Sensor Readings
JSONVar readings;

// Timer variables
unsigned long lastTime = 0;  
unsigned long lastTimeTemperature = 0;
unsigned long lastTimeAcc = 0;
unsigned long gyroDelay = 10;
unsigned long temperatureDelay = 1000;
unsigned long accelerometerDelay = 200;

// Create a sensor object
Adafruit_MPU6050 mpu;

sensors_event_t a, g, temp;

float gyroX, gyroY, gyroZ;
float accX, accY, accZ;
float temperature;

//Gyroscope sensor deviation
float gyroXerror = 0.07;
float gyroYerror = 0.03;
float gyroZerror = 0.01;

//Actualizar los motores:
void updateMotor(int motorNumber, int value) {
  switch (motorNumber) {
    case 1:
      potValue1 = value;
      ESC1.write(potValue1);
      break;
    case 2:
      potValue2 = value;
      ESC2.write(potValue2);
      break;
    case 3:
      potValue3 = value;
      ESC3.write(potValue3);
      break;
    case 4:
      potValue4 = value;
      ESC4.write(potValue4);
      break;
  }
  slider_values = String(potValue1) + "," + String(potValue2) + "," + String(potValue3) + "," + String(potValue4);
}

// Init MPU6050
void initMPU(){
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
}

void initSPIFFS() {
  if (!SPIFFS.begin()) {
    Serial.println("An error has occurred while mounting SPIFFS");
  }
  Serial.println("SPIFFS mounted successfully");
}

// Initialize WiFi
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

//Obtener los valores de los motores:
String processor(const String& var){
  if (var == "SLIDERVALUES"){
    return slider_values;
  } else if (var == "SLIDERVALUE1") {
    return String(potValue1);
  } else if (var == "SLIDERVALUE2") {
    return String(potValue2);
  } else if (var == "SLIDERVALUE3") {
    return String(potValue3);
  } else if (var == "SLIDERVALUE4") {
    return String(potValue4);
  } else if (var == "SLIDERVALUEALL") {
    // Puedes devolver cualquier valor que desees para la quinta barra
    return String((potValue1 + potValue2 + potValue3 + potValue4) / 4);
  }
  return String();
}

//Calibrar motores:
void calibrateESC(Servo& esc, int minImpulse, int maxImpulse) {
  _DEBUG_PORT.println("[*] Force ESC into Calibration Mode...");
  _DEBUG_PORT.println("[*] Set ESC Output Speed Max");
  esc.write(maxImpulse);
  _DEBUG_PORT.println("[+] Please Remove the Power Supply / Battery and Re-Connect the Battery in under 5 seconds!");
  delay(5000);
  _DEBUG_PORT.println("[*] Set ESC Output Speed 0");
  esc.write(0);
  delay(100);
  _DEBUG_PORT.println("[+] If you are hearing a long tone indicating successful calibration was heard, the ESCs are live now and if you raise the throttle a bit they should spin. Test that the motors spin by raising the throttle a bit and then lowering it again. (Do this under 10 seconds)");
  delay(10000);
  _DEBUG_PORT.println("[*] Set ESC Output Speed 0");
  esc.write(0);
  _DEBUG_PORT.println("[+] New Disconnect the battery and connect it again after 5 seconds of disconnection to exit the calibration Mode!");
  _DEBUG_PORT.println("[+] Zero Calibration Complete!");
}

//Obtener las lecturas del giroscopio
String getGyroReadings(){
  mpu.getEvent(&a, &g, &temp);

  float gyroX_temp = g.gyro.x;
  if(abs(gyroX_temp) > gyroXerror)  {
    gyroX += gyroX_temp/50.00;
  }
  
  float gyroY_temp = g.gyro.y;
  if(abs(gyroY_temp) > gyroYerror) {
    gyroY += gyroY_temp/70.00;
  }

  float gyroZ_temp = g.gyro.z;
  if(abs(gyroZ_temp) > gyroZerror) {
    gyroZ += gyroZ_temp/90.00;
  }

  readings["gyroX"] = String(gyroX);
  readings["gyroY"] = String(gyroY);
  readings["gyroZ"] = String(gyroZ);

  String jsonString = JSON.stringify(readings);
  return jsonString;
}

String getAccReadings() {
  mpu.getEvent(&a, &g, &temp);
  // Get current acceleration values
  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;
  readings["accX"] = String(accX);
  readings["accY"] = String(accY);
  readings["accZ"] = String(accZ);
  String accString = JSON.stringify (readings);
  return accString;
}

String getTemperature(){
  mpu.getEvent(&a, &g, &temp);
  temperature = temp.temperature;
  return String(temperature);
}

void setup() {
  Serial.begin(9600);
  initWiFi();
  initSPIFFS();
  initMPU();

  // Handle Web Server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.serveStatic("/", SPIFFS, "/");

  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request){
    gyroX=0;
    gyroY=0;
    gyroZ=0;
    request->send(200, "text/plain", "OK");
  });

  server.on("/resetX", HTTP_GET, [](AsyncWebServerRequest *request){
    gyroX=0;
    request->send(200, "text/plain", "OK");
  });

  server.on("/resetY", HTTP_GET, [](AsyncWebServerRequest *request){
    gyroY=0;
    request->send(200, "text/plain", "OK");
  });

  server.on("/resetZ", HTTP_GET, [](AsyncWebServerRequest *request){
    gyroZ=0;
    request->send(200, "text/plain", "OK");
  });
  // Iniciación de los motores:
  _DEBUG_PORT.println("");
  _DEBUG_PORT.println("[Servo/ESC/Attach] Attached ESC Signal Out Pins to 15, 16, 17, 18");
  ESC1.attach(signalOut1, minPWM, maxPWM);
  ESC2.attach(signalOut2, minPWM, maxPWM);
  ESC3.attach(signalOut3, minPWM, maxPWM);
  ESC4.attach(signalOut4, minPWM, maxPWM);

  /*server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });*/

  server.on("/slider", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String message;
    if (request->hasParam("motor") && request->hasParam(input_parameter)) {
      int motorNumber = request->getParam("motor")->value().toInt();
      int value = request->getParam(input_parameter)->value().toInt();
      if (motorNumber == 0) { // Si motorNumber es 0, actualiza todos los motores
        for (int i = 1; i <= 4; ++i) {
          updateMotor(i, value);
        }
        message = "All motors set to " + String(value);
      } else {
        updateMotor(motorNumber, value);
        message = "Motor " + String(motorNumber) + " set to " + String(value);
      }
    } else {
      message = "Invalid parameters";
    }
    _DEBUG_PORT.println(message);
    request->send(200, "text/plain", "OK");
  });

  server.on("/calibrate", HTTP_GET, [] (AsyncWebServerRequest *request) {
    calibrateESC(ESC1, MinImpulseForce, MaxImpulseForce);
    calibrateESC(ESC2, MinImpulseForce, MaxImpulseForce);
    calibrateESC(ESC3, MinImpulseForce, MaxImpulseForce);
    calibrateESC(ESC4, MinImpulseForce, MaxImpulseForce);
    request->send(200, "text/plain", "Calibration Complete");
  });
  
  // Handle Web Server Events
  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 10000);
  });
  server.addHandler(&events);

  server.begin();
}

void loop() {
  if ((millis() - lastTime) > gyroDelay) {
    // Send Events to the Web Server with the Sensor Readings
    events.send(getGyroReadings().c_str(),"gyro_readings",millis());
    lastTime = millis();
  }
  if ((millis() - lastTimeAcc) > accelerometerDelay) {
    // Send Events to the Web Server with the Sensor Readings
    events.send(getAccReadings().c_str(),"accelerometer_readings",millis());
    lastTimeAcc = millis();
  }
  if ((millis() - lastTimeTemperature) > temperatureDelay) {
    // Send Events to the Web Server with the Sensor Readings
    events.send(getTemperature().c_str(),"temperature_reading",millis());
    lastTimeTemperature = millis();
  }

  //Solo calibración:
  if (esc_impulse_brake) {
    if (potValue1 == 0 && potValue2 == 0 && potValue3 == 0 && potValue4 == 0) {
      if (ESCIB_EN) {
        _DEBUG_PORT.println("[*] Impulsive Brakes Used!");
        ESC1.write(MinImpulseForce);
        ESC2.write(MinImpulseForce);
        ESC3.write(MinImpulseForce);
        ESC4.write(MinImpulseForce);
        delay(10);
        ESC1.write(MaxImpulseForce);
        ESC2.write(MaxImpulseForce);
        ESC3.write(MaxImpulseForce);
        ESC4.write(MaxImpulseForce);
        delay(65);
        ESC1.write(MinImpulseForce);
        ESC2.write(MinImpulseForce);
        ESC3.write(MinImpulseForce);
        ESC4.write(MinImpulseForce);
        ESCIB_EN = 0;
      }
    }
    if (potValue1 > 0 || potValue2 > 0 || potValue3 > 0 || potValue4 > 0) {
      ESCIB_EN = 1;
    }
  }

  if (_DEBUG_PORT.available()) {
    String datainput = _DEBUG_PORT.readString();
    datainput.trim();
    if (datainput.equals("help")) {
      _DEBUG_PORT.println("mode -set normal           -> Set the Motor's Running Mode to normal Mode");
      _DEBUG_PORT.println("mode -set manual           -> Set the Motor's Running Mode to Manual Mode");
      _DEBUG_PORT.println("esc -do calibrate          -> Calibrate the ESC (Calib Type: Zero Calibration)");
      _DEBUG_PORT.println("esc impulse_brake en       -> Enable esc Impulse Braking when throttle is set to Zero");
      _DEBUG_PORT.println("esc impulse_brake dis      -> Disable esc Impulse Braking when throttle is set to Zero");
    }
    if (datainput.equals("mode -set")) {
      _DEBUG_PORT.println("Please use the complete command (mode -set <mode>)");
    }
    if (datainput.equals("esc impulse_brake en")) {
      esc_impulse_brake = 1;
      _DEBUG_PORT.println("Enabled the impulse brake option");
    }
    if (datainput.equals("esc impulse_brake dis")) {
      esc_impulse_brake = 0;
      _DEBUG_PORT.println("Disabled the impulse brake option");
    }
    if (datainput.equals("mode -set normal")) {
      _DEBUG_PORT.println("[*] Force Normal Mode....");
      ESC1.write(180);
      ESC2.write(180);
      ESC3.write(180);
      ESC4.write(180);
      _DEBUG_PORT.println("[+] ESC / Motor in Normal Mode");
    }
    if (datainput.equals("mode -set manual")) {
      _DEBUG_PORT.println("[*] Force Manual Mode....");
      ESC1.write(0);
      ESC2.write(0);
      ESC3.write(0);
      ESC4.write(0);
      _DEBUG_PORT.println("[+] ESC / Motor in Manual Mode / Speed 0");
    }
    if (datainput.equals("esc -do calibrate")) {
      calibrateESC(ESC1, MinImpulseForce, MaxImpulseForce);
      calibrateESC(ESC2, MinImpulseForce, MaxImpulseForce);
      calibrateESC(ESC3, MinImpulseForce, MaxImpulseForce);
      calibrateESC(ESC4, MinImpulseForce, MaxImpulseForce);
    }
  }
}
