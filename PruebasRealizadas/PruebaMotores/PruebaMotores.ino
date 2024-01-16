#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo.h>

#define _DEBUG_PORT Serial

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

int esc_impulse_brake = 0;
int MaxImpulseForce = 180;
int MinImpulseForce = 0;
int ESCIB_EN = 1;

const char* ssid = "ESP32WiFi";
const char* password = "christian25";

String slider_values = "0,0,0,0";

const char* input_parameter = "value";

AsyncWebServer server(80);

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

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Web ESC Control / Calibrate</title>
  <style>
    html {font-family: Times New Roman; display: inline-block; text-align: center;}
    h2 {font-size: 2.3rem;}
    p {font-size: 2.0rem;}
    body {max-width: 400px; margin:0px auto; padding-bottom: 25px;}
    .slider { -webkit-appearance: none; margin: 14px; width: 360px; height: 25px; background: #FF0000;
      outline: none; -webkit-transition: .2s; transition: opacity .2s;}
    .slider::-webkit-slider-thumb {-webkit-appearance: none; appearance: none; width: 35px; height: 35px; background:#01070a; cursor: pointer;}
    .slider::-moz-range-thumb { width: 35px; height: 35px; background: #01070a; cursor: pointer; } 
  </style>
</head>
<body>
  <h2>WebESCCont v2 ESC PWM Frequency Generator</h2>
  <p><span id="textslider_values">%SLIDERVALUES%</span></p>
  <p><input type="range" onchange="updateSliderPWM(this, 1)" id="pwmSlider1" min=pot_min max=pot_max value="%SLIDERVALUE1%" step="1" class="slider"></p>
  <p><input type="range" onchange="updateSliderPWM(this, 2)" id="pwmSlider2" min=pot_min max=pot_max value="%SLIDERVALUE2%" step="1" class="slider"></p>
  <p><input type="range" onchange="updateSliderPWM(this, 3)" id="pwmSlider3" min=pot_min max=pot_max value="%SLIDERVALUE3%" step="1" class="slider"></p>
  <p><input type="range" onchange="updateSliderPWM(this, 4)" id="pwmSlider4" min=pot_min max=pot_max value="%SLIDERVALUE4%" step="1" class="slider"></p>
  <p><input type="range" onchange="updateAllMotors(this)" id="pwmSliderAll" min=pot_min max=pot_max value="%SLIDERVALUEALL%" step="1" class="slider"></p>
  <br><hr><br>
<script>
function updateSliderPWM(element, motorNumber) {
  var slider_value = document.getElementById("pwmSlider" + motorNumber).value;
  document.getElementById("textslider_values").innerHTML = slider_value;
  console.log(slider_value);
  var xhr = new XMLHttpRequest();
  xhr.open("GET", "/slider?motor="+motorNumber+"&value="+slider_value, true);
  xhr.send();
}

function updateAllMotors(element) {
  var slider_value = document.getElementById("pwmSliderAll").value;
  document.getElementById("textslider_values").innerHTML = slider_value;
  console.log(slider_value);
  var xhr = new XMLHttpRequest();
  xhr.open("GET", "/slider?motor=0&value="+slider_value, true); // Use motor=0 to indicate all motors
  xhr.send();
}
</script>
</body>
</html>
)rawliteral";

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

void setup(){
  _DEBUG_PORT.begin(9600);
  delay(1000);
  _DEBUG_PORT.println("WebESCCont v1.0");
  _DEBUG_PORT.println("Made by Ankush Sheoran");
  _DEBUG_PORT.println("");
  _DEBUG_PORT.println("[Servo/ESC/Attach] Attached ESC Signal Out Pins to 15, 16, 17, 18");
  ESC1.attach(signalOut1, minPWM, maxPWM);
  ESC2.attach(signalOut2, minPWM, maxPWM);
  ESC3.attach(signalOut3, minPWM, maxPWM);
  ESC4.attach(signalOut4, minPWM, maxPWM);

  Serial.println("\n[*] Creating AP");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  Serial.print("[+] AP Created with IP Gateway ");
  Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });

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

  server.begin();
}

void loop() {
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
