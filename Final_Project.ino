#include <WiFi.h> //wifi
#include <AsyncTCP.h> //web
#include <ESPAsyncWebServer.h> //web
#include <Adafruit_Sensor.h>  //sensors
#include <DHT.h> //dht11
#include <ESP32_MailClient.h> //email
#include <Adafruit_BMP085.h> //bmp sensor
#include <Wire.h> //SDA SCL
#include <Adafruit_I2CDevice.h> //SDA SCL

#define DHTPIN 18     
#define DHTTYPE    DHT11 
DHT dht(DHTPIN, DHTTYPE);
Adafruit_BMP085 bmp;


// NETWORK CREDENTIALS
const char* ssid = "Galaxy A51E68F";
const char* password = "zmfi660033";
//const char* ssid = "Orange-CB4B";
//const char* password = "G40H8N6DFQ0";

// To send Emails using Gmail
#define emailSenderAccount    "drsfcis2024@gmail.com"
#define emailSenderPassword   "gybq bdfp nsab frvb"
//#define emailSenderPassword   "dFrCsI2S024"
//#define emailSenderAccount    "doaafcis2002"
//#define emailSenderPassword   "gybq bdfp nsab frvb"
#define smtpServer            "smtp.gmail.com"
#define smtpServerPort        465
#define emailSubject          "[ALERT] ESP32 Temperature"

// Recipient Email Address
String inputMessage = "do3a2dia22@gmail.com ";
String enableEmailChecked = "checked";
String inputMessage2 = "true";
// Default Threshold Temperature Value
String inputMessage3 = "200.0";//25
String inputMessage4 = "200.0";//55
String inputMessage5 = "2000.0";//990
String inputMessage6 = "200.0";//160
String lastTemperature;
String lastHumidity;
String lastDewpoint;
String lastPressure;
String lastAltitude;
String lastSeapressure;

// HTML web page 
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <title>Weather Station</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body {
      text-align: center;
    }

    h1,h3 {
      color: black;
    }
    h2{
       color: blue;
    }
    h4 {
      color: red; 
    }
    form{
      font-weight: bold;
    }
    form input {

    text-align: center;
  }
  </style>
</head>
<body>
  <h1>Weather Station</h1>
  <h2>Team 108</h2>
  <hr>
  <h3>ESP32 Temperature</h3>
  <h4>%TEMPERATURE% &deg;C</h4>
  <h3>ESP32 Humidity</h3>
  <h4>%HUMIDITY% &percnt;</h4>
  <h3>ESP32 Pressure</h3>
  <h4>%PRESSURE% hPa </h4>
  <h3>ESP32 Altitude</h3>
  <h4>%ALTITUDE% m</h4>
  <h3>Dew Point</h3>
  <h4>%DEWPOINT% &deg;C </h4>
  <h3>Sea level pressure</h3>
  <h4>%SEAPRESSURE% Pa</h4>
  <hr>
  <h3>ESP Email Notification</h3>
  <form action="/get">
    Email Address
    <br>
    <input type="email" name="email_input" value="%EMAIL_INPUT%" required><br><br>
    
    Temperature Threshold 
    <br>
    <input type="number" step="0.1" name="threshold_input" value="%T_THRESHOLD%" required><br><br>
    
    Humidity Threshold 
    <br>
    <input type="number" step="0.1" name="threshold_input2" value="%H_THRESHOLD%" required><br><br>
    
    Pressure Threshold 
    <br>
    <input type="number" step="0.1" name="threshold_input3" value="%P_THRESHOLD%" required><br><br>
    
    Altitude Threshold 
    <br>
    <input type="number" step="0.1" name="threshold_input4" value="%A_THRESHOLD%" required><br><br>

    Enable Email Notification
    <input type="checkbox" name="enable_email_input" value="true" %ENABLE_EMAIL%><br><br>
    
    <input type="submit" value="Submit">
  </form>
</body>
</html>

)rawliteral";

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

AsyncWebServer server(80);


String readDHTTemperature() {

  // Read temperature as Celsius 
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  //float t = dht.readTemperature(true);
  
  // Check if any reads failed and exit early (to try again).
  if (isnan(t)) {    
    Serial.println("Failed to read from DHT sensor!");
    return "--";
  }
  else {
    Serial.print("Temperature = ");
    Serial.print(t);
    Serial.println(" C");
    return String(t);
  }
}

String readDHTHumidity() {

  float h = dht.readHumidity();

  // Check if any reads failed and exit early (to try again).
  if (isnan(h)) {    
    Serial.println("Failed to read from DHT sensor!");
    return "--";
  }
  else {
    Serial.print("Humidity = ");
    Serial.print(h);
    Serial.println(" %");
    return String(h);
  }
}

String calculateDewPoint() {

  float temp = dht.readTemperature();
  float hum = dht.readHumidity();
  float d = temp - (100 - hum / 5.0);


  // Check if any reads failed and exit early (to try again).
  if (isnan(temp) || isnan(hum)) {
    Serial.println("Failed to read from DHT sensor!");
    return "--";
  }
  else {
    Serial.print("Dew Point = ");
    Serial.print(d);
    Serial.println(" C");
    return String(d);
  }
}

String readPressure() {

  float p = bmp.readPressure()/100; 

  // Check if any reads failed and exit early (to try again).
  if (isnan(p)) {    
    Serial.println("Failed to read from BMP 180 sensor!");
    return "--";
  }
  else {
    Serial.print("Pressure = ");
    Serial.print(p);
    Serial.println(" hPa");//hectopascals (hPa)
    return String(p);
  }
}

String readAltitude() {

  float a = bmp.readAltitude();

  // Check if any reads failed and exit early (to try again).
  if (isnan(a)) {    
    Serial.println("Failed to read from BMP 180 sensor!");
    return "--";
  }
  else {
    Serial.print("Altitude = ");
    Serial.print(a);
    Serial.println(" m");
    return String(a);
  }
}
String calculateSeaLevelPressure() {
  const float lapseRate = 0.0065;
  const float standardTemperatureSeaLevel = 288.15;
  const float gravity = 9.80665;
  const float molarMassAir = 0.0289644;
  const float idealGasConstant = 8.31432;

  float pre = bmp.readPressure();
  float altit = bmp.readAltitude();

  // Calculate sea level pressure using the barometric formula
  float s = pre * pow(1 - (lapseRate * altit) / standardTemperatureSeaLevel,
                                                     (gravity * molarMassAir) / (idealGasConstant * lapseRate));
                                                     
  // Check if any reads failed and exit early (to try again).
    if (isnan(pre) || isnan(altit)) {
      Serial.println("Failed to read from BMP 180!");
      return "--";
    }
    else {
      Serial.print("Sea Level Pressure = ");
      Serial.print(s);
      Serial.println(" Pa");
      Serial.println("-------------------------------------");
      return String(s);
    }
}

// Replaces placeholder with DS18B20 values
String processor(const String& var){
  //Serial.println(var);
  if(var == "TEMPERATURE"){
    return lastTemperature;
  }
  else if(var == "HUMIDITY"){
    return lastHumidity;
  }
  else if(var == "DEWPOINT"){
    return lastDewpoint;
  }
  else if(var == "PRESSURE"){
    return lastPressure;
  }
  else if(var == "ALTITUDE"){
    return lastAltitude;
  }
  else if(var == "SEAPRESSURE"){
  return lastSeapressure;
  }
  else if(var == "EMAIL_INPUT"){
    return inputMessage;
  }
  else if(var == "ENABLE_EMAIL"){
    return enableEmailChecked;
  }
  else if(var == "T_THRESHOLD"){
    return inputMessage3;
  }
  else if(var == "H_THRESHOLD"){
    return inputMessage4;
  }
  else if(var == "P_THRESHOLD"){
    return inputMessage5;
  }
  else if(var == "A_THRESHOLD"){
    return inputMessage6;
  }
  return String();
}

// email notification sent or not flag
bool emailSent = false;

const char* PARAM_INPUT_1 = "email_input";
const char* PARAM_INPUT_2 = "enable_email_input";
const char* PARAM_INPUT_3 = "threshold_input";
const char* PARAM_INPUT_4 = "threshold_input2";
const char* PARAM_INPUT_5 = "threshold_input3";
const char* PARAM_INPUT_6 = "threshold_input4";

// Interval between sensor readings
unsigned long previousMillis = 0;     
const long interval = 5000;    

// Email Sending data object 
SMTPData smtpData;

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed!");
    return;
  }
  Serial.println();
  Serial.print("ESP IP Address: http://");
  Serial.println(WiFi.localIP());
  Serial.println("*************************************");


  // Start the DHT11 sensor
  dht.begin();
  
  if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP 180 sensor, check wiring!");
  }
  // Send web page to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });
  
  // Receive an HTTP GET request 
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    // GET email_input value 
    if (request->hasParam(PARAM_INPUT_1)) {
      inputMessage = request->getParam(PARAM_INPUT_1)->value();
      // GET enable_email_input value 
      if (request->hasParam(PARAM_INPUT_2)) {
        inputMessage2 = request->getParam(PARAM_INPUT_2)->value();
        enableEmailChecked = "checked";
      }
      else {
        inputMessage2 = "false";
        enableEmailChecked = "";
      }
      // GET threshold_input value 
      if ((request->hasParam(PARAM_INPUT_3))&&(request->hasParam(PARAM_INPUT_4))&&(request->hasParam(PARAM_INPUT_5))&&(request->hasParam(PARAM_INPUT_6))) {
        inputMessage3 = request->getParam(PARAM_INPUT_3)->value();
        inputMessage4 = request->getParam(PARAM_INPUT_4)->value();
        inputMessage5 = request->getParam(PARAM_INPUT_5)->value();
        inputMessage6 = request->getParam(PARAM_INPUT_6)->value();
      }
    }
    else {
      inputMessage = "No message sent";
    }
    Serial.print("Recipient Email Address: ");    
    Serial.println(inputMessage);
    Serial.print("Email Checked: "); 
    Serial.println(inputMessage2);
    Serial.print("Temperature: ");
    Serial.println(inputMessage3);
    Serial.print("Humidity: ");
    Serial.println(inputMessage4);
    Serial.print("Pressure: ");
    Serial.println(inputMessage5);
    Serial.print("Altitude: ");
    Serial.println(inputMessage6);
    Serial.println("*************************************");
    request->send(200, "text/html", "HTTP GET request sent to your ESP.<br><a href=\"/\">Return to Home Page</a>");
  });
  server.onNotFound(notFound);
  server.begin();


}
// function to get the Email sending status
void sendCallback(SendStatus msg) {
  // Print the current status
  Serial.println(msg.info());

  if (msg.success()) {
    Serial.println(".....................................");
  }
}
bool sendEmailNotification(String emailMessage){
  // Set the SMTP Server Email host, port, account and password
  smtpData.setLogin(smtpServer, smtpServerPort, emailSenderAccount, emailSenderPassword);

  // Set the sender name and Email
  smtpData.setSender("ESP32", emailSenderAccount);

  // Set Email priority or importance High, Normal, Low or 1 to 5 (1 is highest)
  smtpData.setPriority("High");

  // Set the subject
  smtpData.setSubject(emailSubject);

  // Set the message with HTML format
  smtpData.setMessage(emailMessage, true);

  // Add recipients
  smtpData.addRecipient(inputMessage);

  smtpData.setSendCallback(sendCallback);

  // Start sending Email, can be set callback function to track the status
  if (!MailClient.sendMail(smtpData)) {
    Serial.println("Error sending Email, " + MailClient.smtpErrorReason());
    return false;
  }
  // Clear all data from Email object to free memory
  smtpData.empty();
  return true;
}

void loop() {
  

  
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;   
    
    lastTemperature = readDHTTemperature();
    float temperature = lastTemperature.toFloat();  
    
    lastHumidity = readDHTHumidity();
    float humidity = lastHumidity.toFloat(); 

    lastDewpoint = calculateDewPoint();
    float dewpoint= lastDewpoint.toFloat();  
    
    lastPressure = readPressure();
    float pressure= lastPressure.toFloat();  
    
    lastAltitude = readAltitude();
    float altitude = lastAltitude.toFloat();  

    lastSeapressure = calculateSeaLevelPressure();
    float seapressure = lastSeapressure.toFloat();  
    
    //logSensorReadings(temperature, humidity, pressure, altitude);


    // Check if all readings is above threshold and if it needs to send the Email alert
    if(temperature > inputMessage3.toFloat() &&(humidity > inputMessage4.toFloat())&&(pressure > inputMessage5.toFloat())&&(altitude > inputMessage6.toFloat())&& inputMessage2 == "true" && !emailSent){
      String emailMessage = String("Temperature above threshold. Current temperature: ") + 
                            String(temperature) + String(" C.")+
                            String("  Humidity above threshold. Current humidity: ")+
                            String(humidity) + String(" %.")+
                            String("  Pressure above threshold. Current pressure: ")+
                            String(pressure) + String(" hPa.")+
                            String("  Altitude above threshold. Current altitude: ")+
                            String(altitude) + String(" m.");
                            
      if(sendEmailNotification(emailMessage)) {
        Serial.println(emailMessage);
        Serial.println(".....................................");
        emailSent = true;
      }
      else {
        Serial.println("Email failed to send");
      }    
    }
    else if(temperature > inputMessage3.toFloat() &&(humidity < inputMessage4.toFloat())&&(pressure < inputMessage5.toFloat())&&(altitude < inputMessage6.toFloat())&& inputMessage2 == "true" && !emailSent){
      String emailMessage = String("Temperature above threshold. Current temperature: ") + 
                            String(temperature) + String(" C.");
                            
      if(sendEmailNotification(emailMessage)) {
        Serial.println(emailMessage);
        Serial.println(".....................................");
        emailSent = true;
      }
      else {
        Serial.println("Email failed to send");
      }    
    }
        else if(temperature < inputMessage3.toFloat() &&(humidity > inputMessage4.toFloat())&&(pressure < inputMessage5.toFloat())&&(altitude < inputMessage6.toFloat())&& inputMessage2 == "true" && !emailSent){
      String emailMessage = String("Humidity above threshold. Current humidity: ")+
                            String(humidity) + String(" %.");
                            
      if(sendEmailNotification(emailMessage)) {
        Serial.println(emailMessage);
        Serial.println(".....................................");
        emailSent = true;
      }
      else {
        Serial.println("Email failed to send");
      }    
    }
           else if(temperature < inputMessage3.toFloat() &&(humidity < inputMessage4.toFloat())&&(pressure > inputMessage5.toFloat())&&(altitude < inputMessage6.toFloat())&& inputMessage2 == "true" && !emailSent){
      String emailMessage = String("Pressure above threshold. Current pressure: ")+
                            String(pressure) + String(" hPa.");
                            
      if(sendEmailNotification(emailMessage)) {
        Serial.println(emailMessage);
        Serial.println(".....................................");
        emailSent = true;
      }
      else {
        Serial.println("Email failed to send");
      }    
    }
    else if(temperature < inputMessage3.toFloat() &&(humidity < inputMessage4.toFloat())&&(pressure < inputMessage5.toFloat())&&(altitude > inputMessage6.toFloat())&& inputMessage2 == "true" && !emailSent){
      String emailMessage = String("Altitude above threshold. Current altitude: ")+
                            String(altitude) + String(" m.");;
                            
      if(sendEmailNotification(emailMessage)) {
        Serial.println(emailMessage);
        Serial.println(".....................................");
        emailSent = true;
      }
      else {
        Serial.println("Email failed to send");
      }    
    }
    else if((temperature < inputMessage3.toFloat())&&(humidity < inputMessage4.toFloat()) &&(pressure < inputMessage5.toFloat())&&(altitude < inputMessage6.toFloat())&& inputMessage2 == "true" && emailSent) {
      String emailMessage = String("Temperature below threshold. Current temperature: ") + 
                            String(temperature) + String(" C.")+
                            String("  Humidity below threshold. Current humidity: ")+
                            String(humidity) + String(" %.")+
                            String("  Pressure below threshold. Current pressure: ")+
                            String(pressure) + String(" hPa.")+
                            String("  Altitude below threshold. Current altitude: ")+
                            String(altitude) + String(" m.");
      
      if(sendEmailNotification(emailMessage)) {
        Serial.println(emailMessage);
        Serial.println(".....................................");
        emailSent = false;
      }
      else {
        Serial.println("Email failed to send");
      }
    }
  }
}
