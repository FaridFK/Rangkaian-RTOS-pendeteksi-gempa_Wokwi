#include <WiFi.h>
#include <WiFiClient.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "ThingSpeak.h"
#include <LiquidCrystal_I2C.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// WiFi credentials
#define WIFI_SSID "M114118"
#define WIFI_PASSWORD "your_password"

// Pin definitions
#define pinBuzzer 18
#define LEDmerah 23
#define LEDhijau 19

// LCD settings
LiquidCrystal_I2C lcd(0x27, 20, 4);

// MPU6050 object
Adafruit_MPU6050 mpu;

// ThingSpeak settings
WiFiClient client;
String thingSpeakAddress;
String writeAPIKey;
String tsfield1Name;
String request_string;

// Task handles
TaskHandle_t sensorDataTaskHandle;

// Semaphore handle
SemaphoreHandle_t semaphore;

// Mutex handle
portMUX_TYPE mutex = portMUX_INITIALIZER_UNLOCKED;

// Sensor data
double x, y, z;
double xg, yg, zg;
int t;

void setupWiFi()
{
   WiFi.disconnect();
  WiFi.begin("Wokwi-GUEST", "");
  while ((!(WiFi.status() == WL_CONNECTED))) {
    delay(300);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

}

void setupMPU6050()
{
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);
}

void initializeLCD()
{
  lcd.init();
  lcd.backlight();
}

void displayLCD(const String &line1, const String &line2)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

void readMPU6050()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

   //Acceleration
  x = a.acceleration.x;
  y = a.acceleration.y;
  z = a.acceleration.z;

// Gyroscope
  xg = g.gyro.x;
  yg = g.gyro.y;
  zg = g.gyro.y;

  t = temp.temperature;

  //Tidak Gempa
  if ((x==0)and (y==0)){
    lcd.clear();
    lcd.setCursor(1, 1);
    lcd.println("Aman Tidak Terjadi");
    lcd.setCursor(5, 2);
    lcd.println("GEMPA BUMI");
    Serial.print("Aman : ");
    Serial.print(x);
    Serial.print(",");
    Serial.print(y);
    Serial.print(",");
    Serial.print(z);
    Serial.print(", ");
    delay(1000);
    tone(pinBuzzer, 0);
    digitalWrite(LEDmerah, LOW);
    digitalWrite(LEDhijau, HIGH);
  }
  //Terjadi Gempa
  else{
    lcd.clear();
    lcd.setCursor(1, 1);
    lcd.println("Terjadi Gempa Bumi");
    lcd.setCursor(3, 2);
    lcd.println("GEMPA GEMPA !!!");
    //Acceleration
    Serial.println("Acceleration : ");
    Serial.print(" X: ");
    Serial.print(x);
    Serial.print(", Y: ");
    Serial.print(y);
    Serial.print(", Z: ");
    Serial.print(z);
    Serial.println(" m/s^2");
    tone(pinBuzzer, 800);
    digitalWrite(LEDmerah, HIGH);
    digitalWrite(LEDhijau, LOW);
    delay(100);
    //Rotation
    Serial.println("Rotation : ");
    Serial.print(" X: ");
    Serial.print(xg);
    Serial.print(", Y: ");
    Serial.print(yg);
    Serial.print(", Z: ");
    Serial.print(zg);
    tone(pinBuzzer, 700);
    digitalWrite(LEDmerah, HIGH);
    digitalWrite(LEDhijau, LOW);
    delay(100);
    //Temperature
    Serial.println("Temperature : ");
    Serial.print("Temp :");
    Serial.print(t);
    Serial.println(" degC");
    delay(100);
  }
  
}




void processSensorDataTask(void *parameters)
{
  while (1)
  {
    // Read sensor data
    readMPU6050();

    // Update LCD
    String line1 = "Acceleration (m/s^2)";
    String line2 = "X:" + String(x) + " Y:" + String(y) + " Z:" + String(z);
    displayLCD(line1, line2);

    // Detect earthquake
    bool isEarthquake = false;
    if (x == 0 && y == 0)
    {
      isEarthquake = false;
    }
    else
    {
      isEarthquake = true;
    }

    // Update LED
    digitalWrite(LEDmerah, isEarthquake ? HIGH : LOW);
    digitalWrite(LEDhijau, isEarthquake ? LOW : HIGH);

    // Send data to ThingSpeak
    if (client.connect("api.thingspeak.com", 80))
    {
      request_string = "/update?";
      request_string += "key=";
      request_string += "DJJBY2GVDN935JXJ";
      request_string += "&";
      request_string += "field1";
      request_string += "=";
      request_string += x;
      request_string += "&";
      request_string += "field2";
      request_string += "=";
      request_string += y;
      request_string += "&";
      request_string += "field3";
      request_string += "=";
      request_string += z;

      request_string += "&";
      request_string += "field4";
      request_string += "=";
      request_string += xg;
      request_string += "&";
      request_string += "field5";
      request_string += "=";
      request_string += yg;
      request_string += "&";
      request_string += "field6";
      request_string += "=";
      request_string += zg;

      request_string += "&";
      request_string += "field7";
      request_string += "=";
      request_string += t;

      client.print(String("GET ") + request_string + " HTTP/1.1\r\n" +
                   "Host: " + thingSpeakAddress + "\r\n" +
                   "Connection: close\r\n\r\n");
      client.stop();
    }

    delay(1000);
  }
}

void setup()
{
  pinMode(LEDmerah, OUTPUT);
  pinMode(LEDhijau, OUTPUT);
  pinMode(pinBuzzer, OUTPUT);

  Serial.begin(115200);

  setupWiFi();
  setupMPU6050();
  initializeLCD();

  // Create semaphore
  semaphore = xSemaphoreCreateBinary();

  // Create sensor data task
  xTaskCreatePinnedToCore(
      processSensorDataTask,
      "SensorData",
      4096,
      NULL,
      2,
      &sensorDataTaskHandle,
      1);
}

void loop()
{
  // Do nothing in the main loop
}