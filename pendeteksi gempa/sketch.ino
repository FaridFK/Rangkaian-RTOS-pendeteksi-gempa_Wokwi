#include <WiFi.h>
#include <WiFiClient.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "ThingSpeak.h"
#include <LiquidCrystal_I2C.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define pinBuzzer 18
int LEDmerah = 23;
int LEDhijau = 19;

WiFiClient client;

String thingSpeakAddress = "api.thingspeak.com";
String writeAPIKey;
String tsfield1Name;
String request_string;

LiquidCrystal_I2C lcd(0x27, 20, 4);

Adafruit_MPU6050 mpu;

void mpu6050Task(void *parameter);
void kirimThingspeakTask(void *parameter);

void setup()
{
  pinMode(LEDmerah, OUTPUT); // Set LEDmerah sebagai output
  pinMode(LEDhijau, OUTPUT); // Set LEDhijau sebagai output
  Serial.begin(115200);

  // Koneksi Wifi
  WiFi.disconnect();
  WiFi.begin("Wokwi-GUEST", "");
  while (!(WiFi.status() == WL_CONNECTED))
  {
    delay(300);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Koneksi mpu6050
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
  Serial.println("");
  delay(100);
  lcd.init();
  lcd.backlight();

  // Create tasks
  xTaskCreatePinnedToCore(
      mpu6050Task,    // Task function
      "mpu6050Task",  // Task name
      4096,           // Stack size (bytes)
      NULL,           // Task parameter
      1,              // Task priority
      NULL,           // Task handle
      0);             // Run on core 0

  xTaskCreatePinnedToCore(
      kirimThingspeakTask, // Task function
      "kirimThingspeakTask", // Task name
      4096,                 // Stack size (bytes)
      NULL,                 // Task parameter
      1,                    // Task priority
      NULL,                 // Task handle
      0);                   // Run on core 0
}

// inisialisasi nilai sensor
double x, y, z;
double xg, yg, zg;
int t;

void loop()
{
  // Empty loop
}

void mpu6050Task(void *parameter)
{
  while (1)
  {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Acceleration
    x = a.acceleration.x;
    y = a.acceleration.y;
    z = a.acceleration.z;

    // Gyroscope
    xg = g.gyro.x;
    yg = g.gyro.y;
    zg = g.gyro.y;

    t = temp.temperature;

    // Tidak Gempa
    if (x == 0 && y == 0)
    {
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
    // Terjadi Gempa
    else
    {
      lcd.clear();
      lcd.setCursor(1, 1);
      lcd.println("Terjadi Gempa Bumi");
      lcd.setCursor(3, 2);
      lcd.println("GEMPA GEMPA !!!");
      // Acceleration
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
      // Rotation
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
      // Temperature
      Serial.println("Temperature : ");
      Serial.print("Temp :");
      Serial.print(t);
      Serial.println(" degC");
      delay(100);
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void kirimThingspeakTask(void *parameter)
{
  while (1)
  {
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

      // Send HTTP request
      client.print(String("GET ") + request_string + " HTTP/1.1\r\n" +
                   "Host: " + thingSpeakAddress + "\r\n" +
                   "Connection: close\r\n\r\n");

      // Read server response
      while (client.connected())
      {
        String line = client.readStringUntil('\n');
        if (line == "\r")
          break;
      }
      client.stop();
    }

    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}
