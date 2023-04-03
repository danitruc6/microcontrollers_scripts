/*
 * Referencia de como habilitar I2C con ESP32 CAM https://3iinc.xyz/blog/how-to-use-i2c-sensor-bme280-with-esp32cam/
 * Referencia de como usar el BME280 en thigSpeak https://todomaker.com/blog/envio-de-datos-a-thingspeak-usando-esp32/
 * otra buena referencia https://randomnerdtutorials.com/esp32-thingspeak-publish-arduino/
 * como flashear el ESP32CAM https://www.circuitschools.com/how-to-program-upload-the-code-to-esp32-cam-using-arduino-or-programmer/
 */

// #include "esp_camera.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include <ThingSpeak.h>

// -----------------I2C-----------------
#define I2C_SDA 14  // SDA Connected to GPIO 14
#define I2C_SCL 15  // SCL Connected to GPIO 15
TwoWire I2CSensors = TwoWire(0);


// -----------------WiFi-----------------
const char* ssid = "CyberTruc";
const char* password = "Ppc_2.4!GHz";


/*Definimos las credenciales para la conexión a la plataforma*/
unsigned long channelID = 2045021;
const char* WriteAPIKey = "PQ4UKR8JRBZPRUVB";
/*Definimos el cliente WiFi que usaremos*/
WiFiClient cliente;


// BME 280 (Using I2C)
Adafruit_BME280 bme;

// Sensor Variable (BME280)
float temperature, humidity, pressure;

// avoiding delay funcion, using millis() instead
unsigned long previousMillis = 0UL;
unsigned long interval = 30000UL;

void setup() {
  Serial.begin(115200);
  I2CSensors.begin(I2C_SDA, I2C_SCL, 100000);

  // BME 280 (0x77 or 0x76 will be the address)
  if (!bme.begin(0x77, &I2CSensors)) {
    Serial.println("Couldn't Find BME280 Sensor");
    while (1)
      ;
  } else {
    Serial.println("BME280 Sensor Found");
  }

  /*Iniciamos la conexión a la red WiFi, y se imprimirán caracteres indicando el tiempo que tarda la conexión*/
  WiFi.mode(WIFI_STA);
  ThingSpeak.begin(cliente);
  delay(5000);
}

void loop() {
  // counting time
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    /* The Arduino executes this code once 30 seconds
 	*  (interval = 1000 (ms) = 1 second).
 	*/
    // -------------WiFi connection------------------
    // Connect or reconnect to WiFi
    if (WiFi.status() != WL_CONNECTED) {
      Serial.print("Attempting to connect");
      while (WiFi.status() != WL_CONNECTED) {
        WiFi.begin(ssid, password);
        delay(5000);
      }
      Serial.println("\nConnected.");
    }
    // -------------Temperature (C)------------------

    temperature = bme.readTemperature();
    Serial.print("Temperature = ");
    Serial.print(temperature);
    Serial.print(" *C - ");

    // ----------------------------------------------

    // ---------------Humidity (%)-------------------

    humidity = bme.readHumidity();
    Serial.print("Humidity = ");
    Serial.print(humidity);
    Serial.print(" % - ");

    pressure = bme.readPressure() / 100.0F;
    Serial.print("Pressure = ");
    Serial.print(pressure);
    Serial.println(" hPa");
    // ----------------------------------------------

    // Don't forget to update the previousMillis value

    // writing to thingspeak
    ThingSpeak.setField(1, temperature);
    ThingSpeak.setField(2, humidity);
    ThingSpeak.setField(3, pressure);
    int x = ThingSpeak.writeFields(channelID, WriteAPIKey);
    if (x == 200) {
      Serial.println("Channel update successful.");
    } else {
      Serial.println("Problem updating channel. HTTP error code " + String(x));
    }
    previousMillis = currentMillis;
  }
}