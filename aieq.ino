#include <Wire.h>
#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>
#include <Adafruit_Sensor.h>
#include <bme680.h>
#include <Adafruit_BME680.h>
#include <bme680_defs.h>
#include <Adafruit_TSL2591.h>
#include <Adafruit_SleepyDog.h>
#include <NTPClient.h>

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#include "config.h"

char wifi_ssid[] = WIFI_SSID;
char wifi_pass[] = WIFI_PASS;

WiFiUDP Udp;
NTPClient timeClient(Udp, NTP_SERVER, NTP_OFFSET, NTP_INTERVAL);

Adafruit_BME680 bme; // I2C
// Adafruit_BME680 bme(BME_CS); // hardware SPI
// Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);

Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);

extern "C" char *sbrk(int i);
int get_free_ram() {
    char stack_dummy = 0;
    return &stack_dummy - sbrk(0);
}

void setupWiFi(void) {
    if(WiFi.status() == WL_NO_SHIELD) {
        Serial.println("WiFi not available!");
        while(true) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(1000);
            digitalWrite(LED_BUILTIN, LOW);
            delay(1000);
        }
    }

    WiFi.lowPowerMode();
}

void setupBME(void) {
    if(bme.begin()) {
        Serial.println("Found BME680 sensor!");
    } else {
        Serial.println("No BME680 sensor found... check your wiring?");
        while(true) {
            delay(10000);
        }
    }
    Serial.println("Setting temperature oversampling ...");
    bme.setTemperatureOversampling(BME680_OS_8X);
    Serial.println("Setting humidity oversampling ...");
    bme.setHumidityOversampling(BME680_OS_2X);
    Serial.println("Setting pressure oversampling ...");
    bme.setPressureOversampling(BME680_OS_4X);
    Serial.println("Setting IIR filter size ...");
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    Serial.println("Setting gas heater ...");
    bme.setGasHeater(320, 150); // 320*C for 150 ms
}

void setupTSL(void) {
    if (tsl.begin()) {
        Serial.println("Found TSL2591 sensor!");
    } else {
        Serial.println(F("No TSL2591 sensor found... check your wiring?"));
        while(true) {
            delay(10000);
        }
    }

    sensor_t sensor;
    tsl.getSensor(&sensor);
    Serial.println(F("------------------------------------"));
    Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
    Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
    Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
    Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" lux"));
    Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" lux"));
    Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution, 4); Serial.println(F(" lux"));
    Serial.println(F("------------------------------------"));
    Serial.println(F(""));
}

void connectWiFi(void) {
    while(WiFi.status() != WL_CONNECTED) {
        Serial.print("Attempting to connect to SSID: ");
        Serial.println(wifi_ssid);
        WiFi.begin(wifi_ssid, wifi_pass);

        delay(10000);
    }

    Serial.println("Connected to WiFi!");
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    long rssi = WiFi.RSSI();
    Serial.print("RSSI: ");
    Serial.print(rssi);
    Serial.println(" dBm");
}

void setup() {
    WiFi.setPins(8,7,4,2);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.begin(115200);
    // while(!Serial) {
    //     ;
    // }

    setupWiFi();

    Serial.println("Starting timeClient ...");

    Serial.println("Starting timeClient ...");

    setupBME();
    setupTSL();
}

void loop() {
    Serial.println("Looping ...");

    digitalWrite(LED_BUILTIN, HIGH);

    char line[256];

    connectWiFi();

    Serial.println("Beginning timeClient ...");
    timeClient.begin();
    Serial.println("Updating timeClient ...");
    timeClient.update();

    //tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
    tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
    //tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain

    //tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
    // tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS);
    tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
    // tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS);
    // tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);
    // tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);  // longest integration time (dim light)

    Serial.println("Performing TSL2591 read ...");
    uint32_t lum = tsl.getFullLuminosity();
    uint16_t infrared, spectrum_full;
    infrared = lum >> 16;
    spectrum_full = lum & 0xFFFF;
    int spectrum_visible = infrared - spectrum_full;
    float lux = tsl.calculateLux(spectrum_full, infrared);

    float temperature, pressure, humidity, gas, altitude;

    Serial.println("Performing BME680 read ...");
    if(!bme.performReading()) {
        Serial.println("Failed to perform reading!");
        temperature = 0.0;
        pressure = 0.0;
        humidity = 0.0;
        gas = 0.0;
        altitude = 0.0;
    } else {
        Serial.println("Successfully finished BME680 read!");
        temperature = bme.temperature; // °C
        pressure = (bme.pressure / 100.0); // hPa
        humidity = bme.humidity; // %
        gas = bme.gas_resistance; // Ohms
        altitude = bme.readAltitude(SEALEVELPRESSURE_HPA); // Meters
    }

    uint32_t epoch_seconds = timeClient.getEpochTime();

    snprintf(line, sizeof(line), "aieq,host=%s light_infrared=%u,light_spectrum_full=%u,light_spectrum_visible=%u,light_lux=%f,temperature=%f,pressure=%f,humidity=%f,gas=%f,altitude=%f %u000000000", HOSTNAME, infrared, spectrum_full, spectrum_visible, lux, temperature, pressure, humidity, gas, altitude, epoch_seconds);

    Serial.println("Sending UDP packet...");
    if(Udp.beginPacket(INFLUXDB_IP, INFLUXDB_UDP_PORT) == 1) {
        Serial.println("Began UDP packet ...");
    } else {
        Serial.println("Could not begin UDP packet!");
    }
    int wrote_number = Udp.print(line);
    Serial.println("Wrote:");
    Serial.println(wrote_number);
    if(Udp.endPacket() == 1) {
        Serial.println("Sent UDP packet:");
    } else {
        Serial.println("Could not send UDP packet:");
    }
    yield();

    Serial.println(line);
    Serial.println("Stopping UDP ...");
    Udp.stop();
    Serial.println("Ending timeClient ...");
    timeClient.end();
    yield();
    delay(2000);

    Serial.println("Disconnecting from WiFi ...");
    WiFi.end();

    Serial.println("Sleeping ...");
    digitalWrite(LED_BUILTIN, LOW);
    // Watchdog.sleep(MEASURE_INTERVAL_MS);
    delay(10000);
}
