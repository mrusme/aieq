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

int  wifi_status = WL_IDLE_STATUS;
char wifi_ssid[] = WIFI_SSID;
char wifi_pass[] = WIFI_PASS;

WiFiUDP Udp;

Adafruit_BME680 bme; // I2C
// Adafruit_BME680 bme(BME_CS); // hardware SPI
// Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);

Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);

NTPClient timeClient(Udp, NTP_SERVER, NTP_OFFSET, NTP_INTERVAL);

extern "C" char *sbrk(int i);
int get_free_ram() {
    char stack_dummy = 0;
    return &stack_dummy - sbrk(0);
}

void displaySensorDetails(void)
{
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
    delay(500);
}

void setup() {
    WiFi.setPins(8,7,4,2);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.begin(115200);
    // while(!Serial) {
    //     ;
    // }

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

    while(wifi_status != WL_CONNECTED) {
        Serial.print("Attempting to connect to SSID: ");
        Serial.println(wifi_ssid);
        wifi_status = WiFi.begin(wifi_ssid, wifi_pass);

        delay(10000);
    }

    Serial.println("Connected to WiFi!");
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    long rssi = WiFi.RSSI();
    Serial.print("RSSI: ");
    Serial.print(rssi);
    Serial.println(" dBm");

    Serial.println("Starting timeClient ...");
    timeClient.begin();

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

    if (tsl.begin())
    {
        Serial.println(F("Found a TSL2591 sensor"));
    }
    else
    {
        Serial.println(F("No sensor found ... check your wiring?"));
        while (1);
    }

    displaySensorDetails();
}

void loop() {
    Serial.println("Looping ...");

    digitalWrite(LED_BUILTIN, HIGH);

    String line, local_ip, epoch_seconds;
    String infrared, spectrum_full, spectrum_visible, lux; // TSL2591
    String temperature, pressure, humidity, gas, altitude; // BME680

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
    uint16_t ir, full;
    ir = lum >> 16;
    full = lum & 0xFFFF;

    infrared = String(ir);
    spectrum_full = String(full);
    spectrum_visible = String(full - ir);
    lux = String(tsl.calculateLux(full, ir));

    Serial.println("Performing BME680 read ...");
    if(!bme.performReading()) {
        Serial.println("Failed to perform reading!");
        temperature = "0";
        pressure = "0";
        humidity = "0";
        gas = "0";
        altitude = "0";
    } else {
        Serial.println("Successfully finished BME680 read!");
        temperature = String(bme.temperature, 2); // °C
        pressure = String((bme.pressure / 100.0)); // hPa
        humidity = String(bme.humidity); // %
        gas = String(bme.gas_resistance); // Ohms
        altitude = String(bme.readAltitude(SEALEVELPRESSURE_HPA)); // Meters
    }

    local_ip = WiFi.localIP();
    epoch_seconds = String(timeClient.getEpochTime());

    line = String("aieq,host=" + local_ip + " light_infrared=" + infrared + ",light_spectrum_full=" + spectrum_full + ",light_spectrum_visible=" + spectrum_visible + ",light_lux=" + lux + ",temperature=" + temperature + ",pressure=" + pressure + ",humidity=" + humidity + ",gas=" + gas + ",altitude=" + altitude + " " + epoch_seconds + "000000000");

    Serial.println("Sending UDP packet...");
    Udp.beginPacket(INFLUXDB_IP, INFLUXDB_UDP_PORT);
    Udp.print(line);
    Udp.endPacket();
    Serial.println("Sent UDP packet:");
    Serial.println(line);

    digitalWrite(LED_BUILTIN, LOW);
    Watchdog.sleep(MEASURE_INTERVAL_MS);
}
