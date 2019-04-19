#include <Wire.h>
#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>
#include <Adafruit_Sensor.h>
#include <bme680.h>
#include <Adafruit_BME680.h>
#include <bme680_defs.h>
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
//Adafruit_BME680 bme(BME_CS); // hardware SPI
//Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);

NTPClient timeClient(Udp, NTP_SERVER, NTP_OFFSET, NTP_INTERVAL);

extern "C" char *sbrk(int i);
int get_free_ram() {
    char stack_dummy = 0;
    return &stack_dummy - sbrk(0);
}


void setup() {
    // WiFi.setPins(8,7,4,2);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.begin(115200);
    while(!Serial) {
        ;
    }

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
    Serial.println(rssi);
    Serial.println(" dBm");

    timeClient.begin();

    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms
}

void loop() {
#ifdef USBCON
    USBDevice.attach();
#endif

    digitalWrite(LED_BUILTIN, HIGH);

    String line, local_ip, temperature, pressure, humidity, gas, altitude, epoch_seconds;

    timeClient.update();

    if(!bme.performReading()) {
        Serial.println("Failed to perform reading!");
        return;
    }

    local_ip = String(WiFi.localIP());
    temperature = String(bme.temperature, 2); // °C
    pressure = String((bme.pressure / 100.0)); // hPa
    humidity = String(bme.humidity); // %
    gas = String(bme.gas_resistance); // Ohms
    altitude = String(bme.readAltitude(SEALEVELPRESSURE_HPA)); // Meters
    epoch_seconds = String(timeClient.getEpochTime());

    line = String("aieq,host=" + local_ip + " temperature=" + temperature + ",pressure=" + pressure + ",humidity=" + humidity + ",gas=" + gas + ",altitude=" + altitude + " " + epoch_seconds + "000000000");

    Serial.println("Sending UDP packet...");
    Udp.beginPacket(INFLUXDB_IP, INFLUXDB_UDP_PORT);
    Udp.print(line);
    Udp.endPacket();

    digitalWrite(LED_BUILTIN, LOW);
    Watchdog.sleep(MEASURE_INTERVAL_MS);
}
