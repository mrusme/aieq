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

#define NUMBER_OF_LINES 30
#define LENGTH_OF_LINE 256

char lines[NUMBER_OF_LINES][LENGTH_OF_LINE];
int line_iterator = 0;

WiFiUDP Udp;
WiFiClient Tcp;
NTPClient timeClient(Udp, NTP_SERVER, NTP_OFFSET, NTP_INTERVAL);

uint32_t epoch_seconds = 0;

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

void disconnectWiFi(void) {
    delay(1000);
    Serial.println("Disconnecting from WiFi ...");
    WiFi.end();
    yield();
}

void updateTime(void) {
    Serial.println("Beginning timeClient ...");
    timeClient.begin();
    Serial.println("Updating timeClient ...");
    timeClient.update();

    yield();
    epoch_seconds = timeClient.getEpochTime();

    Serial.println("Ending timeClient ...");
    timeClient.end();
    yield();
    delay(1000);
}

void readSensors(void) {
    if(line_iterator >= NUMBER_OF_LINES) {
        Serial.println("Could not read sensors: Line iterator larger than buffer!");
        return;
    }

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

    snprintf(lines[line_iterator], (LENGTH_OF_LINE * sizeof(char)), "aieq,host=%s light_infrared=%u,light_spectrum_full=%u,light_spectrum_visible=%u,light_lux=%f,temperature=%f,pressure=%f,humidity=%f,gas=%f,altitude=%f %u000000000", HOSTNAME, infrared, spectrum_full, spectrum_visible, lux, temperature, pressure, humidity, gas, altitude, epoch_seconds);
    return;
}

#if INFLUXDB_PROTO == 0
int sendLines(void) {
    int retval = 1;

    Serial.println("Sending UDP packet...");
    if(Udp.beginPacket(INFLUXDB_IP, INFLUXDB_UDP_PORT) == 1) {
        Serial.println("Began UDP packet ...");
    } else {
        Serial.println("Could not begin UDP packet!");
    }

    for(int i = 0; i < NUMBER_OF_LINES; i++) {
        char *line = lines[i];

        int wrote_number = Udp.print(line);
        Serial.println("Wrote:");
        Serial.println(wrote_number);
        if(Udp.endPacket() == 1) {
            Serial.println("Sent UDP packet:");
            Serial.println(line);
        } else {
            Serial.println("Could not send UDP packet:");
            Serial.println(line);
            retval = 0;
        }
        yield();
    }

    Serial.println("Stopping UDP ...");
    Udp.stop();
    yield();

    return retval;
}
#else
int sendLines(void) {
    int retval = 1;
    char url[256];
    char host[64];
    char clength[32];

    snprintf(url, 256, "POST /write?db=%s&u=%s&p=%s HTTP/1.1", INFLUXDB_DATABSE, INFLUXDB_USERNAME, INFLUXDB_PASSWORD);
    snprintf(host, 64, "Host: %u.%u.%u.%u:%i", INFLUXDB_IP[0], INFLUXDB_IP[1], INFLUXDB_IP[2], INFLUXDB_IP[3], INFLUXDB_TCP_PORT);

    for(int i = 0; i < NUMBER_OF_LINES; i++) {
        char *line = lines[i];

        Serial.println("Sending HTTP request...");
        if(Tcp.connect(INFLUXDB_IP, INFLUXDB_TCP_PORT)) {
            Serial.println("Connected to HTTP server ...");

            Tcp.println(url);
            Serial.println(url);
            Tcp.println(host);
            Serial.println(host);
            Tcp.println("User-Agent: aieq/1.1");
            snprintf(clength, 32, "Content-Length: %i", strlen(line));
            Tcp.println(clength);
            Serial.println(clength);
            Tcp.println("Connection: close");
            Tcp.println();
            Tcp.println(line);
            Serial.println(line);
            Tcp.println();
        } else {
            Serial.println("Could not connect to HTTP server!");
            retval = 0;
        }
        Serial.println("Terminating HTTP request...");
        Tcp.stop();
        yield();
    }
    yield();

    return retval;
}
#endif

void flushLines(void) {
    for(int i = 0; i < NUMBER_OF_LINES; i++) {
        memset(lines[i], 0, LENGTH_OF_LINE);
    }
}

int maybeFlushLines(void) {
    int retval = 1;

    line_iterator++;

    if(line_iterator < NUMBER_OF_LINES) {
        Serial.println("Not flushing yet ...");
        return retval;
    }

    Serial.println("Time to flush!");

    connectWiFi();
    updateTime();

    retval = sendLines();
    if(retval == 1) {
        flushLines();
    }

    if(retval == 1) {
        line_iterator = 0;
    }

    disconnectWiFi();
    return retval;
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
    connectWiFi();
    updateTime();
    disconnectWiFi();

    setupBME();
    setupTSL();
    digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
    int slept_ms = Watchdog.sleep(MEASURE_INTERVAL_MS);

    Serial.println("Looping ...");
    digitalWrite(LED_BUILTIN, HIGH);

    // int slept_ms = 1000;
    epoch_seconds += (int)(slept_ms / 1000);
    Serial.println("The time is now:");
    Serial.println(epoch_seconds);

    readSensors();
    maybeFlushLines();

    Serial.println("Sleeping ...");
    digitalWrite(LED_BUILTIN, LOW);
    // delay(1000);
}
