#define WIFI_SSID ""
#define WIFI_PASS ""
#define HOSTNAME ""
#define SEALEVELPRESSURE_HPA (1013.25)

#define NTP_SERVER "pool.ntp.org"
#define NTP_OFFSET 0
#define NTP_INTERVAL 3600000

#define MEASURE_INTERVAL_MS 300000

IPAddress INFLUXDB_IP(10, 10, 10, 10);
int INFLUXDB_UDP_PORT = 8089;
int INFLUXDB_TCP_PORT = 8086;
#define INFLUXDB_PROTO 1 // 0 for UDP, 1 for TCP
// For TCP only:
#define INFLUXDB_DATABSE "database"
#define INFLUXDB_USERNAME ""
#define INFLUXDB_PASSWORD ""
