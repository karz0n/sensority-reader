#include <Preferences.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <SparkFunCCS811.h>
#include <WiFi.h>
#include <bsec.h>

#include "Settings.h"

#define LED_BUILTIN (2)
#define CCS811_ADDR1 (0x5A)
#define CCS811_ADDR2 (0x5B)
#define RECONNECT_TIMEOUT (3000)
#define MEASUREMENTS_INTERVAL (3000)

#define INDEX_TEMPERATURE (0)
#define INDEX_PRESSURE (1)
#define INDEX_GAS (2)
#define INDEX_HUMIDITY (3)
#define INDEX_IAQ (4)
#define INDEX_ECO2 (5)
#define INDEX_EVOC (6)
#define INDEX_TVOC (7)

static const IPAddress MQTT_SERVER(192, 168, 1, 10);
static const uint16_t MQTT_PORT = 1883;
static const char* MQTT_TOPICS[] = {
    "temperature",
    "pressure",
    "gas",
    "humidity",
    "iaq",
    "eBreathVOC",
    "eCO2",
    "TVOC",
};

void setupSerial();
void setupWiFi();
void setupSensor1();
void setupSensor2();
bool checkStatusOnSensor1();
bool checkStatusOnSensor2();
bool gatherDataOnSensor1();
bool gatherDataOnSensor2();
void processDataOnSensor1();
void processDataOnSensor2();
void suspend();
void reconnect();
void publishData(const char* topic, float data);

static WiFiClient wifiClient;
static PubSubClient mqttClient(MQTT_SERVER, MQTT_PORT, wifiClient);
static Bsec sensor1;
static CCS811 sensor2(CCS811_ADDR1);

static Settings settings;
static char dataBuffer[MQTT_MAX_PACKET_SIZE];
static float calibrateTempreture = 0.0;
static float calibrateHumidity = 0.0;
static bool calibrateStatus = false;

void setup()
{
    if (!settings.load()) {
        log_e("Settings are absent");
        suspend();
    }

    setupSerial();
    setupWiFi();
    setupSensor1();
    setupSensor2();
}

void loop()
{
    if (!mqttClient.connected()) {
        reconnect();
    }

    mqttClient.loop();

    if (gatherDataOnSensor1()) {
        processDataOnSensor1();
    }

    if (gatherDataOnSensor2()) {
        processDataOnSensor2();
    }
}

void setupSerial()
{
    Serial.begin(115200);
    while (!Serial) { };

    Wire.begin();
}

void setupWiFi()
{
    log_i("WiFi: Connecting...");
    WiFi.begin(settings.wiFiSSID(), settings.wifiPass());
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }
    log_i("WiFi: Connected");
}

void setupSensor1()
{
    sensor1.begin(BME680_I2C_ADDR_SECONDARY, Wire);
    if (!checkStatusOnSensor1()) {
        suspend();
    }

    bsec_virtual_sensor_t sensorList[10] = {
        BSEC_OUTPUT_RAW_TEMPERATURE,
        BSEC_OUTPUT_RAW_PRESSURE,
        BSEC_OUTPUT_RAW_HUMIDITY,
        BSEC_OUTPUT_RAW_GAS,
        BSEC_OUTPUT_IAQ,
        BSEC_OUTPUT_STATIC_IAQ,
        BSEC_OUTPUT_CO2_EQUIVALENT,
        BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
    };
    sensor1.updateSubscription(sensorList, sizeof(sensorList) / sizeof(bsec_virtual_sensor_t), BSEC_SAMPLE_RATE_LP);
    if (!checkStatusOnSensor1()) {
        suspend();
    }
}

void setupSensor2()
{
    const auto status = sensor2.beginWithStatus();

    if (status != CCS811Core::CCS811_Status_e::CCS811_Stat_SUCCESS) {
        log_e("CCS811: %s", sensor2.statusString(status));
        suspend();
    }
}

bool checkStatusOnSensor1()
{
    if (sensor1.status != BSEC_OK) {
        if (sensor1.status < BSEC_OK) {
            log_e("BME680: BSEC error (%d)", sensor1.status);
            return false;
        } else {
            log_w("BME680: BSEC warning (%d)", sensor1.status);
        }
    }
    if (sensor1.bme680Status != BME680_OK) {
        if (sensor1.bme680Status < BME680_OK) {
            log_e("BME680: Error (%d)", sensor1.bme680Status);
            return false;
        } else {
            log_e("BME680: Warning code (%d)", sensor1.bme680Status);
        }
    }
    return true;
}

bool checkStatusOnSensor2()
{
    if (!sensor2.checkForStatusError()) {
        log_e("CCS811: Invalid status");
        return false;
    }
    return true;
}

bool gatherDataOnSensor1()
{
    static auto timestamp = millis();

    const auto elapsed = millis() - timestamp;
    if (elapsed < MEASUREMENTS_INTERVAL) {
        return false;
    } else {
        timestamp = millis();
    }

    if (!sensor1.run()) {
        if (!checkStatusOnSensor1()) {
            calibrateStatus = false;
            timestamp = millis();
        }
    }

    return true;
}

bool gatherDataOnSensor2()
{
    static auto timestamp = millis();

    const auto elapsed = millis() - timestamp;
    if (elapsed < MEASUREMENTS_INTERVAL) {
        return false;
    } else {
        timestamp = millis();
    }

    if (sensor2.dataAvailable()) {
        timestamp = millis();
        if (calibrateStatus) {
            sensor2.setEnvironmentalData(calibrateHumidity, calibrateTempreture);
        }
        const auto status = sensor2.readAlgorithmResults();
        if (status != CCS811Core::CCS811_Status_e::CCS811_Stat_SUCCESS) {
            log_e("CCS811: %s", sensor2.statusString(status));
            return false;
        }
        return true;
    }

    return false;
}

void processDataOnSensor1()
{
    StaticJsonDocument<MQTT_MAX_PACKET_SIZE> values;
    values["value"] = sensor1.temperature;
    values["raw"] = sensor1.rawTemperature;
    size_t len = serializeJson(values, dataBuffer, sizeof(dataBuffer));
    mqttClient.publish(MQTT_TOPICS[INDEX_TEMPERATURE], dataBuffer, len);

    values.clear();
    values["value"] = sensor1.humidity;
    values["raw"] = sensor1.rawHumidity;
    len = serializeJson(values, dataBuffer, sizeof(dataBuffer));
    mqttClient.publish(MQTT_TOPICS[INDEX_HUMIDITY], dataBuffer, len);

    values.clear();
    values["value"] = sensor1.pressure;
    len = serializeJson(values, dataBuffer, sizeof(dataBuffer));
    mqttClient.publish(MQTT_TOPICS[INDEX_PRESSURE], dataBuffer, len);

    values.clear();
    values["value"] = sensor1.gasResistance;
    len = serializeJson(values, dataBuffer, sizeof(dataBuffer));
    mqttClient.publish(MQTT_TOPICS[INDEX_GAS], dataBuffer, len);

    values.clear();
    values["value"] = sensor1.iaq;
    values["raw"] = sensor1.staticIaq;
    values["accuracy"] = sensor1.iaqAccuracy;
    len = serializeJson(values, dataBuffer, sizeof(dataBuffer));
    mqttClient.publish(MQTT_TOPICS[INDEX_IAQ], dataBuffer, len);

    values.clear();
    values["value"] = sensor1.co2Equivalent;
    values["accuracy"] = sensor1.co2Accuracy;
    values["equivalent"] = sensor1.co2Equivalent;
    len = serializeJson(values, dataBuffer, sizeof(dataBuffer));
    mqttClient.publish(MQTT_TOPICS[INDEX_ECO2], dataBuffer, len);

    values.clear();
    values["value"] = sensor1.breathVocEquivalent;
    values["accuracy"] = sensor1.breathVocAccuracy;
    values["equivalent"] = sensor1.breathVocEquivalent;
    len = serializeJson(values, dataBuffer, sizeof(dataBuffer));
    mqttClient.publish(MQTT_TOPICS[INDEX_EVOC], dataBuffer, len);

    calibrateStatus = true;
    calibrateTempreture = sensor1.temperature;
    calibrateHumidity = sensor1.humidity;
}

void processDataOnSensor2()
{
    StaticJsonDocument<MQTT_MAX_PACKET_SIZE> values;
    values["value"] = sensor2.getTVOC();
    size_t len = serializeJson(values, dataBuffer, sizeof(dataBuffer));
    mqttClient.publish(MQTT_TOPICS[INDEX_TVOC], dataBuffer, len);
}

void suspend()
{
    log_i("Freezing");
    while (true) {
        pinMode(LED_BUILTIN, OUTPUT);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
    }
}

void reconnect()
{
    while (!mqttClient.connected()) {
        log_i("MQTT: Attempting to create connection...");
        String clientId = F("client-");
        clientId += String(random(0xffff), HEX);
        if (mqttClient.connect(clientId.c_str(), settings.mqttUser(), settings.mqttPass())) {
            log_i("MQTT: Connected");
        } else {
            log_e("MQTT: Create connection has failed (%d)", mqttClient.state());
            delay(RECONNECT_TIMEOUT);
        }
    }
}
