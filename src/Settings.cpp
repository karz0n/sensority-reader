#include "Settings.h"

#include <Preferences.h>

bool Settings::valid() const
{
    return !(_wifiSSID.isEmpty() || _wifiPass.isEmpty());
}

bool Settings::load()
{
    Preferences preferences;
    preferences.begin("credentials", true);
    if (preferences.isKey("wifi.ssid") && preferences.isKey("wifi.pswd")) {
        _wifiSSID = preferences.getString("wifi.ssid");
        _wifiPass = preferences.getString("wifi.pswd");
    }
    if (preferences.isKey("mqtt.user") && preferences.isKey("mqtt.pass")) {
        _mqttUser = preferences.getString("mqtt.user");
        _mqttPass = preferences.getString("mqtt.pass");
    }
    preferences.end();
    return valid();
}

const char* Settings::wiFiSSID() const
{
    return _wifiSSID.c_str();
}

const char* Settings::wifiPass() const
{
    return _wifiPass.c_str();
}

const char* Settings::mqttUser() const
{
    return _mqttUser.c_str();
}

const char* Settings::mqttPass() const
{
    return _mqttPass.c_str();
}