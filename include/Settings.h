
#include <WString.h>

class Settings {
public:
    bool valid() const;

    bool load();

    const char* wiFiSSID() const;

    const char* wifiPass() const;

    const char* mqttUser() const;

    const char* mqttPass() const;

private:
    String _wifiSSID;
    String _wifiPass;
    String _mqttUser;
    String _mqttPass;
};