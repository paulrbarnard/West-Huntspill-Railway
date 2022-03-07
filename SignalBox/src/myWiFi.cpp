
#include <myWiFi.h>

const char *ssid = "WHRSignals";
const char *password = ""; // set for suitable password

const String disconnectReasons[] = {"NULL",
                                   "WIFI_REASON_UNSPECIFIED",
                                   "WIFI_REASON_AUTH_EXPIRE",
                                   "WIFI_REASON_AUTH_LEAVE",
                                   "WIFI_REASON_ASSOC_EXPIRE",
                                   "WIFI_REASON_ASSOC_TOOMANY",
                                   "WIFI_REASON_NOT_AUTHED",
                                   "WIFI_REASON_NOT_ASSOCED",
                                   "WIFI_REASON_ASSOC_LEAVE",
                                   "WIFI_REASON_ASSOC_NOT_AUTHED",
                                   "WIFI_REASON_DISASSOC_PWRCAP_BAD",
                                   "WIFI_REASON_DISASSOC_SUPCHAN_BAD",
                                   "WIFI_REASON_IE_INVALID",
                                   "WIFI_REASON_MIC_FAILURE",
                                   "WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT",
                                   "WIFI_REASON_GROUP_KEY_UPDATE_TIMEOUT",
                                   "WIFI_REASON_IE_IN_4WAY_DIFFERS",
                                   "WIFI_REASON_GROUP_CIPHER_INVALID",
                                   "WIFI_REASON_PAIRWISE_CIPHER_INVALID",
                                   "WIFI_REASON_AKMP_INVALID",
                                   "WIFI_REASON_UNSUPP_RSN_IE_VERSION",
                                   "WIFI_REASON_INVALID_RSN_IE_CAP",
                                   "WIFI_REASON_802_1X_AUTH_FAILED",
                                   "WIFI_REASON_CIPHER_SUITE_REJECTED"

};

/*
WIFI_REASON_BEACON_TIMEOUT           = 200,
WIFI_REASON_NO_AP_FOUND              = 201,
WIFI_REASON_AUTH_FAIL                = 202,
WIFI_REASON_ASSOC_FAIL               = 203,
WIFI_REASON_HANDSHAKE_TIMEOUT        = 204,
*/

void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info)
{
    Serial.println("Connected to AP");
}

void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info)
{
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("RSSI: ");
    Serial.println(WiFi.RSSI());
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info)
{
    Serial.println("Disconnected from WiFi access point");
    Serial.print("WiFi lost connection. Reason: ");
    Serial.print(info.disconnected.reason);
    Serial.print(" ");
    Serial.println(disconnectReasons[info.disconnected.reason]);
    Serial.println("Trying to Reconnect");
    WiFi.begin(ssid, password);
}

void setupWiFi()
{

    // delete old config
    WiFi.disconnect(true);

    delay(1000);

    WiFi.onEvent(WiFiStationConnected, SYSTEM_EVENT_STA_CONNECTED);
    WiFi.onEvent(WiFiGotIP, SYSTEM_EVENT_STA_GOT_IP);
    WiFi.onEvent(WiFiStationDisconnected, SYSTEM_EVENT_STA_DISCONNECTED);

    /* Remove WiFi event
    Serial.print("WiFi Event ID: ");
    Serial.println(eventID);
    WiFi.removeEvent(eventID);*/

    WiFi.begin(ssid, password);

    Serial.println();
    Serial.println();
    Serial.println("Waiting for WiFi... ");
    int retry = 0;
    while (WiFi.status() != WL_CONNECTED)
    {
        if (retry == 0)
        {
            Serial.printf("WiFi Status:%d", WiFi.status());
        }
        else
        {
            Serial.printf("%d", WiFi.status());
        }
        retry++;
        if (retry >= 50)
        {
            ESP.restart();
        }
        delay(500);
    }
}
