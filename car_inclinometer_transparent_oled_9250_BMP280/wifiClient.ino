#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>

MDNSResponder mdns;
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

#define FPM_SLEEP_MAX_TIME  0xFFFFFFF

int otaServerExpireTime = 1000 * 60 * 5;  //5 mins

void wificonfig_wifiOn() {
  wifi_fpm_do_wakeup();
  wifi_fpm_close();
  delay(100);
}

void wificonfig_wifiOff() {
  wifi_station_disconnect();
  wifi_set_opmode(NULL_MODE);
  wifi_set_sleep_type(MODEM_SLEEP_T);
  wifi_fpm_open();
  wifi_fpm_do_sleep(FPM_SLEEP_MAX_TIME);
  delay(100);
}

void createWifiAP() {
  showStartDisplayMessage("Setup AP for OTA updates...");

  wificonfig_wifiOn();
  boolean result = WiFi.softAP("ESP_Tilt_AP", "12345678");

  delay(100);
  showStartDisplayMessage(result == true ? "OK" : "Failed", false);
  delay(200);

  IPAddress myIP = WiFi.softAPIP();
  Serial.print("Access Point IP address: ");
  Serial.println(myIP);
  if (mdns.begin("espotaserver", myIP)) {
    Serial.println("MDNS responder started");
  }
  httpUpdater.setup(&httpServer);
  httpServer.begin();

  delay(200);

  showStartDisplayMessage("HTTPUpdateServer ready! Open " + myIP.toString() + "/update in your browser");
  Serial.println("HTTPUpdateServer ready! Open http://espotaserver.local/update");
  Serial.printf("or http://");
  Serial.print(myIP);
  Serial.println("/update in your browser");

  delay(500);
}

void handleWifi() {
  httpServer.handleClient();
  if (millis() > otaServerExpireTime) {
    otaServer = false;
    wificonfig_wifiOff();
    Serial.println("Wifi off");
  }
}