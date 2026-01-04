#include "wifi_manager.h"

// Global server object (defined in main file)
extern WebServer server;

void setupWiFi() {
  // Check if we should connect to existing WiFi or create Access Point
  if (strlen(wifiSSID) > 0) {
    // Try to connect to existing WiFi network (Station mode)
    Serial.println("Attempting to connect to WiFi network...");
    Serial.print("SSID: ");
    Serial.println(wifiSSID);
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifiSSID, wifiPassword);
    
    // Wait for connection (max 20 seconds)
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 40) {
      delay(500);
      Serial.print(".");
      attempts++;
    }
    Serial.println();
    
    if (WiFi.status() == WL_CONNECTED) {
      // Successfully connected to WiFi
      IPAddress IP = WiFi.localIP();
      Serial.println("WiFi connected!");
      Serial.print("IP address: ");
      Serial.println(IP);
      Serial.print("Gateway: ");
      Serial.println(WiFi.gatewayIP());
      Serial.print("Subnet: ");
      Serial.println(WiFi.subnetMask());
      Serial.print("OTA Password: ");
      Serial.println(otaPassword);
      Serial.print("Access web interface at: http://");
      Serial.print(IP);
      Serial.println("/");
      Serial.print("Access OTA at: http://");
      Serial.print(IP);
      Serial.println("/update");
      Serial.println("\nNote: IP address is assigned by your router (DHCP)");
      return;
    } else {
      // Connection failed, fall back to Access Point mode
      Serial.println("\nFailed to connect to WiFi. Starting Access Point mode...");
    }
  }
  
  // Access Point mode (fallback or if wifiSSID is empty)
  WiFi.mode(WIFI_AP);
  WiFi.softAP(apSSID, apPassword);
  
  IPAddress IP = WiFi.softAPIP();
  Serial.println("WiFi Access Point started");
  Serial.print("AP IP address: ");
  Serial.println(IP);
  Serial.print("SSID: ");
  Serial.println(apSSID);
  Serial.print("Password: ");
  Serial.println(apPassword);
  Serial.print("OTA Password: ");
  Serial.println(otaPassword);
  Serial.print("Access web interface at: http://");
  Serial.print(IP);
  Serial.println("/");
  Serial.print("Access OTA at: http://");
  Serial.print(IP);
  Serial.println("/update");
  Serial.println("\nNote: In AP mode, IP is always 192.168.4.1");
}

void setupOTA() {
  // Initialize ElegantOTA with password protection
  ElegantOTA.begin(&server, otaPassword);
  
  // Enable auto-reboot after successful update
  ElegantOTA.setAutoReboot(true);
  
  Serial.println("ElegantOTA initialized");
  Serial.print("OTA Password: ");
  Serial.println(otaPassword);
  
  // Note: Stop steppers before OTA update manually or via web interface
  // ElegantOTA handles the update process automatically
}


