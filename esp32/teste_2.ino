#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <FirebaseFS.h> 

//precisam de instalar esta biblioteca https://github.com/mobizt/Firebase-ESP-Client

#define Web_API_KEY "AIzaSyBZrf6QJGbHiYzkmhm_hhBnb-hlfWAl9OM"
#define DATABASE_URL "https://spaceek-5869c-default-rtdb.firebaseio.com"
#define USER_EMAIL "dvfburke@gmail.com"
#define USER_PASS "7005288evora"


// Credenciais WiFi eduroam (Mantidas do seu código original)
#define EAP_ANONYMOUS_IDENTITY "anonymous@tecnico.ulisboa.pt"
#define EAP_IDENTITY           "ist1107107@tecnico.ulisboa.pt" // Seu usuário eduroam
#define EAP_PASSWORD "" // Sua senha eduroam
#define EAP_USERNAME "ist1107107" // Geralmente o mesmo que EAP_IDENTITY

const char* ssid = "eduroam"; // eduroam SSID

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

void setup() {
  Serial.begin(115200);
  delay(10);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");

  config.api_key = Web_API_KEY;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASS;

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  if (Firebase.ready()) {
    String documentPath = "salas/room1";
    FirebaseJson content;
    content.set("fields/noise_1v1/doubleValue", 0.02); 
    content.set("fields/temperature/integerValue", 28);
    content.set("fields/curOcc/integerValue", 64);
    if (Firebase.Firestore.patchDocument( 
    &fbdo,                     // FirebaseData object
    "spaceek-5869c",           // Project ID
    "",                        // Database ID (empty for default)
    documentPath,              // Document path (e.g., "salas/room1")
    content.raw(),             // JSON content
    "noise_1v1,temperature,curOcc", // Field masks (which fields to update)
    "",                        // Update mask (deprecated, leave empty)
    "",                        // Transaction ID (optional)
    ""                         // Read consistency (optional)
    )) 
    {
      Serial.println("Document updated!");
    } else {
      Serial.println("Error: " + fbdo.errorReason());
    }
  }
}

void loop() {
  yield();
}
