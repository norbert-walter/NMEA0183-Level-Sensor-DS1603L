/*
  Es werden Sensorwerte vom Ultraschallsensor DS1603L erfasst und mittels WiFi als NMEA Stream per UDP in das Netzwerk gesendet.
  Die Übertragung erfolgt auf die örtliche Broadcastadresse xxx.xxx.xxx.255 an den Port 50000.

  Nach dem erstmaligen Start oder bei Start in einem Netzwerk welches noch nicht bekannt ist erfolgt der
  Start mit einer Konfigurationsseite und Timeout
  Nach Ablauf von Timeout wird das WiFi-Modem für 3 Minuten abgeschaltet und dann erfolgt reset Wemos und Neustart mit Konfigurationsseite

  Geht die WiFi-Verbindung verloren wird die sichere Anmeldeseite aufgerufen, nach Timeout erfolgt reset Wemos und die normale Anmeldeschleife wie zuvor beschrieben startet.

  Automatische Wiederverbindung bei Wiederkehr WiFi ohne Neuverbindung

  Die Übertragung der Daten erfolgt über NMEA per UDP an BroadcastIP port 50000.
  Es werden 4 Pakete gesendet damit die auch wirklich nach dem aufwachen ankommen.

  Sensoreinbindung
  Die Sensorwerte werden in der Unterroutine alle 5 Sekunden auslesen.
  Die Sensorwerte werden in ein Schieberegister zum gleitenden Durchschnitt gegeben, jeweils 10 Werte bilden Durchschnitt, Register als FIFO (first In / first out)
  Es wird ein einfacher gleitender Durchschnitt gebildet.

  Der Datenstrom kann in OpenCPN eingelesen werden.
  Dazu unter Verbindungen eine neue Netzwerkverbindung einrichten, die Adresse ist die Broadcastadresse des Netzwerks (letzten drei Stellen .255) und der Port ist 50000
  Es wird eine XDR-Sequenz ausgegeben, die dann mit dem Enginedashboard-Plugin in Opencpn ausgelesen werden kann. Verschiedene Tanktypen können durch Anpassen von "FUEL" in der Unterroutine erfasst werden.
  Dazu bitte die Dokumentation vom EngineDashboard-Plugin lesen.


  adapted from Ethernet library examples
*/

#include <Arduino.h>
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <WiFiUdp.h>
#include "DS1603L.h"              //https://github.com/wvmarle/Arduino_DS1603L
#include <SoftwareSerial.h>
#include <MovingAverage.h>

//Einstellungen für broadcasting
unsigned int portBroadcast = 50000;      // localer port an den gesendet wird
unsigned int broadCast = 0;

//Variablen für Timer um Sensorwerte zu lesen
unsigned long Timer_RX = 0;
long Timeout_RX = 5000;                // Intervall in ms hier 5000ms = 5s

// Angabe wie der Sensor angeschlossen ist. Nutze Dx statt x für Pin-Angabe wenn Wemos benutzt wird
const byte txPin = D3;                               // tx of the Wemos to rx of the sensor
const byte rxPin = D4;                               // rx of the Wemos to tx of the sensor
SoftwareSerial sensorSerial(rxPin, txPin);

// If your sensor is connected to Serial, Serial1, Serial2, AltSoftSerial, etc. pass that object to the sensor constructor.
DS1603L sensor(sensorSerial);

//WiFiUDP
WiFiUDP Udp;

//WiFiManager
WiFiManager wifiManager;

//Variable für Ergebnis Sensorwert
int hoehe = 0;

//Für Berechnungen Prozentwert als Ausgabe
String Fuell = "0";

// Definition eines Arrays von 10 Feldern für gleitenden Durchschnitt
MovingAverage <uint8_t, 10> hoehe_D;  //Gleitender Durchschnitt aus 10 Werten

// Zum Senden des NMEA-Strings nötig Um die richtige Variable-Form zu bilden
char XDR;
String XDR1;

//Subroutine um Sensorwerte zu bekommen. Sensorwerte werden in ein Schieberegister geschrieben um aus 10 Werten den gleitenden Durchschnitt zu bekommen
unsigned int Sensor () {
  if (millis() - Timer_RX > Timeout_RX) {
    Timer_RX = millis ();
    Serial.println(F("Starting reading."));
    unsigned int reading = sensor.readSensor();       // Call this as often or as little as you want - the sensor transmits every 1-2 seconds.
    byte sensorStatus = sensor.getStatus();           // Check the status of the sensor (not detected; checksum failed; reading success).
    switch (sensorStatus) {                           // For possible values see DS1603L.h
      case DS1603L_NO_SENSOR_DETECTED:                // No sensor detected: no valid transmission received for >10 seconds.
        Serial.println(F("No sensor detected (yet). If no sensor after 1 second, check whether your connections are good."));
        break;

      case DS1603L_READING_SUCCESS:                   // Latest reading was valid and received successfully.
        Serial.println(F("Reading success."));
        Serial.println(reading);
        Serial.println(F(" mm."));
        break;

      case DS1603L_READING_CHECKSUM_FAIL:             // Checksum of the latest transmission failed.
        Serial.print(F("Data received; checksum failed. Latest level reading: "));
        break;

    }
    hoehe_D.add(reading);                            //Schieberegister zur Mittelwertbildung
    unsigned int reading_D = hoehe_D.get();
    //Serial.println("Durchschnittshöhe:");           //Zum Test
    //Serial.println(reading_D);                      //Zum Test
    return reading_D;
  }
  unsigned int reading_D = hoehe_D.get();
  return reading_D;
}

//Subroutine zur Ermittlung Höhe und Umrechnung in Prozent Füllung
void Berechnung(int h) {
  int Prozent = (h / 400.00) * 100; //Testrechnung muss an Tank angepasst werden. 400 mm als Gesamthöhe Tank zum Test
  Fuell = String(Prozent, DEC);
  Serial.println("Zur Kontrolle, Variablen h, Prozent und Fuell:");      //Zum Test ggf. auskommentieren
  Serial.println(h);                                                     //Zum Test ggf. auskommentieren
  Serial.println(Prozent);                                               //Zum Test ggf. auskommentieren
  Serial.println(Fuell);                                                 //Zum Test ggf. auskommentieren
  delay(500);                                                            //Zum Test ggf. auskommentieren

}

//Calculates the checksum for the NMEA String
int testsum(String strN) {
  int i;
  int XOR;
  int c;
  // Calculate testsum ignoring any $'s in the string
  for (XOR = 0, i = 0; i < 80; i++) {                                    // strlen(strN)
    c = (unsigned char)strN[i];
    if (c == '*') break;
    if (c != '$') XOR ^= c;
  }
  return XOR;
}

//Create NMEA String XDR
String NMEA_XDR(String Val) {
  String nmea = "$IIXDR,V,";
  nmea += Val, DEC;
  nmea += ",P,FUEL*"; //FUEL für Treibstoff, Anpassen um weitere Tanktypen zu erfassen, siehe Dokumentation EngineDashboard-Plugin OpenCPN
  nmea += String (testsum(nmea), HEX);
  //nmea += '\r';
  //nmea += '\n';
  return nmea;
}

//Subroutine zur Erstellung Datensatz zum Senden per UDP
void Data(String n) {
  XDR1 = n;
}

//########################################################################

void setup() {

  Serial.begin(115200);
  Serial.println();

  sensorSerial.begin(9600);                         // Sensor transmits its data at 9600 bps.
  sensor.begin();                                   // Initialise the sensor library.

  //reset settings - zum Testen
  //wifiManager.resetSettings();

  //Timeout in sek., nach Ablauf wird die Setup-Seite ausgeschaltet
  wifiManager.setTimeout(120);


  //Automatische Startseite und nach Timeout (wifimanager.setTimeout) erfolgt reset
  if (!wifiManager.autoConnect("SSID", "Password")) {  //Gebe eine SSID vor sowie ein Password. Password muss mindestens 7 Zeichen lang sein
    Serial.println("failed to connect, shut down WiFi-Modem for 3 Minutes then reset Wemos");
    //Ausschalten WiFi-Modem
    WiFi.forceSleepBegin();
    delay(1);
    delay(180000);             //Warte 3 Minuten, in dieser Zeit ist das WiFi-Modul abgeschaltet
    ESP.reset();
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");
  Serial.println("local ip");
  Serial.println(WiFi.localIP());
}

//########################################################################

void loop() {

  hoehe = Sensor();  //Aufruf Subroutine Sensor für Sensorwerte
  //Serial.println("Variable hoehe:");     //Zum Test ggf. auskommentieren
  //Serial.println(hoehe);                 //Zum Test ggf. auskommentieren
  //delay (500);                           //Zum Test ggf. auskommentieren
  Berechnung(hoehe); //Sprung Unterroutine Umrechnung Höhe in % Füllgrad Tank

  //Überprüfe ob Verbindung zum Netzwerk steht oder starte Setup-Seite
  if (!wifiManager.autoConnect("SSID", "Password")) {   //Gebe eine SSID vor sowie ein Password. Password muss mindestens 7 Zeichen lang sein
    Serial.println("WiFi lost, reset Wemos");
    delay(3000);
    ESP.reset();
  }

  //Setze Broadcastadresse
  IPAddress broadCast = WiFi.localIP();
  broadCast[3] = 255;

  //Erstelle Datenstring zum Senden
  Data(NMEA_XDR(Fuell));
  // Wandle den String fürs Senden um
  String str = XDR1;
  //Length (with one extra character for the null terminator)
  int str_len = str.length() + 1;
  // Prepare the character array (the buffer)
  char XDR[str_len];
  // Copy it over
  str.toCharArray(XDR, str_len);
  delay(50);

  //Sendeschleife Sende vier Pakete
  for (int i = 0; i < 4; i++) {
    Udp.beginPacket(broadCast, portBroadcast); // send UDP to Port 50000 and BroadcastIP
    Udp.write(XDR);
    Udp.endPacket();
  }
  delay(100);
}