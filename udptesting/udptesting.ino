//Author of source code: Martin Chlebovec (martinius96@gmail.com)
//Project's website: https://arduino.clanweb.eu/udp-control-esp32.php?lang=en
//Buy me coffee: https://paypal.me/chlebovec

#include "WiFi.h"
#include "AsyncUDP.h"
const char* ssid = "B535_38E9";
const char* pass = "AnTb97TNnDR";
const int rele = 23;
AsyncUDP udp;
uint8_t number_of_motor = 0;
int number_of_steps = 0;
void process_new_order(uint8_t *, int){}

void setup()
{
  Serial.begin(115200);
  pinMode(rele, OUTPUT);
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  if (udp.listen(1234)) {
    Serial.print("UDP Listening on IP: ");
    Serial.println(WiFi.localIP());
    udp.onPacket([](AsyncUDPPacket packet) {
      Serial.print("UDP Packet Type: ");
      Serial.print(packet.isBroadcast() ? "Broadcast" : packet.isMulticast() ? "Multicast" : "Unicast");
      Serial.print(", From: ");
      Serial.print(packet.remoteIP());
      Serial.print(":");
      Serial.print(packet.remotePort());
      Serial.print(", To: ");
      Serial.print(packet.localIP());
      Serial.print(":");
      Serial.print(packet.localPort());
      Serial.print(", Length: ");
      Serial.print(packet.length()); //dlzka packetu
      Serial.print(", Data: ");
      Serial.write(packet.data(), packet.length());
      Serial.println();
      String myString = (const char*)packet.data();
      int ind1; // , locations
      int ind2;
      ind1 = myString.indexOf(',');  //finds location of first ,
      number_of_motor = myString.substring(0, ind1).toInt();   //captures first data String
      ind2 = myString.indexOf(':', ind1+1 );   //finds location of second ,
      number_of_steps = myString.substring(ind1+1, ind2+1).toInt();
      Serial.println(number_of_motor);
      Serial.println(number_of_steps);
      packet.printf("Got %u bytes of data", packet.length());
    });
  }
}

void loop()
{
//  delay(1000);
//  udp.broadcast("Anyone here?");
}
