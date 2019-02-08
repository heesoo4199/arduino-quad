#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7,8);

const byte address[6] = "00001";

void setup() {
  radio.begin();
  Serial.begin(9600);
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.stopListening();
}

void loop() {
  String str = "";
  str += analogRead(0);
  int str_len = str.length() + 1;
  char text[str_len];
  str.toCharArray(text, str_len);
  bool yes = radio.write(&text, sizeof(text));
  Serial.println(yes);
}
