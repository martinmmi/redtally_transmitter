//////////////////////////////////////////////////////////////////////
//////////////////// TallyWAN by Martin Mittrenga ////////////////////
//////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <U8g2lib.h>
#include <LoRa.h>
#include <Adafruit_NeoPixel.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

const int csPin = 18;          // LoRa radio chip select
const int resetPin = 23;       // LoRa radio reset
const int irqPin = 26;         // change for your board; must be a hardware interrupt pin

String outgoing;              // outgoing message
String mode;

char buf_sm[12];
char buf_rm[12];

byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xAA;     // address of this device
byte destination = 0xBB;      // destination to send to
long lastDiscoverTime = 0;    // last send time
long lastOfferTime = 0;    // last send time
int interval = 3000;          // interval between sends
int counterOffer = 0;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 22, /* data=*/ 21);   // ESP32 Thing, HW I2C with pin remapping

#define LED_PIN_INTERNAL    25

//////////////////////////////////////////////////////////////////////

void sendMessage(String outgoing) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
}

//////////////////////////////////////////////////////////////////////

String onReceive(int packetSize) {
  if (packetSize == 0) return "";          // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  String incoming = "";

  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  if (incomingLength != incoming.length()) {   // check length for error
    Serial.println("error: message length does not match length");
    return "";                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println("This message is not for me.");
    return "";                             // skip rest of function
  }

  // if message is for this device, or broadcast, print details:
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Message length: " + String(incomingLength));
  Serial.println("Message: " + incoming);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();

  return incoming;

}

//////////////////////////////////////////////////////////////////////

void printDisplay(String sm, String rm) {

  sprintf(buf_sm, "%s", sm);
  sprintf(buf_rm, "%s", rm);

  u8g2.clearBuffer();					                    // clear the internal memory
  u8g2.setFont(u8g2_font_6x13_tf);
  u8g2.drawStr(0,10,"tallyWAN_transmitter");	    // write something to the internal memory
  u8g2.drawStr(0,25,"TxD:");
  u8g2.drawStr(35,25,buf_sm);
  u8g2.drawStr(0,40,"RxD:");
  u8g2.drawStr(35,40,buf_rm);
  u8g2.sendBuffer();

}

//////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);                   // initialize serial
  while (!Serial);

  mode = "nothing";

  Serial.println("");
  Serial.println("tallyWAN_transmitter");

  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(csPin, resetPin, irqPin); // set CS, reset, IRQ pin
  LoRa.setTxPower(17);  //2-20 default 17
  LoRa.setSpreadingFactor(7);    //6-12 default 7
  LoRa.setSignalBandwidth(125E3);   //7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, 250E3, and 500E3 default 125E3
  LoRa.setCodingRate4(5);   //5-8 default 5
  LoRa.setPreambleLength(8);    //6-65535 default 8
  LoRa.begin(868E6);  //set Frequenz 915E6 or 868E6

  if (!LoRa.begin(868E6)) {             // initialize ratio at 868 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

  Serial.println("LoRa init succeeded.");

  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x13_tf);	
  u8g2.drawStr(0,10,"tallyWAN_transmitter");
  u8g2.drawStr(0,25,"TxD:");
  u8g2.drawStr(0,40,"RxD:");
  u8g2.sendBuffer();

  Serial.println("OLED init succeeded.");

  pinMode(LED_PIN_INTERNAL, OUTPUT);

}

//////////////////////////////////////////////////////////////////////

void loop() {

  if (mode == "nothing") {
    mode = "discover";
  }

  //Discover Mode
  if ((mode == "discover") && (millis() - lastDiscoverTime > interval)) {
    digitalWrite(LED_PIN_INTERNAL, HIGH);
    String message = "dis-anyrec?";   // send a message
    sendMessage(message);
    printDisplay(message, "");
    Serial.println("TxD: " + message);
    lastDiscoverTime = millis();            // timestamp the message
    digitalWrite(LED_PIN_INTERNAL, LOW);
    mode = "offer";
  }

  //Offer Mode
  if ((mode == "offer")) {
    // parse for a packet, and call onReceive with the result:
    String incoming = onReceive(LoRa.parsePacket());
    printDisplay("", incoming);

    if ((incoming == "off-tally1") || (millis() - lastOfferTime > 100000)) {
      mode = "discover";
      Serial.println("JO");
      counterOffer++;
    }

    if ((incoming == "off-tally1") && (counterOffer >= 3)) {
      digitalWrite(LED_PIN_INTERNAL, HIGH);
      counterOffer = 0;
      mode = "request";
    }

  }

  //Request Mode
  if ((mode == "request")) {
    Serial.println("Tally 0xBB angemeldet!");
  
  }

  //Acknowledge Mode
  if ((mode == "acknowledge")) {
  
  }

}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////