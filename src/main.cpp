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
char buf_t1[4];
char buf_t2[4];
char buf_t3[4];
char buf_t4[4];
String t1, t2, t3, t4;

byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xaa;     // address of this device
byte destination = 0xff;      // destination to send to
long lastDiscoverTime = 0;    // last send time
long lastOfferTime = 0;       // last send time
long lastAnalogReadTime = 0;
long lastTestTime = 0;

int counter_tallys = 0;
int gpioP1 = 12, gpioP2 = 13, gpioP3 = 14, gpioP4 = 15;
int gpioV1, gpioV2, gpioV3, gpioV4;
float gpioV1Map, gpioV2Map, gpioV3Map, gpioV4Map;

bool gpioC1 = HIGH;
bool gpioC2 = HIGH;
bool tally1 = LOW; 
bool tally2 = LOW; 
bool tally3 = LOW; 
bool tally4 = LOW; 

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

void onReceive(int packetSize, String *ptr_rx_adr, String *ptr_tx_adr, String *ptr_incoming, String *ptr_rssi, String *ptr_snr) {
  if (packetSize == 0) return;          // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  int sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  String incoming = "";

  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  if (incomingLength != incoming.length()) {   // check length for error
    Serial.println("error: message length does not match length");
    return;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xff) {
    Serial.println("This message is not for me.");
    return;                             // skip rest of function
  }

  // if message is for this device, or broadcast, print details:
  /*Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Message length: " + String(incomingLength));
  Serial.println("Message: " + incoming);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();*/

  *ptr_rx_adr = String(recipient, HEX);
  *ptr_tx_adr = String(sender, HEX);
  *ptr_incoming = incoming;
  *ptr_rssi = String(LoRa.packetRssi());
  *ptr_snr = String(LoRa.packetSnr());

  return;

}

//////////////////////////////////////////////////////////////////////

void printDisplay(String sm, String rm) {

  sprintf(buf_sm, "%s", sm);
  sprintf(buf_rm, "%s", rm);
  sprintf(buf_t1, "%s", t1);
  sprintf(buf_t2, "%s", t2);
  sprintf(buf_t3, "%s", t3);
  sprintf(buf_t4, "%s", t4);

  u8g2.clearBuffer();					                    // clear the internal memory
  u8g2.setFont(u8g2_font_6x13_tf);
  u8g2.drawStr(0,10,"tallyWAN_transmitter");	    // write something to the internal memory
  u8g2.drawStr(0,25,"TxD:");
  u8g2.drawStr(35,25,buf_sm);
  u8g2.drawStr(0,40,"RxD:");
  u8g2.drawStr(35,40,buf_rm);
  u8g2.drawStr(0,55,buf_t1);
  u8g2.drawStr(10,55,buf_t2);
  u8g2.drawStr(20,55,buf_t3);
  u8g2.drawStr(30,55,buf_t4);
  u8g2.sendBuffer();

}

//////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);                   // initialize serial
  while (!Serial);

  mode = "discover";

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
  pinMode(gpioP1, INPUT_PULLDOWN);
  pinMode(gpioP2, INPUT_PULLDOWN);
  pinMode(gpioP3, INPUT_PULLDOWN);
  pinMode(gpioP4, INPUT_PULLDOWN);

}

//////////////////////////////////////////////////////////////////////

void loop() {

  if (millis() - lastTestTime > 250) {
      Serial.print("mode: "); Serial.println(mode);
      Serial.print("counter_tallys: "); Serial.println(counter_tallys);
      lastTestTime = millis();
    }
  
  //Discover Mode
  if ((mode == "discover")) {
    digitalWrite(LED_PIN_INTERNAL, HIGH);
    String message = "dis-anyrec?";         //Send a message
    sendMessage(message);
    printDisplay(message, "");
    Serial.println("TxD: " + message);
    digitalWrite(LED_PIN_INTERNAL, LOW);
    lastOfferTime = millis();
    mode = "offer";
  }

  //Offer Mode
  while (mode == "offer") {
    String rx_adr, tx_adr, incoming, rssi, snr;
    onReceive(LoRa.parsePacket(), &rx_adr, &tx_adr, &incoming, &rssi, &snr);    //Parse Packets and Read it
    printDisplay("", incoming);
    //Serial.print("rx_adr: "); Serial.println((String)rx_adr);
    //Serial.print("tx_adr: "); Serial.println((String)tx_adr);
    //Serial.print("RxD: "); Serial.println((String)incoming);
    //Serial.print("rssi: "); Serial.println((String)rssi);
    //Serial.print("snr: "); Serial.println((String)snr);*/
    
    if (incoming == "off-tally1") {
      tally1 = HIGH;
      t1 = "T1";
      counter_tallys++;
      lastOfferTime = millis();
    }
    if (incoming == "off-tally2") {
      tally2 = HIGH;
      t2 = "T2";
      counter_tallys++;
      lastOfferTime = millis();
    }
    if (incoming == "off-tally3") {
      tally3 = HIGH;
      t3 = "T3";
      counter_tallys++;
      lastOfferTime = millis();
    }
    if (incoming == "off-tally4") {
      tally4 = HIGH;
      t4 = "T4";
      counter_tallys++;
      lastOfferTime = millis();
    }
    
    if (millis() - lastOfferTime > 10000) {
      if (counter_tallys >= 1) {
        mode = "request";
        break;
      }
      if (counter_tallys == 0) {
        mode = "discover"; 
        break;
      }
    }
  }
  
  //Request Mode
  if ((mode == "request") && (millis() - lastAnalogReadTime > 250)) {

    gpioV1 = analogRead(gpioP1);
    gpioV2 = analogRead(gpioP2);
    gpioV3 = analogRead(gpioP3);
    gpioV4 = analogRead(gpioP4);
    gpioV1Map = gpioV1 * (3.3 / 4095.0);
    gpioV2Map = gpioV2 * (3.3 / 4095.0);
    gpioV3Map = gpioV3 * (3.3 / 4095.0);
    gpioV4Map = gpioV4 * (3.3 / 4095.0);

    Serial.print("gpioV2: "); Serial.println(gpioV2);
    Serial.print("gpioV2Map: "); Serial.print(gpioV2Map); Serial.println(" V");

    if (gpioV1Map > 2.0 && tally1 == HIGH && gpioC1 == HIGH) {
      digitalWrite(LED_PIN_INTERNAL, HIGH);
      String message = "req-tally1on";         //Send a message
      sendMessage(message);
      printDisplay(message, "");
      Serial.println("TxD: " + message);
      gpioC1 = !gpioC1;
      digitalWrite(LED_PIN_INTERNAL, LOW);
    }

    if (gpioV1Map < 2.0 && tally1 == HIGH && gpioC1 == LOW) {
      digitalWrite(LED_PIN_INTERNAL, HIGH);
      String message = "req-tally1off";         //Send a message
      sendMessage(message);
      printDisplay(message, "");
      Serial.println("TxD: " + message);
      gpioC1 = !gpioC1;
      digitalWrite(LED_PIN_INTERNAL, LOW);
    }

    if (gpioV1Map > 2.0 && tally2 == HIGH && gpioC2 == HIGH) {
      digitalWrite(LED_PIN_INTERNAL, HIGH);
      String message = "req-tally2on";         //Send a message
      sendMessage(message);
      printDisplay(message, "");
      Serial.println("TxD: " + message);
      gpioC2 = !gpioC2;
      digitalWrite(LED_PIN_INTERNAL, LOW);
    }

    if (gpioV1Map < 2.0 && tally2 == HIGH && gpioC2 == LOW) {
      digitalWrite(LED_PIN_INTERNAL, HIGH);
      String message = "req-tally2off";         //Send a message
      sendMessage(message);
      printDisplay(message, "");
      Serial.println("TxD: " + message);
      gpioC2 = !gpioC2;
      digitalWrite(LED_PIN_INTERNAL, LOW);
    }
    lastAnalogReadTime = millis();
    mode = "request";
  }

  //Acknowledge Mode
  if ((mode == "acknowledge")) {
  
  }

  //Control Mode
  if (millis() - lastDiscoverTime > 60000) {
    
  }

}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////