//////////////////////////////////////////////////////////////////////
//////////////////// TallyWAN by Martin Mittrenga ////////////////////
//////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <U8g2lib.h>
#include <LoRa.h>
#include <Adafruit_NeoPixel.h>
#include <Pangodream_18650_CL.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

const int csPin = 18;          // LoRa radio chip select
const int resetPin = 23;       // LoRa radio reset
const int irqPin = 26;         // Change for your board; must be a hardware interrupt pin

String outgoing;               // Outgoing message
String mode = "discover";
String name = "TallyWAN";      // Device Name
String bb, cc, dd, ee;

char buf_tx[12];
char buf_rx[12];
char buf_0xbb[8];
char buf_0xcc[8];
char buf_0xdd[8];
char buf_0xee[8];
char buf_name[12];
char buf_localAddress[4];
char buf_mode[8];
char buf_rxAdr[4];
char buf_txAdr[4];

byte msgCount = 0;            // Count of outgoing messages
byte localAddress = 0xaa;     // Address of this device
byte destination = 0xff;      // Destination to send to
long lastDiscoverTime = 0;    // Last send time
long lastOfferTime = 0;       // Last send time
long lastOfferTimeEnd = 0;
long lastAnalogReadTime = 0;
long lastTestTime = 0;

int counterTallys = 0;
int gpioP1 = 12, gpioP2 = 13, gpioP3 = 14, gpioP4 = 15;
int gpioV1, gpioV2, gpioV3, gpioV4;
int gpioV1Map, gpioV2Map, gpioV3Map, gpioV4Map;
float gpioV1Cal, gpioV2Cal, gpioV3Cal, gpioV4Cal;

bool gpioC1 = HIGH;
bool gpioC2 = HIGH;
bool tally_0xbb = LOW; 
bool tally_0xcc = LOW; 
bool tally_0xdd = LOW; 
bool tally_0xee = LOW; 

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 22, /* data=*/ 21);   // ESP32 Thing, HW I2C with pin remapping

#define LED_PIN_INTERNAL    25
#define ADC_PIN             35
#define CONV_FACTOR        1.7
#define READS               20

Pangodream_18650_CL BL(ADC_PIN, CONV_FACTOR, READS);

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
  int sender = LoRa.read();             // sender address
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

  *ptr_rx_adr = String(recipient, HEX);
  *ptr_tx_adr = String(sender, HEX);
  *ptr_incoming = incoming;
  *ptr_rssi = String(LoRa.packetRssi());
  *ptr_snr = String(LoRa.packetSnr());

  return;

}

//////////////////////////////////////////////////////////////////////

void printDisplay(String tx, String rx, String txAdr) {   //tx Transmit Message,  rx Receive Message,   txAdr Receive Address

  sprintf(buf_tx, "%s", tx);
  sprintf(buf_rx, "%s", rx);
  sprintf(buf_0xbb, "%s", bb);
  sprintf(buf_0xcc, "%s", cc);
  sprintf(buf_0xdd, "%s", dd);
  sprintf(buf_0xee, "%s", ee);
  sprintf(buf_name, "%s", name);
  sprintf(buf_localAddress, "%x", localAddress);    // byte
  sprintf(buf_mode, "%s", mode);                    // string
  sprintf(buf_rxAdr, "%x", destination);            // byte
  sprintf(buf_txAdr, "%s", txAdr);

  u8g2.clearBuffer();					      // clear the internal memory
  u8g2.setFont(u8g2_font_6x13_tf);
  u8g2.drawStr(0,10,buf_name);	    // write something to the internal memory
  u8g2.drawStr(0,22,"Adr:");
  u8g2.drawStr(30,22,"0x");
  u8g2.drawStr(42,22,buf_localAddress);
  u8g2.drawStr(62,22,buf_mode);
  u8g2.drawStr(0,34,"TxD:");
  u8g2.drawStr(30,34,buf_tx);
  u8g2.drawStr(100,34,"0x");
  u8g2.drawStr(112,34,buf_rxAdr);
  u8g2.drawStr(0,46,"RxD:");
  u8g2.drawStr(30,46,buf_rx);
  u8g2.drawStr(100,46,"0x");
  u8g2.drawStr(112,46,buf_txAdr);
  u8g2.drawStr(0,58,"Syn:");
  u8g2.drawStr(30,58,buf_0xbb);
  u8g2.drawStr(55,58,buf_0xcc);
  u8g2.drawStr(80,58,buf_0xdd);
  u8g2.drawStr(105,58,buf_0xee);
  u8g2.sendBuffer();

}

//////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);                   // initialize serial
  while (!Serial);

  Serial.println("");
  Serial.println(name);

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
  printDisplay("", "", "");

  Serial.println("OLED init succeeded.");

  pinMode(LED_PIN_INTERNAL, OUTPUT);
  pinMode(gpioP1, INPUT_PULLDOWN);
  pinMode(gpioP2, INPUT_PULLDOWN);
  pinMode(gpioP3, INPUT_PULLDOWN);
  pinMode(gpioP4, INPUT_PULLDOWN);

}

//////////////////////////////////////////////////////////////////////

void loop() {

  
  if (millis() - lastTestTime > 2000) {
    Serial.print("Value from pin: ");
    Serial.println(analogRead(34));
    Serial.print("Average value from pin: ");
    Serial.println(BL.pinRead());
    Serial.print("Volts: ");
    Serial.println(BL.getBatteryVolts());
    Serial.print("Charge level: ");
    Serial.println(BL.getBatteryChargeLevel());
    Serial.println("");
    lastTestTime = millis();
    }
  

  // Discover Mode
  if ((mode == "discover")) {
    digitalWrite(LED_PIN_INTERNAL, HIGH);
    String message = "dis-anyrec?";         // Send a message
    sendMessage(message);
    printDisplay(message, "", "");
    Serial.println("TxD: " + message);
    digitalWrite(LED_PIN_INTERNAL, LOW);
    lastOfferTime = millis();
    mode = "offer";
  }

  // Offer Mode
  while (mode == "offer") {
    String rx_adr, tx_adr, incoming, rssi, snr;
    onReceive(LoRa.parsePacket(), &rx_adr, &tx_adr, &incoming, &rssi, &snr);    // Parse Packets and Read it
    printDisplay("", incoming, (String)tx_adr);

    if (millis() - lastTestTime > 2000) {
    Serial.print("mode: "); Serial.println(mode);
    Serial.print("counterTallys: "); Serial.println(counterTallys);
    Serial.print("tallys: "); Serial.print(tally_0xbb); Serial.print(tally_0xcc); Serial.print(tally_0xdd); Serial.println(tally_0xee); 
    lastTestTime = millis();
    }

    if (incoming == "off-0xbb") {
      tally_0xbb = HIGH;
      bb = "0xbb";
      counterTallys++;
      lastOfferTime = millis();
      lastOfferTimeEnd = millis();
    }
    if (incoming == "off-0xcc") {
      tally_0xcc = HIGH;
      cc = "0xcc";
      counterTallys++;
      lastOfferTime = millis();
      lastOfferTimeEnd = millis();
    }
    if (incoming == "off-0xdd") {
      tally_0xdd = HIGH;
      dd = "0xdd";
      counterTallys++;
      lastOfferTime = millis();
      lastOfferTimeEnd = millis();
    }
    if (incoming == "off-0xee") {
      tally_0xee = HIGH;
      ee = "0xee";
      counterTallys++;
      lastOfferTime = millis();
      lastOfferTimeEnd = millis();
    }

    if ((millis() - lastOfferTime > 10000) && ((counterTallys >= 0))) {
      mode = "discover"; 
      break;
    }

    if ((millis() - lastOfferTimeEnd > 60000) && (counterTallys >= 1)) {
      mode = "request"; 
      printDisplay("", "", "");
      break;
    }
  }
  
  // Request Mode
  if ((mode == "request") && (millis() - lastAnalogReadTime > 250)) {

    gpioV1 = analogRead(gpioP1);
    gpioV2 = analogRead(gpioP2);
    gpioV3 = analogRead(gpioP3);
    gpioV4 = analogRead(gpioP4);
    gpioV1Map = map(gpioV1, 400, 4095, 0, 3695);
    gpioV2Map = map(gpioV2, 400, 4095, 0, 3695);
    gpioV3Map = map(gpioV3, 400, 4095, 0, 3695);
    gpioV4Map = map(gpioV4, 400, 4095, 0, 3695); 
    gpioV1Cal = gpioV1Map * (3.3 / 3695.0);
    gpioV2Cal = gpioV2Map * (3.3 / 3695.0);
    gpioV3Cal = gpioV3Map * (3.3 / 3695.0);
    gpioV4Cal = gpioV4Map * (3.3 / 3695.0);

    /*
    Serial.print("gpioV1: "); Serial.print(gpioV1); Serial.print(" gpioV1Cal: "); Serial.print(gpioV1Cal); Serial.println(" V");
    Serial.print("gpioV2: "); Serial.print(gpioV2); Serial.print(" gpioV2Cal: "); Serial.print(gpioV2Cal); Serial.println(" V");
    Serial.print("gpioV3: "); Serial.print(gpioV3); Serial.print(" gpioV3Cal: "); Serial.print(gpioV3Cal); Serial.println(" V");
    Serial.print("gpioV4: "); Serial.print(gpioV4); Serial.print(" gpioV4Cal: "); Serial.print(gpioV4Cal); Serial.println(" V");
    */
    if (gpioV1Cal > 2.0 && tally_0xbb == HIGH && gpioC1 == HIGH) {
      digitalWrite(LED_PIN_INTERNAL, HIGH);
      String message = "req-0xbb-high";         // Send a message
      sendMessage(message);
      printDisplay(message, "", "");
      Serial.println("TxD: " + message);
      gpioC1 = !gpioC1;
      digitalWrite(LED_PIN_INTERNAL, LOW);
    }

    if (gpioV1Cal < 2.0 && tally_0xbb == HIGH && gpioC1 == LOW) {
      digitalWrite(LED_PIN_INTERNAL, HIGH);
      String message = "req-0xbb-low";         // Send a message
      sendMessage(message);
      printDisplay(message, "", "");
      Serial.println("TxD: " + message);
      gpioC1 = !gpioC1;
      digitalWrite(LED_PIN_INTERNAL, LOW);
    }

    if (gpioV2Cal > 2.0 && tally_0xcc == HIGH && gpioC2 == HIGH) {
      digitalWrite(LED_PIN_INTERNAL, HIGH);
      String message = "req-0xcc-high";         // Send a message
      sendMessage(message);
      printDisplay(message, "", "");
      Serial.println("TxD: " + message);
      gpioC2 = !gpioC2;
      digitalWrite(LED_PIN_INTERNAL, LOW);
    }

    if (gpioV2Cal < 2.0 && tally_0xcc == HIGH && gpioC2 == LOW) {
      digitalWrite(LED_PIN_INTERNAL, HIGH);
      String message = "req-0xcc-low";         // Send a message
      sendMessage(message);
      printDisplay(message, "", "");
      Serial.println("TxD: " + message);
      gpioC2 = !gpioC2;
      digitalWrite(LED_PIN_INTERNAL, LOW);
    }
    lastAnalogReadTime = millis();
    mode = "request";
  }

  // Acknowledge Mode
  if ((mode == "acknowledge")) {
  
  }

  // Control Mode
  if (millis() - lastDiscoverTime > 60000) {
    
  }

}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////