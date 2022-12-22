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
char buf_bb[3];
char buf_cc[3];
char buf_dd[3];
char buf_ee[3];
char buf_name[9];
char buf_localAddress[5];
char buf_mode[9];
char buf_rxAdr[5];
char buf_txAdr[5];
char buf_bV[5];
char buf_bL[4];
char buf_counterTallys[4];

byte msgCount = 0;            // Count of outgoing messages
byte localAddress = 0xaa;     // Address of this device            
String string_localAddress = "aa";                                 
byte destination = 0xff;      // Destination to send to              
String string_destinationAddress = "ff";            
long lastDiscoverTimebb = 0;    // Last send time
long lastDiscoverTimecc = 0;    // Last send time
long lastDiscoverTimedd = 0;    // Last send time
long lastDiscoverTimeee = 0;    // Last send time
long lastOfferTime = 0;       // Last send time
long lastOfferTimeEnd = 0;
long lastControlTime = 0;
long lastAckTime = 0;      
long lastAckTimeEnd = 0;      
long lastAnalogReadTime = 0;
long lastGetBattery = 0;
long lastTestTime = 0;
long lastDisplayPrint = 0;

int counterSend = 0;
int counterSendMax = 3;
int counterTallys = 0;
int counterTallysNew = 0;
int gpioP1 = 12, gpioP2 = 13, gpioP3 = 14, gpioP4 = 15;
int gpioV1, gpioV2, gpioV3, gpioV4;
int gpioV1Map, gpioV2Map, gpioV3Map, gpioV4Map;
float gpioV1Cal, gpioV2Cal, gpioV3Cal, gpioV4Cal;

bool gpioC1 = HIGH;
bool gpioC2 = HIGH;
bool gpioC3 = HIGH;
bool gpioC4 = HIGH;
bool tally_bb = LOW; 
bool tally_cc = LOW; 
bool tally_dd = LOW; 
bool tally_ee = LOW; 
bool tally_bb_init = LOW; 
bool tally_cc_init = LOW; 
bool tally_dd_init = LOW; 
bool tally_ee_init = LOW;
bool initBattery = HIGH;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 22, /* data=*/ 21);   // ESP32 Thing, HW I2C with pin remapping

#define LED_PIN_INTERNAL    25
#define ADC_PIN             35
#define CONV_FACTOR        1.8
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
  sprintf(buf_bb, "%s", bb);
  sprintf(buf_cc, "%s", cc);
  sprintf(buf_dd, "%s", dd);
  sprintf(buf_ee, "%s", ee);
  sprintf(buf_name, "%s", name);
  sprintf(buf_localAddress, "%x", localAddress);    // byte
  sprintf(buf_mode, "%s", mode);                    // string
  sprintf(buf_rxAdr, "%x", destination);            // byte
  sprintf(buf_txAdr, "%s", txAdr);
  sprintf(buf_counterTallys, "%d", counterTallys);

  if ((millis() - lastGetBattery > 5000) || (initBattery == HIGH)) {
    snprintf(buf_bV, 5, "%f", BL.getBatteryVolts());
    snprintf(buf_bL, 4, "%d", BL.getBatteryChargeLevel());
    initBattery = LOW;
    lastGetBattery = millis();
  }

  u8g2.clearBuffer();					      // clear the internal memory
  u8g2.setFont(u8g2_font_6x13_tf);
  u8g2.drawStr(0,10,buf_name);	    // write something to the internal memory
  u8g2.drawStr(62,10,buf_bV);
  u8g2.drawStr(88,10,"V");
  u8g2.drawStr(100,10,buf_bL);
  u8g2.drawStr(121,10,"%");
  u8g2.drawStr(0,22,"Adr:");
  u8g2.drawStr(30,22,buf_localAddress);
  u8g2.drawStr(62,22,buf_mode);
  u8g2.drawStr(0,34,"TxD:");
  u8g2.drawStr(30,34,buf_tx);
  u8g2.drawStr(117,34,buf_rxAdr);
  u8g2.drawStr(0,46,"RxD:");
  u8g2.drawStr(30,46,buf_rx);
  u8g2.drawStr(117,46,buf_txAdr);
  u8g2.drawStr(0,58,"Syn:");
  u8g2.drawStr(30,58,buf_bb);
  u8g2.drawStr(55,58,buf_cc);
  u8g2.drawStr(80,58,buf_dd);
  u8g2.drawStr(105,58,buf_ee);
  u8g2.drawStr(121,58,buf_counterTallys);
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
  u8g2.clearBuffer();
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

  // Discover Mode
  if ((mode == "discover")) {
    digitalWrite(LED_PIN_INTERNAL, HIGH);
    destination = 0xff;
    String message = "dis-anyrec?";         // Send a message
    sendMessage(message);
    printDisplay(message, "", "");
    Serial.println("TxD: " + message);
    digitalWrite(LED_PIN_INTERNAL, LOW);
    lastOfferTime = millis();
    lastDiscoverTimebb = millis();
    lastDiscoverTimecc = millis();
    lastDiscoverTimedd = millis();
    lastDiscoverTimeee = millis();
    mode = "offer";
  }

  // Offer Mode
  while (mode == "offer") {
    String rx_adr, tx_adr, incoming, rssi, snr;
    onReceive(LoRa.parsePacket(), &rx_adr, &tx_adr, &incoming, &rssi, &snr);    // Parse Packets and Read it
    
    // Print Incoming and RxD und TxD Adress
    if ((incoming != "") || (millis() - lastTestTime > 1500)) {
      printDisplay("", incoming, tx_adr);
      lastTestTime = millis();
    }
    if (incoming != "") {
      Serial.println("RxD: " + incoming);
      Serial.print("RxD_Adr: "); Serial.print(rx_adr); Serial.print(" TxD_Adr: "); Serial.println(tx_adr);
    }

    if ((incoming == "off") && (tx_adr == "bb")) {
      tally_bb = HIGH;
      tally_bb_init = HIGH;
      bb = "bb";
      counterTallys++;
      lastOfferTime = millis();
      lastOfferTimeEnd = millis();
    }
    if ((incoming == "off") && (tx_adr == "cc")) {
      tally_cc = HIGH;
      tally_cc_init = HIGH;
      cc = "cc";
      counterTallys++;
      lastOfferTime = millis();
      lastOfferTimeEnd = millis();
    }
    if ((incoming == "off") && (tx_adr == "dd")) {
      tally_dd = HIGH;
      tally_dd_init = HIGH;
      dd = "dd";
      counterTallys++;
      lastOfferTime = millis();
      lastOfferTimeEnd = millis();
    }
    if ((incoming == "off") && (tx_adr == "ee")) {
      tally_ee = HIGH;
      tally_ee_init = HIGH;
      ee = "ee";
      counterTallys++;
      lastOfferTime = millis();
      lastOfferTimeEnd = millis();
    }

    if ((millis() - lastOfferTime > 7500) && ((counterTallys >= 0))) {      // every 7.5 s on discover message
      mode = "discover"; 
      break;
    }

    if ((millis() - lastOfferTimeEnd > 60000) && (counterTallys >= 1)) {    // after 60 s of no new receivers, request mode
      mode = "request"; 
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

    if (gpioV1Cal > 2.0 && tally_bb == HIGH && gpioC1 == HIGH) {
      digitalWrite(LED_PIN_INTERNAL, HIGH);
      destination = 0xbb;
      String message = "req-high";         // Send a message
      sendMessage(message);
      printDisplay(message, "", "");
      Serial.println("TxD: " + message);
      gpioC1 = !gpioC1;
      mode = "acknowledge";
      lastAckTime = millis();
      digitalWrite(LED_PIN_INTERNAL, LOW);
    }

    if (gpioV1Cal < 2.0 && tally_bb == HIGH && gpioC1 == LOW) {
      digitalWrite(LED_PIN_INTERNAL, HIGH);
      destination = 0xbb;
      String message = "req-low";         // Send a message
      sendMessage(message);
      printDisplay(message, "", "");
      Serial.println("TxD: " + message);
      gpioC1 = !gpioC1;
      mode = "acknowledge";
      lastAckTime = millis();
      digitalWrite(LED_PIN_INTERNAL, LOW);
    }

    if (gpioV2Cal > 2.0 && tally_cc == HIGH && gpioC2 == HIGH) {
      digitalWrite(LED_PIN_INTERNAL, HIGH);
      destination = 0xcc;
      String message = "req-high";         // Send a message
      sendMessage(message);
      printDisplay(message, "", "");
      Serial.println("TxD: " + message);
      gpioC2 = !gpioC2;
      mode = "acknowledge";
      lastAckTime = millis();
      digitalWrite(LED_PIN_INTERNAL, LOW);
    }

    if (gpioV2Cal < 2.0 && tally_cc == HIGH && gpioC2 == LOW) {
      digitalWrite(LED_PIN_INTERNAL, HIGH);
      destination = 0xcc;
      String message = "req-low";         // Send a message
      sendMessage(message);
      printDisplay(message, "", "");
      Serial.println("TxD: " + message);
      gpioC2 = !gpioC2;
      mode = "acknowledge";
      lastAckTime = millis();
      digitalWrite(LED_PIN_INTERNAL, LOW);
    }
    lastAnalogReadTime = millis();

    if (gpioV3Cal > 2.0 && tally_dd == HIGH && gpioC3 == HIGH) {
      digitalWrite(LED_PIN_INTERNAL, HIGH);
      destination = 0xdd;
      String message = "req-high";         // Send a message
      sendMessage(message);
      printDisplay(message, "", "");
      Serial.println("TxD: " + message);
      gpioC3 = !gpioC3;
      mode = "acknowledge";
      lastAckTime = millis();
      digitalWrite(LED_PIN_INTERNAL, LOW);
    }

    if (gpioV3Cal < 2.0 && tally_dd == HIGH && gpioC3 == LOW) {
      digitalWrite(LED_PIN_INTERNAL, HIGH);
      destination = 0xdd;
      String message = "req-low";         // Send a message
      sendMessage(message);
      printDisplay(message, "", "");
      Serial.println("TxD: " + message);
      gpioC3 = !gpioC3;
      mode = "acknowledge";
      lastAckTime = millis();
      digitalWrite(LED_PIN_INTERNAL, LOW);
    }

    if (gpioV4Cal > 2.0 && tally_ee == HIGH && gpioC4 == HIGH) {
      digitalWrite(LED_PIN_INTERNAL, HIGH);
      destination = 0xee;
      String message = "req-high";         // Send a message
      sendMessage(message);
      printDisplay(message, "", "");
      Serial.println("TxD: " + message);
      gpioC4 = !gpioC4;
      mode = "acknowledge";
      lastAckTime = millis();
      digitalWrite(LED_PIN_INTERNAL, LOW);
    }

    if (gpioV4Cal < 2.0 && tally_ee == HIGH && gpioC4 == LOW) {
      digitalWrite(LED_PIN_INTERNAL, HIGH);
      destination = 0xee;
      String message = "req-low";         // Send a message
      sendMessage(message);
      printDisplay(message, "", "");
      Serial.println("TxD: " + message);
      gpioC4 = !gpioC4;
      mode = "acknowledge";
      lastAckTime = millis();
      digitalWrite(LED_PIN_INTERNAL, LOW);
    }
    lastAnalogReadTime = millis();
  }

  // Acknowledge Mode
  while (mode == "acknowledge") {
    String rx_adr, tx_adr, incoming, rssi, snr;
    onReceive(LoRa.parsePacket(), &rx_adr, &tx_adr, &incoming, &rssi, &snr);    // Parse Packets and Read it
    
    // Print Incoming and RxD und TxD Adress
    if ((incoming != "") || (millis() - lastTestTime > 1500)) {
      printDisplay("", incoming, tx_adr);
      lastTestTime = millis();
    }
    if (incoming != "") {
      Serial.println("RxD: " + incoming);
      Serial.print("RxD_Adr: "); Serial.print(rx_adr); Serial.print(" TxD_Adr: "); Serial.println(tx_adr);
    }

    // Back to Request Mode
    if ((incoming == "ack") && ((tx_adr == "bb") || (tx_adr == "cc") || (tx_adr == "dd") ||  (tx_adr == "ee"))) {
      mode = "request";
      break;
    }

    // Toggel and Resend Message, if ACK not arrived after 2.5 secounds
    if ((millis() - lastAckTime > 2500) && (counterSend < counterSendMax)) {
      mode = "request";
      counterSend++;
      gpioC1 = !gpioC1;
      gpioC2 = !gpioC2;
      gpioC3 = !gpioC3;
      gpioC4 = !gpioC4;
      break;
    }

    // Aborting the routine after 3 failed trys
    if ((millis() - lastAckTime > 10000) && (counterSend == counterSendMax)) {
      mode = "request";
      counterSend = 0;
      break;
    }
    
  }

  // Control Mode BB after discover and 3 and 3.5 minutes 
  if ((millis() - lastDiscoverTimebb > 180000) && ((tally_bb == HIGH) || (tally_bb_init == HIGH))) {
    digitalWrite(LED_PIN_INTERNAL, HIGH);
    destination = 0xbb;
    printDisplay("", "", ""); 
    String message = "con-rec?";         // Send a message
    sendMessage(message);
    printDisplay(message, "", "");
    Serial.println("TxD: " + message);
    digitalWrite(LED_PIN_INTERNAL, LOW);
    lastDiscoverTimebb = millis();
    lastControlTime = millis();
    mode = "control";

    while (mode == "control") {
      String rx_adr, tx_adr, incoming, rssi, snr;
      onReceive(LoRa.parsePacket(), &rx_adr, &tx_adr, &incoming, &rssi, &snr);    // Parse Packets and Read it
      
      if ((incoming != "") || (millis() - lastTestTime > 1500)) {
        printDisplay("", incoming, tx_adr);
        lastTestTime = millis();
      }
      if (incoming != "") {
        Serial.println("RxD: " + incoming);
        Serial.print("RxD_Adr: "); Serial.print(rx_adr); Serial.print(" TxD_Adr: "); Serial.println(tx_adr);
      }

      if ((incoming == "con") && (tx_adr == "bb")) {
        tally_bb = HIGH;
        bb = "bb";
        mode = "request";
        break;
      }

      if (millis() - lastControlTime > 2500) {
        tally_bb = LOW;
        bb = "";
        counterTallys--;
        mode = "request"; 
        break;
      }
    }
  Serial.print("Tally: ");Serial.print(tally_bb);Serial.print(tally_cc);Serial.print(tally_dd);Serial.println(tally_ee);
  Serial.print("Tally Init: ");Serial.print(tally_bb_init);Serial.print(tally_cc_init);Serial.print(tally_dd_init);Serial.println(tally_ee_init);
  }
  
  // Control Mode CC after discover and 3 minutes 
  if ((millis() - lastDiscoverTimecc > 190000) && ((tally_cc == HIGH) || (tally_cc_init == HIGH))) {
    digitalWrite(LED_PIN_INTERNAL, HIGH);
    destination = 0xcc;
    printDisplay("", "", ""); 
    String message = "con-rec?";         // Send a message
    sendMessage(message);
    printDisplay(message, "", "");
    Serial.println("TxD: " + message);
    digitalWrite(LED_PIN_INTERNAL, LOW);
    lastDiscoverTimecc = millis();
    lastControlTime = millis();
    mode = "control";

    while (mode == "control") {
      String rx_adr, tx_adr, incoming, rssi, snr;
      onReceive(LoRa.parsePacket(), &rx_adr, &tx_adr, &incoming, &rssi, &snr);    // Parse Packets and Read it
      
      if ((incoming != "") || (millis() - lastTestTime > 1500)) {
        printDisplay("", incoming, tx_adr);
        lastTestTime = millis();
      }
      if (incoming != "") {
        Serial.println("RxD: " + incoming);
        Serial.print("RxD_Adr: "); Serial.print(rx_adr); Serial.print(" TxD_Adr: "); Serial.println(tx_adr);
      }
      
      if ((incoming == "con") && (tx_adr == "cc")) {
        tally_cc = HIGH;
        cc = "cc";
        mode = "request";
        break;
      }

      if (millis() - lastControlTime > 2500) {
        tally_cc = LOW;
        cc = "";
        counterTallys--;
        mode = "request"; 
        break;
      }
    }
  Serial.print("Tally: ");Serial.print(tally_bb);Serial.print(tally_cc);Serial.print(tally_dd);Serial.println(tally_ee);
  Serial.print("Tally Init: ");Serial.print(tally_bb_init);Serial.print(tally_cc_init);Serial.print(tally_dd_init);Serial.println(tally_ee_init);
  }

  // Control Mode DD after discover and 3 minutes 
  if ((millis() - lastDiscoverTimedd > 200000) && ((tally_dd == HIGH) || (tally_dd_init == HIGH))) {
    digitalWrite(LED_PIN_INTERNAL, HIGH);
    destination = 0xdd;
    printDisplay("", "", ""); 
    String message = "con-rec?";         // Send a message
    sendMessage(message);
    printDisplay(message, "", "");
    Serial.println("TxD: " + message);
    digitalWrite(LED_PIN_INTERNAL, LOW);
    lastDiscoverTimedd = millis();
    lastControlTime = millis();
    mode = "control";

    while (mode == "control") {
      String rx_adr, tx_adr, incoming, rssi, snr;
      onReceive(LoRa.parsePacket(), &rx_adr, &tx_adr, &incoming, &rssi, &snr);    // Parse Packets and Read it
      
      if ((incoming != "") || (millis() - lastTestTime > 1500)) {
        printDisplay("", incoming, tx_adr);
        lastTestTime = millis();
      }
      if (incoming != "") {
        Serial.println("RxD: " + incoming);
        Serial.print("RxD_Adr: "); Serial.print(rx_adr); Serial.print(" TxD_Adr: "); Serial.println(tx_adr);
      }
      
      if ((incoming == "con") && (tx_adr == "dd")) {
        tally_dd = HIGH;
        dd = "dd";
        mode = "request";
        break;
      }

      if (millis() - lastControlTime > 2500) {
        tally_dd = LOW;
        dd = "";
        counterTallys--;
        mode = "request"; 
        break;
      }
    }
  Serial.print("Tally: ");Serial.print(tally_bb);Serial.print(tally_cc);Serial.print(tally_dd);Serial.println(tally_ee);
  Serial.print("Tally Init: ");Serial.print(tally_bb_init);Serial.print(tally_cc_init);Serial.print(tally_dd_init);Serial.println(tally_ee_init);
  }

  // Control Mode EE after discover and 3 minutes 
  if ((millis() - lastDiscoverTimeee > 210000) && ((tally_ee == HIGH) || (tally_ee_init == HIGH))) {
    digitalWrite(LED_PIN_INTERNAL, HIGH);
    destination = 0xee;
    printDisplay("", "", ""); 
    String message = "con-rec?";         // Send a message
    sendMessage(message);
    printDisplay(message, "", "");
    Serial.println("TxD: " + message);
    digitalWrite(LED_PIN_INTERNAL, LOW);
    lastDiscoverTimeee = millis();
    lastControlTime = millis();
    mode = "control";

    while (mode == "control") {
      String rx_adr, tx_adr, incoming, rssi, snr;
      onReceive(LoRa.parsePacket(), &rx_adr, &tx_adr, &incoming, &rssi, &snr);    // Parse Packets and Read it
      
      if ((incoming != "") || (millis() - lastTestTime > 1500)) {
        printDisplay("", incoming, tx_adr);
        lastTestTime = millis();
      }
      if (incoming != "") {
        Serial.println("RxD: " + incoming);
        Serial.print("RxD_Adr: "); Serial.print(rx_adr); Serial.print(" TxD_Adr: "); Serial.println(tx_adr);
      }
      
      if ((incoming == "con") && (tx_adr == "ee")) {
        tally_ee = HIGH;
        ee = "ee";
        mode = "request";
        break;
      }

      if (millis() - lastControlTime > 2500) {
        tally_ee = LOW;
        ee = "";
        counterTallys--;
        mode = "request"; 
        break;
      }
    }
  Serial.print("Tally: ");Serial.print(tally_bb);Serial.print(tally_cc);Serial.print(tally_dd);Serial.println(tally_ee);
  Serial.print("Tally Init: ");Serial.print(tally_bb_init);Serial.print(tally_cc_init);Serial.print(tally_dd_init);Serial.println(tally_ee_init);
  }

  // Function Print Display if nothing work
  if (millis() - lastDisplayPrint > 10000) {
    printDisplay("", "", "");
    lastDisplayPrint = millis();
  }

}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////