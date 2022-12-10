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

int count = 0;
int countDiscover = 0;
int interval = 2000;           // interval between sends

const int dis = 1;
const int off = 2;
const int req = 3;
const int ack = 4;
const int mode;

mode = dis;

const int csPin = 18;          // LoRa radio chip select
const int resetPin = 23;       // LoRa radio reset
const int irqPin = 26;         // change for your board; must be a hardware interrupt pin

String outgoing;               // outgoing message
String message;
String message2;
byte sender2;
byte recipient2;
byte msgCount = 0;             // count of outgoing messages
byte localAddress = 0xAA;      // address of this device
byte destination = 0xFF;       // destination to send to

char buf_la[4];
char buf_da[4];
char buf_mo[4];
char buf_ms[4];

bool clk_state;
bool last_clk_state;

unsigned long lastDisplayTime = 0;
unsigned long lastTaktTime1 = 0;
unsigned long lastTaktTime2 = 0;
unsigned long lastDiscoverTime = 0;        // last packed send time

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 22, /* data=*/ 21);   // ESP32 Thing, HW I2C with pin remapping

#define LED_PIN_INTERNAL    25

//////////////////////////////////////////////////////////////////////

void printDisplay(byte la, byte da, int mo, String ms) {

  sprintf(buf_la, "%c", la);
  sprintf(buf_da, "%c", da);
  sprintf(buf_mo, "%i", mo);
  sprintf(buf_ms, "%s", ms);

  u8g2.clearBuffer();					                    // clear the internal memory
  u8g2.setFont(u8g2_font_6x13_tf);	              // choose a suitable font
  u8g2.drawStr(0,10,"tallyWAN_transmitter");	    // write something to the internal memory
  u8g2.drawStr(0,30,"LA:");
  u8g2.drawStr(30,30,buf_la);
  u8g2.drawStr(60,30,"DA:");
  u8g2.drawStr(90,30,buf_da);
  u8g2.drawStr(0,60,"MODE:");
  u8g2.drawStr(30,60,buf_mo);
  u8g2.drawStr(0,90,"MES:");
  u8g2.drawStr(30,90,buf_ms);
  u8g2.sendBuffer();				                    	// transfer internal memory to the display
}

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

void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return

  // read packet header bytes:
  byte recipient = LoRa.read();         // recipient address
  byte sender = LoRa.read();            // sender address
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
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println("This message is not for me.");
    return;                             // skip rest of function
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

  message2 = incoming;
  sender2 = sender;
  recipient2 = recipient;
  
}

//////////////////////////////////////////////////////////////////////

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial);
  Serial.println("LoRa Sender");

  lastDisplayTime = millis();
  lastTaktTime1 = millis();
  lastTaktTime2 = millis();
  lastDiscoverTime = millis();

  pinMode(LED_PIN_INTERNAL, OUTPUT);

  LoRa.setPins(csPin, resetPin, irqPin); // set CS, reset, IRQ pin
  LoRa.setTxPower(17);  //2-20 default 17
  LoRa.setSpreadingFactor(7);    //6-12 default 7
  LoRa.setSignalBandwidth(125E3);   //7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, 250E3, and 500E3 default 125E3
  LoRa.setCodingRate4(5);   //5-8 default 5
  LoRa.setPreambleLength(8);    //6-65535 default 8
  LoRa.begin(868E6);  //set Frequenz 915E6 or 868E6

  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed");
    while (1);
  }

  u8g2.begin();

}


//////////////////////////////////////////////////////////////////////

void loop() {

  switch(mode)
  {
    case dis:

      if (millis() - lastDiscoverTime > interval) {
          message = "Any Receiver?";       // send a message
          sendMessage(message);
          printDisplay(localAddress, destination, mode, message);
          Serial.println("Sending " + message);
          
          lastDiscoverTime = millis();            // timestamp the message
          countDiscover++;
        }
      if (countDiscover == 3) {
          countDiscover = 0;
          mode = off;
          break;
      }

    case off:

      onReceive(LoRa.parsePacket());
      printDisplay(sender2, recipient2, mode, message);
      break;

  }


}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

/*

//CLOCK
  if (clk_state == 0 && millis() - lastTaktTime1 < 1000) {
    clk_state = 1;
  }
    
  if (clk_state == 1 && millis() - lastTaktTime1 >= 1000) {
    if(millis() - lastTaktTime1 <= 2000) {
    clk_state = 0;
    }
  }
    
  if (last_clk_state != clk_state) {   
    last_clk_state = clk_state;
    if (clk_state == 1) {

      char buf_c[8];

      itoa(counter, buf_c, 10);
      String message_status = ("REQ/A/0/B/");
      String message = message_status + buf_c;

      Serial.print("Sending packet: ");
      Serial.println(message);

      // send lora packet
      LoRa.beginPacket();
      LoRa.print(message);    //LoRa.write(byte) //255 bytes maximum //LoRa.write(buffer, length);
      LoRa.endPacket();

      // convert message for display
      char buf[16];
      sprintf(buf, "%s%s", message_status, buf_c);

      // print message on the display
      u8g2.clearBuffer();					// clear the internal memory
      u8g2.setFont(u8g2_font_6x13_tf);	// choose a suitable font
      u8g2.drawStr(0,10,"LoRa Sender");	// write something to the internal memory
      u8g2.drawStr(0,30,buf);
      u8g2.sendBuffer();					// transfer internal memory to the display

      digitalWrite(LED_PIN_INTERNAL, clk_state);

      counter++;
    }

    if (clk_state == 0) {
      digitalWrite(LED_PIN_INTERNAL, clk_state);
    }
  }

  if (millis() - lastTaktTime1 >= 2000){
    lastTaktTime1 = millis();
  }

*/

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////