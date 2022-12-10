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

int counter = 0;

const int csPin = 18;          // LoRa radio chip select
const int resetPin = 23;       // LoRa radio reset
const int irqPin = 26;         // change for your board; must be a hardware interrupt pin

bool clk_state;
bool last_clk_state;
bool high = 1;
bool low = 0;

unsigned long lastMillisDisplay = 0;
unsigned long lastMillisTakt_1 = 0;
unsigned long lastMillisTakt_2 = 0;

//U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 13, /* data=*/ 11, /* reset=*/ 8);
//U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the Display
//U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 16, /* data=*/ 17, /* reset=*/ U8X8_PIN_NONE);   // ESP32 Thing, pure SW emulated I2C
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 22, /* data=*/ 21);   // ESP32 Thing, HW I2C with pin remapping

#define LED_PIN_INTERNAL    25

//////////////////////////////////////////////////////////////////////


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial);

  lastMillisDisplay = millis();
  lastMillisTakt_1 = millis();
  lastMillisTakt_2 = millis();

  pinMode(LED_PIN_INTERNAL, OUTPUT);

  Serial.println("LoRa Sender");

  LoRa.setPins(csPin, resetPin, irqPin); // set CS, reset, IRQ pin

  LoRa.setTxPower(17);  //2-20 default 17
  LoRa.setSpreadingFactor(7);    //6-12 default 7
  LoRa.setSignalBandwidth(125E3);   //7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, 250E3, and 500E3 default 125E3
  LoRa.setCodingRate4(5);   //5-8 default 5
  LoRa.setPreambleLength(8);    //6-65535 default 8

  LoRa.begin(868E6);  //set Frequenz

  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed");
    while (1);
  }

  u8g2.begin();

}


//////////////////////////////////////////////////////////////////////


void loop() {

//CLOCK
  if (clk_state == 0 && millis() - lastMillisTakt_1 < 1000) {
    clk_state = 1;
  }
    
  if (clk_state == 1 && millis() - lastMillisTakt_1 >= 1000) {
    if(millis() - lastMillisTakt_1 <= 2000) {
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

  if (millis() - lastMillisTakt_1 >= 2000){
    lastMillisTakt_1 = millis();
  }

}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////