/*  This is an example sketch to test the core functionalities of SIMCom-based cellular modules.
    This code supports the SIM7000-series modules (LTE CAT-M/NB-IoT shields) for low-power IoT devices!

    Note: this code is specifically meant for AVR microcontrollers (Arduino Uno, Mega, Leonardo, etc)
    However, if you are using an ESP8266 please make the minor modifications mentioned below in the
    comments for the pin definitions and software serial initialization.

    For ESP32 please use the ESP32_LTE_Demo instead: https://github.com/botletics/SIM7000-LTE-Shield/blob/master/Code/examples/ESP32_LTE_Demo/ESP32_LTE_Demo.ino

    Author: Timothy Woo (www.botletics.com)
    Github: https://github.com/botletics/SIM7000-LTE-Shield
    Last Updated: 11/22/2022
    License: GNU GPL v3.0
*/

#include "BotleticsSIM7000.h" // https://github.com/botletics/Botletics-SIM7000/tree/main/src

// For botletics SIM7000 shield
#define PWRKEY 6
#define RST 7
#define TX 10 // Microcontroller RX
#define RX 11 // Microcontroller TX

// We default to using software serial. If you want to use hardware serial
// (because softserial isnt supported) comment out the following three lines 
// and uncomment the HardwareSerial line
#include <SoftwareSerial.h>
SoftwareSerial modemSS = SoftwareSerial(TX, RX);

// Use the following line for ESP8266 instead of the line above (comment out the one above)
//SoftwareSerial modemSS = SoftwareSerial(TX, RX, false, 256); // TX, RX, inverted logic, buffer size

SoftwareSerial *modemSerial = &modemSS;

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
uint8_t type;
char setupString[128] = {0};
char *setupStrPtr = setupString;

bool setupComplete = false;

void setup() {
  //  while (!Serial);

  pinMode(RST, OUTPUT);
  digitalWrite(RST, HIGH); // Default state

  pinMode(PWRKEY, OUTPUT);
  digitalWrite(PWRKEY, HIGH);
  delay(100);
  digitalWrite(PWRKEY, LOW);
  delay(100);
  digitalWrite(PWRKEY, HIGH);

  delay(2000);

  Serial.begin(9600);
  Serial.println(F("ARD=INIT"));

  // SIM7000 takes about 3s to turn on and SIM7500 takes about 15s
  // Press Arduino reset button if the module is still turning on and the board doesn't find it.
  // When the module is on it should communicate right after pressing reset

  // Software serial:
  modemSS.begin(115200); // Default SIM7000 shield baud rate

  Serial.println(F("ARD=SETBAUD"));
  modemSS.println("AT+IPR=9600"); // Set baud rate
  delay(100); // Short pause to let the command run
  modemSS.begin(9600);
  // if (! modem.begin(modemSS)) {
  //   Serial.println(F("ARD=SETBAUD_FAIL"));
  //   while (1); // Don't proceed if it couldn't find the device
  // }

  while (!setupComplete) {

    if ((millis() % (unsigned long)3000) == 0) {
      Serial.print(F("ARD=ALIVE\r"));
      modemSS.println("AT");
    }

    if (modemSS.available()) {
      char c = modemSS.read();
      Serial.print(c);

      *(setupStrPtr++) = c;
      if (setupStrPtr >= &setupString[128]) {
        setupStrPtr = setupString;
      }

      if (c == '\n') {
        for (char *ptr = setupString; ptr <= &setupString[127]; ptr++) {
          if (*ptr == 'O' && *(ptr + 1) == 'K') {
            setupComplete = true;
          }
        }
      }
    }
  }

  Serial.print("ARD=READY\r");
}

void loop() {
  while (1) {
    while (Serial.available()) {
      delay(1);
      modemSS.write(Serial.read());
    }
    if (modemSS.available()) {
      Serial.write(modemSS.read());
    }
  }
}