// Remote node with RFM69, Power 12V, temperature(A6), iButton reader with 2 LEDs
// v.1.40

#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466

#include <OneWire.h>
#include <SPI.h>
#include <RFM69.h>
#include <RFM69_ATC.h>
#include <avr/eeprom.h> // Global configuration for in chip EEPROM
#define VERSION     140
// Pins
#define LED_GREEN   5    // iButton probe LED
#define LED_RED     4    // iButton probe LED
#define SPEAKER     7    // Speaker pin
// Radio
#define NETWORKID   100
#define GATEWAYID   1    // Do not change gateway address
#define NODEID      2    // This is our address 
#define FREQUENCY   RF69_868MHZ // Match this with the version of your gateway (others: RF69_433MHZ, RF69_915MHZ)
#define KEY         "ABCDABCDABCDABCD" // Has to be same 16 characters/bytes on all nodes, not more not less!
#define ENABLE_ATC  // Comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI    -75
#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif
// OHS
#define REG_LEN       21   // size of one conf. element
#define RADIO_REPEAT  6    // repeat sending

OneWire ds(6);          // Dallas reader on pin with 4k7 pull-up rezistor

// Global variables
uint8_t  addr[8];             // Dallas chip
uint8_t  key[8];              // Dallas chip
uint8_t  mode = 0;
uint8_t  pos;
uint8_t  msg[REG_LEN+1];
uint8_t  radioLength;
int8_t   iButtonRead = 0;
unsigned long previousMillis = 0;
unsigned long readerMillis = 0;
unsigned long tempMillis = 0;
unsigned long aliveMillis = 0;

// Notes and LEDs patterns
char *goodkey  = "G1,,G5,,g0,.";
char *wrongkey = "R1,,R1,,r0,.";
char *auth0    = "R5,r0,.";
char *auth1    = "R5,r0,,,,.";
char *auth2    = "R5,r0,,,,,,.";
char *auth3    = "R5,r0,,,,,,,,.";
char *p        = ".";
char *ok       = "G,g,,,,,,,,,.";
char *armed    = "R,r,,,,,,,,,.";
char *arming   = "R7,r5,R7,r0,,,,,,,,.";
char *iButton  = "G5,g0,,,,,.";
int notes[] = { NOTE_A3, NOTE_B3, NOTE_C4, NOTE_D4, NOTE_E4, NOTE_F4, NOTE_G4, NOTE_A4 };

// Configuration struct
struct config_t {
  uint16_t version;
  char     reg[REG_LEN * 2]; // Number of elements on this node
} conf; 

// Float conversion 
union u_tag {
  byte  b[4]; 
  float fval;
} u;

// Registration
void send_conf(){ 
  Serial.print(F("Conf"));
  delay(NODEID*100); // Wait some time to avoid contention
  pos = 0; 
  while (pos < sizeof(conf.reg)) {
    msg[0] = 'R'; // Registration flag
    for (uint8_t ii=0; ii < REG_LEN; ii++){ 
      msg[1+ii] = conf.reg[pos+ii];
    }
    Serial.print(F("-"));
    Serial.print(radio.sendWithRetry(GATEWAYID, msg, REG_LEN + 1, RADIO_REPEAT));
    pos+=REG_LEN;
  }
  Serial.println(F(" end"));
  tone(SPEAKER, notes[2]);  delay(100); noTone(SPEAKER); // Just beep as confirmation
}
// Set defaults on first time
void setDefault(){
  conf.version = VERSION;   // Change VERSION to take effect
  conf.reg[0]  = 'K';       // Key
  conf.reg[1]  = 'i';       // iButton
  conf.reg[2]  = 0;         // Local address, must be odd number
  conf.reg[3]  = B00000000; // Default setting
  conf.reg[4]  = B00011111; // Default setting, group=16, disabled
  for (uint8_t ii=0; ii < 17; ii++){ conf.reg[5+ii] = 0;} // Placeholder for name
  conf.reg[21] = 'S';       // Sensor
  conf.reg[22] = 'T';       // Temperature
  conf.reg[23] = 0;         // Local address
  conf.reg[24] = B00000000; // Default setting
  conf.reg[25] = B00011111; // Default setting, group=16, disabled
  for (uint8_t ii=0; ii < 17; ii++){ conf.reg[26+ii] = 0;}
}

void checkRadio(){
  // Look for incomming transmissions
  if (radio.receiveDone()) {
    radioLength = radio.DATALEN; 
    if (radio.ACKRequested()) { 
      delay(5); // wait after receive, we need this delay or gateway will not see ACK!!!
      radio.sendACK();
      Serial.print(F("ACK:"));
    }
    for (uint8_t ii=0; ii < radioLength; ii++){ Serial.print((char)radio.DATA[ii], HEX); Serial.print("-"); }; Serial.println(F("<"));
    if ((char)radio.DATA[0] == 'C') {
      Serial.print(F("C:")); Serial.println(radio.DATA[1],HEX);
      // Commands from gateway
      switch (radio.DATA[1]) {
        case 1: send_conf(); break; // Request for registration
        case 10 ... 16 : mode = radio.DATA[1]; // Auth. commands
        default: break;
      }
    }
    if ((char)radio.DATA[0] == 'R') { // Registration
      Serial.print(F("R:"));
      // Replace part of conf string with new paramters.
      pos = 0; 
      while (((conf.reg[pos] != radio.DATA[1]) || (conf.reg[pos+1] != radio.DATA[2]) ||
              (conf.reg[pos+2] != radio.DATA[3])) && (pos < sizeof(conf.reg))) {
        pos += REG_LEN; // size of one conf. element
      }
      if (pos < sizeof(conf.reg)) {
        Serial.println(pos);       
        memcpy(&conf.reg[pos], &radio.DATA[1], REG_LEN);
        // Save it to EEPROM
        conf.version = VERSION;
        eeprom_update_block((const void*)&conf, (void*)0, sizeof(conf)); // Save current configuration
        tone(SPEAKER, notes[pos/REG_LEN]);  delay(100); noTone(SPEAKER);               
      }
    }
  }
}

void setup() {
  // Set pins
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT); 
  pinMode(SPEAKER, OUTPUT);

  // RFM69
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  radio.setHighPower();  // uncomment only for RFM69HW!
  // radio.encrypt(KEY); // uncomment if you use encryption
  #ifdef ENABLE_ATC
    radio.enableAutoPower(ATC_RSSI);
  #endif
 
  eeprom_read_block((void*)&conf, (void*)0, sizeof(conf)); // Read current configuration
  if (conf.version != VERSION) setDefault();
   
  Serial.begin(115200); 

  delay(2000);
  send_conf(); 

  previousMillis = millis();
  readerMillis   = millis();
  tempMillis     = millis();
  aliveMillis    = millis() + 600000; // Do ping at start
  
  p = iButton; // Do beep at startup
}

void loop() {

  checkRadio();

  // Tone and leds
  if ((long)(millis() - previousMillis) >= 200) {
    previousMillis = millis();  
    if (*p == '.') {
      // reset all sound and LED
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_GREEN, LOW);
      noTone(SPEAKER);
      // change the mode
      switch (mode) {
        case 10: p = arming; break;
        case 11: p = auth0; break;
        case 12: p = auth1; break;
        case 13: p = auth2; break;
        case 14: p = auth3; break;         
        case 15: p = armed; break;
        default: p = ok; break; // Case 20
      }
    }
    while (*p != ',') {
      switch (*p) {
        case 'G': digitalWrite(LED_GREEN, HIGH); break;
        case 'g': digitalWrite(LED_GREEN, LOW); break;
        case 'R': digitalWrite(LED_RED, HIGH); break;
        case 'r': digitalWrite(LED_RED, LOW); break;
        case '1'...'9': tone(SPEAKER, notes[*p-49]); break;
        case '0': noTone(SPEAKER); break; 
        default: break;
      }
      p++; 
    }
    p++;

    // iButton
    if (iButtonRead >= 0) {
      if (!ds.search(addr)) {
        ds.reset_search();
      } else { // we have chip at reader
        // valid crc
        if ( OneWire::crc8(addr, 7) == addr[7]) {
          if (iButtonRead == 0) {
            p = iButton; // play
            readerMillis = millis();
          }
          iButtonRead++;
          memcpy(&key[0], &addr[0], sizeof(key));
        }
        ds.reset_search(); 
      } 
    }
    if ((unsigned long)(millis() - readerMillis) > 1400){
      // We have at least one good key
      if (iButtonRead > 0) {
        msg[0] = 'K'; 
        msg[1] = 'i'; 
        // If iButton is held at reader or just touched
        if (iButtonRead > 4) msg[2] = 1; 
        else                 msg[2] = 0; 
        memcpy(&msg[3], &key[0], sizeof(key));
        // Send        ;
        if (radio.sendWithRetry(GATEWAYID, msg, 11)) {
          p = goodkey; // play 
        }
        else {
          p = wrongkey; // play
        }
        iButtonRead = -1; // Temporarly disable scanning 
        memset(&key[0], 0x0, sizeof(key)); // Clear the key
      } else {
        iButtonRead = 0; // Enable scanning
      }
      readerMillis = millis();
    }
  } 

  // Sensors readings every 300 secodns
  if ((long)(millis() - tempMillis) >= 300000) {
    tempMillis = millis();
    // Temperature 
    u.fval = (((float)analogRead(A6) * 0.003223)-0.5)*100; 
    msg[0] = 'S'; // Sensor
    msg[1] = 'T'; // Temperature
    msg[2] = 0;   // local address
    msg[3] = u.b[0]; msg[4] = u.b[1]; msg[5] = u.b[2]; msg[6] = u.b[3];
    // Send to GW 
    radio.sendWithRetry(GATEWAYID, msg, 7);
  }

  // Send alive packet every 10 minutes
  if ((unsigned long)(millis() - aliveMillis) > 600000){
    aliveMillis = millis();
    msg[0] = 'C'; // Command
    msg[1] = 2;   // PING = 2
    // Send to GW 
    radio.sendWithRetry(GATEWAYID, msg, 2);
  }
}
