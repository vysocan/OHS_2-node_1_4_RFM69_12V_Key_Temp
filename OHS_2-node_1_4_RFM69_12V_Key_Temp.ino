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

// Node settings
#define VERSION         140
#define PING_DELAY      1200000 // In milliseconds, 20 minutes
#define SENSOR_DELAY    600000  // In milliseconds, 10 minutes
// Constants
#define REG_LEN         21   // Size of one conf. element, defined in gateway
#define NODE_NAME_SIZE  16   // As defined in gateway
#define KEY_DELAY_ARMED 400  // Send key delay, for other then disarmed mode, allows faster disarm. 
#define KEY_DELAY       1400 // Send key delay, for disarmed mode
// Pins
#define LED_GREEN       5    // iButton probe LED
#define LED_RED         4    // iButton probe LED
#define SPEAKER         7    // Speaker pin
// Radio
#define NETWORKID       100  // Do not change, defined in gateway
#define GATEWAYID       1    // Do not change gateway address
#define NODEID          2    // This is our address 
#define RADIO_REPEAT    5    // Repeat sending
#define FREQUENCY       RF69_868MHZ // Match this with the version of your gateway (others: RF69_433MHZ, RF69_915MHZ)
#define KEY             "ABCDABCDABCDABCD" // Has to be same 16 characters/bytes on all nodes, not more not less!
//#define ENABLE_ATC      // Comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI        -75
#ifdef ENABLE_ATC 
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

OneWire ds(6);          // Dallas reader on pin with 4k7 pull-up rezistor

// Global variables
uint8_t  addr[8];             // Dallas chip
uint8_t  key[8];              // Dallas chip
uint8_t  mode = 16;           // Disarmed mode on start
uint8_t  pos;
uint8_t  msg[REG_LEN+1];
uint8_t  radioLength;
int8_t   iButtonRead = 0;
uint16_t keyDelay = KEY_DELAY;
unsigned long previousMillis = 0;
unsigned long readerMillis = 0;
unsigned long tempMillis = 0;
unsigned long aliveMillis = 0;

// Notes and LEDs patterns
char *keyOK    = "G1,,G5,,g0,.";
char *keyNOK   = "R1,,R1,,r0,.";
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
/*
 * Registration
 */
void sendConf(){ 
  int8_t result;
  uint16_t count = 0;
  
  // Wait some time to avoid contention
  delay(NODEID * 1000);

  Serial.print(F("Conf:"));

  while (count < sizeof(conf.reg)) {
    msg[0] = 'R'; // Registration flag
    memcpy(&msg[1], &conf.reg[count], REG_LEN);    
    result = radio.sendWithRetry(GATEWAYID, msg, REG_LEN + 1, RADIO_REPEAT);
    Serial.print(F(" ")); Serial.print(result);
    count+=REG_LEN;
  }
  
  Serial.println(F("."));
  
  // Play tones to confirm at least last send
  if (result == 1) {
    tone(SPEAKER, notes[0]);  delay(100); tone(SPEAKER, notes[7]);  delay(100); noTone(SPEAKER);
  } else {
    tone(SPEAKER, notes[7]);  delay(100); tone(SPEAKER, notes[0]);  delay(100); noTone(SPEAKER);
  }
}
/*
 * Set defaults on first time
 */
void setDefault(){
  conf.version = VERSION;   // Change VERSION to take effect
  conf.reg[0+(REG_LEN*0)]  = 'K';       // Key
  conf.reg[1+(REG_LEN*0)]  = 'i';       // iButton
  conf.reg[2+(REG_LEN*0)]  = 0;         // Local address, must be odd number
  conf.reg[3+(REG_LEN*0)]  = B00000000; // Default setting
  conf.reg[4+(REG_LEN*0)]  = B00011111; // Default setting, group='not set', enabled
  memset(&conf.reg[5+(REG_LEN*0)], 0, NODE_NAME_SIZE);
  conf.reg[0+(REG_LEN*1)]  = 'S';       // Sensor
  conf.reg[1+(REG_LEN*1)]  = 'T';       // Temperature
  conf.reg[2+(REG_LEN*1)]  = 0;         // Local address
  conf.reg[3+(REG_LEN*1)]  = B00000000; // Default setting
  conf.reg[4+(REG_LEN*1)]  = B00011111; // Default setting, group='not set', enabled
  memset(&conf.reg[5+(REG_LEN*1)], 0, NODE_NAME_SIZE);
  strcpy(&conf.reg[5+(REG_LEN*1)], "Temperature"); // Set default name
}
/*
 * Process incoming radio data
 */
void checkRadio(){
  // Look for incomming transmissions
  if (radio.receiveDone()) {
    radioLength = radio.DATALEN; 
    if (radio.ACKRequested()) { 
      delay(5); // wait after receive, we need this delay or gateway will not see ACK!!!
      radio.sendACK();
      Serial.print(F("ACK:"));
    }
    //for (uint8_t ii=0; ii < radioLength; ii++){ Serial.print((char)radio.DATA[ii], HEX); Serial.print("-"); }; Serial.println(F("<"));
    if ((char)radio.DATA[0] == 'C') {
      Serial.print(F("C:")); Serial.println(radio.DATA[1]);
      // Commands from gateway
      switch (radio.DATA[1]) {
        case 1: sendConf(); break; // Request for registration
        case 10 ... 16 : // Auth. commands
          mode = radio.DATA[1];
          // Change keyDelay based on mode, armed or arming allows faster disarm.        
          if (mode == 16) keyDelay = KEY_DELAY;
          else keyDelay = KEY_DELAY_ARMED;
          break;
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
/*
 * Send float value to gateway 
 */
void sendValue(uint8_t element, float value) {
  u.fval = value; 
  msg[0] = conf.reg[(REG_LEN*element)];
  msg[1] = conf.reg[1+(REG_LEN*element)];
  msg[2] = conf.reg[2+(REG_LEN*element)];
  msg[3] = u.b[0]; msg[4] = u.b[1]; msg[5] = u.b[2]; msg[6] = u.b[3];
  // Send to GW 
  radio.sendWithRetry(GATEWAYID, msg, 7);
}
/*
 * setup
 */
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

  sendConf(); 

  previousMillis = millis();
  readerMillis   = millis();
  tempMillis     = millis();
  aliveMillis    = millis();
}
/*
 * main task
 */
void loop() {
  // Process radio data
  checkRadio();

  // Tone, leds and iButton
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
            /*
            // Print key
            Serial.print(F("Key: "));
            for (uint8_t j = 0; j < 8; j++) {
              Serial.print(addr[j], HEX);
              Serial.print(F(", "));
            }
            Serial.println();
            */
          }
          iButtonRead++;
          memcpy(&key[0], &addr[0], sizeof(key));
        }
        ds.reset_search(); 
      } 
    }
    // Try to send key after keyDelay based on mode
    if ((unsigned long)(millis() - readerMillis) > keyDelay){
      // We have at least one good key
      if (iButtonRead > 0) {
        msg[0] = conf.reg[0]; 
        msg[1] = conf.reg[1];
        // If iButton is held at reader or just touched
        if (iButtonRead > 4) msg[2] = conf.reg[2] + 1;  
        else                 msg[2] = conf.reg[2] + 0; 
        memcpy(&msg[3], &key[0], sizeof(key));
        // Send        ;
        if (radio.sendWithRetry(GATEWAYID, msg, 11)) {
          p = keyOK; // play
        } else {
          p = keyNOK; // play
        }
        iButtonRead = -3; // Temporarily disable scanning 
        memset(&key[0], 0x0, sizeof(key)); // Clear the key
      } else {
        if (iButtonRead < 0) iButtonRead++; // Add up to 0, to enable scanning
      }
      readerMillis = millis();
    }
  } // Tone, leds and iButton

  // Temperature readings every SENSOR_DELAY
  if ((long)(millis() - tempMillis) >= SENSOR_DELAY) {
    tempMillis = millis();
    sendValue(1, ((((float)analogRead(A6) * 0.003223)-0.5)*100)); // Send it to GW
  }

  // Send alive packet every PING_DELAY
  if ((unsigned long)(millis() - aliveMillis) > PING_DELAY){
    aliveMillis = millis();
    msg[0] = 'C'; // Command
    msg[1] = 2;   // PING = 2
    // Send to GW 
    radio.sendWithRetry(GATEWAYID, msg, 2);
  }
  
}// End main loop
