/*
  HeltecPager 1.2 - single-file group text pager for Heltec WiFi LoRa 32 V3.
  Flash this SAME sketch onto every device; Settings > Name personalizes it.

  ARDUINO SETUP
    Board package: esp32 by Espressif Systems
    Board: Heltec WiFi LoRa 32(V3) (ESP32-S3, not V2 or V4)
    USB CDC On Boot: Disabled if offered (onboard CP210x USB-UART).
    Core 2.0.17 sets this automatically for the V3; no menu change needed.
    Libraries: RadioLib, Adafruit SSD1306, Adafruit GFX Library
               (accept their dependencies, including Adafruit BusIO)
    Serial Monitor: 115200 baud, Newline

  VALIDATION
    Compiled with esp32 core 2.0.17, RadioLib 7.1.2, Adafruit SSD1306 2.5.17,
    Adafruit GFX Library 1.12.6, Adafruit BusIO 1.17.4.
    Packet parser passed roundtrip, boundary and 100,000 malformed-input tests
    under AddressSanitizer/UndefinedBehaviorSanitizer (leak check disabled).
    Not tested on physical hardware; confirm operation on two boards first.

  EXTERNAL WIRING - GPIO numbers, NOT header positions
    Up -> GPIO4       Down -> GPIO5       Left -> GPIO6
    Right -> GPIO7    Select -> GPIO38
    Each normally-open button connects its GPIO to GND when pressed.
    Internal pull-ups enabled. No button connects to 3.3V or 5V.
    GPIO39 -> 330-ohm resistor -> LED anode (+); LED cathode (-) -> GND.
    Onboard GPIO35 user LED mirrors unread status; orange charge LED untouched.
    Attach the matching antenna before use. Disconnect power while wiring.

  INTERNAL V3 CONNECTIONS (already wired on board)
    OLED: SDA17, SCL18, reset21; Vext36 active LOW
    SX1262: SCK9, MISO11, MOSI10, CS8, DIO1=14, reset12, BUSY13
    TCXO 1.8V, DIO2 RF switch control

  CONTROLS
    Menus: Up/Down selects, Select opens, Left returns.
    Keyboard: arrows move, Select types. '_'=space, '<'=delete, '>'=done.
    Hold Select 700ms to toggle upper/lower case in keyboard.
    Hold Left 700ms to leave keyboard (message draft retained in RAM).
    Send confirmation: Select sends; Left edits.
    Reader: Up/Down scroll; Left returns; Select opens Quick messages.
    Tap Right on Home to open newest unread message.
    New messages open automatically ONLY from Home; remain unread until an
    explicit open/Select. Editing and menu state are never overwritten.

  RADIO / LIMITS
    Direct group broadcast; no mesh, gateway, encryption, authentication or ACK.
    "Transmitted" means local TX completed, NOT delivery confirmed.
    Same channel and firmware required. Names are labels, not trusted identity.
    Max name 12 printable ASCII characters; text 120; inbox holds latest 20.
    Presets: 903/909/915/921/927 MHz; default 915; BW500kHz/SF12/CR4:8,
    20dBm, sync0x12, CRC on, explicit header, 16-symbol preamble.
    Boosted RX gain enabled. All pagers MUST run this radio profile; versions
    1.0/1.1 cannot receive this profile. Existing names/inbox/channel retained.
    Higher power and longer airtime increase energy per message; boosted RX
    also increases listening current. No guaranteed distance multiplier.
    Packet airtime determines TX timeout and busy-channel retry spacing.
    Typical airtime is about 0.5-2 seconds; backoff adds further delay.
    500kHz bandwidth retained for this fixed-channel US-band design; these
    settings alone do not establish FCC compliance. /status shows RF settings
    and the last valid incoming packet RSSI/SNR for manual range tests.
    These are experimental US-band settings, NOT a certification or compliance
    guarantee. Verify your exact board, antenna and permitted operating profile.
    Do not use this build on 433/868-only hardware or outside applicable regions.
    Always-on RX; OLED blanks after 60s. No deep sleep. Not an emergency pager.
    Bounded random backoff + LoRa activity detection; hidden-node losses remain.
    Incoming packets are strictly length/version checked, then deduplicated.
    Inbox is saved to NVS after a 2-second quiet period (recent entries can be
    lost if power is removed immediately). Name/channel saved on explicit change.
    Inbox times are receiver uptime with boot ID; there is no real-time clock.

  BATTERY (Heltec V3.2, standard 4.2V-full single-cell LiPo)
    Built-in divider: GPIO1 ADC, GPIO37 enable HIGH on V3.2.
    Older V3 revisions may require LOW: change BATTERY_ENABLE_LEVEL below.
    No external battery-sense wiring. Home shows icon and estimated percentage;
    Settings > Battery shows voltage. /battery prints diagnostics.
    Samples every 10 seconds, averaging 16 readings and smoothing between reads.
    Sampling pauses during queued/TX activity and for 2 seconds after TX.
    Percentage is a generic voltage estimate, NOT a measured fuel gauge.
    USB/charging changes readings; assess remaining charge with USB unplugged.
    Voltage alone cannot reliably detect charging or battery presence.
    Low warning below 3.50V for 3 samples; re-arms above 3.65V.
    No automatic shutdown; radio continues listening. Capacity (1000mAh, etc.)
    does not change the voltage curve. BATTERY_CALIBRATION can correct readings
    against a multimeter: new factor = old factor * measured / displayed.

  SERIAL COMMANDS (newline terminated)
    /help /battery /status /name Amber /channel 3 /send Hello /inbox /read 1 /demo
    /up /down /left /right /ok /case /back
    /channel uses 1..5; /demo creates a LOCAL simulated incoming message.
    Serial control permits testing before buttons are wired.

  Primary pin reference:
  https://github.com/espressif/arduino-esp32/blob/master/variants/heltec_wifi_lora_32_V3/pins_arduino.h
*/
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RadioLib.h>
#include <esp_system.h>
#include <ctype.h>
#include <stddef.h>

#if !defined(CONFIG_IDF_TARGET_ESP32S3)
#error "Select Heltec WiFi LoRa 32(V3), an ESP32-S3 board."
#endif

constexpr uint8_t PIN_UP=4, PIN_DOWN=5, PIN_LEFT=6, PIN_RIGHT=7, PIN_OK=38;
constexpr uint8_t PIN_ALERT=39, PIN_USER_LED=35;
constexpr uint8_t PIN_OLED_SDA=17, PIN_OLED_SCL=18, PIN_OLED_RST=21, PIN_VEXT=36;
constexpr uint8_t PIN_BATTERY_ADC=1, PIN_BATTERY_ENABLE=37;
constexpr uint8_t BATTERY_ENABLE_LEVEL=HIGH; // V3.2; older V3 may use LOW.
constexpr float BATTERY_DIVIDER=4.9f, BATTERY_CALIBRATION=1.0f;
constexpr uint32_t BATTERY_INTERVAL_MS=10000;
bool batteryValid=false, batterySampling=false, batteryLow=false;
bool batteryWarningPending=false, batteryHasRead=false, batteryTxSeen=false;
float batteryVolts=0.0f, batteryRawVolts=0.0f;
uint8_t batteryPercent=0, batterySamples=0, batteryLowCount=0;
uint32_t batterySumMv=0, batteryLastRead=0, batteryNextSample=0, batteryLastTx=0;

constexpr size_t NAME_MAX_LEN=12, TEXT_MAX_LEN=120, INBOX_MAX=20, HEADER_LEN=22;
constexpr size_t PACKET_MAX=HEADER_LEN+NAME_MAX_LEN+TEXT_MAX_LEN;
// Shared long-range profile. Keep identical on every pager.
constexpr float RADIO_BW_KHZ=500.0f;
constexpr uint8_t RADIO_SF=12, RADIO_CR=8;
constexpr int8_t RADIO_TX_DBM=20;
constexpr uint16_t RADIO_PREAMBLE=16;
constexpr uint32_t SEND_GAP_MS=5000;
uint32_t txTimeoutMs=6000, maxPacketAirMs=2000;
bool haveLinkSample=false;
float lastRxRssi=0.0f, lastRxSnr=0.0f;
uint32_t lastRxAt=0;
constexpr uint8_t CHANNEL_COUNT=5;
const float CHANNEL_MHZ[CHANNEL_COUNT]={903.0f,909.0f,915.0f,921.0f,927.0f};
const char *QUICK[]={"Come here please.","On my way.","Where are you?","I am home.",
                     "Please call me.","Yes.","No.","Thank you!"};
constexpr uint8_t QUICK_COUNT=sizeof(QUICK)/sizeof(QUICK[0]);
const char KEYBOARD[]="ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.<>";
static_assert(sizeof(KEYBOARD)-1 == 40, "Keyboard must contain 40 keys");

Adafruit_SSD1306 oled(128,64,&Wire,-1,100000,100000);
SX1262 radio = new Module(8,14,12,13);
Preferences prefs;
bool prefsReady=false, oledReady=false, oledSleeping=false, radioReady=false;
int16_t lastRadioError=0;
volatile bool radioIRQ=false;
uint64_t deviceId=0;
uint32_t bootId=0, sequenceNumber=0;
char deviceName[NAME_MAX_LEN+1]={0};
uint8_t channelIndex=2;

struct Message {
  uint64_t sender;
  uint32_t senderBoot, sequence, receivedBoot, receivedSeconds;
  int16_t rssi;
  uint8_t unread;
  char name[NAME_MAX_LEN+1];
  char text[TEXT_MAX_LEN+1];
};
// Explicit declarations keep Arduino auto-prototypes after the Message type.
bool decodePacket(const uint8_t *p, size_t len, Message &m);
void acceptMessage(Message &m);

struct InboxStore {
  uint32_t magic;
  uint16_t version, count;
  Message messages[INBOX_MAX];
};
InboxStore inbox={};
bool inboxDirty=false;
uint32_t inboxChangedAt=0;
struct Seen { uint64_t sender; uint32_t boot, sequence; };
Seen seen[64]={};
uint8_t seenNext=0;

// Use int UI state so Arduino's generated function prototypes require no custom types.
enum { HOME, INBOX, READER, EDIT_TEXT, EDIT_NAME, QUICK_MENU, SETTINGS,
       CHANNEL_MENU, CONFIRM_SEND, ABOUT, CLEAR_CONFIRM, BATTERY_VIEW };
enum { UP, DOWN, LEFT, RIGHT, SELECT };
int view=HOME, homeSelection=0, inboxSelection=0, quickSelection=0;
int settingsSelection=0, channelSelection=2, keyboardSelection=0, readerLine=0;
bool lowercase=false;
char draft[TEXT_MAX_LEN+1]={0}, editedName[NAME_MAX_LEN+1]={0};
Message reading={}; // Copy keeps reader stable when a new message shifts inbox.
uint32_t lastInteraction=0, lastDraw=0;
bool drawNeeded=true;
char notice[22]={0};
uint32_t noticeUntil=0;

struct Button {
  explicit Button(uint8_t p) : pin(p) {}
  uint8_t pin;
  bool raw=false, stable=false, held=false;
  uint32_t changed=0, pressed=0;
};
Button buttons[5]={Button(PIN_UP),Button(PIN_DOWN),Button(PIN_LEFT),Button(PIN_RIGHT),Button(PIN_OK)};

uint8_t txBuffer[PACKET_MAX];
size_t txLength=0;
bool txQueued=false, txActive=false;
uint8_t busyAttempts=0;
uint32_t txDue=0, txStarted=0, lastSendAt=0;
bool haveSent=false;
char txText[TEXT_MAX_LEN+1]={0};
char serialLine[180]={0};
size_t serialLength=0;
bool serialOverflow=false;

void IRAM_ATTR onRadioIRQ() { radioIRQ=true; }

void copyText(char *out, size_t size, const char *in) {
  if (!size) return;
  size_t n=strnlen(in,size-1);
  memcpy(out,in,n); out[n]=0;
}
bool printable(const char *s, size_t n) {
  for(size_t i=0;i<n;++i) if((uint8_t)s[i]<32 || (uint8_t)s[i]>126) return false;
  return true;
}
bool hasNonSpace(const char *s) {
  while(*s) { if(*s!=' ') return true; ++s; } return false;
}
void put32(uint8_t *p,uint32_t v) { for(int i=0;i<4;++i) p[i]=(v>>(8*i))&255; }
uint32_t get32(const uint8_t *p) {
  uint32_t v=0; for(int i=0;i<4;++i) v|=((uint32_t)p[i])<<(8*i); return v;
}
void put64(uint8_t *p,uint64_t v) { for(int i=0;i<8;++i) p[i]=(v>>(8*i))&255; }
uint64_t get64(const uint8_t *p) {
  uint64_t v=0; for(int i=0;i<8;++i) v|=((uint64_t)p[i])<<(8*i); return v;
}
size_t makePacket(uint8_t *out,const char *text) {
  size_t nl=strlen(deviceName), tl=strlen(text);
  if(!nl || nl>NAME_MAX_LEN || !tl || tl>TEXT_MAX_LEN) return 0;
  out[0]='P'; out[1]='G'; out[2]=1; out[3]=1;
  put64(out+4,deviceId); put32(out+12,bootId); put32(out+16,++sequenceNumber);
  out[20]=(uint8_t)nl; out[21]=(uint8_t)tl;
  memcpy(out+HEADER_LEN,deviceName,nl); memcpy(out+HEADER_LEN+nl,text,tl);
  return HEADER_LEN+nl+tl;
}
bool decodePacket(const uint8_t *p,size_t len,Message &m) {
  if(len<HEADER_LEN || len>PACKET_MAX) return false;
  if(p[0]!='P' || p[1]!='G' || p[2]!=1 || p[3]!=1) return false;
  size_t nl=p[20],tl=p[21];
  if(!nl || nl>NAME_MAX_LEN || !tl || tl>TEXT_MAX_LEN || len!=HEADER_LEN+nl+tl) return false;
  if(!printable((const char*)p+HEADER_LEN,nl+tl)) return false;
  memset(&m,0,sizeof(m));
  m.sender=get64(p+4); m.senderBoot=get32(p+12); m.sequence=get32(p+16);
  memcpy(m.name,p+HEADER_LEN,nl); memcpy(m.text,p+HEADER_LEN+nl,tl);
  return hasNonSpace(m.name) && hasNonSpace(m.text);
}

uint8_t unreadCount() {
  uint8_t n=0; for(unsigned i=0;i<inbox.count;++i) if(inbox.messages[i].unread) ++n;
  return n;
}
void updateLED() {
  bool on=unreadCount()>0;
  digitalWrite(PIN_ALERT,on); digitalWrite(PIN_USER_LED,on);
}
void wakeDisplay() {
  lastInteraction=millis();
  if(oledReady && oledSleeping) { oled.ssd1306_command(SSD1306_DISPLAYON); oledSleeping=false; }
  drawNeeded=true;
}
void tell(const char *text,uint32_t duration=2500) {
  copyText(notice,sizeof(notice),text); noticeUntil=millis()+duration;
  Serial.println(text); wakeDisplay();
}
// Piecewise approximation for a typical 1S LiPo, under a light load.
uint8_t batteryPercentForVoltage(float v) {
  const float volts[]={3.30f,3.50f,3.60f,3.70f,3.75f,3.80f,3.85f,3.90f,4.00f,4.10f,4.20f};
  const uint8_t pct[]={0,5,10,20,30,40,50,60,75,90,100};
  if(!isfinite(v) || v<=volts[0]) return 0;
  for(unsigned i=1;i<sizeof(pct);++i) {
    if(v<=volts[i]) return (uint8_t)(pct[i-1]+(v-volts[i-1]) *
      (pct[i]-pct[i-1])/(volts[i]-volts[i-1])+0.5f);
  }
  return 100;
}
void stopBatterySampling() {
  digitalWrite(PIN_BATTERY_ENABLE,BATTERY_ENABLE_LEVEL==HIGH?LOW:HIGH);
  batterySampling=false;
}
void initializeBattery() {
  digitalWrite(PIN_BATTERY_ENABLE,BATTERY_ENABLE_LEVEL==HIGH?LOW:HIGH);
  pinMode(PIN_BATTERY_ENABLE,OUTPUT);
  pinMode(PIN_BATTERY_ADC,INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_BATTERY_ADC,ADC_2_5db);
}
void recordBatteryVoltage(float v) {
  batteryRawVolts=v;
  if(!isfinite(v) || v<2.5f || v>4.45f) {
    batteryValid=false; batteryLowCount=0; batteryWarningPending=false;
    drawNeeded=true; return;
  }
  batteryVolts=batteryValid ? batteryVolts+0.25f*(v-batteryVolts) : v;
  batteryValid=true;
  batteryPercent=batteryPercentForVoltage(batteryVolts);
  // Raw averaged voltage makes the alert responsive even while UI is smoothed.
  if(v<=3.50f) {
    if(batteryLowCount<3) ++batteryLowCount;
    if(batteryLowCount>=3 && !batteryLow) {
      batteryLow=true; batteryWarningPending=true;
    }
  } else {
    batteryLowCount=0;
    if(v>=3.65f) { batteryLow=false; batteryWarningPending=false; }
  }
  drawNeeded=true; // Does not wake an idle display on routine measurements.
}
void serviceBattery() {
  uint32_t now=millis();
  if(txActive || txQueued || radioIRQ ||
     (batteryTxSeen && (uint32_t)(now-batteryLastTx)<2000)) {
    if(batterySampling) stopBatterySampling();
    return;
  }
  if(batteryWarningPending && (!notice[0] || (int32_t)(now-noticeUntil)>=0)) {
    batteryWarningPending=false; tell("Battery low: recharge",5000);
  }
  if(!batterySampling) {
    if(batteryHasRead && (uint32_t)(now-batteryLastRead)<BATTERY_INTERVAL_MS) return;
    digitalWrite(PIN_BATTERY_ENABLE,BATTERY_ENABLE_LEVEL);
    batterySampling=true; batterySamples=0; batterySumMv=0;
    batteryNextSample=now+20; // Divider settling; no blocking delay.
    return;
  }
  if((int32_t)(now-batteryNextSample)<0) return;
  batterySumMv+=analogReadMilliVolts(PIN_BATTERY_ADC);
  batteryNextSample=now+3;
  if(++batterySamples<16) return;
  stopBatterySampling(); batteryHasRead=true; batteryLastRead=now;
  recordBatteryVoltage((batterySumMv/16.0f)*0.001f*BATTERY_DIVIDER*BATTERY_CALIBRATION);
}
void printBattery() {
  if(batteryValid) Serial.printf("Battery: ~%u%%, filtered %.3f V, latest %.3f V%s; age %lu s\n",
    batteryPercent,batteryVolts,batteryRawVolts,batteryLow?" LOW":"",
    (unsigned long)((millis()-batteryLastRead)/1000));
  else Serial.printf("Battery: %s; latest %.3f V\n",
    batteryHasRead?"reading unavailable":"waiting for first sample",batteryRawVolts);
  Serial.println("Voltage estimate only; USB/charging affects it. V3.2 GPIO37 enable=HIGH.");
}
void drawBatteryIcon(int x,int y) {
  oled.drawRect(x,y,19,8,SSD1306_WHITE);
  oled.fillRect(x+19,y+2,2,4,SSD1306_WHITE);
  if(!batteryValid) { oled.setCursor(x+6,y); oled.print('?'); return; }
  if(batteryLow) { oled.setCursor(x+6,y); oled.print('!'); return; }
  unsigned bars=(batteryPercent+19)/20;
  for(unsigned i=0;i<bars;++i) oled.fillRect(x+2+i*3,y+2,2,4,SSD1306_WHITE);
}

void dirtyInbox() { inboxDirty=true; inboxChangedAt=millis(); }
void saveSettings() {
  if(!prefsReady) return;
  if(!prefs.putString("name",deviceName) || !prefs.putUChar("channel",channelIndex))
    Serial.println("WARNING: could not save settings.");
}
void saveInbox() {
  if(!inboxDirty) return;
  inboxDirty=false;
  if(prefsReady && prefs.putBytes("inbox",&inbox,sizeof(inbox))!=sizeof(inbox))
    Serial.println("WARNING: inbox save failed.");
}
void remember(uint64_t id,uint32_t boot,uint32_t seq) {
  seen[seenNext]={id,boot,seq}; seenNext=(seenNext+1)%64;
}
bool alreadySeen(uint64_t id,uint32_t boot,uint32_t seq) {
  for(const auto &s:seen) if(s.sender==id && s.boot==boot && s.sequence==seq) return true;
  return false;
}
void loadStorage() {
  inbox.magic=0x50414731; inbox.version=1;
  prefsReady=prefs.begin("heltecpager",false);
  if(!prefsReady) { Serial.println("NVS unavailable: settings/inbox volatile."); return; }
  String saved=prefs.getString("name",deviceName);
  if(saved.length()>0 && saved.length()<=NAME_MAX_LEN &&
     printable(saved.c_str(),saved.length()) && hasNonSpace(saved.c_str()))
    copyText(deviceName,sizeof(deviceName),saved.c_str());
  channelIndex=prefs.getUChar("channel",2);
  if(channelIndex>=CHANNEL_COUNT) channelIndex=2;
  if(prefs.getBytesLength("inbox")==sizeof(inbox)) {
    if(prefs.getBytes("inbox",&inbox,sizeof(inbox))!=sizeof(inbox) ||
       inbox.magic!=0x50414731 || inbox.version!=1 || inbox.count>INBOX_MAX) inbox.count=0;
    for(unsigned i=0;i<inbox.count;++i) {
      auto &m=inbox.messages[i];
      if(strnlen(m.name,sizeof(m.name))>NAME_MAX_LEN || strnlen(m.text,sizeof(m.text))>TEXT_MAX_LEN ||
         !printable(m.name,strnlen(m.name,sizeof(m.name))) ||
         !printable(m.text,strnlen(m.text,sizeof(m.text))) || m.unread>1) { inbox.count=0; break; }
    }
  }
  inbox.magic=0x50414731; inbox.version=1;
  for(unsigned i=0;i<inbox.count;++i)
    remember(inbox.messages[i].sender,inbox.messages[i].senderBoot,inbox.messages[i].sequence);
}
void markReadingRead() {
  for(unsigned i=0;i<inbox.count;++i) {
    auto &m=inbox.messages[i];
    if(m.sender==reading.sender && m.senderBoot==reading.senderBoot && m.sequence==reading.sequence) {
      if(m.unread) { m.unread=0; dirtyInbox(); }
      break;
    }
  }
  reading.unread=0; updateLED(); drawNeeded=true;
}
void openMessage(int index) {
  if(index<0 || index>=inbox.count) return;
  reading=inbox.messages[index]; readerLine=0; view=READER; markReadingRead(); wakeDisplay();
}
void acceptMessage(Message &m) {
  if(alreadySeen(m.sender,m.senderBoot,m.sequence)) return;
  remember(m.sender,m.senderBoot,m.sequence);
  m.receivedBoot=bootId; m.receivedSeconds=millis()/1000; m.unread=1;
  unsigned shift=inbox.count<INBOX_MAX ? inbox.count : INBOX_MAX-1;
  memmove(&inbox.messages[1],&inbox.messages[0],shift*sizeof(Message));
  inbox.messages[0]=m;
  if(inbox.count<INBOX_MAX) ++inbox.count;
  if(view==INBOX && inbox.count>1 && inboxSelection<(int)inbox.count-1) ++inboxSelection;
  dirtyInbox(); updateLED();
  Serial.printf("RX %s: %s [RSSI %d]\n",m.name,m.text,m.rssi);
  if(view==HOME) { reading=m; readerLine=0; view=READER; }
  else { snprintf(notice,sizeof(notice),"New: %.12s",m.name); noticeUntil=millis()+2500; }
  wakeDisplay();
}

void startListening() {
  radioIRQ=false;
  int16_t status=radio.startReceive();
  if(status!=RADIOLIB_ERR_NONE) {
    radioReady=false; lastRadioError=status;
    Serial.printf("RX start error %d; Settings > Retry radio\n",status);
    drawNeeded=true;
  }
}
void initializeRadio() {
  radioReady=false; radioIRQ=false;
  int16_t s=radio.begin(CHANNEL_MHZ[channelIndex],RADIO_BW_KHZ,RADIO_SF,RADIO_CR,
                        0x12,RADIO_TX_DBM,RADIO_PREAMBLE,1.8,false);
  if(s==RADIOLIB_ERR_NONE) s=radio.setDio2AsRfSwitch(true);
  if(s==RADIOLIB_ERR_NONE) s=radio.setCRC(2);
  if(s==RADIOLIB_ERR_NONE) s=radio.setRxBoostedGainMode(true);
  lastRadioError=s;
  if(s!=RADIOLIB_ERR_NONE) { Serial.printf("Radio init error %d\n",s); drawNeeded=true; return; }
  maxPacketAirMs=(radio.getTimeOnAir(PACKET_MAX)+999UL)/1000UL;
  radio.setDio1Action(onRadioIRQ);
  radioReady=true; startListening();
  Serial.printf("Radio %s: %.1f MHz, BW%.0f SF%u CR4:%u, %ddBm, preamble %u, boosted RX\n",
                radioReady?"ready":"failed",CHANNEL_MHZ[channelIndex],RADIO_BW_KHZ,
                RADIO_SF,RADIO_CR,RADIO_TX_DBM,RADIO_PREAMBLE);
  drawNeeded=true;
}
void queueMessage(const char *text) {
  size_t len=strlen(text);
  if(!len || len>TEXT_MAX_LEN || !printable(text,len) || !hasNonSpace(text)) {
    tell("Use 1-120 ASCII chars"); return;
  }
  if(!radioReady) { tell("Radio unavailable"); return; }
  if(txQueued || txActive) { tell("Already sending"); return; }
  if(haveSent && (uint32_t)(millis()-lastSendAt)<SEND_GAP_MS) { tell("Wait a few seconds"); return; }
  copyText(txText,sizeof(txText),text);
  txLength=makePacket(txBuffer,txText);
  if(!txLength) { tell("Packet build failed"); return; }
  txQueued=true; busyAttempts=0; txDue=millis()+250+(esp_random()%1250);
  tell("Queued for broadcast");
}
void finishTx(bool timedOut) {
  batteryTxSeen=true; batteryLastTx=millis();
  int16_t s=radio.finishTransmit();
  txActive=false;
  if(!timedOut && s==RADIOLIB_ERR_NONE) {
    lastSendAt=millis(); haveSent=true;
    Serial.printf("TX complete: %s (no delivery ACK)\n",txText);
    if(strcmp(draft,txText)==0) draft[0]=0;
    if(view==CONFIRM_SEND) view=HOME;
    tell("Transmitted (no ACK)");
  } else { lastRadioError=timedOut?RADIOLIB_ERR_TX_TIMEOUT:s; tell("TX failed; try again"); }
  startListening();
}
void serviceRadio() {
  if(!radioReady) return;
  if(txActive) {
    if(radioIRQ) { radioIRQ=false; finishTx(false); }
    else if((uint32_t)(millis()-txStarted)>txTimeoutMs) finishTx(true);
    return;
  }
  if(radioIRQ) {
    radioIRQ=false;
    uint8_t buffer[255];
    size_t length=radio.getPacketLength();
    if(length>0 && length<=sizeof(buffer)) {
      int16_t status=radio.readData(buffer,length);
      if(status==RADIOLIB_ERR_NONE) {
        Message m;
        if(decodePacket(buffer,length,m) && m.sender!=deviceId) {
          lastRxRssi=radio.getRSSI(); lastRxSnr=radio.getSNR();
          lastRxAt=millis(); haveLinkSample=true;
          Serial.printf("Link RSSI %.1f dBm, SNR %.1f dB\n",lastRxRssi,lastRxSnr);
          m.rssi=(int16_t)lastRxRssi; acceptMessage(m);
        }
      } else Serial.printf("Packet rejected, radio code %d\n",status);
    }
    startListening();
  }
  if(!radioReady || !txQueued || (int32_t)(millis()-txDue)<0) return;
  radio.standby(); radioIRQ=false;
  int16_t channelStatus=radio.scanChannel();
  if(channelStatus==RADIOLIB_LORA_DETECTED) {
    if(++busyAttempts>=5) { txQueued=false; tell("Channel busy; retry"); }
    else txDue=millis()+maxPacketAirMs+250+(esp_random()%(maxPacketAirMs+1000));
    startListening(); return;
  }
  if(channelStatus!=RADIOLIB_CHANNEL_FREE) {
    txQueued=false; lastRadioError=channelStatus; tell("Radio scan failed"); startListening(); return;
  }
  radioIRQ=false; txQueued=false;
  if(batterySampling) stopBatterySampling();
  batteryTxSeen=true; batteryLastTx=millis();
  uint32_t airMs=(radio.getTimeOnAir(txLength)+999UL)/1000UL;
  txTimeoutMs=airMs*2UL+2000UL;
  Serial.printf("TX %u bytes: expected airtime %lu ms, timeout %lu ms\n",
                (unsigned)txLength,(unsigned long)airMs,(unsigned long)txTimeoutMs);
  int16_t s=radio.startTransmit(txBuffer,txLength);
  if(s!=RADIOLIB_ERR_NONE) { lastRadioError=s; tell("TX start failed"); startListening(); return; }
  txActive=true; txStarted=millis(); drawNeeded=true;
}

void setName(const char *name) {
  size_t n=strlen(name);
  if(!n || n>NAME_MAX_LEN || !printable(name,n) || !hasNonSpace(name)) { tell("Name: 1-12 chars"); return; }
  copyText(deviceName,sizeof(deviceName),name); saveSettings(); tell("Name saved");
}
void setChannel(int index) {
  if(index<0 || index>=CHANNEL_COUNT) { tell("Channel must be 1-5"); return; }
  if(txQueued || txActive) { tell("Wait for transmit"); return; }
  if(radioReady) radio.standby();
  channelIndex=(uint8_t)index; saveSettings(); initializeRadio();
  tell(radioReady?"Channel saved":"Saved; radio error");
}
void beginEditor(bool name) {
  if(name) copyText(editedName,sizeof(editedName),deviceName);
  keyboardSelection=0; lowercase=false; view=name?EDIT_NAME:EDIT_TEXT; drawNeeded=true;
}
void editKey() {
  char *target=view==EDIT_NAME?editedName:draft;
  size_t cap=view==EDIT_NAME?NAME_MAX_LEN:TEXT_MAX_LEN;
  size_t n=strlen(target);
  char c=KEYBOARD[keyboardSelection];
  if(c=='<') { if(n) target[n-1]=0; }
  else if(c=='>') {
    if(!hasNonSpace(target)) { tell("Please enter text"); return; }
    if(view==EDIT_NAME) { setName(target); view=SETTINGS; }
    else view=CONFIRM_SEND;
  } else if(n<cap) {
    if(c=='_') c=' ';
    else if(lowercase && c>='A' && c<='Z') c=c-'A'+'a';
    target[n]=c; target[n+1]=0;
  } else tell("Text limit reached");
  drawNeeded=true;
}
void handleKey(int key,bool longPress=false) {
  wakeDisplay();
  bool editing=view==EDIT_TEXT || view==EDIT_NAME;
  if(longPress) {
    if(editing && key==SELECT) lowercase=!lowercase;
    if(editing && key==LEFT) view=view==EDIT_NAME?SETTINGS:HOME;
    return;
  }
  if(editing) {
    int row=keyboardSelection/10,col=keyboardSelection%10;
    if(key==UP) row=(row+3)%4;
    if(key==DOWN) row=(row+1)%4;
    if(key==LEFT) col=(col+9)%10;
    if(key==RIGHT) col=(col+1)%10;
    keyboardSelection=row*10+col;
    if(key==SELECT) editKey();
    return;
  }
  if(view==HOME) {
    if(key==UP) homeSelection=(homeSelection+3)%4;
    if(key==DOWN) homeSelection=(homeSelection+1)%4;
    if(key==RIGHT) {
      for(unsigned i=0;i<inbox.count;++i) if(inbox.messages[i].unread) { openMessage(i); return; }
      tell("No unread messages");
    }
    if(key==SELECT) {
      if(homeSelection==0) { inboxSelection=0; view=INBOX; }
      if(homeSelection==1) beginEditor(false);
      if(homeSelection==2) { quickSelection=0; view=QUICK_MENU; }
      if(homeSelection==3) { settingsSelection=0; view=SETTINGS; }
    }
  } else if(view==INBOX) {
    if(key==LEFT) view=HOME;
    if(inbox.count) {
      if(key==UP) inboxSelection=(inboxSelection+inbox.count-1)%inbox.count;
      if(key==DOWN) inboxSelection=(inboxSelection+1)%inbox.count;
      if(key==SELECT) openMessage(inboxSelection);
    }
  } else if(view==READER) {
    // Automatic preview remains unread until Select; explicit inbox open marks read.
    if(key==UP && readerLine>0) --readerLine;
    int lines=(strlen(reading.text)+20)/21;
    if(key==DOWN && readerLine+4<lines) ++readerLine;
    if(key==LEFT) { view=INBOX; inboxSelection=0; }
    if(key==SELECT) {
      if(reading.unread) markReadingRead();
      else { quickSelection=0; view=QUICK_MENU; }
    }
  } else if(view==QUICK_MENU) {
    if(key==LEFT) view=HOME;
    if(key==UP) quickSelection=(quickSelection+QUICK_COUNT-1)%QUICK_COUNT;
    if(key==DOWN) quickSelection=(quickSelection+1)%QUICK_COUNT;
    if(key==SELECT) {
      if(draft[0]) { tell("Finish/clear draft"); return; }
      copyText(draft,sizeof(draft),QUICK[quickSelection]); view=CONFIRM_SEND;
    }
  } else if(view==CONFIRM_SEND) {
    if(key==LEFT && !txQueued && !txActive) beginEditor(false);
    if(key==SELECT) queueMessage(draft);
  } else if(view==SETTINGS) {
    if(key==LEFT) view=HOME;
    if(key==UP) settingsSelection=(settingsSelection+5)%6;
    if(key==DOWN) settingsSelection=(settingsSelection+1)%6;
    if(key==SELECT) {
      if(settingsSelection==0) beginEditor(true);
      if(settingsSelection==1) { channelSelection=channelIndex; view=CHANNEL_MENU; }
      if(settingsSelection==2) view=ABOUT;
      if(settingsSelection==3) {
        if(txQueued || txActive) tell("Wait for transmit");
        else { initializeRadio(); tell(radioReady?"Radio ready":"Radio init failed"); }
      }
      if(settingsSelection==4) view=CLEAR_CONFIRM;
      if(settingsSelection==5) view=BATTERY_VIEW;
    }
  } else if(view==CHANNEL_MENU) {
    if(key==LEFT) view=SETTINGS;
    if(key==UP) channelSelection=(channelSelection+CHANNEL_COUNT-1)%CHANNEL_COUNT;
    if(key==DOWN) channelSelection=(channelSelection+1)%CHANNEL_COUNT;
    if(key==SELECT) { setChannel(channelSelection); view=SETTINGS; }
  } else if(view==ABOUT || view==BATTERY_VIEW) {
    if(key==LEFT || key==SELECT) view=SETTINGS;
  } else if(view==CLEAR_CONFIRM) {
    if(key==LEFT) view=SETTINGS;
    if(key==SELECT) { inbox.count=0; dirtyInbox(); updateLED(); view=SETTINGS; tell("Inbox cleared"); }
  }
}
void pollButtons() {
  uint32_t now=millis();
  for(int i=0;i<5;++i) {
    Button &b=buttons[i]; bool pressed=digitalRead(b.pin)==LOW;
    if(pressed!=b.raw) { b.raw=pressed; b.changed=now; }
    if((uint32_t)(now-b.changed)>=30 && b.stable!=b.raw) {
      b.stable=b.raw;
      if(b.stable) { b.pressed=now; b.held=false; wakeDisplay(); }
      else if(!b.held) handleKey(i);
    }
    if(b.stable && !b.held && (i==LEFT || i==SELECT) &&
       (view==EDIT_TEXT || view==EDIT_NAME) && (uint32_t)(now-b.pressed)>=700) {
      b.held=true; handleKey(i,true);
    }
  }
}

void line(int y,const char *text,bool selected=false) {
  if(selected) oled.fillRect(0,y,128,9,SSD1306_WHITE);
  oled.setTextColor(selected?SSD1306_BLACK:SSD1306_WHITE);
  oled.setCursor(0,y); for(int i=0;text[i] && i<21;++i) oled.write(text[i]);
  oled.setTextColor(SSD1306_WHITE);
}
void textLines(const char *text,int first,int y,int count) {
  size_t len=strlen(text);
  for(int row=0;row<count;++row) {
    size_t offset=(first+row)*21;
    if(offset>=len) break;
    char part[22]={0}; size_t n=len-offset; if(n>21) n=21;
    memcpy(part,text+offset,n); line(y+row*9,part);
  }
}
void drawUI() {
  if(!oledReady || oledSleeping) return;
  oled.clearDisplay(); oled.setTextSize(1); oled.setTextWrap(false);
  char buf[32];
  if(view==HOME) {
    line(0,deviceName);
    oled.setCursor(76,0);
    if(batteryValid) { snprintf(buf,sizeof(buf),"%3u%%",batteryPercent); oled.print(buf); }
    else oled.print(" --%");
    drawBatteryIcon(105,0);
    const char *items[]={"Inbox","Compose","Quick messages","Settings"};
    for(int i=0;i<4;++i) line(12+i*10,items[i],homeSelection==i);
    snprintf(buf,sizeof(buf),"CH%d U:%u OK:open",channelIndex+1,unreadCount());
    line(55,radioReady?buf:"RADIO ERROR: Settings");
  } else if(view==INBOX) {
    snprintf(buf,sizeof(buf),"Inbox %u / unread %u",inbox.count,unreadCount()); line(0,buf);
    int first=(inboxSelection/4)*4;
    if(!inbox.count) line(20,"No messages yet");
    for(int row=0;row<4 && first+row<inbox.count;++row) {
      const auto &m=inbox.messages[first+row];
      snprintf(buf,sizeof(buf),"%c%-12.12s %.6s",m.unread?'*':' ',m.name,m.text);
      line(12+row*10,buf,first+row==inboxSelection);
    }
    line(55,"OK:read Left:home");
  } else if(view==READER) {
    snprintf(buf,sizeof(buf),"%s%.12s",reading.unread?"NEW ":"From ",reading.name); line(0,buf);
    textLines(reading.text,readerLine,12,4);
    line(55,reading.unread?"OK:mark read Up/Dn":"OK:reply Left:inbox");
  } else if(view==EDIT_TEXT || view==EDIT_NAME) {
    const char *t=view==EDIT_NAME?editedName:draft; size_t n=strlen(t);
    snprintf(buf,sizeof(buf),"%s %u/%u %s",view==EDIT_NAME?"Name":"Text",(unsigned)n,
             (unsigned)(view==EDIT_NAME?NAME_MAX_LEN:TEXT_MAX_LEN),lowercase?"abc":"ABC");
    line(0,buf); line(12,t+(n>20?n-20:0));
    for(int i=0;i<40;++i) {
      int x=(i%10)*12,y=24+(i/10)*8;
      bool selected=i==keyboardSelection;
      if(selected) oled.fillRect(x,y,11,8,SSD1306_WHITE);
      oled.setTextColor(selected?SSD1306_BLACK:SSD1306_WHITE); oled.setCursor(x+3,y);
      char c=KEYBOARD[i]; if(lowercase && c>='A' && c<='Z') c=c-'A'+'a'; oled.write(c);
    }
    oled.setTextColor(SSD1306_WHITE); line(56,"_=space <=del >=done");
  } else if(view==QUICK_MENU) {
    line(0,"Quick messages"); int first=(quickSelection/4)*4;
    for(int row=0;row<4 && first+row<QUICK_COUNT;++row)
      line(12+row*10,QUICK[first+row],first+row==quickSelection);
    line(55,"OK:choose Left:home");
  } else if(view==CONFIRM_SEND) {
    line(0,"Broadcast to channel?"); textLines(draft,0,12,4);
    line(55,(txQueued || txActive)?"Sending...":"OK:send Left:edit");
  } else if(view==SETTINGS) {
    line(0,"Settings");
    const char *items[]={"Name","Channel","About / status","Retry radio","Clear inbox","Battery"};
    int first=settingsSelection>=4?settingsSelection-3:0;
    for(int row=0;row<4;++row) line(12+row*10,items[first+row],settingsSelection==first+row);
    line(55,"OK:open Left:home");
  } else if(view==CHANNEL_MENU) {
    line(0,"Radio channel");
    int first=channelSelection>=4?1:0;
    for(int row=0;row<4;++row) {
      int i=first+row; snprintf(buf,sizeof(buf),"CH%d  %.1f MHz%s",i+1,CHANNEL_MHZ[i],i==channelIndex?" *":"");
      line(12+row*10,buf,i==channelSelection);
    }
    line(55,"OK:save Left:cancel");
  } else if(view==ABOUT) {
    line(0,"HeltecPager 1.2"); snprintf(buf,sizeof(buf),"ID %012llX",(unsigned long long)deviceId); line(12,buf);
    snprintf(buf,sizeof(buf),"CH%d %.1fMHz %ddBm",channelIndex+1,CHANNEL_MHZ[channelIndex],RADIO_TX_DBM); line(22,buf);
    snprintf(buf,sizeof(buf),"Radio:%s err:%d",radioReady?"OK":"ERR",lastRadioError); line(32,buf);
    if(haveLinkSample) snprintf(buf,sizeof(buf),"R:%.0f S:%.1f dB",lastRxRssi,lastRxSnr);
    else snprintf(buf,sizeof(buf),"SF%u BW%.0f CR4:%u",RADIO_SF,RADIO_BW_KHZ,RADIO_CR);
    line(42,buf); line(55,"Long range / no ACK");
  } else if(view==BATTERY_VIEW) {
    line(0,"Battery (estimate)");
    if(batteryValid) {
      snprintf(buf,sizeof(buf),"Charge: ~%u%%",batteryPercent); line(12,buf);
      snprintf(buf,sizeof(buf),"Voltage: %.2f V",batteryVolts); line(22,buf);
      line(32,batteryLow?"LOW - please recharge":"1S LiPo / voltage est");
    } else { line(12,"Reading unavailable"); line(22,"Check pack / ADC"); }
    line(42,"USB affects estimate"); line(55,"OK/Left:back");
  } else if(view==CLEAR_CONFIRM) {
    line(0,"Delete entire inbox?"); line(22,"This cannot be undone"); line(55,"OK:delete Left:cancel");
  }
  if(notice[0] && (int32_t)(noticeUntil-millis())>0) {
    oled.fillRect(0,55,128,9,SSD1306_BLACK); line(55,notice);
  }
  oled.display(); drawNeeded=false; lastDraw=millis();
}

void printStatus() {
  Serial.printf("RF: SF%u BW%.0fkHz CR4:%u %ddBm, preamble %u, boosted RX\n",
                RADIO_SF,RADIO_BW_KHZ,RADIO_CR,RADIO_TX_DBM,RADIO_PREAMBLE);
  if(haveLinkSample) Serial.printf("Last RX: RSSI %.1f dBm, SNR %.1f dB, %lu s ago\n",
    lastRxRssi,lastRxSnr,(unsigned long)((millis()-lastRxAt)/1000));
  else Serial.println("Last RX: no valid remote message received this boot.");
  Serial.printf("\nHeltecPager 1.2 | %s | ID %012llX | boot %08lX\n",deviceName,
                (unsigned long long)deviceId,(unsigned long)bootId);
  Serial.printf("CH%u %.1f MHz | radio=%s err=%d | OLED=%s | NVS=%s\n",channelIndex+1,
                CHANNEL_MHZ[channelIndex],radioReady?"OK":"ERROR",lastRadioError,
                oledReady?"initialized":"absent",prefsReady?"OK":"ERROR");
  Serial.printf("Inbox=%u unread=%u heap=%u uptime=%lu s\n",inbox.count,unreadCount(),
                (unsigned)ESP.getFreeHeap(),(unsigned long)(millis()/1000));
}
void serialCommand(char *cmd) {
  if(!strcmp(cmd,"/help")) {
    Serial.println("/battery /status /name Amber /channel 3 /send Hello /inbox /read 1 /demo");
    Serial.println("/up /down /left /right /ok /case /back. Channels 1..5; default 3=915MHz.");
  } else if(!strcmp(cmd,"/status")) { printStatus(); printBattery(); }
  else if(!strcmp(cmd,"/battery")) printBattery();
  else if(!strncmp(cmd,"/name ",6)) setName(cmd+6);
  else if(!strncmp(cmd,"/channel ",9)) {
    char *end=nullptr; long n=strtol(cmd+9,&end,10);
    if(end==cmd+9 || *end || n<1 || n>CHANNEL_COUNT) tell("Channel must be 1-5"); else setChannel(n-1);
  } else if(!strncmp(cmd,"/send ",6)) queueMessage(cmd+6);
  else if(!strcmp(cmd,"/inbox")) {
    for(unsigned i=0;i<inbox.count;++i) {
      auto &m=inbox.messages[i]; Serial.printf("%u %c %s: %s\n",i+1,m.unread?'*':' ',m.name,m.text);
    }
  } else if(!strncmp(cmd,"/read ",6)) {
    char *end=nullptr; long n=strtol(cmd+6,&end,10);
    if(end==cmd+6 || *end || n<1 || n>inbox.count) tell("Invalid inbox number"); else openMessage(n-1);
  } else if(!strcmp(cmd,"/demo")) {
    Message m={}; m.sender=0xDEADBEEF; m.senderBoot=bootId; m.sequence=esp_random(); m.rssi=0;
    copyText(m.name,sizeof(m.name),"Demo"); copyText(m.text,sizeof(m.text),"Local test. Your unread LED should be on. Press Select to mark this message read.");
    acceptMessage(m);
  } else if(!strcmp(cmd,"/up")) handleKey(UP);
  else if(!strcmp(cmd,"/down")) handleKey(DOWN);
  else if(!strcmp(cmd,"/left")) handleKey(LEFT);
  else if(!strcmp(cmd,"/right")) handleKey(RIGHT);
  else if(!strcmp(cmd,"/ok")) handleKey(SELECT);
  else if(!strcmp(cmd,"/case")) handleKey(SELECT,true);
  else if(!strcmp(cmd,"/back")) handleKey(LEFT,true);
  else if(cmd[0]) Serial.println("Unknown command. Type /help.");
}
void serviceSerial() {
  unsigned budget=64;
  while(Serial.available() && budget--) {
    char c=Serial.read(); if(c=='\r') continue;
    if(c=='\n') {
      if(serialOverflow) Serial.println("Command too long; discarded.");
      else { serialLine[serialLength]=0; serialCommand(serialLine); }
      serialLength=0; serialOverflow=false;
    } else if(!serialOverflow) {
      if(serialLength<sizeof(serialLine)-1) serialLine[serialLength++]=c;
      else serialOverflow=true;
    }
  }
}
void initializeOLED() {
  pinMode(PIN_VEXT,OUTPUT); digitalWrite(PIN_VEXT,LOW); delay(100);
  pinMode(PIN_OLED_RST,OUTPUT); digitalWrite(PIN_OLED_RST,HIGH); delay(10);
  digitalWrite(PIN_OLED_RST,LOW); delay(20); digitalWrite(PIN_OLED_RST,HIGH); delay(100);
  if(!Wire.begin(PIN_OLED_SDA,PIN_OLED_SCL,100000)) return;
  Wire.setTimeOut(50);
  Wire.beginTransmission(0x3C);
  if(Wire.endTransmission()!=0) { Serial.println("OLED absent at 0x3C; serial remains usable."); return; }
  oledReady=oled.begin(SSD1306_SWITCHCAPVCC,0x3C,false,false);
  if(oledReady) { oled.clearDisplay(); oled.display(); }
}
void setup() {
  Serial.begin(115200); delay(1000);
  Serial.printf("\nStarting HeltecPager; reset reason %d\n",(int)esp_reset_reason());
  pinMode(PIN_ALERT,OUTPUT); digitalWrite(PIN_ALERT,LOW);
  pinMode(PIN_USER_LED,OUTPUT); digitalWrite(PIN_USER_LED,LOW);
  for(auto &b:buttons) pinMode(b.pin,INPUT_PULLUP);
  deviceId=ESP.getEfuseMac(); bootId=esp_random();
  snprintf(deviceName,sizeof(deviceName),"Pager-%04X",(unsigned)(deviceId&0xFFFF));
  loadStorage(); initializeOLED(); initializeBattery();
  SPI.begin(9,11,10,8); initializeRadio();
  updateLED(); lastInteraction=millis(); drawUI(); printStatus();
  Serial.println("Type /help. Set name with /name Amber, or Settings > Name.");
}
void loop() {
  serviceRadio(); pollButtons(); serviceSerial(); serviceBattery();
  uint32_t now=millis();
  if(inboxDirty && !txActive && !txQueued && (uint32_t)(now-inboxChangedAt)>=2000) saveInbox();
  if(notice[0] && (int32_t)(now-noticeUntil)>=0) { notice[0]=0; drawNeeded=true; }
  if(oledReady && !oledSleeping && (uint32_t)(now-lastInteraction)>=60000) {
    oled.ssd1306_command(SSD1306_DISPLAYOFF); oledSleeping=true;
  }
  if(drawNeeded && (uint32_t)(now-lastDraw)>=40) drawUI();
  delay(2);
}
