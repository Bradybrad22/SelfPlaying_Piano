/*
 * 61-Key Automatic Piano (C2~C7) using ESP32-C3 + 4×PCA9685 + UDP Trigger
 * ------------------------------------------------------------------
 * - 上电不播放；收到 UDP 文本里包含 WIFI_TRIGGER(默认 "S") 时，延迟 1 秒后播放一次
 * - 之后再收包也不会再播（one-shot）
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>

/* ------------ 前置结构声明 & 全局切换指针 ---------------- */
struct KeyMap { uint8_t driverIndex; uint8_t channel; };
struct NoteEvent;                       // 前向声明
extern const NoteEvent score[] PROGMEM; // 后面定义
extern const size_t SCORE_LEN;          // 后面定义

/* ================ 可配置宏区域 ================= */

#define MIDI_LOW   36    // C2
#define MIDI_HIGH  96    // C7 (包含)
#define KEY_COUNT  (MIDI_HIGH - MIDI_LOW + 1)
static const uint8_t BAD_KEYS[] = {59, 43}; // 坏键列表

inline bool isBadKey(uint8_t m){
  for(uint8_t k : BAD_KEYS) if(k == m) return true;
  return false;
}
inline uint8_t remapIfBad(uint8_t m){
  if(!isBadKey(m)) return m;
  uint8_t up = (m < MIDI_HIGH)? (uint8_t)(m+1) : m;
  if(!isBadKey(up) && up <= MIDI_HIGH) return up;
  uint8_t down = (m > MIDI_LOW)? (uint8_t)(m-1) : m;
  if(!isBadKey(down) && down >= MIDI_LOW) return down;
  return m;
}

/* I2C & 模式 */
#define SDA_PIN   4
#define SCL_PIN   5
#define I2C_FREQ  400000UL
#define CALIBRATION_MODE 1

/* 舵机脉宽极限 (需根据实际舵机微调) */
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
#define REST_PULSE_US 600

/* 短音策略与恢复窗口 */
#define MIN_HOLD_MS                55
#define SHORT_NOTE_THRESHOLD_MS    150
#define SHORT_NOTE_HOLD_SCALE_NUM  70
#define SHORT_NOTE_HOLD_SCALE_DEN  100
#define KEY_RECOVERY_EXTRA_MS      40

/* 行程角度参数 */
#define SERVO_US_RANGE      (SERVO_MAX_US - SERVO_MIN_US)
#define SERVO_FULL_DEG      180
#define KEY_PRESS_DEG       18
#define BASE_PRESS_DELTA_US ((SERVO_US_RANGE * KEY_PRESS_DEG)/SERVO_FULL_DEG)

/* 力度映射 */
#define VELOCITY_EXTRA_MIN 0
#define VELOCITY_EXTRA_MAX 200

/* 活动音容量 */
#define MAX_ACTIVE_NOTES 20

/* ======= 键重复/压力测试配置 ======= */
#define TEST_ENABLE_PHASE_SINGLE   1
#define TEST_ENABLE_PHASE_RAPID1   1
#define TEST_ENABLE_PHASE_RAPID2   1
#define TEST_ENABLE_PHASE_CHORDS   1
#define TEST_SKIP_BAD_KEYS         1

#define TEST_SINGLE_HOLD_MS        260
#define TEST_SINGLE_GAP_MS         320

#define TEST_RAPID1_REPEAT_COUNT   4
#define TEST_RAPID1_INTERVAL_MS    120
#define TEST_RAPID1_HOLD_MS        85
#define TEST_RAPID1_BLOCK_GAP_MS   60

#define TEST_RAPID2_REPEAT_COUNT   5
#define TEST_RAPID2_INTERVAL_MS    90
#define TEST_RAPID2_HOLD_MS        60
#define TEST_RAPID2_BLOCK_GAP_MS   40

#define TEST_CHORD_HOLD_MS         180
#define TEST_CHORD_GAP_MS          300

#define TEST_MAX_EVENTS            800

/* ================ WiFi / UDP 配置 ================= */
#define WIFI_SSID     "ESAPROBO"
#define WIFI_PASS     "shimmyshimmy"
#define UDP_PORT      2808
#define WIFI_TRIGGER  "S"   // 收到该字符串就播放一次

// 是否使用静态 IP（0=DHCP，1=静态）
#define USE_STATIC_IP 1
#if USE_STATIC_IP
IPAddress localIP (192,168,1,87);
IPAddress gateway (192,168,0,1);
IPAddress subnet  (255,255,255,0);
IPAddress dns1    (192,168,0,1);
#endif

WiFiUDP UDPServer;

/* ================ 数据结构 ================== */
struct NoteEvent {
  uint32_t startMs;
  uint16_t durMs;
  uint8_t  midi;
  uint8_t  vel;
};

struct ActiveNote { uint8_t midi; uint32_t offTime; bool active; };

/* ================ PCA9685 简化封装 ================ */
class PCA9685 {
public:
   PCA9685(uint8_t addr=0x40):_addr(addr){}
  void begin(){
    write8(0x00, 0x00);
    delay(5);
    setPWMFreq(50);
  }
  void setPWMFreq(float freq){
    float prescaleval = (25000000.0f / (4096.0f * freq)) - 1.0f;
    uint8_t prescale = (uint8_t)floor(prescaleval + 0.5f);
    uint8_t oldmode = read8(0x00);
    uint8_t sleep   = (oldmode & 0x7F) | 0x10;
    write8(0x00, sleep);
    write8(0xFE, prescale);
    write8(0x00, oldmode);
    delay(5);
    write8(0x00, oldmode | 0xA1);
  }
  void setPWM(uint8_t channel, uint16_t on, uint16_t off){
    uint8_t reg = 0x06 + 4*channel;
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(on & 0xFF);
    Wire.write(on >> 8);
    Wire.write(off & 0xFF);
    Wire.write(off >> 8);
    Wire.endTransmission();
  }
private:
  uint8_t _addr;
  void write8(uint8_t reg, uint8_t val){
    Wire.beginTransmission(_addr); Wire.write(reg); Wire.write(val); Wire.endTransmission();
  }
  uint8_t read8(uint8_t reg){
    Wire.beginTransmission(_addr); Wire.write(reg); Wire.endTransmission();
    Wire.requestFrom((int)_addr, 1);
    if(Wire.available()) return Wire.read();
    return 0xFF;
  }
};

/* ================ 全局变量 ================== */
static const uint8_t driverAddrs[4] = {0x40,0x41,0x42,0x44};
PCA9685 drivers[4] = { {0x40},{0x41},{0x42},{0x44} };

int16_t  pressOffset[KEY_COUNT] = {0};
uint8_t  velocityPercent = 60;
uint16_t tempoPercent    = 100;
bool     paused          = true;
uint32_t startMillis     = 0;
size_t   currentEventIndex = 0;

volatile bool udpTriggerPending = false;   // UDP已收到并待执行
bool playedOnce = false;                   // 已经播过一次？
uint32_t triggerAtMs = 0;                  // 计划开始播放的时间戳(ms)

ActiveNote activeNotes[MAX_ACTIVE_NOTES];
uint32_t   keyRecoverUntil[KEY_COUNT] = {0};

/* 动态当前曲目指针 */
const NoteEvent* currentScore = nullptr;
size_t currentLen = 0;

/* 测试模式数据 */
static NoteEvent testScore[TEST_MAX_EVENTS];
static size_t    TEST_LEN = 0;
static bool      testScoreBuilt = false;
static uint16_t  skipCount[KEY_COUNT] = {0};
static uint16_t  hitCount[KEY_COUNT]  = {0};
static uint16_t  maxConcurrency = 0;
static const NoteEvent* savedScore = nullptr;
static size_t   savedLen = 0;
static size_t   savedIndex = 0;
static bool     inKeyTest = false;

/* 单键快速重复测试 */
static NoteEvent burstScore[128];
static size_t    burstLen = 0;
static bool      inBurst  = false;
static const NoteEvent* savedScoreBurst = nullptr;
static size_t    savedLenBurst  = 0;
static size_t    savedIdxBurst  = 0;

/* UDP 播放请求次数（已不用，可删） */
volatile uint16_t playRequests = 0;

/* ================ 工具函数 ================== */
inline int keyIndex(uint8_t midi){ return (midi < MIDI_LOW || midi > MIDI_HIGH)? -1 : (midi - MIDI_LOW); }

inline bool mapMidi(uint8_t midi, KeyMap &km){
  if(midi < MIDI_LOW || midi > MIDI_HIGH) return false;
  uint8_t idx = midi - MIDI_LOW;
  uint8_t driverIndex = idx / 16;
  uint8_t channel = idx % 16;
  if(driverIndex == 3 && channel > 12) return false; // 第4板只用 0..12
  km.driverIndex = driverIndex; km.channel = channel; return true;
}

inline uint16_t usToCount(uint16_t us){ return (uint32_t)us * 4096UL / 20000UL; }

void setServoPulse(uint8_t midi, uint16_t pulseUs){
  KeyMap km; if(!mapMidi(midi, km)) return;
  if(pulseUs < SERVO_MIN_US) pulseUs = SERVO_MIN_US;
  if(pulseUs > SERVO_MAX_US) pulseUs = SERVO_MAX_US;
  uint16_t off = usToCount(pulseUs);
  drivers[km.driverIndex].setPWM(km.channel, 0, off);
}

/* 计算按压脉宽 */
static uint16_t computePressPulse(uint8_t midi){
  int k = keyIndex(midi);
  if(k < 0) return REST_PULSE_US;
  uint16_t delta = BASE_PRESS_DELTA_US;
  int32_t pulse = REST_PULSE_US + delta;
  int32_t extra = (int32_t)VELOCITY_EXTRA_MIN +
                  (int32_t)velocityPercent * (VELOCITY_EXTRA_MAX - VELOCITY_EXTRA_MIN) / 100;
  pulse += extra;
  pulse += pressOffset[k];
  if(pulse < SERVO_MIN_US) pulse = SERVO_MIN_US;
  if(pulse > SERVO_MAX_US) pulse = SERVO_MAX_US;
  return (uint16_t)pulse;
}

/* ================ 整体键测试构建 & 统计（保持原样） ================ */
inline bool keyTestIsBad(uint8_t m){
#if TEST_SKIP_BAD_KEYS
  return isBadKey(m);
#else
  return false;
#endif
}
static void addTestNote(uint32_t start, uint16_t dur, uint8_t midi, uint8_t vel=80){
  if(TEST_LEN >= TEST_MAX_EVENTS) return;
  testScore[TEST_LEN++] = { start, dur, midi, vel };
}
static void resetTestStats(){
  memset(skipCount,0,sizeof(skipCount));
  memset(hitCount,0,sizeof(hitCount));
  maxConcurrency = 0;
}
static void buildTestScore(){
  if(testScoreBuilt) return;
  TEST_LEN = 0;
  uint32_t t=0;
  uint8_t keys[KEY_COUNT]; int kcount=0;
  for(uint8_t m=MIDI_LOW; m<=MIDI_HIGH; ++m){
    if(keyTestIsBad(m)) continue;
    keys[kcount++] = m;
  }
#if TEST_ENABLE_PHASE_SINGLE
  for(int i=0;i<kcount;i++){
    addTestNote(t, TEST_SINGLE_HOLD_MS, keys[i], 90);
    t += TEST_SINGLE_GAP_MS;
  }
#endif
#if TEST_ENABLE_PHASE_RAPID1
  for(int i=0;i<kcount;i++){
    uint32_t bs = t;
    for(int r=0;r<TEST_RAPID1_REPEAT_COUNT;r++){
      addTestNote(bs + r*TEST_RAPID1_INTERVAL_MS, TEST_RAPID1_HOLD_MS, keys[i], 85);
    }
    t = bs + (TEST_RAPID1_REPEAT_COUNT-1)*TEST_RAPID1_INTERVAL_MS + TEST_RAPID1_HOLD_MS + TEST_RAPID1_BLOCK_GAP_MS;
  }
#endif
#if TEST_ENABLE_PHASE_RAPID2
  for(int i=0;i<kcount;i++){
    uint32_t bs = t;
    for(int r=0;r<TEST_RAPID2_REPEAT_COUNT;r++){
      addTestNote(bs + r*TEST_RAPID2_INTERVAL_MS, TEST_RAPID2_HOLD_MS, keys[i], 80);
    }
    t = bs + (TEST_RAPID2_REPEAT_COUNT-1)*TEST_RAPID2_INTERVAL_MS + TEST_RAPID2_HOLD_MS + TEST_RAPID2_BLOCK_GAP_MS;
  }
#endif
#if TEST_ENABLE_PHASE_CHORDS
  auto chord = [&](std::initializer_list<int> idxList){
    for(int idx: idxList){
      if(idx>=0 && idx<kcount) addTestNote(t, TEST_CHORD_HOLD_MS, keys[idx], 95);
    }
    t += TEST_CHORD_GAP_MS;
  };
  chord({0,1,2,3,4,5});
  int mid = kcount/2;
  chord({mid-2,mid-1,mid,mid+1,mid+2});
  chord({kcount-6,kcount-5,kcount-4,kcount-3,kcount-2,kcount-1});
  chord({0, mid, kcount-1});
  for(int rep=0; rep<4; rep++){
    for(int j=0;j<10 && j<kcount;j++){
      addTestNote(t, 140, keys[j], 88);
      int idx2 = (j + mid/(rep+2)) % kcount;
      addTestNote(t, 140, keys[idx2], 88);
    }
    t += 280;
  }
#endif
  testScoreBuilt = true;
  Serial.printf("[TEST] build complete: events=%u span≈%lu ms (%.1f s)\n",
    (unsigned)TEST_LEN, (unsigned long)t, t/1000.0f);
}
static void startKeyTest(){
  if(inKeyTest) return;
  buildTestScore();
  resetTestStats();
  savedScore = currentScore;
  savedLen   = currentLen;
  savedIndex = currentEventIndex;
  currentScore = testScore;
  currentLen   = TEST_LEN;
  currentEventIndex = 0;
  paused = false;
  startMillis = millis();
  inKeyTest = true;
  Serial.println(F("[TEST] Key test started."));
}
static void stopKeyTest(bool resumeOriginal){
  if(!inKeyTest) return;
  if(resumeOriginal && savedScore){
    currentScore = savedScore;
    currentLen   = savedLen;
    currentEventIndex = savedIndex;
    paused = true;
    Serial.println(F("[TEST] Restored original score (paused)."));
  }
  inKeyTest = false;
}
static void printKeyStats(){
  Serial.println(F("MIDI:hit/skip"));
  uint16_t totalSkip=0,totalHit=0;
  for(int i=0;i<KEY_COUNT;i++){
    if(skipCount[i] || hitCount[i]){
      uint8_t m = MIDI_LOW + i;
      Serial.printf("%d:%u/%u  ", m, hitCount[i], skipCount[i]);
      totalSkip += skipCount[i]; totalHit += hitCount[i];
      if((i%8)==7) Serial.println();
    }
  }
  Serial.printf("\n[TEST] totalHit=%u totalSkip=%u maxConc=%u\n", totalHit, totalSkip, maxConcurrency);
}
static void maybeAutoReport(){
  if(inKeyTest && currentEventIndex >= currentLen){
    Serial.println(F("[TEST] Sequence finished."));
    printKeyStats();
  }
  if(inBurst && currentEventIndex >= currentLen){
    Serial.println(F("[F] 单键快速测试结束。输入 FX 可恢复原曲或继续查看。"));
  }
}

/* ============ 单键快速重复测试实现（保持原样） ============ */
static void startBurst(uint8_t midi, int repeat=20, int intervalMs=120, int holdMs=70){
  midi = remapIfBad(midi);
  if(isBadKey(midi) || keyIndex(midi)<0){
    Serial.println(F("[F] 该 MIDI 无法用于测试(坏键或超范围)。"));
    return;
  }
  if(repeat < 1) repeat = 1;
  if(repeat > 120) repeat = 120;
  if(intervalMs < 10) intervalMs = 10;
  if(intervalMs > 2000) intervalMs = 2000;
  if(holdMs < 20) holdMs = 20;
  if(holdMs > intervalMs) holdMs = intervalMs;

  savedScoreBurst = currentScore;
  savedLenBurst   = currentLen;
  savedIdxBurst   = currentEventIndex;

  burstLen = 0;
  uint32_t t = 0;
  for(int i=0;i<repeat && burstLen < (sizeof(burstScore)/sizeof(burstScore[0])-1); ++i){
    burstScore[burstLen++] = { t, (uint16_t)holdMs, midi, 90 };
    t += intervalMs;
  }
  burstScore[burstLen++] = { 0xFFFFFFFF, 0, 0, 0 };

  currentScore = burstScore;
  currentLen   = burstLen - 1;
  currentEventIndex = 0;
  startMillis = millis();
  paused = false;
  inBurst = true;
  Serial.printf("[F] 单键快速测试开始: midi=%u repeat=%d interval=%d hold=%d span≈%lu ms\n",
      midi, repeat, intervalMs, holdMs, (unsigned long)((repeat>0? (repeat-1)*intervalMs : 0) + holdMs));
}
static void stopBurst(bool restore){
  if(!inBurst) return;
  inBurst = false;
  if(restore && savedScoreBurst){
    currentScore = savedScoreBurst;
    currentLen   = savedLenBurst;
    currentEventIndex = savedIdxBurst;
    paused = true;
    Serial.println(F("[F] 已恢复原曲 (暂停)。"));
  }
}
static void burstStatus(){
  if(!inBurst){
    Serial.println(F("[F] 当前不在单键快速测试模式。"));
  } else {
    Serial.printf("[F] Burst 进度: idx=%u/%u\n",
      (unsigned)currentEventIndex, (unsigned)currentLen);
  }
}

/* ================ 核心播放逻辑 ================== */
uint32_t scaledTime(){
  if(paused) return startMillis;
  uint32_t elapsed = millis() - startMillis;
  return (uint64_t)elapsed * tempoPercent / 100ULL;
}

void noteOn(uint8_t midi, uint32_t now, uint32_t dur){
  midi = remapIfBad(midi);
  if(isBadKey(midi)) return;
  int k = keyIndex(midi); if(k<0) return;
  if(now < keyRecoverUntil[k]){ if(inKeyTest) skipCount[k]++; return; }

  int slot=-1; for(int i=0;i<MAX_ACTIVE_NOTES;i++) if(!activeNotes[i].active){ slot=i; break; }
  if(slot<0) return;

  uint32_t origDur = dur;
  uint16_t pressPulse = computePressPulse(midi);

  if(origDur < SHORT_NOTE_THRESHOLD_MS){
    uint32_t effectiveBase = (origDur > MIN_HOLD_MS)? (origDur - MIN_HOLD_MS) : 0;
    uint32_t scaledPart = (effectiveBase * SHORT_NOTE_HOLD_SCALE_NUM) / SHORT_NOTE_HOLD_SCALE_DEN;
    dur = MIN_HOLD_MS + scaledPart;
    if(dur > origDur) dur = origDur;
  }
  uint32_t hold = dur < MIN_HOLD_MS ? MIN_HOLD_MS : dur;

  setServoPulse(midi, pressPulse);
  activeNotes[slot] = { midi, now + hold, true };
  keyRecoverUntil[k] = now + hold + KEY_RECOVERY_EXTRA_MS;

  if(inKeyTest){
    hitCount[k]++;
    uint16_t conc=0; for(int i=0;i<MAX_ACTIVE_NOTES;i++) if(activeNotes[i].active) conc++;
    if(conc > maxConcurrency) maxConcurrency = conc;
  }
}

void scanNoteOff(uint32_t now){
  for(int i=0;i<MAX_ACTIVE_NOTES;i++){
    if(activeNotes[i].active && now >= activeNotes[i].offTime){
      setServoPulse(activeNotes[i].midi, REST_PULSE_US);
      activeNotes[i].active = false;
    }
  }
}

void dispatch(){
  uint32_t vtime = scaledTime();
  while(currentEventIndex < currentLen){
    NoteEvent ev;
    memcpy_P(&ev, &currentScore[currentEventIndex], sizeof(NoteEvent));
    if(ev.startMs <= vtime){
      uint32_t durScaled = (uint64_t)ev.durMs * 100 / tempoPercent;
      noteOn(ev.midi, millis(), durScaled);
      currentEventIndex++;
    } else break;
  }
}

void restartPlayback(){
  currentEventIndex = 0;
  startMillis = millis();
  paused = false;
  for(auto &a: activeNotes){
    if(a.active){
      setServoPulse(a.midi, REST_PULSE_US);
      a.active=false;
    }
  }
}

/* 复位所有舵机并暂停 */
void resetAllServos(bool restart=false){
  paused = true;
  for(int i=0;i<MAX_ACTIVE_NOTES;i++){
    if(activeNotes[i].active){
      setServoPulse(activeNotes[i].midi, REST_PULSE_US);
      activeNotes[i].active=false;
    }
  }
  for(uint8_t m=MIDI_LOW; m<=MIDI_HIGH; ++m) setServoPulse(m, REST_PULSE_US);
  Serial.println(F("[X] All servos reset. Playback paused."));
  if(restart){
    restartPlayback();
    Serial.println(F("[X] Restart after reset."));
  }
}

/* ================ 示例主曲谱 (score[]) ================== */
const NoteEvent score[] PROGMEM = {
  {0,283,72,82},
    {300,141,72,56},
    {450,141,72,62},
    {600,283,72,62},
    {900,141,72,80},
    {1050,141,72,80},
    {1200,283,72,80},
    {1500,141,72,80},
    {1650,141,72,80},
    {1800,283,72,80},
    {2100,283,71,80},
    {2400,283,72,82},
    {2700,141,72,56},
    {2850,141,72,62},
    {3000,283,72,62},
    {3300,141,72,80},
    {3450,141,72,80},
    {3600,283,72,80},
    {3900,141,72,80},
    {4050,141,72,80},
    {4200,283,72,80},
    {4500,298,30,96},
    {4500,298,42,63},
    {4500,283,71,80},
    {4800,1798,35,106},
    {4800,1798,47,51},
    {4800,298,60,82},
    {4800,283,72,82},
    {5100,73,60,56},
    {5100,141,72,56},
    {5250,73,60,62},
    {5250,141,72,62},
    {5400,298,60,62},
    {5400,283,72,62},
    {5700,73,60,80},
    {5700,141,72,80},
    {5850,73,60,80},
    {5850,141,72,80},
    {6000,298,60,80},
    {6000,283,72,80},
    {6300,73,60,80},
    {6300,141,72,80},
    {6450,73,60,80},
    {6450,141,72,80},
    {6600,298,60,80},
    {6600,283,72,80},
    {6900,298,30,96},
    {6900,298,42,63},
    {6900,148,59,80},
    {6900,283,71,80},
    {7200,1708,35,106},
    {7200,1798,35,106},
    {7200,1708,47,51},
    {7200,1798,47,51},
    {7200,283,72,82},
    {7500,141,72,56},
    {7650,141,72,62},
    {7800,283,72,62},
    {8100,141,72,80},
    {8250,141,72,80},
    {8400,283,72,80},
    {8700,141,72,80},
    {8850,141,72,80},
    {9000,283,72,80},
    {9300,283,30,96},
    {9300,298,30,96},
    {9300,283,42,63},
    {9300,298,42,63},
    {9300,283,71,80},
    {9600,1708,35,106},
    {9600,1798,35,106},
    {9600,1708,47,51},
    {9600,1798,47,51},
    {9600,283,72,82},
    {9900,141,72,56},
    {10050,141,72,62},
    {10200,283,72,62},
    {10500,141,72,80},
    {10650,141,72,80},
    {10800,283,72,80},
    {11100,141,72,80},
    {11250,141,72,80},
    {11400,283,72,80},
    {11700,283,30,96},
    {11700,298,30,96},
    {11700,283,42,63},
    {11700,298,42,63},
    {11700,283,71,80},
    {12000,1708,35,106},
    {12000,1798,35,106},
    {12000,1708,47,51},
    {12000,1798,47,51},
    {12000,283,72,82},
    {12300,141,72,56},
    {12450,141,72,62},
    {12600,283,72,62},
    {12900,141,72,80},
    {13050,141,72,80},
    {13200,283,72,80},
    {13500,141,72,80},
    {13650,141,72,80},
    {13800,283,72,80},
    {14100,283,30,96},
    {14100,298,30,96},
    {14100,283,42,63},
    {14100,298,42,63},
    {14100,283,71,80},
    {24000,1708,35,106},
    {24000,1798,35,106},
    {24000,1708,47,51},
    {24000,1798,47,51},
    {24000,283,72,82},
    {24300,141,72,56},
    {24450,141,72,62},
    {24600,283,72,62},
    {24900,141,72,80},
    {25050,141,72,80},
    {25200,283,72,80},
    {25500,141,72,80},
    {25650,141,72,80},
    {25800,283,72,80},
    {26100,283,30,96},
    {26100,298,30,96},
    {26100,283,42,63},
    {26100,298,42,63},
    {26100,283,71,80},
    {26400,1708,35,106},
    {26400,1798,35,106},
    {26400,1708,47,51},
    {26400,1798,47,51},
    {26400,283,72,82},
    {26700,141,72,56},
    {26850,141,72,62},
    {27000,283,72,62},
    {27300,141,72,80},
    {27450,141,72,80},
    {27600,283,72,80},
    {27900,141,72,80},
    {28050,141,72,80},
    {28200,283,72,80},
    {28500,283,30,96},
    {28500,298,30,96},
    {28500,283,42,63},
    {28500,298,42,63},
    {28500,283,71,80},
    {28800,1708,35,106},
    {28800,1798,35,106},
    {28800,1708,47,51},
    {28800,1798,47,51},
    {28800,283,72,82},
    {29100,141,72,56},
    {29250,141,72,62},
    {29400,283,72,62},
    {29700,141,72,80},
    {29850,141,72,80},
    {30000,283,72,80},
    {30300,141,72,80},
    {30450,141,72,80},
    {30600,283,72,80},
    {30900,283,30,96},
    {30900,298,30,96},
    {30900,283,42,63},
    {30900,298,42,63},
    {30900,283,71,80},
    {31200,1708,35,106},
    {31200,1798,35,106},
    {31200,1708,47,51},
    {31200,1798,47,51},
    {31200,283,72,82},
    {31500,141,72,56},
    {31650,141,72,62},
    {31800,283,72,62},
    {32100,141,72,80},
    {32250,141,72,80},
    {32400,283,72,80},
    {32700,141,72,80},
    {32850,141,72,80},
    {33000,283,72,80},
    {33300,283,71,80},
    {0xFFFFFFFF,0,0,0}
};
const size_t SCORE_LEN = (sizeof(score)/sizeof(score[0])) - 1;

/* ================ 串口命令处理（原样） ================== */
void handleSerial(){
  if(!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim(); if(!line.length()) return;
  int sp = line.indexOf(' ');
  String cmd = (sp<0)? line : line.substring(0, sp);
  cmd.toUpperCase();
  String rest = (sp<0)? "" : line.substring(sp+1);

  if(cmd=="S"){
    restartPlayback(); Serial.println(F("[S] Restart."));
  } else if(cmd=="P"){
    paused = !paused;
    if(!paused){
      uint32_t vt = scaledTime();
      startMillis = millis() - (uint64_t)vt * 100 / tempoPercent;
    }
    Serial.printf("[P] %s\n", paused?"Paused":"Resume");
  } else if(cmd=="B"){
    int v = rest.toInt(); if(v<30) v=30; if(v>400) v=400; tempoPercent=v;
    Serial.printf("[B] tempo=%u\n", tempoPercent);
  } else if(cmd=="V"){
    int v = rest.toInt(); if(v<0) v=0; if(v>100) v=100; velocityPercent=v;
    Serial.printf("[V] velocity=%u\n", velocityPercent);
  } else if(cmd=="L"){
    Serial.printf("[L] idx=%u/%u paused=%d tempo=%u vel=%u virtualT=%lu ms\n",
      (unsigned)currentEventIndex,(unsigned)currentLen,paused,tempoPercent,velocityPercent,(unsigned long)scaledTime());
  } else if(cmd=="T"){
#if CALIBRATION_MODE
    int m = rest.toInt();
    if(keyIndex(m)>=0){ noteOn(m, millis(), 120); Serial.printf("[T] midi %d\n", m); }
    else Serial.println(F("[T] invalid midi"));
#endif
  } else if(cmd=="O"){
#if CALIBRATION_MODE
    int sp2 = rest.indexOf(' ');
    if(sp2>0){
      int m = rest.substring(0,sp2).toInt();
      int d = rest.substring(sp2+1).toInt();
      int k = keyIndex(m);
      if(k>=0){ pressOffset[k]+=d; Serial.printf("[O] midi %d offset=%d\n", m, pressOffset[k]); }
      else Serial.println(F("[O] invalid midi"));
    }
#endif
  } else if(cmd=="W"){
#if CALIBRATION_MODE
    for(int i=0;i<KEY_COUNT;i++) if(pressOffset[i]!=0)
      Serial.printf("midi %d : %d\n", i+MIDI_LOW, pressOffset[i]);
#endif
  } else if(cmd=="R"){
#if CALIBRATION_MODE
    for(int i=0;i<KEY_COUNT;i++) pressOffset[i]=0;
    Serial.println(F("[R] offsets cleared"));
#endif
  } else if(cmd=="X"){
    bool r=false;
    if(rest.length()){ rest.toUpperCase(); if(rest=="R") r=true; }
    resetAllServos(r);
  } else if(cmd=="K"){
    startKeyTest();
  } else if(cmd=="KX"){
    stopKeyTest(true);
  } else if(cmd=="KS"){
    printKeyStats();
  } else if(cmd=="KR"){
    if(inKeyTest){
      resetTestStats();
      currentEventIndex=0; paused=false; startMillis=millis();
      Serial.println(F("[TEST] Stats reset & restarted."));
    } else Serial.println(F("[TEST] Not in test mode."));
  } else if(cmd=="KD"){
    Serial.printf("[TEST] Built=%d events=%u inTest=%d\n",
      testScoreBuilt,(unsigned)TEST_LEN,inKeyTest);

  } else if(cmd=="F"){
    int nums[4]; int n=0;
    int from=0;
    while(from < (int)rest.length() && n<4){
      int spc = rest.indexOf(' ', from);
      String token = (spc<0)? rest.substring(from) : rest.substring(from, spc);
      token.trim();
      if(token.length()){
        nums[n++] = token.toInt();
      }
      if(spc<0) break;
      from = spc+1;
    }
    if(n < 1){
      Serial.println(F("[F] 用法: F <midi> [repeat] [intervalMs] [holdMs]"));
    } else {
      startBurst( (uint8_t)nums[0],
                  (n>1 && nums[1]>0)?nums[1]:20,
                  (n>2 && nums[2]>0)?nums[2]:120,
                  (n>3 && nums[3]>0)?nums[3]:70 );
    }
  } else if(cmd=="FX"){
    stopBurst(true);
  } else if(cmd=="FS"){
    burstStatus();

  } else {
    Serial.println(F("[?] Unknown command"));
  }
}

/* ================ UDP 触发处理 ================== */
void handleUDPTrigger() {
  char packetBuffer[128];
  int cb = UDPServer.parsePacket();
  if (cb > 0) {
    int len = UDPServer.read(packetBuffer, sizeof(packetBuffer)-1);
    if (len <= 0) return;
    packetBuffer[len] = 0;

    // 去掉尾部 \r\n
    for (int i=len-1; i>=0; --i){
      if(packetBuffer[i]=='\r' || packetBuffer[i]=='\n') packetBuffer[i]=0;
      else break;
    }

    Serial.printf("[UDP] payload:'%s'\n", packetBuffer);

    if (!playedOnce && strstr(packetBuffer, WIFI_TRIGGER)) {
      udpTriggerPending = true;
      triggerAtMs = millis() + 900;   // 延迟 1 秒启动
      Serial.println("[UDP] trigger matched! will start in 1s");
    }
  }
}

/* ================ Setup & Loop ================== */

void setup(){
  Serial.begin(115200);
  while(!Serial && millis()<2000){}

  Serial.println(F("\n[Init] 61-Key Piano Controller"));

  // I2C & PCA
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_FREQ);
  for(int i=0;i<4;i++){
    Serial.printf("[Init] Driver %d @0x%02X...", i, driverAddrs[i]);
    drivers[i].begin();
    Serial.println(F("OK"));
  }
  for(uint8_t m=MIDI_LOW; m<=MIDI_HIGH; ++m) setServoPulse(m, REST_PULSE_US);
  memset(activeNotes,0,sizeof(activeNotes));

  currentScore = score;
  currentLen   = SCORE_LEN;

  // 不自动播放
  paused = true;
  currentEventIndex = currentLen; // 视为已播完

  // WiFi
  Serial.println(F("[WiFi] Connecting..."));
  WiFi.mode(WIFI_STA);
#if USE_STATIC_IP
  WiFi.config(localIP, gateway, subnet, dns1);
#endif
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WiFi] OK IP=%s RSSI=%d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
    UDPServer.begin(UDP_PORT);
    Serial.printf("[UDP] Listening on %u\n", UDP_PORT);
  } else {
    Serial.printf("[WiFi] FAIL status=%d (continue offline)\n", WiFi.status());
  }

  Serial.println(F("[Ready] Commands: S P B V L T O W R X K KX KS KR KD F FX FS"));
  Serial.printf("[Ready] Waiting UDP '%s' to start.\n", WIFI_TRIGGER);
}

void loop(){
  handleSerial();
  handleUDPTrigger();

  // 只允许第一次 UDP 触发，且延迟 1 秒后启动
  if (paused && udpTriggerPending && !playedOnce && !inKeyTest && !inBurst && millis() >= triggerAtMs) {
    udpTriggerPending = false;
    playedOnce = true;
    restartPlayback();
    paused = false;
    Serial.println(F("[PLAY] Start after 1s delay (one-shot)."));
  }

  if(!paused) dispatch();
  scanNoteOff(millis());
  maybeAutoReport();

  // 播放结束自动停（仅主曲目）
  if(!paused && !inKeyTest && !inBurst && currentEventIndex >= currentLen){
    paused = true;
    Serial.println(F("[DONE] Song finished. Waiting next UDP (ignored)."));
  }
}
