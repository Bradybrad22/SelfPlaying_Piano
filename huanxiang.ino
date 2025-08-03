/*
 * 61-Key Automatic Piano (C2~C7) using ESP32-C3 + 4×PCA9685
 * ------------------------------------------------------------------
 * 概述：ESP32-C3 通过 4 块 PCA9685 控制 61 路伺服舵机，实现自动弹奏。
 * 功能：事件调度、速度(tempo)缩放、力度(脉宽附加)、单键校准、快速重复测试、复位命令、单键突发(Burst)快速能力测试。
 * 特色：短音符自动缩短保持时间 + 15° 减行程策略，预留扩展（踏板/多曲目/NVS）。
 */

#include <Arduino.h>
#include <Wire.h>

/* ------------ 前置结构声明 & 全局切换指针 ---------------- */
struct KeyMap { uint8_t driverIndex; uint8_t channel; };
struct NoteEvent;                       // 前向声明
extern const NoteEvent score[] PROGMEM; // 后面定义
extern const size_t SCORE_LEN;          // 后面定义

/* ================ 可配置宏区域 ================= */

// (坏键 + MIDI 范围) —— remap/跳过使用
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
  return m; // 保持 -> 上层可能直接忽略
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

/* 行程角度参数 (按压角度=18° 可调) */
#define SERVO_US_RANGE      (SERVO_MAX_US - SERVO_MIN_US)
#define SERVO_FULL_DEG      180
#define KEY_PRESS_DEG       18
#define BASE_PRESS_DELTA_US ((SERVO_US_RANGE * KEY_PRESS_DEG)/SERVO_FULL_DEG)

/* 力度映射 (线性加成) */
#define VELOCITY_EXTRA_MIN 0
#define VELOCITY_EXTRA_MAX 200

/* 活动音容量 */
#define MAX_ACTIVE_NOTES 20

/* ========== 键重复/压力测试配置 (整套遍历测试) ========== */
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
    write8(0x00, oldmode | 0xA1); // Auto-Inc + ALLCALL
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
bool     paused          = false;
uint32_t startMillis     = 0;
size_t   currentEventIndex = 0;

ActiveNote activeNotes[MAX_ACTIVE_NOTES];
uint32_t   keyRecoverUntil[KEY_COUNT] = {0};

/* 动态当前曲目指针 */
const NoteEvent* currentScore = nullptr;
size_t currentLen = 0;

/* 测试模式数据（全键测试） */
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

/* ============ 单键快速重复测试 (Burst) ============ */
static NoteEvent burstScore[128];
static size_t    burstLen = 0;
static bool      inBurst  = false;
static const NoteEvent* savedScoreBurst = nullptr;
static size_t    savedLenBurst  = 0;
static size_t    savedIdxBurst  = 0;

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

/* ================ 整体键测试构建 & 统计 ================ */
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

/* ============ 单键快速重复测试实现 ============ */
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
/* 这里仅保留哨兵；请将你生成的数组粘贴在此处（哨兵前）。 */
const NoteEvent score[] PROGMEM = {
  {0,2730,56,79},
    {0,2730,44,79},
    {2730,114,49,75},
    {2730,114,37,75},
    {2844,114,56,60},
    {2958,114,61,64},
    {3072,114,64,65},
    {3185,114,61,62},
    {3299,114,56,61},
    {3413,114,49,57},
    {3527,114,56,58},
    {3641,114,61,60},
    {3754,114,64,60},
    {3868,114,61,58},
    {3982,114,56,57},
    {4096,114,49,53},
    {4209,114,56,54},
    {4323,114,61,55},
    {4437,114,64,54},
    {4551,114,61,52},
    {4664,114,56,50},
    {4778,114,49,45},
    {4892,114,56,47},
    {5006,114,61,49},
    {5119,114,64,46},
    {5233,114,61,43},
    {5347,114,56,38},
    {5461,114,49,27},
    {5546,85,68,37},
    {5575,114,56,30},
    {5631,85,69,41},
    {5688,114,61,33},
    {5717,85,68,38},
    {5802,85,67,39},
    {5802,114,64,36},
    {5887,85,68,40},
    {5916,114,61,34},
    {5973,85,73,42},
    {6030,114,56,32},
    {6058,85,76,43},
    {6143,85,75,45},
    {6143,114,52,31},
    {6229,85,73,43},
    {6257,114,56,33},
    {6314,85,75,45},
    {6371,114,61,36},
    {6399,85,73,43},
    {6485,85,72,44},
    {6485,114,64,37},
    {6570,85,73,46},
    {6598,114,61,34},
    {6655,85,76,49},
    {6712,114,56,32},
    {6741,85,80,47},
    {6826,114,49,32},
    {6911,85,68,45},
    {6940,114,56,36},
    {6997,85,69,49},
    {7053,114,61,40},
    {7082,85,68,45},
    {7167,85,67,46},
    {7167,114,64,43},
    {7253,85,68,48},
    {7281,114,61,41},
    {7338,85,73,50},
    {7395,114,56,39},
    {7423,85,76,51},
    {7509,85,75,54},
    {7509,114,52,37},
    {7594,85,73,51},
    {7622,114,56,39},
    {7679,85,75,53},
    {7736,114,61,43},
    {7765,85,73,51},
    {7850,85,72,52},
    {7850,114,64,44},
    {7935,85,73,54},
    {7964,114,61,41},
    {8020,85,76,57},
    {8077,114,56,38},
    {8106,85,80,55},
    {8191,114,51,37},
    {8276,85,69,55},
    {8305,114,57,42},
    {8362,85,73,55},
    {8419,114,61,47},
    {8447,85,75,58},
    {8532,85,78,61},
    {8532,114,66,53},
    {8618,85,81,64},
    {8646,114,61,47},
    {8703,85,85,67},
    {8760,114,57,44},
    {8788,85,87,70},
    {8874,85,95,74},
    {8874,114,54,44},
    {8959,85,93,72},
    {8987,114,61,48},
    {9044,85,92,71},
    {9101,114,63,52},
    {9130,85,90,70},
    {9215,85,88,69},
    {9215,114,69,55},
    {9300,85,87,68},
    {9329,114,63,52},
    {9386,85,90,67},
    {9443,114,61,49},
    {9471,85,85,66},
    {9556,85,84,68},
    {9556,114,44,39},
    {9642,85,87,63},
    {9670,114,51,42},
    {9727,85,81,63},
    {9784,114,54,45},
    {9812,85,80,63},
    {9898,85,78,60},
    {9898,114,60,46},
    {9983,85,81,58},
    {10011,114,54,43},
    {10068,85,76,54},
    {10125,114,51,40},
    {10154,85,75,54},
    {10239,85,78,53},
    {10239,114,44,35},
    {10324,85,73,48},
    {10353,114,51,37},
    {10410,85,72,48},
    {10466,114,54,39},
    {10495,85,75,47},
    {10580,85,69,45},
    {10580,114,60,39},
    {0xFFFFFFFF,0,0,0}
};

/* 自动计算长度（不包含哨兵） */
const size_t SCORE_LEN = (sizeof(score)/sizeof(score[0])) - 1;

/* ================ 串口命令处理 ================== */
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

  } else if(cmd=="F"){          // 单键快速突发测试 F <midi> [repeat] [interval] [hold]
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

/* ================ Setup & Loop ================== */
void setup(){
  Serial.begin(115200);
  while(!Serial && millis()<2000){}
  Serial.println(F("\n[Init] 61-Key Piano Controller"));

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
  restartPlayback();

  Serial.println(F("[Ready] Commands: S P B V L T O W R X K KX KS KR KD F FX FS"));
}

void loop(){
  handleSerial();
  if(!paused) dispatch();
  scanNoteOff(millis());
  maybeAutoReport();
}
