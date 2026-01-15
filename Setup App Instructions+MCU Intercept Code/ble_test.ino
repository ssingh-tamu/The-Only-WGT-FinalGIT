// ---------- Forward declarations to placate Arduino's auto-prototyper ----------
struct ParsedStd;
struct ArbMeta;

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Arduino.h>
#include <string>
#include <utility>          // std::forward
#include <esp_heap_caps.h>  // heap_caps_malloc (PSRAM)

// ---------- NUS UUIDs ----------
static BLEUUID SERVICE_UUID("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
static BLEUUID RX_UUID     ("6e400002-b5a3-f393-e0a9-e50e24dcca9e");   // phone -> ESP32 (Write/WriteNR)
static BLEUUID TX_UUID     ("6e400003-b5a3-f393-e0a9-e50e24dcca9e");   // ESP32 -> phone (Notify)

#define SEND_ACK    1
#define HEARTBEAT   0          // keep quiet during plotting

// ---------- Plot controls ----------
#define PLOT_STRIDE 8          // 1 = output every point; larger = decimate

// ---------- Quiet logging (only numeric lines during plot) ----------
static bool g_quiet = false;

template <typename... Args>
void QPRINT(Args&&... args) { if (!g_quiet) Serial.print(std::forward<Args>(args)...); }

template <typename... Args>
void QPRINTLN(Args&&... args) { if (!g_quiet) Serial.println(std::forward<Args>(args)...); }

// ---------- BLE ----------
BLECharacteristic* txChar = nullptr;
unsigned long lastBeat = 0;

// ---------- ARB session state ----------
static const size_t SRC_FULL = 65536;        // source index space coming from phone

// Dynamic buffers (prefer PSRAM)
static int16_t* g_lut    = nullptr;          // Q15 samples (capacity decided at runtime)
static uint8_t* g_bitmap = nullptr;          // bit per destination sample
static size_t   g_capacity = 0;              // actual LUT size allocated (<= 65536)

static uint32_t g_unique_received = 0;       // unique dest samples written
static bool     g_plottedOnce     = false;   // avoid re-plotting

// ---------- Structs ----------
struct ParsedStd {
  uint8_t  version, msgType;
  uint16_t seq;
  uint8_t  shape;
  uint32_t freq_mHz;
  uint16_t amp_mVpp;
  int32_t  offset_mV;
  int32_t  phase_mdeg;
  uint8_t  channel;
};

struct ArbMeta {
  uint32_t freq_mHz   = 0;
  uint16_t amp_mVpp   = 0;
  int32_t  offset_mV  = 0;
  int32_t  phase_mdeg = 0;
  uint8_t  channel    = 0;
  uint16_t seq        = 0;
  bool     active     = false;
} g_meta;

static int16_t g_q15_min =  32767;
static int16_t g_q15_max = -32768;

// ---------- Utilities ----------
static void hexDump(const uint8_t* p, size_t n, const char* tag) {
  QPRINT(tag); QPRINT(" ["); QPRINT(n); QPRINTLN(" bytes]:");
  for (size_t i = 0; i < n; ++i) {
    if ((i % 16) == 0) QPRINT("  ");
    if (p[i] < 16) QPRINT('0');
    QPRINT(String(p[i], HEX)); QPRINT(' ');
    if ((i % 16) == 15 || i + 1 == n) QPRINTLN("");
  }
}

static uint16_t rd_u16_le(const uint8_t* p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static uint32_t rd_u32_le(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static int32_t rd_i32_le(const uint8_t* p) {
  return (int32_t)rd_u32_le(p);
}

static const char* shapeName(uint8_t code) {
  switch (code) { case 0: return "sine"; case 1: return "square"; case 2: return "triangle"; case 3: return "sinc"; default: return "unknown"; }
}
static const char* channelName(uint8_t code) {
  switch (code) { case 0: return "A"; case 1: return "B"; default: return "?"; }
}

// ---------- Standard 32B packet (MsgType=0x00) ----------
static bool parseStd32(const uint8_t* buf, size_t len, ParsedStd& out) {
  if (len != 32) return false;
  if (buf[0] != 0x01) return false;
  if (buf[1] != 0x00) return false;
  out.version = buf[0];
  out.msgType = buf[1];
  out.seq = rd_u16_le(buf + 2);
  size_t i = 4;
  while (i + 2 <= len) {
    uint8_t T = buf[i], L = buf[i + 1]; i += 2;
    if (i + L > len) return false;
    const uint8_t* V = buf + i;
    switch (T) {
      case 0x01: if (L != 1) return false; out.shape = V[0]; break;
      case 0x02: if (L != 4) return false; out.freq_mHz = rd_u32_le(V); break;
      case 0x03: if (L != 2) return false; out.amp_mVpp = rd_u16_le(V); break;
      case 0x04: if (L != 4) return false; out.offset_mV = rd_i32_le(V); break;
      case 0x05: if (L != 4) return false; out.phase_mdeg = rd_i32_le(V); break;
      case 0x06: if (L != 1) return false; out.channel = V[0]; break;
    }
    i += L;
    if (i == len) break;
  }
  return true;
}

// ---------- ARB META (MsgType=0x01) ----------
static bool parseArbMeta(const uint8_t* buf, size_t len, ArbMeta& m) {
  if (len != 32) { QPRINT("Bad ARB META len="); QPRINTLN((int)len); return false; }
  if (buf[0] != 0x01 || buf[1] != 0x01) { QPRINTLN("Bad header (expect ver=1,msg=1)"); return false; }
  m.seq = rd_u16_le(buf + 2);
  size_t i = 4;
  while (i + 2 <= len) {
    uint8_t T = buf[i], L = buf[i + 1]; i += 2;
    if (i + L > len) { QPRINTLN("META TLV OOB"); return false; }
    const uint8_t* V = buf + i;
    switch (T) {
      case 0x01: /*shape (ignored for ARB)*/ break;
      case 0x02: if (L != 4) return false; m.freq_mHz = rd_u32_le(V); break;
      case 0x03: if (L != 2) return false; m.amp_mVpp = rd_u16_le(V); break;
      case 0x04: if (L != 4) return false; m.offset_mV = rd_i32_le(V); break;
      case 0x05: if (L != 4) return false; m.phase_mdeg = rd_i32_le(V); break;
      case 0x06: if (L != 1) return false; m.channel = V[0]; break;
      default: break;
    }
    i += L;
    if (i == len) break;
  }
  return true;
}

// ---------- ARB DATA (MsgType=0x02) ----------
struct ArbBlockHdr { uint16_t start_idx; uint16_t count; };

static bool parseArbDataHeader(const uint8_t* buf, size_t len, uint16_t& seq, uint8_t& T, uint8_t& L) {
  if (len < 6) return false;
  if (buf[0] != 0x01 || buf[1] != 0x02) { QPRINTLN("Bad header (expect ver=1,msg=2)"); return false; }
  seq = rd_u16_le(buf + 2);
  T = buf[4];
  L = buf[5];
  if (T != 0x20) { QPRINT("Bad TLV T=0x"); QPRINTLN(String(T, HEX)); return false; }
  if (L < 4) { QPRINTLN("Bad TLV L (must be >=4)"); return false; }
  return true;
}

// ---------- bitmap helpers over dest capacity ----------
static inline bool testBit(uint32_t idx) { return (g_bitmap[idx >> 3] >> (idx & 7)) & 1; }
static inline void setBit (uint32_t idx) { g_bitmap[idx >> 3] |= (uint8_t)(1u << (idx & 7)); }

// ---------- Map SRC_FULL indices to dest capacity (even decimation) ----------
static inline uint32_t mapSrcToDst(uint32_t srcIndex) {
  // Scale 0..65535 -> 0..(g_capacity-1)
  // Use 64-bit math to avoid rounding bias.
  uint64_t num = (uint64_t)srcIndex * (uint64_t)g_capacity;
  uint32_t dst = (uint32_t)(num / (uint64_t)SRC_FULL);
  if (dst >= g_capacity) dst = g_capacity - 1;
  return dst;
}

// ---------- Apply ARB block with on-the-fly decimation to fit g_capacity ----------
static void applyArbBlock(uint16_t start_idx, uint16_t count, const int16_t* samples) {
  if (!g_lut || !g_bitmap || g_capacity == 0) return;

  uint32_t prevDst = UINT32_MAX;
  for (uint16_t i = 0; i < count; ++i) {
    uint32_t src = (uint32_t)start_idx + i;          // 0..65535
    if (src >= SRC_FULL) break;

    uint32_t dst = mapSrcToDst(src);                 // 0..g_capacity-1
    if (dst == prevDst) continue;                    // avoid writing same dest slot repeatedly

    int16_t s = samples[i];
    if (s < g_q15_min) g_q15_min = s;
    if (s > g_q15_max) g_q15_max = s;

    g_lut[dst] = s;
    if (!testBit(dst)) { setBit(dst); ++g_unique_received; }

    prevDst = dst;
  }
}

// ---------- Plot (ONLY after complete table is received) ----------
static void printArbPlot() {
  g_quiet = true;                                    // silence logs
  const int stride = PLOT_STRIDE < 1 ? 1 : PLOT_STRIDE;

  Serial.println("wave");                             // Arduino Serial Plotter series name
  for (size_t i = 0; i < g_capacity; i += stride) {
    const float y = (float)g_lut[i] / 32768.0f;       // Q15 -> [-1,1)
    Serial.println(y, 6);
  }
  g_quiet = false;
}

// ---------- Print ARB done & trigger plot ----------
static void maybePrintDone() {
  if (g_capacity == 0) return;

  if (g_unique_received >= g_capacity && !g_plottedOnce) {
    QPRINT("** [ARB DONE] LUT ready. cap="); QPRINT(g_capacity);
    QPRINT(" f="); QPRINT(g_meta.freq_mHz / 1000.0, 3); QPRINT(" Hz, Vpp=");
    QPRINT(g_meta.amp_mVpp / 1000.0, 3); QPRINT(" V, offset=");
    QPRINT(g_meta.offset_mV / 1000.0, 3); QPRINT(" V, phase=");
    QPRINT(g_meta.phase_mdeg / 1000.0, 3); QPRINT(" deg, ch=");
    QPRINT(channelName(g_meta.channel));
    QPRINT(" | min="); QPRINT(g_q15_min);
    QPRINT(" max=");   QPRINTLN(g_q15_max);

    printArbPlot();      // only once
    g_plottedOnce = true;
  }
}

// ---------- Reassembler for ALL messages ----------
static uint8_t g_buf[320];       // header + TLV L<=255
static size_t  g_have = 0;
static size_t  g_need = 0;       // total frame length when known (0=unknown)

static int findHeaderIdx(const uint8_t* d, int n) {
  for (int i = 0; i + 1 < n; ++i) {
    if (d[i] == 0x01 && (d[i+1] == 0x00 || d[i+1] == 0x01 || d[i+1] == 0x02)) return i;
  }
  return -1;
}
static void resetAsm() { g_have = 0; g_need = 0; }

static void processFrame(const uint8_t* f, size_t n) {
  uint8_t msg = f[1];
  if (msg == 0x00) {
    ParsedStd p{};
    if (parseStd32(f, n, p)) {
      QPRINTLN("[PKT] Parsed standard waveform");
      QPRINT("  shape="); QPRINT(p.shape); QPRINT(" ("); QPRINT(shapeName(p.shape)); QPRINTLN(")");
      QPRINT("  f="); QPRINT(p.freq_mHz / 1000.0, 3); QPRINTLN(" Hz");
      QPRINT("  Vpp="); QPRINT(p.amp_mVpp / 1000.0, 3); QPRINTLN(" V");
      QPRINT("  ofs="); QPRINT(p.offset_mV / 1000.0, 3); QPRINTLN(" V");
      QPRINT("  ph="); QPRINT(p.phase_mdeg / 1000.0, 3); QPRINTLN(" deg");
      QPRINT("  ch="); QPRINTLN(channelName(p.channel));
    } else {
      QPRINTLN("Std parse failed.");
    }
  } else if (msg == 0x01) {
    ArbMeta m{};
    if (parseArbMeta(f, n, m)) {
      // Reset ARB session
      if (g_lut && g_capacity) memset(g_lut, 0, g_capacity * sizeof(int16_t));
      if (g_bitmap && g_capacity) memset(g_bitmap, 0, (g_capacity + 7) / 8);
      g_unique_received = 0;
      g_q15_min =  32767;
      g_q15_max = -32768;
      g_meta = m; g_meta.active = true;
      g_plottedOnce = false;

      QPRINT("[PKT] Parsed arbitrary META | dest-capacity="); QPRINTLN((int)g_capacity);
    } else {
      QPRINTLN("ARB META parse failed.");
    }
  } else if (msg == 0x02) {
    uint16_t seq; uint8_t T, L;
    if (!parseArbDataHeader(f, n, seq, T, L)) { QPRINTLN("ARB DATA header parse failed."); return; }
    if ((size_t)(6 + L) != n) { QPRINTLN("ARB DATA length mismatch."); return; }
    const uint8_t* V = f + 6;
    if (L < 4) { QPRINTLN("ARB DATA L<4"); return; }

    ArbBlockHdr bh;
    bh.start_idx = rd_u16_le(V + 0);
    bh.count     = rd_u16_le(V + 2);
    if (L != (uint8_t)(4 + 2 * bh.count)) {
      QPRINT("ARB DATA bad L vs count. L="); QPRINT(L);
      QPRINT(" count="); QPRINTLN(bh.count);
      return;
    }
    const int16_t* samples = (const int16_t*)(V + 4);

    // per-block min/max for logs
    int16_t bmin =  32767, bmax = -32768;
    for (uint16_t i = 0; i < bh.count; ++i) {
      int16_t s = samples[i];
      if (s < bmin) bmin = s;
      if (s > bmax) bmax = s;
    }

    QPRINT("[PKT] Parsed arbitrary DATA | Seq="); QPRINT(seq);
    QPRINT(" srcIdx="); QPRINT(bh.start_idx);
    QPRINT(" count=");  QPRINT(bh.count);
    QPRINT(" q15[min="); QPRINT(bmin);
    QPRINT(" max=");     QPRINT(bmax); QPRINTLN("]");

    if (!g_meta.active) {
      QPRINTLN("[ARB] Warning: DATA received before META; still assembling.");
    }
    if (!g_lut || !g_bitmap || g_capacity == 0) {
      QPRINTLN("[ARB] ERROR: Buffers not allocated.");
      return;
    }

    applyArbBlock(bh.start_idx, bh.count, samples);

    QPRINT("[ARB LUT-BLOCKED] dest-capacity="); QPRINT((int)g_capacity);
    QPRINT(" total_received="); QPRINT(g_unique_received);
    QPRINTLN("/" + String((int)g_capacity));

    maybePrintDone();
  } else {
    QPRINT("Unknown MsgType="); QPRINTLN(msg);
  }
}

static void feedAsm(const uint8_t* d, int n) {
  int off = 0;
  while (off < n) {
    if (g_have == 0) {
      int h = findHeaderIdx(d + off, n - off);
      if (h < 0) return;
      off += h;
    }

    if (g_need > 0) {
      int toCopy = min((int)(g_need - g_have), n - off);
      memcpy(g_buf + g_have, d + off, toCopy);
      g_have += toCopy; off += toCopy;
      if (g_have == g_need) {
        hexDump(g_buf, g_need, "RX frame");
        processFrame(g_buf, g_need);
        resetAsm();
      }
      continue;
    }

    if (g_have < 4) {
      int need = 4 - (int)g_have;
      int toCopy = min(need, n - off);
      memcpy(g_buf + g_have, d + off, toCopy);
      g_have += toCopy; off += toCopy;
      if (g_have < 4) continue;
    }

    uint8_t msg = g_buf[1];
    if (msg == 0x00 || msg == 0x01) {
      g_need = 32;
      continue;
    } else if (msg == 0x02) {
      if (g_have < 6) {
        int need = 6 - (int)g_have;
        int toCopy = min(need, n - off);
        memcpy(g_buf + g_have, d + off, toCopy);
        g_have += toCopy; off += toCopy;
        if (g_have < 6) continue;
      }
      uint8_t T = g_buf[4], L = g_buf[5];
      if (T != 0x20) {
        int h2 = findHeaderIdx(d + off, n - off);
        if (h2 < 0) { resetAsm(); return; }
        resetAsm(); off += h2;
        continue;
      }
      g_need = 4 /*hdr*/ + 2 /*TL*/ + L;
      if (g_need > sizeof(g_buf)) {
        QPRINTLN("Frame too large; dropping.");
        resetAsm();
        continue;
      }
    } else {
      int h2 = findHeaderIdx(d + off, n - off);
      if (h2 < 0) { resetAsm(); return; }
      resetAsm(); off += h2;
    }
  }
}

// ---------- BLE write handler ----------
static void handleWriteCommon(BLECharacteristic* c, const char* which) {
  if (!c) return;
  String v = c->getValue();                    // binary payload
  std::string s; s.assign(v.c_str(), v.length());
  const uint8_t* data = reinterpret_cast<const uint8_t*>(s.data());
  const size_t   len  = s.size();

  QPRINTLN(">>> onWrite HIT");
  hexDump(data, len, which);
  feedAsm(data, (int)len);

#if SEND_ACK
  if (txChar) {
    char buf[32];
    snprintf(buf, sizeof(buf), "ack:%u", (unsigned)len);
    txChar->setValue((uint8_t*)buf, strlen(buf));
    txChar->notify();
    QPRINTLN("→ Sent ack to client");
  }
#endif
}

// ---------- BLE callbacks ----------
class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override { handleWriteCommon(c, "RX write"); }
};
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* /*pServer*/) override { QPRINTLN("Central connected."); }
  void onDisconnect(BLEServer* /*pServer*/) override {
    QPRINTLN("Central disconnected. Restart advertising.");
    BLEDevice::startAdvertising();
    resetAsm();
  }
};

// ---------- setup/loop ----------
static bool allocWithFallback() {
  // Try PSRAM first, then internal RAM, with capacities: 65536 -> 32768 -> 16384 -> 8192 -> 4096
  const size_t tries[] = {65536, 32768, 16384, 8192, 4096};
  for (size_t cap : tries) {
    size_t lutBytes = cap * sizeof(int16_t);
    size_t bmpBytes = (cap + 7) / 8;

    // free any previous
    if (g_lut)    { heap_caps_free(g_lut);    g_lut = nullptr; }
    if (g_bitmap) { heap_caps_free(g_bitmap); g_bitmap = nullptr; }

    // try PSRAM
    g_lut    = (int16_t*) heap_caps_malloc(lutBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    g_bitmap = (uint8_t*) heap_caps_malloc(bmpBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    // if PSRAM fails, try internal RAM
    if (!g_lut)    g_lut    = (int16_t*) heap_caps_malloc(lutBytes, MALLOC_CAP_8BIT);
    if (!g_bitmap) g_bitmap = (uint8_t*) heap_caps_malloc(bmpBytes, MALLOC_CAP_8BIT);

    if (g_lut && g_bitmap) {
      g_capacity = cap;
      memset(g_lut, 0, lutBytes);
      memset(g_bitmap, 0, bmpBytes);
      QPRINT("[MEM] ARB capacity set to "); QPRINT((int)g_capacity); QPRINTLN(" samples.");
      return true;
    }
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  QPRINTLN("\nBooting…");

  if (!allocWithFallback()) {
    QPRINTLN("❌ FATAL: Could not allocate ARB buffers (try a board with PSRAM).");
  }

  g_unique_received = 0;
  g_q15_min =  32767;
  g_q15_max = -32768;
  g_plottedOnce = false;

  BLEDevice::init("ESP32-NUS");
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService* svc = server->createService(SERVICE_UUID);

  // TX: Notify
  txChar = svc->createCharacteristic(TX_UUID, BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  txChar->addDescriptor(new BLE2902());

  // RX: Write / WriteNR
  BLECharacteristic* rxChar = svc->createCharacteristic(RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rxChar->setCallbacks(new RxCallbacks());

  svc->start();

  // Advertising
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  BLEAdvertisementData advData; advData.setName("ESP32-NUS"); advData.setCompleteServices(SERVICE_UUID); adv->setAdvertisementData(advData);
  BLEAdvertisementData scanResp; scanResp.setName("ESP32-NUS"); adv->setScanResponseData(scanResp);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  adv->setMinPreferred(0x00);
  BLEDevice::startAdvertising();

  QPRINTLN("✅ Advertising ESP32-NUS (RX=Write, TX=Notify)");
  resetAsm();
}

void loop() {
#if HEARTBEAT
  unsigned long now = millis();
  if (now - lastBeat > 2000) {
    lastBeat = now;
    char beat[24];
    snprintf(beat, sizeof(beat), "beat %lu", now / 1000);
    if (txChar) {
      txChar->setValue((uint8_t*)beat, strlen(beat));
      txChar->notify();
      QPRINT("heartbeat -> "); QPRINTLN(beat);
    }
  }
#endif
}
