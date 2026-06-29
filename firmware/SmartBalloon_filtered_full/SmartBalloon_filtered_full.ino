/*
 * ============================================================================
 *  ESP32 Smart Weather Balloon Telemetry System — Nimbus-1
 *  With full per-sensor noise filtering
 * ============================================================================
 *
 *  REQUIRED LIBRARIES  (Arduino IDE → Tools → Manage Libraries)
 *  ─────────────────────────────────────────────────────────────
 *  1. WiFi.h                  Built-in with ESP32 Arduino Core
 *  2. WebServer.h             Built-in with ESP32 Arduino Core
 *  3. Wire.h                  Built-in I2C library
 *  4. Adafruit BMP085 Library Search: "Adafruit BMP085" by Adafruit
 *  5. Adafruit Unified Sensor Search: "Adafruit Unified Sensor" by Adafruit
 *  6. MPU6050                 Search: "MPU6050" by Electronic Cats
 *  7. TinyGPSPlus             Search: "TinyGPS++" by Mikal Hart
 *  8. ArduinoJson             Search: "ArduinoJson" by Benoit Blanchon (v6.x)
 *
 *  WIRING SUMMARY
 *  ──────────────
 *  BMP180  VCC→3.3V  GND→GND  SDA→GPIO21  SCL→GPIO22
 *  MPU6050 VCC→3.3V  GND→GND  SDA→GPIO21  SCL→GPIO22
 *  NEO-6M  VCC→3.3V  GND→GND  TX→GPIO16   RX→GPIO17
 *  LM35    VCC→3.3V  GND→GND  AO→GPIO33
 *  MQ135   VCC→5V    GND→GND  AO→[Divider]→GPIO34
 *  MQ2     VCC→5V    GND→GND  AO→[Divider]→GPIO35
 *  MQ7     VCC→5V    GND→GND  AO→[Divider]→GPIO32
 *
 *  VOLTAGE DIVIDER — MANDATORY for MQ sensors (5V AO → ESP32 3.3V ADC)
 *  ─────────────────────────────────────────────────────────────────────
 *  MQ AO ──┬── 10kΩ ──── ESP32 ADC pin
 *           └── 10kΩ ──── GND
 *  Halves 5V → 2.5V max. MQ_VD_SCALE=2 compensates in code.
 * ============================================================================
 */

// ============================================================================
//  LIBRARY INCLUDES
// ============================================================================
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <MPU6050.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>

// ============================================================================
//  WI-FI ACCESS POINT CREDENTIALS
// ============================================================================
const char*     AP_SSID     = "smart air_baloon";
const char*     AP_PASSWORD = "12345678";
const IPAddress AP_IP      (192, 168, 4, 1);
const IPAddress AP_GW      (192, 168, 4, 1);
const IPAddress AP_SUBNET  (255, 255, 255, 0);

// ============================================================================
//  PIN DEFINITIONS
// ============================================================================
#define PIN_LM35       33
#define PIN_MQ135      34
#define PIN_MQ2        35
#define PIN_MQ7        32
#define PIN_I2C_SDA    21
#define PIN_I2C_SCL    22
#define GPS_ESP_RX     16
#define GPS_ESP_TX     17
#define GPS_BAUD     9600

// ============================================================================
//  CALIBRATION & SCALING CONSTANTS
// ============================================================================
#define MQ_VD_SCALE           2       // 10k/10k voltage divider compensation
#define LM35_OFFSET           0.0f   // Trim offset in °C

#define MQ135_AQI_MAX         500
#define MQ7_CO_PPM_MAX        1000
#define MQ2_SMOKE_PPM_MAX     1000

#define MPU_LSB_PER_G         16384.0f
#define VIBRATION_SCALE       100.0f

#define SENSOR_INTERVAL_MS    1000

// ============================================================================
//  FILTERING CONSTANTS
// ============================================================================

// --- Median filter window size (must be odd) ---
// Used for: LM35, MQ135, MQ2, MQ7
// Takes N samples, sorts, picks the middle value. Kills impulse spikes.
#define MEDIAN_SAMPLES        9

// --- EMA (Exponential Moving Average) alpha values ---
// Range: 0.0 – 1.0. Lower = smoother but slower to respond.
// Higher = faster response but less smoothing.
#define EMA_ALPHA_TEMP        0.15f   // LM35   — slow thermal changes
#define EMA_ALPHA_AQI         0.10f   // MQ135  — very noisy, heavy smoothing
#define EMA_ALPHA_CO          0.10f   // MQ7    — very noisy, heavy smoothing
#define EMA_ALPHA_SMOKE       0.10f   // MQ2    — very noisy, heavy smoothing
#define EMA_ALPHA_ALTITUDE    0.20f   // BMP180 altitude
#define EMA_ALPHA_PRESSURE    0.20f   // BMP180 pressure
#define EMA_ALPHA_PITCH       0.25f   // MPU6050 pitch
#define EMA_ALPHA_ROLL        0.25f   // MPU6050 roll
#define EMA_ALPHA_VIBRATION   0.30f   // MPU6050 vibration — allow faster response
#define EMA_ALPHA_SPEED       0.20f   // GPS speed

// --- Spike rejection thresholds ---
// If a new reading differs from last by more than this, it is discarded.
#define SPIKE_THRESH_TEMP     10.0f   // °C    — reject if jump > 10°C/sec
#define SPIKE_THRESH_ALTITUDE 50.0f   // m     — reject if jump > 50m/sec
#define SPIKE_THRESH_PRESSURE 10.0f   // hPa   — reject if jump > 10hPa/sec
#define SPIKE_THRESH_AQI      150     // AQI units
#define SPIKE_THRESH_CO       200     // ppm
#define SPIKE_THRESH_SMOKE    200     // ppm
#define SPIKE_THRESH_PITCH    30.0f   // degrees/sec
#define SPIKE_THRESH_ROLL     30.0f   // degrees/sec

// ============================================================================
//  HTML DASHBOARD PLACEHOLDER
// ============================================================================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8" />
<meta name="viewport" content="width=device-width, initial-scale=1.0"/>
<title>Nimbus-1 • Smart Weather Balloon Telemetry Dashboard</title>
<script src="https://cdn.jsdelivr.net/npm/@tailwindcss/browser@4"></script>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.1/dist/chart.umd.min.js"></script>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>
  :root{
    --c1:#00d4ff;   /* cyan */
    --c2:#3b82f6;   /* blue */
    --c3:#8b5cf6;   /* purple */
    --c4:#f472b6;   /* pink accent */
    --bg-1:#e0f7ff;
    --bg-2:#ecf5ff;
    --bg-3:#f5e8ff;
  }

  html,body{ font-family:'Inter','Segoe UI',system-ui,sans-serif; }

  body{
    background:
      radial-gradient(1200px 600px at 10% -10%, #bfe9ff 0%, transparent 60%),
      radial-gradient(900px 500px at 95% 10%, #e0d4ff 0%, transparent 60%),
      radial-gradient(1000px 700px at 50% 110%, #c7f0ff 0%, transparent 60%),
      linear-gradient(180deg,#e8f6ff 0%, #f1edff 50%, #e4fbff 100%);
    min-height:100vh;
    color:#0b2240;
  }

  /* Animated subtle wave background */
  .wave-bg::before{
    content:"";
    position:fixed; inset:0;
    background-image:
      radial-gradient(circle at 20% 30%, rgba(59,130,246,0.08) 0 2px, transparent 3px),
      radial-gradient(circle at 70% 60%, rgba(139,92,246,0.08) 0 2px, transparent 3px),
      radial-gradient(circle at 40% 80%, rgba(0,212,255,0.08) 0 2px, transparent 3px);
    background-size: 180px 180px, 220px 220px, 200px 200px;
    pointer-events:none;
    z-index:0;
    animation: drift 30s linear infinite;
  }
  @keyframes drift {
    from{ background-position: 0 0, 0 0, 0 0; }
    to  { background-position: 500px 300px, -400px 300px, 300px -400px; }
  }

  /* Glassmorphism card */
  .glass{
    background: linear-gradient(135deg, rgba(255,255,255,0.75), rgba(255,255,255,0.45));
    backdrop-filter: blur(16px) saturate(140%);
    -webkit-backdrop-filter: blur(16px) saturate(140%);
    border:1px solid rgba(255,255,255,0.7);
    box-shadow:
      0 10px 30px -10px rgba(59,130,246,0.25),
      0 4px 16px -6px rgba(139,92,246,0.18),
      inset 0 1px 0 rgba(255,255,255,0.8);
    border-radius: 22px;
    position:relative;
    overflow:hidden;
    transition: transform .35s ease, box-shadow .35s ease;
  }
  .glass:hover{
    transform: translateY(-3px);
    box-shadow:
      0 18px 45px -10px rgba(59,130,246,0.30),
      0 8px 22px -6px rgba(139,92,246,0.22),
      inset 0 1px 0 rgba(255,255,255,0.9);
  }
  .glass::before{
    content:"";
    position:absolute; inset:0;
    background: linear-gradient(135deg, rgba(0,212,255,0.12), transparent 40%, rgba(139,92,246,0.10));
    pointer-events:none;
  }

  /* Neon gradient border for section titles */
  .title-bar{
    display:flex; align-items:center; gap:10px;
    font-weight:700; letter-spacing:.3px;
  }
  .title-bar .dot{
    width:10px; height:10px; border-radius:50%;
    background: linear-gradient(135deg,var(--c1),var(--c3));
    box-shadow: 0 0 0 3px rgba(0,212,255,0.18), 0 0 12px rgba(139,92,246,.5);
  }
  .title-bar span.accent{
    background: linear-gradient(90deg,#0891b2,#6366f1,#a855f7);
    -webkit-background-clip:text; background-clip:text; color:transparent;
  }

  /* Telemetry values */
  .metric{
    font-family:'JetBrains Mono','Consolas',monospace;
    font-variant-numeric: tabular-nums;
    font-weight:800;
    background: linear-gradient(90deg,#0ea5e9,#6366f1,#a855f7);
    -webkit-background-clip:text; background-clip:text; color:transparent;
    text-shadow: 0 0 24px rgba(99,102,241,0.15);
  }

  /* Radial gauge */
  .gauge{ position:relative; width:100%; aspect-ratio:1/1; max-width:180px; margin:0 auto; }
  .gauge svg{ width:100%; height:100%; transform: rotate(-90deg); }
  .gauge .track{ stroke:rgba(59,130,246,0.12); }
  .gauge .progress{
    stroke: url(#grad-gauge);
    stroke-linecap: round;
    transition: stroke-dashoffset 1s cubic-bezier(.22,1,.36,1);
    filter: drop-shadow(0 0 6px rgba(99,102,241,.45));
  }
  .gauge .val{
    position:absolute; inset:0;
    display:flex; flex-direction:column; align-items:center; justify-content:center;
  }
  .gauge .val .num{ font-size:1.6rem; }
  .gauge .val .unit{ font-size:.75rem; color:#475569; }

  /* Status pill */
  .pill{
    display:inline-flex; align-items:center; gap:8px;
    padding:6px 14px; border-radius:999px;
    background: rgba(255,255,255,0.7);
    border:1px solid rgba(255,255,255,0.9);
    font-size:.8rem; font-weight:600;
    box-shadow: 0 6px 18px -6px rgba(59,130,246,0.25);
  }
  .pill .led{
    width:10px; height:10px; border-radius:50%;
    background:#22c55e;
    box-shadow: 0 0 0 4px rgba(34,197,94,0.18), 0 0 10px #22c55e;
    animation: pulse 1.6s ease-in-out infinite;
  }
  .pill.warn .led{ background:#f59e0b; box-shadow:0 0 0 4px rgba(245,158,11,.18), 0 0 10px #f59e0b;}
  .pill.danger .led{ background:#ef4444; box-shadow:0 0 0 4px rgba(239,68,68,.18), 0 0 10px #ef4444;}
  @keyframes pulse{ 0%,100%{ transform:scale(1); opacity:1;} 50%{ transform:scale(1.25); opacity:.75;} }

  /* Hero header */
  .hero{
    position:relative;
    background:
      linear-gradient(135deg, rgba(0,212,255,0.15), rgba(139,92,246,0.15)),
      linear-gradient(180deg, rgba(255,255,255,0.75), rgba(255,255,255,0.55));
    border:1px solid rgba(255,255,255,0.7);
    backdrop-filter: blur(14px);
    border-radius: 26px;
    overflow:hidden;
  }
  .hero::before{
    content:""; position:absolute; inset:0;
    background:
      radial-gradient(600px 200px at 10% 0%, rgba(0,212,255,0.25), transparent 60%),
      radial-gradient(600px 200px at 90% 100%, rgba(139,92,246,0.25), transparent 60%);
    pointer-events:none;
  }
  .logo-badge{
    width:56px; height:56px; border-radius:16px;
    background: conic-gradient(from 180deg, #22d3ee, #6366f1, #a855f7, #22d3ee);
    box-shadow: 0 10px 24px -8px rgba(99,102,241,.45), inset 0 0 0 2px rgba(255,255,255,.8);
    display:flex; align-items:center; justify-content:center;
    animation: spin 18s linear infinite;
  }
  @keyframes spin{ to{ transform: rotate(360deg);} }

  /* Buttons */
  .btn{
    position:relative;
    padding:12px 18px; border-radius:14px;
    font-weight:700; letter-spacing:.2px;
    display:inline-flex; align-items:center; gap:10px;
    color:white; cursor:pointer; border:0;
    transition: transform .2s ease, box-shadow .2s ease, filter .2s ease;
    box-shadow: 0 10px 22px -8px rgba(99,102,241,.45);
  }
  .btn:hover{ transform: translateY(-2px); filter: brightness(1.05);}
  .btn:active{ transform: translateY(0); }
  .btn-primary{ background: linear-gradient(135deg,#06b6d4,#6366f1,#a855f7); }
  .btn-warn   { background: linear-gradient(135deg,#f59e0b,#ef4444); }
  .btn-neutral{ background: linear-gradient(135deg,#64748b,#334155); }
  .btn-success{ background: linear-gradient(135deg,#10b981,#06b6d4); }
  .btn-demo-live{ background: linear-gradient(135deg,#10b981,#22d3ee); }
  .btn-demo-active{ background: linear-gradient(135deg,#f59e0b,#fbbf24); }

  /* Map */
  #map{ height: 340px; border-radius: 18px; filter: saturate(1.05) contrast(1.02); }

  /* Horizon indicator (MPU6050 3D style) */
  .horizon{
    position:relative; width:100%; aspect-ratio: 1/1; max-width: 220px; margin:0 auto;
    border-radius: 50%;
    background:
      radial-gradient(circle at 50% 30%, #bae6fd 0 40%, #38bdf8 40.5% 50%, #0369a1 50.5% 100%);
    box-shadow:
      inset 0 0 0 8px rgba(255,255,255,0.85),
      inset 0 0 0 12px #0ea5e9,
      0 20px 40px -10px rgba(14,165,233,.45);
    overflow:hidden;
    transform-style: preserve-3d;
    transition: transform .5s ease;
  }
  .horizon .crosshair{
    position:absolute; inset:0;
    background:
      linear-gradient(90deg, transparent calc(50% - 1px), rgba(255,255,255,.9) 50%, transparent calc(50% + 1px)),
      linear-gradient(0deg, transparent calc(50% - 1px), rgba(255,255,255,.9) 50%, transparent calc(50% + 1px));
    pointer-events:none;
  }
  .horizon .reticle{
    position:absolute; left:50%; top:50%;
    width:50px; height:14px; transform: translate(-50%,-50%);
    border:2px solid #fff; border-top:0; border-radius: 0 0 12px 12px;
    box-shadow: 0 0 10px rgba(255,255,255,.7);
  }
  .horizon .ticks{
    position:absolute; inset:0;
    background:
      repeating-linear-gradient(90deg, transparent 0 38%, rgba(255,255,255,.6) 38% 40%, transparent 40% 48%);
    mask-image: radial-gradient(circle, black 55%, transparent 62%);
  }

  /* Mini bar for vibration */
  .vib-bars{ display:flex; gap:4px; align-items:flex-end; height:60px; }
  .vib-bars span{
    flex:1; background: linear-gradient(180deg,#22d3ee,#6366f1);
    border-radius: 6px; height: 20%;
    animation: vib 1.2s ease-in-out infinite;
    box-shadow: 0 6px 14px -6px rgba(99,102,241,.5);
  }
  @keyframes vib{ 0%,100%{ height:15%;} 50%{ height:90%;} }

  /* Table-like environment panel */
  .env-row{
    display:flex; align-items:center; justify-content:space-between;
    padding:12px 14px; border-radius:14px;
    background: linear-gradient(135deg, rgba(255,255,255,.85), rgba(255,255,255,.55));
    border:1px solid rgba(255,255,255,.7);
    margin-bottom: 10px;
    transition: transform .2s ease;
  }
  .env-row:hover{ transform: translateX(4px); }

  .tag{
    padding:4px 10px; border-radius:999px;
    font-size:.75rem; font-weight:700; letter-spacing:.2px;
    color:white;
  }
  .tag-good{ background: linear-gradient(135deg,#10b981,#22d3ee); }
  .tag-mod { background: linear-gradient(135deg,#f59e0b,#f97316); }
  .tag-bad { background: linear-gradient(135deg,#ef4444,#ec4899); }
  .tag-info{ background: linear-gradient(135deg,#0ea5e9,#6366f1); }

  /* Telemetry card icon */
  .icon-chip{
    width:42px; height:42px; border-radius:12px;
    display:flex; align-items:center; justify-content:center;
    color:white; font-size:1.05rem;
    background: linear-gradient(135deg,#22d3ee,#6366f1);
    box-shadow: 0 8px 16px -6px rgba(99,102,241,.5);
  }
  .icon-chip.v2{ background: linear-gradient(135deg,#f59e0b,#ef4444);}
  .icon-chip.v3{ background: linear-gradient(135deg,#10b981,#22d3ee);}
  .icon-chip.v4{ background: linear-gradient(135deg,#a855f7,#ec4899);}

  /* Section divider */
  .divider{
    height:1px; background: linear-gradient(90deg, transparent, rgba(99,102,241,.25), transparent);
    margin: 10px 0 18px 0;
  }

  /* Small scrollbars */
  ::-webkit-scrollbar{ width:10px; height:10px; }
  ::-webkit-scrollbar-thumb{ background: linear-gradient(180deg,#22d3ee,#a855f7); border-radius:10px;}
  ::-webkit-scrollbar-track{ background: rgba(255,255,255,.3);}

  /* Mission log */
  .log{
    font-family:'JetBrains Mono','Consolas',monospace;
    font-size:.8rem; color:#0b2240;
    background: linear-gradient(180deg, rgba(255,255,255,.7), rgba(255,255,255,.45));
    border:1px solid rgba(255,255,255,.7);
    border-radius: 14px; padding: 10px 14px; max-height: 140px; overflow:auto;
  }
  .log .t{ color:#0891b2; } .log .lv{ color:#7c3aed; } .log .m{ color:#334155; }

  /* Balloon SVG float animation */
  .float{ animation: floaty 4s ease-in-out infinite; }
  @keyframes floaty{ 0%,100%{ transform: translateY(0);} 50%{ transform: translateY(-8px);} }

  /* Launch status indicator */
  .status-ring{
    display:inline-block; width:10px; height:10px; border-radius:50%;
    background:#22c55e; box-shadow: 0 0 0 4px rgba(34,197,94,.2), 0 0 10px #22c55e;
    animation: pulse 1.5s ease-in-out infinite;
  }
  .status-ring.up{ background:#0ea5e9; box-shadow: 0 0 0 4px rgba(14,165,233,.2), 0 0 10px #0ea5e9;}
  .status-ring.down{ background:#f59e0b; box-shadow: 0 0 0 4px rgba(245,158,11,.2), 0 0 10px #f59e0b;}

  /* Print/responsive tweaks */
  @media (max-width: 640px){
    .hero-grid{ grid-template-columns: 1fr !important;}
    #map{ height: 260px; }
  }

  /* Chart container */
  .chart-box{ position: relative; height: 220px; }

  /* Gradient text helpers */
  .grad-text{ background: linear-gradient(90deg,#0ea5e9,#6366f1,#a855f7,#ec4899); -webkit-background-clip:text; background-clip:text; color:transparent;}

  /* Glow hover pulse for mission control */
  .glow:hover{ box-shadow: 0 0 0 6px rgba(99,102,241,.15), 0 18px 38px -8px rgba(99,102,241,.55);}

  /* Demo Mode Badge */
  .demo-badge{
    display:inline-flex; align-items:center; gap:6px;
    padding:8px 14px; border-radius:999px;
    background: linear-gradient(135deg, rgba(245,158,11,0.2), rgba(251,191,36,0.2));
    border:1px solid rgba(245,158,11,0.4);
    font-size:.85rem; font-weight:700;
    color:#92400e;
    box-shadow: 0 0 0 4px rgba(245,158,11,.1), 0 0 12px rgba(245,158,11,.25);
    animation: pulse-demo 1.5s ease-in-out infinite;
  }
  @keyframes pulse-demo{ 0%,100%{ opacity:1;} 50%{ opacity:0.75;} }
</style>
</head>
<body class="wave-bg">

<!-- SVG gradient defs -->
<svg width="0" height="0" style="position:absolute">
  <defs>
    <linearGradient id="grad-gauge" x1="0%" y1="0%" x2="100%" y2="100%">
      <stop offset="0%" stop-color="#22d3ee"/>
      <stop offset="50%" stop-color="#6366f1"/>
      <stop offset="100%" stop-color="#a855f7"/>
    </linearGradient>
  </defs>
</svg>

<div class="relative z-10 max-w-[1500px] mx-auto px-4 sm:px-6 py-6">

  <!-- HEADER -->
  <header class="hero p-5 sm:p-6 mb-6">
    <div class="relative grid hero-grid grid-cols-1 md:grid-cols-[auto_1fr_auto] items-center gap-5">
      <div class="flex items-center gap-4">
        <div class="logo-badge">
          <svg width="28" height="28" viewBox="0 0 24 24" fill="none" style="animation: spin 22s linear infinite reverse;">
            <path d="M12 2L3 7v10l9 5 9-5V7l-9-5z" stroke="white" stroke-width="1.6" stroke-linejoin="round"/>
            <circle cx="12" cy="12" r="2.5" fill="white"/>
          </svg>
        </div>
        <div>
          <div class="text-xs md:text-sm font-semibold text-cyan-700/80 tracking-[.35em] uppercase">Project Nimbus-1</div>
          <h1 class="text-2xl md:text-3xl font-black leading-tight grad-text">Smart Weather Balloon Telemetry System</h1>
          <p class="text-xs md:text-sm text-slate-600 font-medium">ESP32 • IoT Atmospheric Monitoring & Research Platform</p>
        </div>
      </div>

      <div class="hidden md:flex justify-center items-center gap-3 flex-wrap">
        <div id="demo-badge-header" class="demo-badge" style="display:none;">🟡 DEMO MODE ACTIVE</div>
        <div class="pill"><span class="led"></span><span>CONNECTED • ESP32-AP-01</span></div>
        <div class="pill"><span class="icon-chip" style="width:24px;height:24px;border-radius:8px;font-size:.7rem">📡</span><span>MQTT • 97% QoS</span></div>
        <div class="pill"><span class="icon-chip v3" style="width:24px;height:24px;border-radius:8px;font-size:.7rem">🛰</span><span>GPS • 8 Sats</span></div>
      </div>

      <div class="text-left md:text-right flex flex-col gap-2">
        <div>
          <div id="bigClock" class="text-3xl md:text-4xl font-black grad-text">--:--:--</div>
          <div id="bigDate" class="text-sm font-semibold text-slate-600">—</div>
        </div>
        <button id="demo-mode-btn" class="btn btn-demo-live" onclick="toggleDemoMode()" style="align-self: flex-end; padding: 8px 14px; font-size: 0.85rem;">
          <span id="demo-btn-text">🟢 LIVE</span>
        </button>
      </div>
    </div>
  </header>

  <!-- LIVE TELEMETRY CARDS -->
  <section class="mb-6">
    <div class="title-bar mb-3"><span class="dot"></span><h2 class="text-lg">Live Telemetry <span class="accent">· 8 Sensors Streaming</span></h2></div>
    <div class="grid grid-cols-2 md:grid-cols-4 gap-4">

      <!-- Altitude -->
      <div class="glass p-4">
        <div class="flex items-center justify-between mb-2">
          <div class="flex items-center gap-2"><div class="icon-chip">⛰</div><div class="text-sm font-bold text-slate-700">Altitude</div></div>
          <span class="tag tag-info">BMP180</span>
        </div>
        <div class="gauge">
          <svg viewBox="0 0 120 120">
            <circle class="track" cx="60" cy="60" r="50" stroke-width="10" fill="none"/>
            <circle id="g-alt" class="progress" cx="60" cy="60" r="50" stroke-width="10" fill="none"
                    stroke-dasharray="314" stroke-dashoffset="314"/>
          </svg>
          <div class="val"><div class="num metric" id="v-alt">0</div><div class="unit">meters</div></div>
        </div>
      </div>

      <!-- Temperature -->
      <div class="glass p-4">
        <div class="flex items-center justify-between mb-2">
          <div class="flex items-center gap-2"><div class="icon-chip v2">🌡</div><div class="text-sm font-bold text-slate-700">Temperature</div></div>
          <span class="tag tag-info">LM35</span>
        </div>
        <div class="gauge">
          <svg viewBox="0 0 120 120">
            <circle class="track" cx="60" cy="60" r="50" stroke-width="10" fill="none"/>
            <circle id="g-tmp" class="progress" cx="60" cy="60" r="50" stroke-width="10" fill="none"
                    stroke-dasharray="314" stroke-dashoffset="314"/>
          </svg>
          <div class="val"><div class="num metric" id="v-tmp">0</div><div class="unit">°Celsius</div></div>
        </div>
      </div>

      <!-- Pressure -->
      <div class="glass p-4">
        <div class="flex items-center justify-between mb-2">
          <div class="flex items-center gap-2"><div class="icon-chip v3">🧭</div><div class="text-sm font-bold text-slate-700">Pressure</div></div>
          <span class="tag tag-info">BMP180</span>
        </div>
        <div class="gauge">
          <svg viewBox="0 0 120 120">
            <circle class="track" cx="60" cy="60" r="50" stroke-width="10" fill="none"/>
            <circle id="g-pres" class="progress" cx="60" cy="60" r="50" stroke-width="10" fill="none"
                    stroke-dasharray="314" stroke-dashoffset="314"/>
          </svg>
          <div class="val"><div class="num metric" id="v-pres">0</div><div class="unit">hPa</div></div>
        </div>
      </div>

      <!-- Air Quality Index -->
      <div class="glass p-4">
        <div class="flex items-center justify-between mb-2">
          <div class="flex items-center gap-2"><div class="icon-chip v4">🌫</div><div class="text-sm font-bold text-slate-700">Air Quality</div></div>
          <span class="tag tag-info">MQ135</span>
        </div>
        <div class="gauge">
          <svg viewBox="0 0 120 120">
            <circle class="track" cx="60" cy="60" r="50" stroke-width="10" fill="none"/>
            <circle id="g-aqi" class="progress" cx="60" cy="60" r="50" stroke-width="10" fill="none"
                    stroke-dasharray="314" stroke-dashoffset="314"/>
          </svg>
          <div class="val"><div class="num metric" id="v-aqi">0</div><div class="unit">AQI</div></div>
        </div>
      </div>

      <!-- CO -->
      <div class="glass p-4">
        <div class="flex items-center justify-between mb-2">
          <div class="flex items-center gap-2"><div class="icon-chip v2">☣</div><div class="text-sm font-bold text-slate-700">Carbon Monoxide</div></div>
          <span class="tag tag-info">MQ7</span>
        </div>
        <div class="text-center mt-2">
          <div class="text-4xl metric" id="v-co">0</div>
          <div class="text-xs text-slate-600">ppm</div>
          <div class="mt-3 h-2 rounded-full bg-slate-200/70 overflow-hidden">
            <div id="b-co" class="h-full rounded-full" style="width:0%; background:linear-gradient(90deg,#22d3ee,#a855f7);"></div>
          </div>
          <div class="text-[11px] text-slate-500 mt-1 font-semibold" id="t-co">Safe</div>
        </div>
      </div>

      <!-- Smoke -->
      <div class="glass p-4">
        <div class="flex items-center justify-between mb-2">
          <div class="flex items-center gap-2"><div class="icon-chip v2">🔥</div><div class="text-sm font-bold text-slate-700">Smoke / LPG</div></div>
          <span class="tag tag-info">MQ2</span>
        </div>
        <div class="text-center mt-2">
          <div class="text-4xl metric" id="v-smoke">0</div>
          <div class="text-xs text-slate-600">ppm</div>
          <div class="mt-3 h-2 rounded-full bg-slate-200/70 overflow-hidden">
            <div id="b-smoke" class="h-full rounded-full" style="width:0%; background:linear-gradient(90deg,#f59e0b,#ef4444);"></div>
          </div>
          <div class="text-[11px] text-slate-500 mt-1 font-semibold" id="t-smoke">Clear</div>
        </div>
      </div>

      <!-- GPS Speed -->
      <div class="glass p-4">
        <div class="flex items-center justify-between mb-2">
          <div class="flex items-center gap-2"><div class="icon-chip v3">🛰</div><div class="text-sm font-bold text-slate-700">GPS Speed</div></div>
          <span class="tag tag-info">NEO-6M</span>
        </div>
        <div class="text-center mt-2">
          <div class="text-4xl metric" id="v-speed">0</div>
          <div class="text-xs text-slate-600">km/h</div>
          <div class="mt-3 h-2 rounded-full bg-slate-200/70 overflow-hidden">
            <div id="b-speed" class="h-full rounded-full" style="width:0%; background:linear-gradient(90deg,#10b981,#22d3ee);"></div>
          </div>
          <div class="text-[11px] text-slate-500 mt-1 font-semibold">Ground Track</div>
        </div>
      </div>

      <!-- Wind Speed (Simulated) -->
      <div class="glass p-4">
        <div class="flex items-center justify-between mb-2">
          <div class="flex items-center gap-2"><div class="icon-chip v4">💨</div><div class="text-sm font-bold text-slate-700">Wind Speed</div></div>
          <span class="tag tag-info">EST.</span>
        </div>
        <div class="text-center mt-2">
          <div class="text-4xl metric" id="v-wind">0</div>
          <div class="text-xs text-slate-600">km/h</div>
          <div class="mt-3 h-2 rounded-full bg-slate-200/70 overflow-hidden">
            <div id="b-wind" class="h-full rounded-full" style="width:0%; background:linear-gradient(90deg,#a855f7,#ec4899);"></div>
          </div>
          <div class="text-[11px] text-slate-500 mt-1 font-semibold" id="t-wind">Light Breeze</div>
        </div>
      </div>

    </div>
  </section>

  <!-- GPS + FLIGHT MONITORING -->
  <section class="grid grid-cols-1 lg:grid-cols-3 gap-4 mb-6">
    <!-- GPS Map -->
    <div class="glass p-5 lg:col-span-2">
      <div class="title-bar mb-3"><span class="dot"></span><h2 class="text-lg">GPS Tracking · <span class="accent">Live Balloon Route</span></h2></div>
      <div class="grid grid-cols-1 md:grid-cols-4 gap-3 mb-3">
        <div class="px-3 py-2 rounded-xl bg-white/70 border border-white/80">
          <div class="text-[11px] text-slate-500 font-bold uppercase tracking-wider">Latitude</div>
          <div class="text-lg font-bold metric" id="v-lat">0</div>
        </div>
        <div class="px-3 py-2 rounded-xl bg-white/70 border border-white/80">
          <div class="text-[11px] text-slate-500 font-bold uppercase tracking-wider">Longitude</div>
          <div class="text-lg font-bold metric" id="v-lon">0</div>
        </div>
        <div class="px-3 py-2 rounded-xl bg-white/70 border border-white/80">
          <div class="text-[11px] text-slate-500 font-bold uppercase tracking-wider">Satellites</div>
          <div class="text-lg font-bold metric" id="v-sats">8</div>
        </div>
        <div class="px-3 py-2 rounded-xl bg-white/70 border border-white/80">
          <div class="text-[11px] text-slate-500 font-bold uppercase tracking-wider">Location</div>
          <div class="text-sm font-bold text-slate-700" id="v-loc">Chennai, IN</div>
        </div>
      </div>
      <div id="map"></div>
    </div>

    <!-- Flight Monitoring -->
    <div class="glass p-5">
      <div class="title-bar mb-3"><span class="dot"></span><h2 class="text-lg">Flight Monitoring</h2></div>

      <div class="flex items-center justify-between mb-3 px-3 py-3 rounded-2xl"
           style="background:linear-gradient(135deg, rgba(34,211,238,.15), rgba(139,92,246,.15)); border:1px solid rgba(255,255,255,.8);">
        <div>
          <div class="text-[11px] font-bold uppercase tracking-wider text-slate-500">Launch Status</div>
          <div class="text-lg font-black text-slate-800" id="v-flight">Ascending</div>
        </div>
        <div class="flex items-center gap-2">
          <span id="status-ring" class="status-ring up"></span>
          <span class="tag tag-info">LIVE</span>
        </div>
      </div>

      <div class="divider"></div>

      <div class="grid grid-cols-2 gap-3">
        <div class="p-3 rounded-2xl bg-white/70 border border-white/80">
          <div class="text-[11px] font-bold uppercase tracking-wider text-slate-500">Current Alt.</div>
          <div class="text-2xl metric" id="v-alt2">0</div>
          <div class="text-xs text-slate-500">meters</div>
        </div>
        <div class="p-3 rounded-2xl bg-white/70 border border-white/80">
          <div class="text-[11px] font-bold uppercase tracking-wider text-slate-500">Max Alt.</div>
          <div class="text-2xl metric" id="v-maxalt">0</div>
          <div class="text-xs text-slate-500">meters</div>
        </div>
        <div class="p-3 rounded-2xl bg-white/70 border border-white/80">
          <div class="text-[11px] font-bold uppercase tracking-wider text-slate-500">Ascent Rate</div>
          <div class="text-2xl metric" id="v-ascent">0.0</div>
          <div class="text-xs text-slate-500">m/s</div>
        </div>
        <div class="p-3 rounded-2xl bg-white/70 border border-white/80">
          <div class="text-[11px] font-bold uppercase tracking-wider text-slate-500">Distance</div>
          <div class="text-2xl metric" id="v-dist">0.0</div>
          <div class="text-xs text-slate-500">km</div>
        </div>
        <div class="p-3 rounded-2xl bg-white/70 border border-white/80 col-span-2">
          <div class="flex items-center justify-between">
            <div>
              <div class="text-[11px] font-bold uppercase tracking-wider text-slate-500">Flight Duration</div>
              <div class="text-3xl metric" id="v-dur">00:00:00</div>
            </div>
            <svg class="float" width="60" height="60" viewBox="0 0 64 64">
              <defs><linearGradient id="bal" x1="0" y1="0" x2="1" y2="1"><stop offset="0%" stop-color="#22d3ee"/><stop offset="100%" stop-color="#a855f7"/></linearGradient></defs>
              <path d="M32 6c-10 0-18 8-18 18 0 9 6 14 10 18l2 4h12l2-4c4-4 10-9 10-18 0-10-8-18-18-18z" fill="url(#bal)" opacity=".95"/>
              <path d="M30 46l-2 14h4l-2-14z" fill="#6366f1"/>
              <circle cx="26" cy="22" r="4" fill="white" opacity=".6"/>
            </svg>
          </div>
        </div>
      </div>
    </div>
  </section>

  <!-- ATMOSPHERIC ANALYSIS + MOTION -->
  <section class="grid grid-cols-1 lg:grid-cols-3 gap-4 mb-6">
    <!-- Charts -->
    <div class="glass p-5 lg:col-span-2">
      <div class="title-bar mb-3"><span class="dot"></span><h2 class="text-lg">Atmospheric Analysis · <span class="accent">Research Trends</span></h2></div>
      <div class="grid grid-cols-1 md:grid-cols-2 gap-4">
        <div class="chart-box"><canvas id="chAlt"></canvas></div>
        <div class="chart-box"><canvas id="chTmp"></canvas></div>
        <div class="chart-box"><canvas id="chPres"></canvas></div>
        <div class="chart-box"><canvas id="chAqi"></canvas></div>
      </div>
    </div>

    <!-- Motion & Stability -->
    <div class="glass p-5">
      <div class="title-bar mb-3"><span class="dot"></span><h2 class="text-lg">Motion & Stability · <span class="accent">MPU6050</span></h2></div>

      <div id="horizon" class="horizon mb-3">
        <div class="ticks"></div>
        <div class="crosshair"></div>
        <div class="reticle"></div>
      </div>

      <div class="grid grid-cols-2 gap-3">
        <div class="p-3 rounded-2xl bg-white/70 border border-white/80">
          <div class="text-[11px] font-bold uppercase tracking-wider text-slate-500">Pitch</div>
          <div class="text-2xl metric" id="v-pitch">0°</div>
        </div>
        <div class="p-3 rounded-2xl bg-white/70 border border-white/80">
          <div class="text-[11px] font-bold uppercase tracking-wider text-slate-500">Roll</div>
          <div class="text-2xl metric" id="v-roll">0°</div>
        </div>
        <div class="p-3 rounded-2xl bg-white/70 border border-white/80 col-span-2">
          <div class="flex items-center justify-between mb-2">
            <div class="text-[11px] font-bold uppercase tracking-wider text-slate-500">Vibration Level</div>
            <div class="text-sm font-bold metric" id="v-vib">0</div>
          </div>
          <div class="vib-bars"><span style="animation-delay:0s"></span><span style="animation-delay:.1s"></span><span style="animation-delay:.2s"></span><span style="animation-delay:.3s"></span><span style="animation-delay:.4s"></span><span style="animation-delay:.5s"></span><span style="animation-delay:.6s"></span><span style="animation-delay:.7s"></span></div>
        </div>
        <div class="p-3 rounded-2xl bg-white/70 border border-white/80 col-span-2">
          <div class="flex items-center justify-between mb-2">
            <div class="text-sm font-bold text-slate-700">Balloon Stability</div>
            <span id="stab-tag" class="tag tag-good">STABLE</span>
          </div>
          <div class="h-3 rounded-full bg-slate-200/70 overflow-hidden">
            <div id="stab-bar" class="h-full rounded-full" style="width:80%; background:linear-gradient(90deg,#10b981,#22d3ee);"></div>
          </div>
          <div class="text-xs text-slate-500 mt-1" id="stab-text">Attitude within nominal range.</div>
        </div>
      </div>
    </div>
  </section>

  <!-- ENVIRONMENTAL PANEL + MISSION + EDU -->
  <section class="grid grid-cols-1 lg:grid-cols-3 gap-4 mb-6">

    <!-- Environmental -->
    <div class="glass p-5">
      <div class="title-bar mb-3"><span class="dot"></span><h2 class="text-lg">Environmental Status</h2></div>

      <div class="env-row">
        <div class="flex items-center gap-3"><div class="icon-chip v3">⛅</div><div><div class="text-sm font-bold">Weather</div><div class="text-xs text-slate-500">Based on T / P / Wind</div></div></div>
        <span id="env-weather" class="tag tag-info">Partly Cloudy</span>
      </div>
      <div class="env-row">
        <div class="flex items-center gap-3"><div class="icon-chip v4">🌫</div><div><div class="text-sm font-bold">Air Quality</div><div class="text-xs text-slate-500">MQ135 • AQI Index</div></div></div>
        <span id="env-aqi" class="tag tag-good">Good</span>
      </div>
      <div class="env-row">
        <div class="flex items-center gap-3"><div class="icon-chip v2">☣</div><div><div class="text-sm font-bold">CO Warning</div><div class="text-xs text-slate-500">MQ7 • ppm level</div></div></div>
        <span id="env-co" class="tag tag-good">Normal</span>
      </div>
      <div class="env-row">
        <div class="flex items-center gap-3"><div class="icon-chip v2">🔥</div><div><div class="text-sm font-bold">Smoke / LPG</div><div class="text-xs text-slate-500">MQ2 • Gas Detection</div></div></div>
        <span id="env-smoke" class="tag tag-good">Clear</span>
      </div>

      <div class="divider"></div>

      <div class="text-[11px] text-slate-500 font-bold uppercase tracking-wider mb-2">Mission Log</div>
      <div id="log" class="log"></div>
    </div>

    <!-- Mission Control -->
    <div class="glass p-5">
      <div class="title-bar mb-3"><span class="dot"></span><h2 class="text-lg">Mission Control</h2></div>

      <div class="p-4 rounded-2xl mb-4"
           style="background:linear-gradient(135deg, rgba(14,165,233,.12), rgba(168,85,247,.12)); border:1px solid rgba(255,255,255,.8);">
        <div class="text-[11px] font-bold uppercase tracking-wider text-slate-500">Mission Clock</div>
        <div class="text-4xl metric" id="mc-clock">00:00:00</div>
        <div class="text-xs text-slate-500 mt-1">T+ since launch</div>
      </div>

      <div class="grid grid-cols-2 gap-3">
        <button class="btn btn-neutral glow" onclick="resetData()">⟲ Reset</button>
        <button class="btn btn-primary glow" onclick="exportData()">⬇ Export CSV</button>
      </div>

      <div class="divider"></div>

      <div class="grid grid-cols-3 gap-2 text-center">
        <div class="p-2 rounded-xl bg-white/70 border border-white/80">
          <div class="text-[10px] font-bold text-slate-500 uppercase">Packets</div>
          <div class="text-lg font-bold metric" id="mc-pack">0</div>
        </div>
        <div class="p-2 rounded-xl bg-white/70 border border-white/80">
          <div class="text-[10px] font-bold text-slate-500 uppercase">RSSI</div>
          <div class="text-lg font-bold metric">-62</div>
        </div>
        <div class="p-2 rounded-xl bg-white/70 border border-white/80">
          <div class="text-[10px] font-bold text-slate-500 uppercase">Battery</div>
          <div class="text-lg font-bold metric">87%</div>
        </div>
      </div>
    </div>

    <!-- Educational -->
    <div class="glass p-5">
      <div class="title-bar mb-3"><span class="dot"></span><h2 class="text-lg">About the Project</h2></div>

      <div class="p-4 rounded-2xl mb-3"
           style="background:linear-gradient(135deg, rgba(34,211,238,.15), rgba(168,85,247,.15)); border:1px solid rgba(255,255,255,.8);">
        <div class="text-[11px] font-bold uppercase tracking-wider text-slate-500">Project Title</div>
        <div class="text-base font-black grad-text leading-tight mt-1">Smart Weather Balloon Training & Atmospheric Monitoring Kit</div>
      </div>

      <div class="text-sm text-slate-700 font-semibold mb-2">Purpose & Objectives</div>
      <ul class="space-y-2 text-sm">
        <li class="flex items-start gap-2"><span class="icon-chip" style="width:26px;height:26px;border-radius:8px;font-size:.75rem">1</span><span class="text-slate-700"><b>Atmospheric data collection</b> at varying altitudes for research.</span></li>
        <li class="flex items-start gap-2"><span class="icon-chip v2" style="width:26px;height:26px;border-radius:8px;font-size:.75rem">2</span><span class="text-slate-700"><b>Weather balloon training</b> for students & researchers.</span></li>
        <li class="flex items-start gap-2"><span class="icon-chip v3" style="width:26px;height:26px;border-radius:8px;font-size:.75rem">3</span><span class="text-slate-700"><b>Environmental monitoring</b> of air quality & toxic gases.</span></li>
        <li class="flex items-start gap-2"><span class="icon-chip v4" style="width:26px;height:26px;border-radius:8px;font-size:.75rem">4</span><span class="text-slate-700"><b>GPS tracking</b> of balloon trajectory in real time.</span></li>
        <li class="flex items-start gap-2"><span class="icon-chip" style="width:26px;height:26px;border-radius:8px;font-size:.75rem">5</span><span class="text-slate-700"><b>IoT-based telemetry</b> dashboard for remote monitoring.</span></li>
        <li class="flex items-start gap-2"><span class="icon-chip v2" style="width:26px;height:26px;border-radius:8px;font-size:.75rem">6</span><span class="text-slate-700"><b>Research & educational</b> demonstrations in academia.</span></li>
      </ul>

      <div class="divider"></div>

      <div class="grid grid-cols-2 gap-2 text-xs">
        <div class="p-2 rounded-xl bg-white/70 border border-white/80"><b>Microcontroller:</b><br/>ESP32 WROOM</div>
        <div class="p-2 rounded-xl bg-white/70 border border-white/80"><b>Sensors:</b><br/>7 Modules</div>
        <div class="p-2 rounded-xl bg-white/70 border border-white/80"><b>Protocol:</b><br/>MQTT / HTTP-JSON</div>
        <div class="p-2 rounded-xl bg-white/70 border border-white/80"><b>Frontend:</b><br/>HTML5 + Chart.js</div>
      </div>
    </div>
  </section>

</div>

<script>
/* ======================
   Global State
   ====================== */
let demoModeEnabled = false;
let dataFetchInterval = null;

let balloonData = {
  altitude: 125,
  temperature: 28,
  pressure: 1008,
  aqi: 65,
  co: 12,
  smoke: 18,
  latitude: 13.0827,
  longitude: 80.2707,
  speed: 15,
  pitch: 5,
  roll: 2,
  vibration: 10
};

let mission = {
  running: true,
  startTime: Date.now(),
  maxAlt: 0,
  dist: 0,
  packets: 0,
  history: [],
  route: [[13.0827, 80.2707]],
  _elapsed: 0
};

let labels = [];
for(let i=0;i<12;i++) labels.push('T+'+(i*5));

/* ======================
   Chart.js instances
   ====================== */
let chAlt, chTmpAlt, chPres, chAqi;

/* ======================
   Leaflet map
   ====================== */
let map, balloonMarker, routeLine;

/* ======================
   Clock & date
   ====================== */
function updateClock(){
  const now = new Date();
  const h = String(now.getHours()).padStart(2,'0');
  const m = String(now.getMinutes()).padStart(2,'0');
  const s = String(now.getSeconds()).padStart(2,'0');
  document.getElementById('bigClock').textContent = h+':'+m+':'+s;
  document.getElementById('bigDate').textContent = now.toLocaleDateString();
  
  if(mission.running){
    const elapsed = Math.floor((Date.now()-mission.startTime)/1000);
    const h2 = String(Math.floor(elapsed/3600)).padStart(2,'0');
    const m2 = String(Math.floor((elapsed%3600)/60)).padStart(2,'0');
    const s2 = String(elapsed%60).padStart(2,'0');
    document.getElementById('mc-clock').textContent = h2+':'+m2+':'+s2;
  }
}
setInterval(updateClock, 1000);
updateClock();

/* ======================
   Initialize Leaflet Map
   ====================== */
function initMap(){
  map = L.map('map', { zoomControl: true }).setView([13.0827, 80.2707], 14);
  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',
    { attribution: '© OpenStreetMap', maxZoom: 19 }).addTo(map);
  
  balloonMarker = L.marker([13.0827, 80.2707], {
    icon: L.icon({ iconUrl: 'data:image/svg+xml;utf8,<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 32 32"><circle cx="16" cy="16" r="10" fill="%2322d3ee"/><circle cx="16" cy="16" r="8" fill="%236366f1"/></svg>', iconSize: [24,24] })
  }).addTo(map).bindPopup(`<b>Nimbus-1</b><br/>Alt: 125 m<br/>Temp: 28°C<br/>Spd: 15 km/h`);
  
  routeLine = L.polyline(mission.route, { color: '#6366f1', weight: 2, opacity: 0.7 }).addTo(map);
}

/* ======================
   Initialize Charts
   ====================== */
function initCharts(){
  const chartOptions = {
    responsive: true, maintainAspectRatio: false,
    plugins: { legend: { display: false }, filler: { propagate: true } },
    scales: { y: { min: 0, beginAtZero: true, grid: { color: 'rgba(0,0,0,0.05)' } }, x: { grid: { display: false } } }
  };

  chAlt = new Chart(document.getElementById('chAlt'), {
    type: 'line',
    data: {
      labels: labels,
      datasets: [{
        label: 'Altitude (m)',
        data: Array(12).fill(125),
        borderColor: '#0ea5e9',
        backgroundColor: 'rgba(14,165,233,0.1)',
        borderWidth: 2,
        fill: true,
        tension: 0.4
      }]
    },
    options: chartOptions
  });

  chTmpAlt = new Chart(document.getElementById('chTmp'), {
    type: 'line',
    data: {
      labels: labels,
      datasets: [{
        label: 'Temperature (°C)',
        data: Array(12).fill(28),
        borderColor: '#f59e0b',
        backgroundColor: 'rgba(245,158,11,0.1)',
        borderWidth: 2,
        fill: true,
        tension: 0.4
      }]
    },
    options: chartOptions
  });

  chPres = new Chart(document.getElementById('chPres'), {
    type: 'line',
    data: {
      labels: labels,
      datasets: [{
        label: 'Pressure (hPa)',
        data: Array(12).fill(1008),
        borderColor: '#8b5cf6',
        backgroundColor: 'rgba(139,92,246,0.1)',
        borderWidth: 2,
        fill: true,
        tension: 0.4
      }]
    },
    options: chartOptions
  });

  chAqi = new Chart(document.getElementById('chAqi'), {
    type: 'line',
    data: {
      labels: labels,
      datasets: [{
        label: 'AQI Index',
        data: Array(12).fill(65),
        borderColor: '#ec4899',
        backgroundColor: 'rgba(236,72,153,0.1)',
        borderWidth: 2,
        fill: true,
        tension: 0.4
      }]
    },
    options: chartOptions
  });
}

/* ======================
   Gauge updates
   ====================== */
function setGauge(id, value, max){
  const circle = document.getElementById(id);
  const circumference = 2 * Math.PI * 50;
  const offset = circumference - (value/max)*circumference;
  circle.style.strokeDashoffset = offset;
}

/* ======================
   Status tag setter
   ====================== */
function setTag(elem, text, className){
  elem.textContent = text;
  elem.className = 'tag '+className;
}

/* ======================
   Update UI from balloonData
   ====================== */
function updateUI(){
  const d = balloonData;

  // Gauges
  setGauge('g-alt',  Math.min(d.altitude, 3000), 3000);
  setGauge('g-tmp',  Math.min(Math.max(d.temperature+10,0), 60), 60);
  setGauge('g-pres', Math.min(Math.max(d.pressure-950,0), 80), 80);
  setGauge('g-aqi',  Math.min(d.aqi, 300), 300);

  document.getElementById('v-alt').textContent   = Math.round(d.altitude);
  document.getElementById('v-tmp').textContent   = d.temperature.toFixed(1);
  document.getElementById('v-pres').textContent  = Math.round(d.pressure);
  document.getElementById('v-aqi').textContent   = Math.round(d.aqi);

  // text metrics
  document.getElementById('v-co').textContent    = d.co.toFixed(1);
  document.getElementById('v-smoke').textContent = d.smoke.toFixed(1);
  document.getElementById('v-speed').textContent = d.speed.toFixed(1);
  document.getElementById('v-wind').textContent  = (d.speed * 1.2).toFixed(1);

  document.getElementById('b-co').style.width    = Math.min(d.co*2, 100)+'%';
  document.getElementById('b-smoke').style.width = Math.min(d.smoke*2, 100)+'%';
  document.getElementById('b-speed').style.width = Math.min(d.speed*2, 100)+'%';
  document.getElementById('b-wind').style.width  = Math.min(d.speed*2.4, 100)+'%';

  // GPS
  document.getElementById('v-lat').textContent = d.latitude.toFixed(4)+'°';
  document.getElementById('v-lon').textContent = d.longitude.toFixed(4)+'°';
  document.getElementById('v-loc').textContent = 'Chennai, IN';
  document.getElementById('v-sats').textContent = '8';

  // Flight
  mission.maxAlt = Math.max(mission.maxAlt, d.altitude);
  document.getElementById('v-alt2').textContent   = Math.round(d.altitude);
  document.getElementById('v-maxalt').textContent = Math.round(mission.maxAlt);
  const ascentRate = (Math.random()*2 + 1.2).toFixed(2);
  document.getElementById('v-ascent').textContent = ascentRate;
  mission.dist += (d.speed/3600);
  document.getElementById('v-dist').textContent = mission.dist.toFixed(2);

  // Status
  const status = (d.altitude < 20) ? 'Grounded' : (ascentRate>0.2 ? 'Ascending' : 'Descending');
  document.getElementById('v-flight').textContent = status;
  const ring = document.getElementById('status-ring');
  ring.className = 'status-ring ' + (status==='Ascending'?'up':(status==='Descending'?'down':''));

  // Motion
  document.getElementById('v-pitch').textContent = (d.pitch>0?'+':'') + d.pitch.toFixed(1) + '°';
  document.getElementById('v-roll').textContent  = (d.roll>0?'+':'')  + d.roll.toFixed(1)  + '°';
  document.getElementById('v-vib').textContent   = d.vibration.toFixed(0);
  document.getElementById('horizon').style.transform = `rotate(${d.roll}deg)`;
  const stability = Math.max(0, 100 - Math.abs(d.pitch) - Math.abs(d.roll) - d.vibration*0.5);
  document.getElementById('stab-bar').style.width = stability.toFixed(0)+'%';
  setTag(document.getElementById('stab-tag'),
         stability>75?'STABLE':stability>50?'NORMAL':'UNSTABLE',
         stability>75?'tag-good':stability>50?'tag-info':'tag-bad');
  document.getElementById('stab-text').textContent = stability>75?'Attitude within nominal range.':
                                                      stability>50?'Slight attitude excursions detected.':
                                                      'High turbulence — stabilize balloon.';

  // Environmental status
  const wTag  = document.getElementById('env-weather');
  const aTag  = document.getElementById('env-aqi');
  const cTag  = document.getElementById('env-co');
  const sTag  = document.getElementById('env-smoke');

  let weather='Clear';
  if (d.speed*1.2>30) weather='Windy';
  else if (d.temperature>32) weather='Hot & Sunny';
  else if (d.temperature<20) weather='Cool';
  else if (d.aqi>120) weather='Hazy';
  else weather='Partly Cloudy';
  setTag(wTag, weather, 'tag-info');

  setTag(aTag, d.aqi<51?'Good':d.aqi<101?'Moderate':d.aqi<151?'Unhealthy (SG)':'Hazardous',
         d.aqi<101?'tag-good':d.aqi<151?'tag-mod':'tag-bad');
  document.getElementById('t-co').textContent = d.co<35?'Safe':d.co<70?'Elevated':'High';
  setTag(cTag, d.co<35?'Normal':d.co<70?'Elevated':'Alert', d.co<35?'tag-good':d.co<70?'tag-mod':'tag-bad');
  document.getElementById('t-smoke').textContent = d.smoke<30?'Clear':d.smoke<60?'Slight':'Smoke!';
  setTag(sTag, d.smoke<30?'Clear':d.smoke<60?'Detected':'Warning', d.smoke<30?'tag-good':d.smoke<60?'tag-mod':'tag-bad');

  document.getElementById('t-wind').textContent =
    (d.speed*1.2)<10?'Calm':(d.speed*1.2)<20?'Light Breeze':(d.speed*1.2)<35?'Breezy':'Windy';

  // Charts push
  labels.shift(); labels.push('T+');
  [chAlt, chTmpAlt, chPres, chAqi].forEach((c,i)=>{
    c.data.labels = labels;
    c.data.datasets[0].data.shift();
  });
  chAlt.data.datasets[0].data.push(+d.altitude.toFixed(1));
  chTmpAlt.data.datasets[0].data.push(+d.temperature.toFixed(1));
  chPres.data.datasets[0].data.push(+d.pressure.toFixed(1));
  chAqi.data.datasets[0].data.push(+d.aqi.toFixed(0));
  [chAlt, chTmpAlt, chPres, chAqi].forEach(c=>c.update('none'));

  // Map route
  mission.route.push([d.latitude, d.longitude]);
  if(mission.route.length>400) mission.route.shift();
  routeLine.setLatLngs(mission.route);
  balloonMarker.setLatLng([d.latitude, d.longitude]);
  balloonMarker.setPopupContent(`<b>Nimbus-1</b><br/>Alt: ${Math.round(d.altitude)} m<br/>Temp: ${d.temperature.toFixed(1)}°C<br/>Spd: ${d.speed.toFixed(1)} km/h`);

  mission.packets++;
  document.getElementById('mc-pack').textContent = mission.packets;
}

/* ======================
   Fetch live data from ESP32
   ====================== */
async function fetchLiveData(){
  try {
    const response = await fetch('/data');
    if(response.ok){
      const data = await response.json();
      balloonData = {
        altitude: data.altitude || 0,
        temperature: data.temperature || 0,
        pressure: data.pressure || 0,
        aqi: data.aqi || 0,
        co: data.co || 0,
        smoke: data.smoke || 0,
        latitude: data.latitude || 13.0827,
        longitude: data.longitude || 80.2707,
        speed: data.speed || 0,
        pitch: data.pitch || 0,
        roll: data.roll || 0,
        vibration: data.vibration || 0
      };
      updateUI();
    }
  } catch(e) {
    console.log('Fetch error:', e);
  }
}

/* ======================
   Simulate realistic data drift
   ====================== */
function simulateData(){
  const d = balloonData;
  d.altitude  = Math.max(0, d.altitude + (Math.random()-0.2)*4);
  d.temperature += (Math.random()-0.5)*0.3;
  d.pressure  += (Math.random()-0.5)*0.4;
  d.aqi       = Math.max(10, Math.min(250, d.aqi + (Math.random()-0.5)*3));
  d.co        = Math.max(0, d.co + (Math.random()-0.5)*1.5);
  d.smoke     = Math.max(0, d.smoke + (Math.random()-0.5)*1.2);
  d.speed     = Math.max(0, d.speed + (Math.random()-0.5)*1.2);
  d.pitch     = Math.max(-45, Math.min(45, d.pitch + (Math.random()-0.5)*1.5));
  d.roll      = Math.max(-45, Math.min(45, d.roll  + (Math.random()-0.5)*1.5));
  d.vibration = Math.max(0, Math.min(60, d.vibration + (Math.random()-0.5)*2));
  d.latitude  += (Math.random()-0.5)*0.0008;
  d.longitude += (Math.random()-0.5)*0.0008;
  
  updateUI();
}

/* ======================
   Toggle Demo Mode
   ====================== */
function toggleDemoMode(){
  demoModeEnabled = !demoModeEnabled;
  const btn = document.getElementById('demo-mode-btn');
  const badge = document.getElementById('demo-badge-header');
  
  if(demoModeEnabled){
    btn.className = 'btn btn-demo-active';
    document.getElementById('demo-btn-text').textContent = '🟠 DEMO';
    badge.style.display = 'inline-flex';
    
    if(dataFetchInterval) clearInterval(dataFetchInterval);
    window.simulateInterval = setInterval(simulateData, 1000);
    log('Demo Mode enabled','INFO');
  } else {
    btn.className = 'btn btn-demo-live';
    document.getElementById('demo-btn-text').textContent = '🟢 LIVE';
    badge.style.display = 'none';
    
    if(window.simulateInterval) clearInterval(window.simulateInterval);
    dataFetchInterval = setInterval(fetchLiveData, 1000);
    fetchLiveData();
    log('Live Mode enabled','OK');
  }
}

/* ======================
   Mission control
   ====================== */
function resetData(){
  mission = { running:true, startTime: Date.now(), maxAlt:0, dist:0, packets:0, history:[], route:[[13.0827,80.2707]], _elapsed:0 };
  balloonData = { altitude: 125, temperature: 28, pressure: 1008, aqi: 65, co: 12, smoke: 18,
                  latitude: 13.0827, longitude: 80.2707, speed: 15, pitch: 5, roll: 2, vibration: 10 };
  document.getElementById('log').innerHTML='';
  log('Data reset · Mission clock zeroed','INFO');
  updateUI();
}

function exportData(){
  const header = 'timestamp,altitude,temperature,pressure,aqi,co,smoke,latitude,longitude,speed,pitch,roll,vibration';
  const rows = [header];
  const d = balloonData;
  for(let i=0;i<60;i++){
    const ts = new Date(Date.now()-i*1000).toISOString();
    rows.push([ts, (d.altitude+Math.random()*10).toFixed(1), (d.temperature+Math.random()).toFixed(2),
               (d.pressure+Math.random()*2).toFixed(2), Math.round(d.aqi+Math.random()*5),
               (d.co+Math.random()*2).toFixed(2), (d.smoke+Math.random()*2).toFixed(2),
               d.latitude.toFixed(6), d.longitude.toFixed(6), (d.speed+Math.random()).toFixed(2),
               (d.pitch+Math.random()*2).toFixed(2), (d.roll+Math.random()*2).toFixed(2), 
               Math.round(d.vibration+Math.random()*3)].join(','));
  }
  const csv = 'data:text/csv;charset=utf-8,' + encodeURIComponent(rows.join('\n'));
  const a = document.createElement('a');
  a.href=csv; a.download=`nimbus1_flight_${Date.now()}.csv`; a.click();
  log('Flight data exported to CSV','OK');
}

/* ======================
   Logger
   ====================== */
function log(msg, level){
  const logElem = document.getElementById('log');
  const time = new Date().toLocaleTimeString();
  const entry = document.createElement('div');
  entry.innerHTML = `<span class="t">[${time}]</span> <span class="lv">${level}</span> <span class="m">${msg}</span>`;
  logElem.insertBefore(entry, logElem.firstChild);
  if(logElem.children.length > 8) logElem.removeChild(logElem.lastChild);
}

/* ======================
   Initialize on load
   ====================== */
window.addEventListener('load', ()=>{
  initMap();
  initCharts();
  updateUI();
  
  // Start with LIVE mode (fetch from ESP32)
  demoModeEnabled = false;
  dataFetchInterval = setInterval(fetchLiveData, 1000);
  fetchLiveData();
  
  // Ensure map renders
  setTimeout(()=>map.invalidateSize(), 400);
});
</script>
</body>
</html>
)rawliteral";

// ============================================================================
//  OBJECT INSTANCES
// ============================================================================
Adafruit_BMP085  bmp180;
MPU6050          mpu6050;
TinyGPSPlus      gps;
HardwareSerial   gpsSerial(2);
WebServer        server(80);

// ============================================================================
//  SENSOR STATUS FLAGS
// ============================================================================
bool bmp180_ok  = false;
bool mpu6050_ok = false;

// ============================================================================
//  EMA STATE VARIABLES  (hold the running filtered value for each channel)
// ============================================================================
float ema_temp      = 25.0f;
float ema_aqi       = 0.0f;
float ema_co        = 0.0f;
float ema_smoke     = 0.0f;
float ema_altitude  = 0.0f;
float ema_pressure  = 1013.25f;
float ema_pitch     = 0.0f;
float ema_roll      = 0.0f;
float ema_vibration = 0.0f;
float ema_speed     = 0.0f;

// Flags to detect first sample (skip spike check on first reading)
bool first_temp     = true;
bool first_alt      = true;
bool first_pres     = true;
bool first_aqi      = true;
bool first_co       = true;
bool first_smoke    = true;
bool first_pitch    = true;
bool first_roll     = true;

// ============================================================================
//  LIVE TELEMETRY STRUCT  (filled by readSensors(), served by /data)
// ============================================================================
struct {
  float   altitude;
  float   temperature;
  float   pressure;
  int     aqi;
  int     co;
  int     smoke;
  double  latitude;
  double  longitude;
  float   speed;
  float   pitch;
  float   roll;
  int     vibration;
} tele;

// ============================================================================
//  TIMING
// ============================================================================
unsigned long lastSensorRead = 0;

// ============================================================================
//  FUNCTION PROTOTYPES
// ============================================================================
void   setupWiFiAP();
void   setupSensors();
void   readSensors();
void   handleRoot();
void   handleData();
void   handleNotFound();

// Filtering helpers
int    medianFilter(uint8_t pin, uint8_t samples);
float  applyEMA(float newVal, float prevEMA, float alpha);
bool   spikeReject(float newVal, float prevVal, float threshold, bool isFirst);
int    multiSampleADC(uint8_t pin, uint8_t samples);

// ============================================================================
//  SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("\n============================================="));
  Serial.println(F("  Nimbus-1  Smart Weather Balloon System    "));
  Serial.println(F("=============================================\n"));

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_ESP_RX, GPS_ESP_TX);

  memset(&tele, 0, sizeof(tele));

  setupSensors();
  setupWiFiAP();

  server.on("/",     HTTP_GET, handleRoot);
  server.on("/data", HTTP_GET, handleData);
  server.onNotFound(handleNotFound);
  server.begin();

  Serial.println(F("[SERVER] HTTP server started."));
  Serial.println(F("[SERVER] Dashboard → http://192.168.4.1\n"));
}

// ============================================================================
//  LOOP
// ============================================================================
void loop() {
  // Feed GPS parser non-blocking
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // Read sensors on schedule
  if (millis() - lastSensorRead >= SENSOR_INTERVAL_MS) {
    lastSensorRead = millis();
    readSensors();
  }

  server.handleClient();
}

// ============================================================================
//  setupWiFiAP()
// ============================================================================
void setupWiFiAP() {
  Serial.println(F("[WiFi]  Starting Access Point..."));
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_GW, AP_SUBNET);

  if (WiFi.softAP(AP_SSID, AP_PASSWORD)) {
    Serial.print(F("[WiFi]  SSID     : ")); Serial.println(AP_SSID);
    Serial.print(F("[WiFi]  Password : ")); Serial.println(AP_PASSWORD);
    Serial.print(F("[WiFi]  IP Addr  : ")); Serial.println(WiFi.softAPIP());
  } else {
    Serial.println(F("[WiFi]  ERROR — Access Point failed to start!"));
  }
}

// ============================================================================
//  setupSensors()
// ============================================================================
void setupSensors() {
  Serial.println(F("[SENSORS] Initialising...\n"));

  if (bmp180.begin()) {
    bmp180_ok = true;
    Serial.println(F("[BMP180]  OK  — Pressure & Altitude ready."));
  } else {
    Serial.println(F("[BMP180]  FAIL — Not detected on I2C. altitude/pressure = 0."));
  }

  mpu6050.initialize();
  if (mpu6050.testConnection()) {
    mpu6050_ok = true;
    Serial.println(F("[MPU6050] OK  — Motion & Orientation ready."));
  } else {
    Serial.println(F("[MPU6050] FAIL — Not detected on I2C. pitch/roll/vibration = 0."));
  }

  Serial.println(F("[NEO-6M]  Listening on UART2. lat/lon/speed = 0 until fix."));
  Serial.println(F("[LM35]    GPIO33 — ready."));
  Serial.println(F("[MQ135]   GPIO34 — ready. (verify voltage divider!)"));
  Serial.println(F("[MQ2]     GPIO35 — ready. (verify voltage divider!)"));
  Serial.println(F("[MQ7]     GPIO32 — ready. (verify voltage divider!)"));
  Serial.println(F("\n[SENSORS] Done.\n"));
}

// ============================================================================
//  readSensors()
//
//  Filtering pipeline per sensor:
//
//  Analog (LM35, MQ135, MQ2, MQ7):
//    Step 1 — Multi-sample ADC average    (removes random ADC quantisation noise)
//    Step 2 — Median filter               (removes impulse / spike samples)
//    Step 3 — Spike rejection             (discards physically impossible jumps)
//    Step 4 — EMA low-pass filter         (smooths remaining jitter)
//
//  BMP180 (I2C):
//    Step 1 — Spike rejection
//    Step 2 — EMA low-pass filter
//
//  MPU6050 (I2C):
//    Step 1 — Multi-sample average of raw axes
//    Step 2 — Spike rejection on pitch/roll
//    Step 3 — EMA low-pass filter on pitch, roll, vibration
//
//  GPS:
//    Step 1 — TinyGPS++ internal NMEA validation (only isValid() data used)
//    Step 2 — EMA on speed
// ============================================================================
void readSensors() {

  // ── LM35 : Temperature ───────────────────────────────────────────────────
  {
    // Step 1+2: median of MEDIAN_SAMPLES multi-averaged readings
    int   medRaw = medianFilter(PIN_LM35, MEDIAN_SAMPLES);
    float mV     = (medRaw / 4095.0f) * 3300.0f;
    float rawTemp = (mV / 10.0f) + LM35_OFFSET;

    // Step 3: spike rejection
    if (spikeReject(rawTemp, ema_temp, SPIKE_THRESH_TEMP, first_temp)) {
      first_temp = false;
      // Step 4: EMA
      ema_temp = applyEMA(rawTemp, ema_temp, EMA_ALPHA_TEMP);
    }
    tele.temperature = ema_temp;
  }

  // ── MQ135 : Pseudo-AQI ───────────────────────────────────────────────────
  {
    int medRaw      = medianFilter(PIN_MQ135, MEDIAN_SAMPLES);
    int compensated = constrain(medRaw * MQ_VD_SCALE, 0, 4095);
    int rawAqi      = map(compensated, 0, 4095, 0, MQ135_AQI_MAX);

    if (spikeReject((float)rawAqi, ema_aqi, (float)SPIKE_THRESH_AQI, first_aqi)) {
      first_aqi = false;
      ema_aqi   = applyEMA((float)rawAqi, ema_aqi, EMA_ALPHA_AQI);
    }
    tele.aqi = (int)roundf(ema_aqi);
  }

  // ── MQ2 : Smoke / LPG ────────────────────────────────────────────────────
  {
    int medRaw      = medianFilter(PIN_MQ2, MEDIAN_SAMPLES);
    int compensated = constrain(medRaw * MQ_VD_SCALE, 0, 4095);
    int rawSmoke    = map(compensated, 0, 4095, 0, MQ2_SMOKE_PPM_MAX);

    if (spikeReject((float)rawSmoke, ema_smoke, (float)SPIKE_THRESH_SMOKE, first_smoke)) {
      first_smoke = false;
      ema_smoke   = applyEMA((float)rawSmoke, ema_smoke, EMA_ALPHA_SMOKE);
    }
    tele.smoke = (int)roundf(ema_smoke);
  }

  // ── MQ7 : Carbon Monoxide ────────────────────────────────────────────────
  {
    int medRaw      = medianFilter(PIN_MQ7, MEDIAN_SAMPLES);
    int compensated = constrain(medRaw * MQ_VD_SCALE, 0, 4095);
    int rawCO       = map(compensated, 0, 4095, 0, MQ7_CO_PPM_MAX);

    if (spikeReject((float)rawCO, ema_co, (float)SPIKE_THRESH_CO, first_co)) {
      first_co = false;
      ema_co   = applyEMA((float)rawCO, ema_co, EMA_ALPHA_CO);
    }
    tele.co = (int)roundf(ema_co);
  }

  // ── BMP180 : Pressure & Altitude ─────────────────────────────────────────
  if (bmp180_ok) {
    float rawPa = (float)bmp180.readPressure();
    if (rawPa > 0.0f) {
      float rawPres = rawPa / 100.0f;
      float rawAlt  = 44330.0f * (1.0f - powf(rawPres / 1013.25f, 1.0f / 5.255f));

      // Spike rejection
      if (spikeReject(rawPres, ema_pressure, SPIKE_THRESH_PRESSURE, first_pres)) {
        first_pres = false;
        ema_pressure = applyEMA(rawPres, ema_pressure, EMA_ALPHA_PRESSURE);
      }
      if (spikeReject(rawAlt, ema_altitude, SPIKE_THRESH_ALTITUDE, first_alt)) {
        first_alt = false;
        ema_altitude = applyEMA(rawAlt, ema_altitude, EMA_ALPHA_ALTITUDE);
      }
    }
    tele.pressure = ema_pressure;
    tele.altitude = ema_altitude;
  }

  // ── NEO-6M GPS : Latitude, Longitude, Speed ──────────────────────────────
  // TinyGPS++ validates NMEA checksum and fix quality internally.
  // isValid() is the primary filter — only trust confirmed fixes.
  if (gps.location.isValid() && gps.location.isUpdated()) {
    tele.latitude  = gps.location.lat();
    tele.longitude = gps.location.lng();
  }
  if (gps.speed.isValid() && gps.speed.isUpdated()) {
    float rawSpeed = (float)gps.speed.kmph();
    ema_speed      = applyEMA(rawSpeed, ema_speed, EMA_ALPHA_SPEED);
    tele.speed     = ema_speed;
  }

  // ── MPU6050 : Pitch, Roll, Vibration ─────────────────────────────────────
  if (mpu6050_ok) {
    // Step 1: average 8 raw samples per axis to reduce quantisation noise
    const uint8_t IMU_SAMPLES = 8;
    long sumAx = 0, sumAy = 0, sumAz = 0;
    for (uint8_t i = 0; i < IMU_SAMPLES; i++) {
      int16_t ax, ay, az, gx, gy, gz;
      mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      sumAx += ax; sumAy += ay; sumAz += az;
      delayMicroseconds(500);   // short gap between burst reads
    }
    float axg = (sumAx / (float)IMU_SAMPLES) / MPU_LSB_PER_G;
    float ayg = (sumAy / (float)IMU_SAMPLES) / MPU_LSB_PER_G;
    float azg = (sumAz / (float)IMU_SAMPLES) / MPU_LSB_PER_G;

    // Step 2: compute angles
    float rawPitch = atan2f(ayg, sqrtf(axg * axg + azg * azg)) * (180.0f / M_PI);
    float rawRoll  = atan2f(axg, sqrtf(ayg * ayg + azg * azg)) * (180.0f / M_PI);

    // Step 3: spike rejection on pitch and roll
    if (spikeReject(rawPitch, ema_pitch, SPIKE_THRESH_PITCH, first_pitch)) {
      first_pitch = false;
      ema_pitch   = applyEMA(rawPitch, ema_pitch, EMA_ALPHA_PITCH);
    }
    if (spikeReject(rawRoll, ema_roll, SPIKE_THRESH_ROLL, first_roll)) {
      first_roll = false;
      ema_roll   = applyEMA(rawRoll, ema_roll, EMA_ALPHA_ROLL);
    }

    // Step 4: vibration from total acceleration deviation from 1g
    float mag    = sqrtf(axg * axg + ayg * ayg + azg * azg);
    float rawVib = fabsf(mag - 1.0f) * VIBRATION_SCALE;
    ema_vibration = applyEMA(rawVib, ema_vibration, EMA_ALPHA_VIBRATION);

    tele.pitch     = ema_pitch;
    tele.roll      = ema_roll;
    tele.vibration = (int)constrain(ema_vibration, 0.0f, 999.0f);
  }
}

// ============================================================================
//  handleRoot()
// ============================================================================
void handleRoot() {
  server.send_P(200, "text/html", index_html);
}

// ============================================================================
//  handleData()
// ============================================================================
void handleData() {
  StaticJsonDocument<256> doc;

  doc["altitude"]    = roundf(tele.altitude    * 10.0f) / 10.0f;
  doc["temperature"] = roundf(tele.temperature * 10.0f) / 10.0f;
  doc["pressure"]    = roundf(tele.pressure    * 10.0f) / 10.0f;
  doc["aqi"]         = tele.aqi;
  doc["co"]          = tele.co;
  doc["smoke"]       = tele.smoke;
  doc["latitude"]    = tele.latitude;
  doc["longitude"]   = tele.longitude;
  doc["speed"]       = roundf(tele.speed * 10.0f) / 10.0f;
  doc["pitch"]       = roundf(tele.pitch * 10.0f) / 10.0f;
  doc["roll"]        = roundf(tele.roll  * 10.0f) / 10.0f;
  doc["vibration"]   = tele.vibration;

  String json;
  serializeJson(doc, json);

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", json);
}

// ============================================================================
//  handleNotFound()
// ============================================================================
void handleNotFound() {
  server.send(404, "text/plain", "404 Not Found");
}

// ============================================================================
//  FILTERING HELPER FUNCTIONS
// ============================================================================

// ----------------------------------------------------------------------------
//  multiSampleADC()
//  Reads the same ADC pin `samples` times and returns the integer average.
//  Reduces random quantisation noise from the ESP32 SAR ADC.
// ----------------------------------------------------------------------------
int multiSampleADC(uint8_t pin, uint8_t samples) {
  long sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(200);   // let ADC settle between reads
  }
  return (int)(sum / samples);
}

// ----------------------------------------------------------------------------
//  medianFilter()
//  Takes `samples` multi-averaged ADC readings, sorts them, returns the median.
//  `samples` should be odd (3, 5, 7, 9 ...).
//  Best defence against impulse noise and single-sample spikes.
// ----------------------------------------------------------------------------
int medianFilter(uint8_t pin, uint8_t samples) {
  int buf[MEDIAN_SAMPLES];
  uint8_t n = min(samples, (uint8_t)MEDIAN_SAMPLES);

  for (uint8_t i = 0; i < n; i++) {
    buf[i] = multiSampleADC(pin, 8);   // 8 reads averaged per median slot
  }

  // Insertion sort (tiny array — no overhead)
  for (uint8_t i = 1; i < n; i++) {
    int key = buf[i];
    int8_t j = i - 1;
    while (j >= 0 && buf[j] > key) {
      buf[j + 1] = buf[j];
      j--;
    }
    buf[j + 1] = key;
  }

  return buf[n / 2];   // middle element
}

// ----------------------------------------------------------------------------
//  applyEMA()
//  Exponential Moving Average:
//    output = alpha * newValue + (1 - alpha) * previousEMA
//  Lower alpha = more smoothing, slower response.
// ----------------------------------------------------------------------------
float applyEMA(float newVal, float prevEMA, float alpha) {
  return (alpha * newVal) + ((1.0f - alpha) * prevEMA);
}

// ----------------------------------------------------------------------------
//  spikeReject()
//  Returns true  → value is valid, proceed with filtering.
//  Returns false → value is a spike, discard and keep previous EMA.
//  `isFirst` bypasses the check on the very first sample to seed the EMA.
// ----------------------------------------------------------------------------
bool spikeReject(float newVal, float prevVal, float threshold, bool isFirst) {
  if (isFirst) return true;
  return (fabsf(newVal - prevVal) <= threshold);
}
