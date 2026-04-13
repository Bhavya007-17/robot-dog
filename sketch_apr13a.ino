
// Libraries: Install "ESP32Servo" from Library Manager (ESP32 Arduino core includes WiFi/WebServer).

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ESP32Servo.h>
#include <cctype>

// --- Wi‑Fi Access Point ---
static const char *AP_SSID = "RobotDog-ESP32";
static const char *AP_PASS = "robotdog123";  // min 8 chars for WPA2

// --- Servo pins (ESP32-safe outputs) ---
static const int PIN_SERVO_FL = 13;    // front left leg  (was 2)
static const int PIN_SERVO_BR = 14;    // back right leg  (was 3)
static const int PIN_SERVO_FR = 25;    // front right leg (was 4)
static const int PIN_SERVO_BL = 26;    // back left leg   (was 5)
static const int PIN_SERVO_SLIDE = 27; // slider          (was 6)
static const int PIN_SERVO_ROT = 33;   // rotation        (was 7)

// Per-servo trim (microseconds) so “logical” poses match the mechanics. Edit defaults here,
// then use Serial: +1/-1 … +6/-6 (±10 µs), p=print, s=save to flash, d=reload defaults.
static const uint32_t CAL_PREFS_VER = 1;
static const int DEFAULT_SERVO_OFFSET_US[6] = { 0, 0, 0, 0, 0, 100 };
static const int SERVO_US_MIN = 500;
static const int SERVO_US_MAX = 2500;
static const int CAL_STEP_US = 10;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;

WebServer server(80);

float servo1Pos;
float servo1PosFiltered;
float servo2Pos;
float servo2PosFiltered;
float servo3Pos;
float servo3PosFiltered;
float servo4Pos;
float servo4PosFiltered;
float servo5Pos;
float servo5PosFiltered;
float servo6Pos;
float servo6PosFiltered;

int servo1Offset;
int servo2Offset;
int servo3Offset;
int servo4Offset;
int servo5Offset;
int servo6Offset;

unsigned long currentMillis;
long previousMillis = 0;
long previousWalkMillis = 0;

// Delay between gait phases (ms). Walk uses the slider + four legs so the same
// global delay feels slower than turn (mostly body twist); tune independently.
int stepTimeWalk = 110;
int stepTimeTurn = 200;
int stepTimeTrick = 180;
int filterVal = 5;

int walkAction = 0;
int walkCount = 0;
int inMotionFlag = 0;

unsigned long lastCmdMs = 0;
const unsigned long CMD_TIMEOUT_MS = 800;

static int constrainedMicros(float filtered, int offset) {
  return constrain((int)(filtered + offset), SERVO_US_MIN, SERVO_US_MAX);
}

// Pin 26 / back left: mount direction opposite other legs — mirror logical µs around 1500.
static float servo4PulseOut(float logicalUs) {
  return 3000.0f - logicalUs;
}

void applyDefaultOffsets() {
  servo1Offset = DEFAULT_SERVO_OFFSET_US[0];
  servo2Offset = DEFAULT_SERVO_OFFSET_US[1];
  servo3Offset = DEFAULT_SERVO_OFFSET_US[2];
  servo4Offset = DEFAULT_SERVO_OFFSET_US[3];
  servo5Offset = DEFAULT_SERVO_OFFSET_US[4];
  servo6Offset = DEFAULT_SERVO_OFFSET_US[5];
}

void loadCalibrationFromNVS() {
  Preferences p;
  if (!p.begin("dogcal", true)) {
    applyDefaultOffsets();
    return;
  }
  if (p.getUInt("ver", 0) != CAL_PREFS_VER) {
    p.end();
    applyDefaultOffsets();
    return;
  }
  servo1Offset = p.getInt("o1", DEFAULT_SERVO_OFFSET_US[0]);
  servo2Offset = p.getInt("o2", DEFAULT_SERVO_OFFSET_US[1]);
  servo3Offset = p.getInt("o3", DEFAULT_SERVO_OFFSET_US[2]);
  servo4Offset = p.getInt("o4", DEFAULT_SERVO_OFFSET_US[3]);
  servo5Offset = p.getInt("o5", DEFAULT_SERVO_OFFSET_US[4]);
  servo6Offset = p.getInt("o6", DEFAULT_SERVO_OFFSET_US[5]);
  p.end();
}

void saveCalibrationToNVS() {
  Preferences p;
  if (!p.begin("dogcal", false)) {
    Serial.println("NVS open failed");
    return;
  }
  p.putUInt("ver", CAL_PREFS_VER);
  p.putInt("o1", servo1Offset);
  p.putInt("o2", servo2Offset);
  p.putInt("o3", servo3Offset);
  p.putInt("o4", servo4Offset);
  p.putInt("o5", servo5Offset);
  p.putInt("o6", servo6Offset);
  p.end();
  Serial.println("Calibration saved to flash.");
}

void printCalibrationSerial() {
  Serial.printf(
      "Offsets us: 1=%d 2=%d 3=%d 4=%d 5=%d 6=%d  (FL,BR,FR,BL,slide,rot)\n",
      servo1Offset, servo2Offset, servo3Offset, servo4Offset, servo5Offset, servo6Offset);
}

void processSerialCalibration() {
  if (!Serial.available()) {
    return;
  }
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) {
    return;
  }

  if ((line.charAt(0) == '+' || line.charAt(0) == '-') && line.length() >= 2) {
    int ch = line.charAt(1);
    if (ch >= '1' && ch <= '6') {
      int idx = ch - '1';
      int delta = (line.charAt(0) == '+') ? CAL_STEP_US : -CAL_STEP_US;
      switch (idx) {
        case 0: servo1Offset += delta; break;
        case 1: servo2Offset += delta; break;
        case 2: servo3Offset += delta; break;
        case 3: servo4Offset += delta; break;
        case 4: servo5Offset += delta; break;
        case 5: servo6Offset += delta; break;
        default: break;
      }
      printCalibrationSerial();
    }
    return;
  }

  if (line.equalsIgnoreCase("p") || line.equalsIgnoreCase("print")) {
    printCalibrationSerial();
    return;
  }
  if (line.equalsIgnoreCase("s") || line.equalsIgnoreCase("save")) {
    saveCalibrationToNVS();
    return;
  }
  if (line.equalsIgnoreCase("d") || line.equalsIgnoreCase("defaults")) {
    applyDefaultOffsets();
    printCalibrationSerial();
    Serial.println("Defaults applied in RAM (upload again or type s to overwrite flash).");
    return;
  }
  if (line.equalsIgnoreCase("h") || line.equalsIgnoreCase("help")) {
    Serial.println("Cal: +1..+6 / -1..-6 = nudge offset ±10us | p=print | s=save | d=defaults");
  }
}

void applyRestingPose() {
  servo1Pos = 1000;
  servo3Pos = 1000;
  servo2Pos = 2000;
  servo4Pos = 2000;
  servo5Pos = 1500;
  servo6Pos = 1500;
}

void applyCommand(char key) {
  key = static_cast<char>(std::toupper(static_cast<unsigned char>(key)));

  if (key == 'X' || key == ' ') {
    walkAction = 5;
    walkCount = 0;
    inMotionFlag = 1;
    return;
  }

  if (inMotionFlag) {
    return;
  }

  int next = 0;
  switch (key) {
    case 'W': next = 1; break;
    case 'S': next = 2; break;
    case 'A': next = 3; break;
    case 'D': next = 4; break;
    case 'L': next = 6; break;  // lift one leg (front left)
    case 'B': next = 7; break;  // bow — weight on front two legs
    case 'G': next = 8; break;  // shake / wag (body rotate)
    default: return;
  }

  walkAction = next;
  walkCount = 0;
}

void checkCommandTimeout() {
  if (walkAction == 0 || walkAction == 5) {
    return;
  }
  if (inMotionFlag) {
    return;
  }
  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    walkAction = 5;
    walkCount = 0;
  }
}

static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Mini Dog</title>
<style>
body{font-family:sans-serif;text-align:center;background:#111;color:#eee;padding:1rem;}
h1{font-size:1.2rem;}
h2{font-size:1rem;margin:1.25rem 0 0.5rem;color:#aaa;font-weight:600;}
.grid{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;max-width:320px;margin:1rem auto;align-items:center;}
.tricks{grid-template-columns:1fr 1fr 1fr;max-width:320px;}
button{padding:14px 10px;font-size:0.95rem;border-radius:8px;border:none;cursor:pointer;background:#333;color:#fff;}
button:active{background:#555;}
.w{background:#2a6;}
.s{background:#a62;}
.a{background:#26a;}
.d{background:#26a;}
.x{background:#444;}
.t1{background:#374;}
.t2{background:#537;}
.t3{background:#735;}
kbd{background:#222;padding:2px 6px;border-radius:4px;}
.hint{font-size:0.85rem;color:#888;margin-top:0.5rem;}
</style></head>
<body>
<h1>RobotDog ESP32</h1>
<p>Join Wi‑Fi <strong>RobotDog-ESP32</strong> then use buttons or keys <kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd> <kbd>L</kbd><kbd>B</kbd><kbd>G</kbd></p>
<div class="grid">
<div></div><button class="w" id="btnW">Forward</button><div></div>
<button class="a" id="btnA">Left</button><button class="x" id="btnX">Stop</button><button class="d" id="btnD">Right</button>
<div></div><button class="s" id="btnS">Back</button><div></div>
</div>
<h2>Tricks</h2>
<div class="grid tricks">
<button class="t1" id="btnL">Lift one leg</button>
<button class="t2" id="btnB">Bow (front 2)</button>
<button class="t3" id="btnG">Shake</button>
</div>
<p class="hint">Lift raises front-left; bow leans on both front legs; shake wags the body.</p>
<script>
function cmd(k){
  fetch('/cmd?key='+encodeURIComponent(k)).catch(function(){});
}
document.getElementById('btnW').onclick=function(){cmd('W');};
document.getElementById('btnS').onclick=function(){cmd('S');};
document.getElementById('btnA').onclick=function(){cmd('A');};
document.getElementById('btnD').onclick=function(){cmd('D');};
document.getElementById('btnX').onclick=function(){cmd('X');};
document.getElementById('btnL').onclick=function(){cmd('L');};
document.getElementById('btnB').onclick=function(){cmd('B');};
document.getElementById('btnG').onclick=function(){cmd('G');};
document.addEventListener('keydown',function(e){
  var m={87:'W',83:'S',65:'A',68:'D',32:'X',88:'X',76:'L',66:'B',71:'G'};
  if(m[e.keyCode]){e.preventDefault();cmd(m[e.keyCode]);}
});
</script>
</body></html>
)rawliteral";

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleCmd() {
  if (!server.hasArg("key")) {
    server.send(400, "text/plain", "missing key");
    return;
  }
  String k = server.arg("key");
  if (k.length() == 0) {
    server.send(400, "text/plain", "empty key");
    return;
  }
  lastCmdMs = millis();
  applyCommand(k.charAt(0));
  server.send(200, "text/plain", "ok");
}

void setup() {
  Serial.begin(115200);

  servo1.attach(PIN_SERVO_FL);
  servo2.attach(PIN_SERVO_BR);
  servo3.attach(PIN_SERVO_FR);
  servo4.attach(PIN_SERVO_BL);
  servo5.attach(PIN_SERVO_SLIDE);
  servo6.attach(PIN_SERVO_ROT);

  loadCalibrationFromNVS();

  applyRestingPose();
  servo1PosFiltered = servo1Pos;
  servo2PosFiltered = servo2Pos;
  servo3PosFiltered = servo3Pos;
  servo4PosFiltered = servo4Pos;
  servo5PosFiltered = servo5Pos;
  servo6PosFiltered = servo6Pos;

  servo1.writeMicroseconds(constrainedMicros(servo1Pos, servo1Offset));
  servo2.writeMicroseconds(constrainedMicros(servo2Pos, servo2Offset));
  servo3.writeMicroseconds(constrainedMicros(servo3Pos, servo3Offset));
  servo4.writeMicroseconds(constrainedMicros(servo4PulseOut(servo4Pos), servo4Offset));
  servo5.writeMicroseconds(constrainedMicros(servo5Pos, servo5Offset));
  servo6.writeMicroseconds(constrainedMicros(servo6Pos, servo6Offset));

  walkAction = 5;
  walkCount = 0;

  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(AP_SSID, AP_PASS)) {
    Serial.println("AP start failed");
  } else {
    IPAddress ip = WiFi.softAPIP();
    Serial.print("AP IP: ");
    Serial.println(ip);
  }

  server.on("/", HTTP_GET, handleRoot);
  server.on("/cmd", HTTP_GET, handleCmd);
  server.begin();
  Serial.println("HTTP server on port 80");
  printCalibrationSerial();
  Serial.println("Cal: +1/-1 … +6/-6 (10us), p, s=save, d=defaults, h=help");
}

void loop() {
  server.handleClient();
  processSerialCalibration();

  currentMillis = millis();
  if (currentMillis - previousMillis >= 10) {
    previousMillis = currentMillis;

    checkCommandTimeout();

    // *** STAGGERED STAND-UP (walkAction 5) ***
    if (walkAction == 5) {
      inMotionFlag = 1;
      if (walkCount == 0) {
        servo5Pos = 1500;
        servo6Pos = 1500;
        previousWalkMillis = currentMillis;
        walkCount = 1;
      } else if (walkCount == 1 && currentMillis - previousWalkMillis >= 500) {
        servo1Pos = 1800;
        servo3Pos = 1800;
        previousWalkMillis = currentMillis;
        walkCount = 2;
      } else if (walkCount == 2 && currentMillis - previousWalkMillis >= 500) {
        servo2Pos = 1200;
        servo4Pos = 1200;
        previousWalkMillis = currentMillis;
        walkCount = 3;
      } else if (walkCount == 3 && currentMillis - previousWalkMillis >= 500) {
        inMotionFlag = 0;
        walkAction = 0;
      }
    }

    // *** WALK FORWARD ***
    if (walkAction == 1) {
      inMotionFlag = 1;
      if (walkCount == 0) {
        servo1Pos = 1000;
        servo2Pos = 2000;
        previousWalkMillis = currentMillis;
        walkCount = 1;
      } else if (walkCount == 1 && currentMillis - previousWalkMillis >= stepTimeWalk) {
        servo5Pos = 1000;
        previousWalkMillis = currentMillis;
        walkCount = 2;
      } else if (walkCount == 2 && currentMillis - previousWalkMillis >= stepTimeWalk) {
        servo1Pos = 1800;
        servo2Pos = 1200;
        previousWalkMillis = currentMillis;
        walkCount = 3;
      } else if (walkCount == 3 && currentMillis - previousWalkMillis >= stepTimeWalk) {
        servo3Pos = 1000;
        servo4Pos = 2000;
        previousWalkMillis = currentMillis;
        walkCount = 4;
      } else if (walkCount == 4 && currentMillis - previousWalkMillis >= stepTimeWalk) {
        servo5Pos = 2000;
        previousWalkMillis = currentMillis;
        walkCount = 5;
      } else if (walkCount == 5 && currentMillis - previousWalkMillis >= stepTimeWalk) {
        servo3Pos = 1800;
        servo4Pos = 1200;
        previousWalkMillis = currentMillis;
        walkCount = 6;
      } else if (walkCount == 6 && currentMillis - previousWalkMillis >= stepTimeWalk) {
        previousWalkMillis = currentMillis;
        inMotionFlag = 0;
      }
    }

    // *** WALK BACKWARD ***
    if (walkAction == 2) {
      inMotionFlag = 1;
      if (walkCount == 0) {
        servo1Pos = 1000;
        servo2Pos = 2000;
        previousWalkMillis = currentMillis;
        walkCount = 1;
      } else if (walkCount == 1 && currentMillis - previousWalkMillis >= stepTimeWalk) {
        servo5Pos = 2000;
        previousWalkMillis = currentMillis;
        walkCount = 2;
      } else if (walkCount == 2 && currentMillis - previousWalkMillis >= stepTimeWalk) {
        servo1Pos = 1800;
        servo2Pos = 1200;
        previousWalkMillis = currentMillis;
        walkCount = 3;
      } else if (walkCount == 3 && currentMillis - previousWalkMillis >= stepTimeWalk) {
        servo3Pos = 1000;
        servo4Pos = 2000;
        previousWalkMillis = currentMillis;
        walkCount = 4;
      } else if (walkCount == 4 && currentMillis - previousWalkMillis >= stepTimeWalk) {
        servo5Pos = 1000;
        previousWalkMillis = currentMillis;
        walkCount = 5;
      } else if (walkCount == 5 && currentMillis - previousWalkMillis >= stepTimeWalk) {
        servo3Pos = 1800;
        servo4Pos = 1200;
        previousWalkMillis = currentMillis;
        walkCount = 6;
      } else if (walkCount == 6 && currentMillis - previousWalkMillis >= stepTimeWalk) {
        previousWalkMillis = currentMillis;
        inMotionFlag = 0;
      }
    }

    // *** TURN LEFT ***
    if (walkAction == 3) {
      inMotionFlag = 1;
      if (walkCount == 0) {
        servo1Pos = 1000;
        servo2Pos = 2000;
        previousWalkMillis = currentMillis;
        walkCount = 1;
      } else if (walkCount == 1 && currentMillis - previousWalkMillis >= stepTimeTurn) {
        servo5Pos = 1500;
        servo6Pos = 800;
        previousWalkMillis = currentMillis;
        walkCount = 2;
      } else if (walkCount == 2 && currentMillis - previousWalkMillis >= stepTimeTurn) {
        servo1Pos = 1800;
        servo2Pos = 1200;
        previousWalkMillis = currentMillis;
        walkCount = 3;
      } else if (walkCount == 3 && currentMillis - previousWalkMillis >= stepTimeTurn) {
        servo3Pos = 1000;
        servo4Pos = 2000;
        previousWalkMillis = currentMillis;
        walkCount = 4;
      } else if (walkCount == 4 && currentMillis - previousWalkMillis >= stepTimeTurn) {
        servo6Pos = 1500;
        previousWalkMillis = currentMillis;
        walkCount = 5;
      } else if (walkCount == 5 && currentMillis - previousWalkMillis >= stepTimeTurn) {
        servo3Pos = 1800;
        servo4Pos = 1200;
        previousWalkMillis = currentMillis;
        walkCount = 6;
      } else if (walkCount == 6 && currentMillis - previousWalkMillis >= stepTimeTurn) {
        previousWalkMillis = currentMillis;
        inMotionFlag = 0;
      }
    }

    // *** TURN RIGHT ***
    if (walkAction == 4) {
      inMotionFlag = 1;
      if (walkCount == 0) {
        servo3Pos = 1000;
        servo4Pos = 2000;
        previousWalkMillis = currentMillis;
        walkCount = 1;
      } else if (walkCount == 1 && currentMillis - previousWalkMillis >= stepTimeTurn) {
        servo5Pos = 1500;
        servo6Pos = 800;
        previousWalkMillis = currentMillis;
        walkCount = 2;
      } else if (walkCount == 2 && currentMillis - previousWalkMillis >= stepTimeTurn) {
        servo3Pos = 1800;
        servo4Pos = 1200;
        previousWalkMillis = currentMillis;
        walkCount = 3;
      } else if (walkCount == 3 && currentMillis - previousWalkMillis >= stepTimeTurn) {
        servo1Pos = 1000;
        servo2Pos = 2000;
        previousWalkMillis = currentMillis;
        walkCount = 4;
      } else if (walkCount == 4 && currentMillis - previousWalkMillis >= stepTimeTurn) {
        servo6Pos = 1500;
        previousWalkMillis = currentMillis;
        walkCount = 5;
      } else if (walkCount == 5 && currentMillis - previousWalkMillis >= stepTimeTurn) {
        servo1Pos = 1800;
        servo2Pos = 1200;
        previousWalkMillis = currentMillis;
        walkCount = 6;
      } else if (walkCount == 6 && currentMillis - previousWalkMillis >= stepTimeTurn) {
        previousWalkMillis = currentMillis;
        inMotionFlag = 0;
      }
    }

    // *** TRICK: lift front-left leg only (others stay planted) ***
    if (walkAction == 6) {
      inMotionFlag = 1;
      if (walkCount == 0) {
        servo1Pos = 1800;
        servo3Pos = 1800;
        servo2Pos = 1200;
        servo4Pos = 1200;
        servo5Pos = 1500;
        servo6Pos = 1500;
        previousWalkMillis = currentMillis;
        walkCount = 1;
      } else if (walkCount == 1 && currentMillis - previousWalkMillis >= stepTimeTrick) {
        servo1Pos = 1000;
        previousWalkMillis = currentMillis;
        walkCount = 2;
      } else if (walkCount == 2 && currentMillis - previousWalkMillis >= 900) {
        servo1Pos = 1800;
        previousWalkMillis = currentMillis;
        walkCount = 3;
      } else if (walkCount == 3 && currentMillis - previousWalkMillis >= stepTimeTrick) {
        walkAction = 0;
        walkCount = 0;
        inMotionFlag = 0;
      }
    }

    // *** TRICK: bow — lean on front two legs, rear lifted ***
    if (walkAction == 7) {
      inMotionFlag = 1;
      if (walkCount == 0) {
        servo1Pos = 1800;
        servo3Pos = 1800;
        servo2Pos = 1200;
        servo4Pos = 1200;
        servo5Pos = 1500;
        servo6Pos = 1500;
        previousWalkMillis = currentMillis;
        walkCount = 1;
      } else if (walkCount == 1 && currentMillis - previousWalkMillis >= stepTimeTrick) {
        // Left/right legs are usually mirror-mounted: one side needs low µs, the other high,
        // or only one front reaches “down”. Fronts both load; rears both lift.
        const int bowFrontL = 1000;
        const int bowFrontR = 2200;
        const int bowRearL = 1000;
        const int bowRearR = 2200;
        servo1Pos = bowFrontL;
        servo3Pos = bowFrontR;
        servo4Pos = bowRearL;
        servo2Pos = bowRearR;
        servo5Pos = 1280;
        previousWalkMillis = currentMillis;
        walkCount = 2;
      } else if (walkCount == 2 && currentMillis - previousWalkMillis >= 1000) {
        servo1Pos = 1800;
        servo3Pos = 1800;
        servo2Pos = 1200;
        servo4Pos = 1200;
        servo5Pos = 1500;
        previousWalkMillis = currentMillis;
        walkCount = 3;
      } else if (walkCount == 3 && currentMillis - previousWalkMillis >= stepTimeTrick) {
        walkAction = 0;
        walkCount = 0;
        inMotionFlag = 0;
      }
    }

    // *** TRICK: shake — quick left/right on rotation servo ***
    if (walkAction == 8) {
      inMotionFlag = 1;
      const int wagMs = 68;
      const int wagL = 850;
      const int wagR = 2150;
      const int wagLastPhase = 21;
      const int wagCenterPhase = 22;
      if (walkCount == 0) {
        servo1Pos = 1800;
        servo3Pos = 1800;
        servo2Pos = 1200;
        servo4Pos = 1200;
        servo5Pos = 1500;
        servo6Pos = 1500;
        previousWalkMillis = currentMillis;
        walkCount = 1;
      } else if (walkCount == 1 && currentMillis - previousWalkMillis >= 160) {
        servo6Pos = wagL;
        previousWalkMillis = currentMillis;
        walkCount = 2;
      } else if (walkCount >= 2 && walkCount <= wagLastPhase) {
        if (currentMillis - previousWalkMillis >= wagMs) {
          servo6Pos = (walkCount % 2 == 0) ? wagR : wagL;
          previousWalkMillis = currentMillis;
          walkCount++;
        }
      } else if (walkCount == wagCenterPhase && currentMillis - previousWalkMillis >= wagMs) {
        servo6Pos = 1500;
        previousWalkMillis = currentMillis;
        walkCount = 23;
      } else if (walkCount == 23 && currentMillis - previousWalkMillis >= 320) {
        walkAction = 0;
        walkCount = 0;
        inMotionFlag = 0;
      }
    }

    servo1PosFiltered = filter(servo1Pos, servo1PosFiltered, filterVal);
    servo2PosFiltered = filter(servo2Pos, servo2PosFiltered, filterVal);
    servo3PosFiltered = filter(servo3Pos, servo3PosFiltered, filterVal);
    servo4PosFiltered = filter(servo4Pos, servo4PosFiltered, filterVal);
    servo5PosFiltered = filter(servo5Pos, servo5PosFiltered, filterVal);
    servo6PosFiltered = filter(servo6Pos, servo6PosFiltered, filterVal);

    servo1.writeMicroseconds(constrainedMicros(servo1PosFiltered, servo1Offset));
    servo2.writeMicroseconds(constrainedMicros(servo2PosFiltered, servo2Offset));
    servo3.writeMicroseconds(constrainedMicros(servo3PosFiltered, servo3Offset));
    servo4.writeMicroseconds(constrainedMicros(servo4PosFiltered, servo4Offset));
    servo5.writeMicroseconds(constrainedMicros(servo5PosFiltered, servo5Offset));
    servo6.writeMicroseconds(constrainedMicros(servo6PosFiltered, servo6Offset));
  }
}

float filter(float prevValue, float currentValue, int filterAmount) {
  return (prevValue + (currentValue * filterAmount)) / (filterAmount + 1);
}
