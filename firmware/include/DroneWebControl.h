#ifndef DRONE_WEB_H
#define DRONE_WEB_H
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

// =============================================================================
//  DroneWebControl — soft-AP web joystick UI + control link failsafe.
//
//  Changes vs previous revision:
//    • Level TRIM (roll/pitch, degrees) — /trim endpoint + buttons in the UI.
//      This is the fix for the classic "sensor level ≠ thrust level" drift
//      that neither the accelerometer nor the drag observer can see.
//    • "Recalibrate level" button (/calib) — honoured by main only when
//      disarmed, drone flat on a level surface.
//    • Live telemetry (attitude, velocity estimate, trims) in /status and UI,
//      polled at 2 Hz so you can watch what the FC believes while diagnosing.
//    • Command path, failsafe and joystick semantics are UNCHANGED.
// =============================================================================
class DroneWebControl
{
public:
  struct DroneCommand
  {
    float roll = 0.0f;   // [-1,1]
    float pitch = 0.0f;  // [-1,1]
    float yaw = 0.0f;    // [-1,1]
    float thrust = 0.0f; // [0,1]
    bool armed = false;
  };

  static constexpr float TRIM_MAX_DEG = 6.0f;

  DroneWebControl() : server(80) {}

  // Start AP + web server (no DNS)
  bool begin(const char *ssid, const char *password = nullptr)
  {
    WiFi.softAPdisconnect(true);
    WiFi.disconnect(true, true);

    WiFi.mode(WIFI_AP);

    bool ok;
    if (password && password[0] != '\0')
      ok = WiFi.softAP(ssid, password);
    else
      ok = WiFi.softAP(ssid);

    if (!ok)
      return false;

    server.on("/", HTTP_GET, [&]()
              { handleRoot(); });
    server.on("/cmd", HTTP_GET, [&]()
              { handleCmd(); });
    server.on("/status", HTTP_GET, [&]()
              { handleStatus(); });
    server.on("/trim", HTTP_GET, [&]()
              { handleTrim(); });
    server.on("/calib", HTTP_GET, [&]()
              { handleCalib(); });
    server.on("/cal6", HTTP_GET, [&]()
              { handleCal6(); });
    server.onNotFound([&]()
                      { handleRoot(); });

    server.begin();
    lastRxMs = millis();
    return true;
  }

  void loop()
  {
    server.handleClient();

    const uint32_t now = millis();

    // Consider "link lost" if either:
    //  A) no stations connected to the AP, or
    //  B) no /cmd received for failsafeMs (control stream stopped)
    const bool noStations = (WiFi.softAPgetStationNum() == 0);
    const bool cmdTimeout = (now - lastRxMs > failsafeMs);

    const bool linkLost =
        (cutThrottleOnClientLost && noStations) ||
        (cutThrottleOnCmdTimeout && cmdTimeout);

    if (linkLost)
    {
      cmd.roll = cmd.pitch = cmd.yaw = 0.0f;
      cmd.thrust = 0.0f;
      cmd.armed = false; // disarm on link lost
      newCmdFlag = true;
      // NOTE: don't touch lastRxMs; it should remain stale until client returns
    }
  }

  // Compatibility with existing loop code:
  bool hasNewRight() const { return newCmdFlag; }   // "new input arrived"
  bool hasNewCommand() const { return newCmdFlag; } // alias

  DroneCommand getDroneCommand(bool clearNewFlag = true)
  {
    DroneCommand out = cmd;
    if (clearNewFlag)
      newCmdFlag = false;
    return out;
  }

  IPAddress apIP() const { return WiFi.softAPIP(); }

  void setFailsafeMs(uint32_t ms) { failsafeMs = ms; }

  // ── Trim (degrees, + roll = right wing down, + pitch = nose up) ───────────
  void setTrim(float rollDeg, float pitchDeg)
  {
    trimRollDeg = clampf(rollDeg, -TRIM_MAX_DEG, TRIM_MAX_DEG);
    trimPitchDeg = clampf(pitchDeg, -TRIM_MAX_DEG, TRIM_MAX_DEG);
  }
  void getTrim(float &rollDeg, float &pitchDeg) const
  {
    rollDeg = trimRollDeg;
    pitchDeg = trimPitchDeg;
  }
  bool takeTrimChanged()
  {
    const bool c = trimChangedFlag;
    trimChangedFlag = false;
    return c;
  }

  // ── Recalibration request (main honours it only when disarmed) ───────────
  bool takeCalibrateRequest()
  {
    const bool c = calibRequested;
    calibRequested = false;
    return c;
  }

  // ── 6-face accel calibration wizard (main executes, UI drives) ───────────
  //  request: 0 = none, 1 = capture next face, 2 = restart
  int takeCal6Request()
  {
    const int r = cal6Request;
    cal6Request = 0;
    return r;
  }
  // main reports progress: face index (0..6 = done) and a short message
  void setCal6State(int nextFace, const String &msg)
  {
    cal6Face = nextFace;
    cal6Msg = msg;
  }

  // ── Telemetry pushed by main (shown in /status) ───────────────────────────
  void setTelemetry(float rollDeg, float pitchDeg, float yawDeg,
                    float velX, float velY, const char *state)
  {
    tRoll = rollDeg;
    tPitch = pitchDeg;
    tYaw = yawDeg;
    tVx = velX;
    tVy = velY;
    tState = state;
  }

private:
  WebServer server;
  DroneCommand cmd;
  volatile bool newCmdFlag = false;
  uint32_t lastRxMs = 0;
  uint32_t failsafeMs = 3000;

  bool cutThrottleOnClientLost = true;
  bool cutThrottleOnCmdTimeout = true; // "control link lost" = no /cmd updates

  float trimRollDeg = 0.0f, trimPitchDeg = 0.0f;
  bool trimChangedFlag = false;
  bool calibRequested = false;
  int cal6Request = 0;
  int cal6Face = 0;
  String cal6Msg = "not started";

  float tRoll = 0, tPitch = 0, tYaw = 0, tVx = 0, tVy = 0;
  const char *tState = "boot";

  static float clampf(float v, float lo, float hi)
  {
    if (v < lo)
      return lo;
    if (v > hi)
      return hi;
    return v;
  }

  void handleCmd()
  {
    // Copy current command
    DroneCommand next = cmd;

    // Parse into 'next'
    if (server.hasArg("roll"))
      next.roll = clampf(server.arg("roll").toFloat(), -1.0f, 1.0f);
    if (server.hasArg("pitch"))
      next.pitch = clampf(server.arg("pitch").toFloat(), -1.0f, 1.0f);
    if (server.hasArg("yaw"))
      next.yaw = clampf(server.arg("yaw").toFloat(), -1.0f, 1.0f);
    if (server.hasArg("thrust"))
      next.thrust = clampf(server.arg("thrust").toFloat(), 0.0f, 1.0f);
    if (server.hasArg("arm"))
      next.armed = (server.arg("arm").toInt() != 0);

    // Always refresh RX timestamp (keeps failsafe happy)
    lastRxMs = millis();

    // Only raise "new command" if something changed
    const bool changed =
        (next.roll != cmd.roll) ||
        (next.pitch != cmd.pitch) ||
        (next.yaw != cmd.yaw) ||
        (next.thrust != cmd.thrust) ||
        (next.armed != cmd.armed);

    if (changed)
    {
      cmd = next;
      newCmdFlag = true;
    }

    server.send(200, "text/plain", "OK");
  }

  void handleTrim()
  {
    if (server.hasArg("roll") || server.hasArg("pitch"))
    {
      float r = trimRollDeg, p = trimPitchDeg;
      if (server.hasArg("roll"))
        r = server.arg("roll").toFloat();
      if (server.hasArg("pitch"))
        p = server.arg("pitch").toFloat();
      setTrim(r, p);
      trimChangedFlag = true;
    }
    server.send(200, "application/json", trimJson());
  }

  void handleCalib()
  {
    if (cmd.armed)
    {
      server.send(200, "text/plain", "ARMED");
      return;
    }
    calibRequested = true;
    server.send(200, "text/plain", "OK");
  }

  void handleCal6()
  {
    if (cmd.armed)
    {
      server.send(200, "text/plain", "ARMED");
      return;
    }
    const String c = server.hasArg("cmd") ? server.arg("cmd") : String("capture");
    cal6Request = (c == "reset") ? 2 : 1;
    server.send(200, "text/plain", "OK");
  }

  static String jsonEscape(const String &in)
  {
    String o;
    o.reserve(in.length() + 8);
    for (unsigned i = 0; i < in.length(); ++i)
    {
      const char ch = in[i];
      if (ch == '"' || ch == '\\')
      {
        o += '\\';
        o += ch;
      }
      else if (ch == '\n')
        o += "\\n";
      else
        o += ch;
    }
    return o;
  }

  String trimJson() const
  {
    String j = "{\"roll\":" + String(trimRollDeg, 2) + ",\"pitch\":" + String(trimPitchDeg, 2) + "}";
    return j;
  }

  void handleStatus()
  {
    String json;
    json.reserve(600);
    json += "{";
    json += "\"ip\":\"" + apIP().toString() + "\",";
    json += "\"roll\":" + String(cmd.roll, 3) + ",";
    json += "\"pitch\":" + String(cmd.pitch, 3) + ",";
    json += "\"yaw\":" + String(cmd.yaw, 3) + ",";
    json += "\"thrust\":" + String(cmd.thrust, 3) + ",";
    json += "\"armed\":" + String(cmd.armed ? "true" : "false") + ",";
    json += "\"trim\":" + trimJson() + ",";
    json += "\"att\":{\"roll\":" + String(tRoll, 1) + ",\"pitch\":" + String(tPitch, 1) + ",\"yaw\":" + String(tYaw, 1) + "},";
    json += "\"vel\":{\"x\":" + String(tVx, 2) + ",\"y\":" + String(tVy, 2) + "},";
    json += "\"cal6\":{\"face\":" + String(cal6Face) + ",\"msg\":\"" + jsonEscape(cal6Msg) + "\"},";
    json += "\"state\":\"" + String(tState) + "\"";
    json += "}";
    server.send(200, "application/json", json);
  }

  void handleRoot()
  {
    server.send_P(200, "text/html", htmlPage());
  }

  // --- Web UI (2 joysticks + thrust slider + arm toggle + trim + telemetry) ---
  static const char *htmlPage()
  {
    return R"HTML(
<!doctype html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no"/>
  <title>Drone Web Control</title>
  <style>
    :root { --bg:#1f1f1f; --panel:#2a2a2a; --text:#eaeaea; --acc:#6b5cff; }
    html, body { height:100%; margin:0; font-family: Helvetica, Arial, sans-serif; background:var(--bg); color:var(--text); }
    .wrap { max-width: 920px; margin: 0 auto; padding: 16px; }
    h1 { margin: 8px 0 4px; font-size: 22px; }
    .sub { opacity: .85; margin-bottom: 12px; }
    .grid { display:grid; grid-template-columns: 1fr; gap: 12px; }
    @media (min-width: 780px) { .grid { grid-template-columns: 1fr 1fr; } }
    .card { background:var(--panel); border-radius: 14px; padding: 14px; box-shadow: 0 6px 18px rgba(0,0,0,.25); }
    .row { display:flex; align-items:center; justify-content:space-between; gap:10px; }
    .lbl { font-size: 14px; opacity: .9; }
    .val { font-family: ui-monospace, SFMono-Regular, Menlo, monospace; font-size: 13px; opacity:.9; }
    .joyWrap { display:flex; gap:14px; justify-content:space-around; flex-wrap:wrap; }
    .joyBox { display:flex; flex-direction:column; align-items:center; gap:8px; }
    .joyTitle { font-size: 14px; opacity:.9; }
    .base {
      position: relative;
      width: 150px; height: 150px;
      border-radius: 50%;
      background: #151515;
      outline: 2px solid rgba(255,255,255,.06);
      touch-action: none;
      user-select: none;
    }
    .stick {
      position:absolute;
      width: 64px; height: 64px;
      left: 50%; top: 50%;
      transform: translate(-50%, -50%);
      border-radius: 50%;
      background: #9a9a9a;
      opacity: .9;
    }
    input[type="range"] { width: 100%; }
    .btn {
      background: var(--acc);
      border: none;
      color: white;
      padding: 10px 12px;
      border-radius: 10px;
      cursor:pointer;
      font-weight: 600;
    }
    .btn.off { background: #444; }
    .btn.sm { padding: 8px 10px; font-size: 13px; }
    .btn.warn { background: #8a5a12; }
    .small { font-size: 12px; opacity:.85; }
    .pill { padding: 6px 10px; border-radius: 999px; background: rgba(255,255,255,.08); }
    .trimPad { display:grid; grid-template-columns: 1fr 1fr 1fr; gap:6px; max-width: 260px; margin: 8px auto; }
    .trimPad .btn { width:100%; }
    .trimPad .center { text-align:center; align-self:center; }
  </style>
</head>
<body>
<div class="wrap">
  <h1>Drone Web Control</h1>
  <div class="sub">Right joystick: Roll/Pitch • Left joystick: Yaw • Slider: Thrust</div>

  <div class="grid">
    <div class="card">
      <div class="row">
        <div class="lbl">ARM</div>
        <button id="armBtn" class="btn off">DISARMED</button>
      </div>
      <div style="height:10px"></div>
      <div class="row">
        <div class="lbl">Thrust</div>
        <div class="val"><span id="thrustVal">0.00</span></div>
      </div>
      <input id="thrust" type="range" min="0" max="100" value="0" />
      <div class="small">Tip: if control stops updating, failsafe will cut thrust.</div>
    </div>

    <div class="card">
      <div class="row">
        <div class="lbl">Live</div>
        <div class="pill val" id="live">roll 0.00 • pitch 0.00 • yaw 0.00</div>
      </div>
      <div style="height:12px"></div>

      <div class="joyWrap">
        <div class="joyBox">
          <div class="joyTitle">Yaw</div>
          <div class="base" id="joyL"><div class="stick" id="stickL"></div></div>
        </div>
        <div class="joyBox">
          <div class="joyTitle">Roll / Pitch</div>
          <div class="base" id="joyR"><div class="stick" id="stickR"></div></div>
        </div>
      </div>
    </div>

    <div class="card">
      <div class="row">
        <div class="lbl">Level trim</div>
        <div class="pill val" id="trimVal">roll 0.00° • pitch 0.00°</div>
      </div>
      <div class="trimPad">
        <div></div>
        <button class="btn sm" id="tFwd">&#9650; Fwd</button>
        <div></div>
        <button class="btn sm" id="tLeft">&#9664; Left</button>
        <button class="btn sm off" id="tReset">Reset</button>
        <button class="btn sm" id="tRight">Right &#9654;</button>
        <div></div>
        <button class="btn sm" id="tBack">&#9660; Back</button>
        <div></div>
      </div>
      <div class="small">Press the direction the drone should move <b>toward</b> (opposite to the drift). 0.2° per press, saved on the drone.</div>
    </div>

    <div class="card">
      <div class="row">
        <div class="lbl">Telemetry</div>
        <div class="pill val" id="stateVal">—</div>
      </div>
      <div style="height:8px"></div>
      <div class="val" id="attVal">att: — </div>
      <div class="val" id="velVal">vel: — </div>
      <div style="height:10px"></div>
      <div class="row">
        <div class="small">Disarmed, drone flat on a level surface, don't touch it for ~2 s.</div>
        <button class="btn sm warn" id="calibBtn">Recalibrate level</button>
      </div>
    </div>

    <div class="card">
      <div class="row">
        <div class="lbl">Accel 6-face calibration (silicon offsets)</div>
        <button class="btn sm off" id="cal6Reset">Restart</button>
      </div>
      <div style="height:8px"></div>
      <div class="val" id="cal6Next">—</div>
      <div style="height:6px"></div>
      <div class="val" id="cal6Msg" style="white-space:pre-line">—</div>
      <div style="height:10px"></div>
      <div class="row">
        <div class="small">Disarmed. Hold the drone still against a flat wall/table edge in the shown pose, then press Capture (~2.5 s). Do the six poses in any order except the LAST: flat, right side up, on your LEVEL surface — the level calibration runs right after it.</div>
        <button class="btn sm warn" id="cal6Btn">Capture</button>
      </div>
    </div>
  </div>
</div>

<script>
  // Normalized commands:
  let roll = 0, pitch = 0, yaw = 0, thrust = 0, armed = 0;

  const thrustEl = document.getElementById('thrust');
  const thrustValEl = document.getElementById('thrustVal');
  const liveEl = document.getElementById('live');
  const armBtn = document.getElementById('armBtn');

  function clamp(v, lo, hi){ return Math.max(lo, Math.min(hi, v)); }

  // Throttle send rate
  let lastSend = 0;
  const intervalMs = 70;
  let pending = false;

  function sendCmd() {
    const now = Date.now();
    if (now - lastSend < intervalMs) {
      if (!pending) {
        pending = true;
        setTimeout(() => { pending = false; sendCmd(); }, intervalMs - (now - lastSend));
      }
      return;
    }
    lastSend = now;

    liveEl.textContent = `roll ${roll.toFixed(2)} • pitch ${pitch.toFixed(2)} • yaw ${yaw.toFixed(2)}`;

    const url = `/cmd?roll=${roll.toFixed(3)}&pitch=${pitch.toFixed(3)}&yaw=${yaw.toFixed(3)}&thrust=${thrust.toFixed(3)}&arm=${armed}`;
    fetch(url).catch(()=>{});
  }

  // Keepalive / heartbeat: keep sending current state even if inputs don't change
  const heartbeatMs = 100; // 50-150ms is typical; must be < failsafeMs
  setInterval(() => { if (armed) sendCmd(); }, heartbeatMs);

  // Arm toggle
  armBtn.addEventListener('click', () => {
    armed = armed ? 0 : 1;
    armBtn.textContent = armed ? "ARMED" : "DISARMED";
    armBtn.className = armed ? "btn" : "btn off";
    if (!armed) {
      thrust = 0; thrustEl.value = 0; thrustValEl.textContent = thrust.toFixed(2);
      roll = pitch = yaw = 0;
      resetStick('stickL'); resetStick('stickR');
    }
    sendCmd();
  });

  // Thrust slider (0..1)
  thrustEl.addEventListener('input', () => {
    thrust = clamp(parseInt(thrustEl.value, 10) / 100.0, 0, 1);
    thrustValEl.textContent = thrust.toFixed(2);
    if (!armed && thrust > 0) { thrust = 0; thrustEl.value = 0; thrustValEl.textContent = "0.00"; }
    sendCmd();
  });

  // Joystick helpers
  function resetStick(stickId){
    const s = document.getElementById(stickId);
    s.style.transform = 'translate(-50%, -50%)';
  }

  function attachJoystick(baseId, stickId, onMove, onEnd) {
    const base = document.getElementById(baseId);
    const stick = document.getElementById(stickId);

    const radius = 60; // px from center
    let active = false;

    function setFromEvent(ev) {
      const rect = base.getBoundingClientRect();
      const cx = rect.left + rect.width/2;
      const cy = rect.top + rect.height/2;
      const x = ev.clientX - cx;
      const y = ev.clientY - cy;

      const dist = Math.sqrt(x*x + y*y);
      const k = dist > radius ? (radius / dist) : 1.0;
      const px = x * k;
      const py = y * k;

      stick.style.transform = `translate(${px}px, ${py}px) translate(-50%, -50%)`;

      // Normalize to [-1,1]
      const nx = clamp(px / radius, -1, 1);
      const ny = clamp(py / radius, -1, 1);
      onMove(nx, ny);
      sendCmd();
    }

    base.addEventListener('pointerdown', (ev) => {
      if (!armed) return;
      active = true;
      base.setPointerCapture(ev.pointerId);
      setFromEvent(ev);
    });

    base.addEventListener('pointermove', (ev) => {
      if (!active) return;
      setFromEvent(ev);
    });

    function end(ev) {
      if (!active) return;
      active = false;
      resetStick(stickId);
      onEnd();
      sendCmd();
    }

    base.addEventListener('pointerup', end);
    base.addEventListener('pointercancel', end);
    base.addEventListener('pointerleave', () => { /* ignore */ });
  }

  // Left joystick: yaw (x), ignore y
  attachJoystick('joyL', 'stickL',
    (nx, ny) => { yaw = nx; },
    () => { yaw = 0; }
  );

  // Right joystick: roll (x), pitch (y inverted so up = +pitch)
  attachJoystick('joyR', 'stickR',
    (nx, ny) => { roll = nx; pitch = -ny; },
    () => { roll = 0; pitch = 0; }
  );

  // ── Trim ─────────────────────────────────────────────────────────────────
  // + roll trim = lean right (drone moves right); + pitch trim = nose up (drone moves back)
  let trimRoll = 0, trimPitch = 0;
  const TRIM_STEP = 0.2;
  const trimValEl = document.getElementById('trimVal');
  function showTrim(){ trimValEl.textContent = `roll ${trimRoll.toFixed(2)}° • pitch ${trimPitch.toFixed(2)}°`; }
  function sendTrim(r, p){
    fetch(`/trim?roll=${r.toFixed(2)}&pitch=${p.toFixed(2)}`)
      .then(x => x.json()).then(j => { trimRoll = j.roll; trimPitch = j.pitch; showTrim(); })
      .catch(()=>{});
  }
  document.getElementById('tLeft').addEventListener('click',  () => sendTrim(trimRoll - TRIM_STEP, trimPitch));
  document.getElementById('tRight').addEventListener('click', () => sendTrim(trimRoll + TRIM_STEP, trimPitch));
  document.getElementById('tFwd').addEventListener('click',   () => sendTrim(trimRoll, trimPitch - TRIM_STEP)); // nose down → forward
  document.getElementById('tBack').addEventListener('click',  () => sendTrim(trimRoll, trimPitch + TRIM_STEP)); // nose up → back
  document.getElementById('tReset').addEventListener('click', () => sendTrim(0, 0));

  // ── Recalibrate level ────────────────────────────────────────────────────
  document.getElementById('calibBtn').addEventListener('click', () => {
    if (armed) { alert('Disarm first.'); return; }
    if (!confirm('Drone flat on a LEVEL surface and untouched? Calibration takes ~2 s.')) return;
    fetch('/calib').catch(()=>{});
  });

  // ── 6-face accel calibration ─────────────────────────────────────────────
  const CAL6_POSES = [
    'Pose 1/6: NOSE to the ceiling (drone standing on its tail)',
    'Pose 2/6: NOSE to the floor',
    'Pose 3/6: RIGHT side to the ceiling (drone on its left side)',
    'Pose 4/6: LEFT side to the ceiling',
    'Pose 5/6: UPSIDE DOWN, flat',
    'Pose 6/6: FLAT, right side up, on the LEVEL surface (last!)',
    'Done — results below. Level calibration was run on pose 6.'
  ];
  const cal6NextEl = document.getElementById('cal6Next');
  const cal6MsgEl = document.getElementById('cal6Msg');
  document.getElementById('cal6Btn').addEventListener('click', () => {
    if (armed) { alert('Disarm first.'); return; }
    fetch('/cal6?cmd=capture').catch(()=>{});
  });
  document.getElementById('cal6Reset').addEventListener('click', () => {
    fetch('/cal6?cmd=reset').catch(()=>{});
  });

  // ── Telemetry poll (2 Hz) ────────────────────────────────────────────────
  const stateEl = document.getElementById('stateVal');
  const attEl = document.getElementById('attVal');
  const velEl = document.getElementById('velVal');
  function poll(){
    fetch('/status').then(x => x.json()).then(j => {
      trimRoll = j.trim.roll; trimPitch = j.trim.pitch; showTrim();
      stateEl.textContent = j.state;
      attEl.textContent = `att: roll ${j.att.roll}° pitch ${j.att.pitch}° yaw ${j.att.yaw}°`;
      velEl.textContent = `vel est: x ${j.vel.x} m/s  y ${j.vel.y} m/s`;
      const f = Math.max(0, Math.min(6, j.cal6.face|0));
      cal6NextEl.textContent = CAL6_POSES[f];
      cal6MsgEl.textContent = j.cal6.msg;
    }).catch(()=>{});
  }
  setInterval(poll, 500);
  poll();

  // On page close: send safe command
  window.addEventListener('beforeunload', () => {
    roll = pitch = yaw = 0; thrust = 0; armed = 0;
    navigator.sendBeacon(`/cmd?roll=0&pitch=0&yaw=0&thrust=0&arm=0`);
  });

  // Start at zero
  resetStick('stickL'); resetStick('stickR');
  thrustValEl.textContent = "0.00";
</script>
</body>
</html>
)HTML";
  }
};

#endif // DRONE_WEB_H
