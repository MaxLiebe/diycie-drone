#ifndef DRONE_WEB_H
#define DRONE_WEB_H
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

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

  // Compatibility with your existing loop code:
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

private:
  WebServer server;
  DroneCommand cmd;
  volatile bool newCmdFlag = false;
  uint32_t lastRxMs = 0;
  uint32_t failsafeMs = 3000;

  bool cutThrottleOnClientLost = true;
  bool cutThrottleOnCmdTimeout = true; // "control link lost" = no /cmd updates

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

  void handleStatus()
  {
    String json = "{";
    json += "\"ip\":\"" + apIP().toString() + "\",";
    json += "\"roll\":" + String(cmd.roll, 3) + ",";
    json += "\"pitch\":" + String(cmd.pitch, 3) + ",";
    json += "\"yaw\":" + String(cmd.yaw, 3) + ",";
    json += "\"thrust\":" + String(cmd.thrust, 3) + ",";
    json += "\"armed\":" + String(cmd.armed ? "true" : "false");
    json += "}";
    server.send(200, "application/json", json);
  }

  void handleRoot()
  {
    server.send_P(200, "text/html", htmlPage());
  }

  // --- Web UI (2 joysticks + thrust slider + arm toggle) ---
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
    .small { font-size: 12px; opacity:.85; }
    .pill { padding: 6px 10px; border-radius: 999px; background: rgba(255,255,255,.08); }
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