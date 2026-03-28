#pragma once
#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>
#include "ct_sensor.h"
#include "http_sender.h"
#include "diagnostics.h"
#include "wifi_manager.h"

static WebServer _statusServer(80);
static bool _statusServerRunning = false;
static AllCTReadings _lastReadings = {};

void updateLastReadings(const AllCTReadings& readings) {
    _lastReadings = readings;
}

void _handleStatus() {
    String json = "{";
    json += "\"device_id\":\"" + getDeviceId() + "\"";
    json += ",\"location\":\"" + getLocationName() + "\"";
    json += ",\"ip\":\"" + WiFi.localIP().toString() + "\"";
    json += ",\"rssi\":" + String(WiFi.RSSI());
    json += ",\"uptime_s\":" + String(getUptimeSeconds());
    json += ",\"free_heap\":" + String(ESP.getFreeHeap());
    json += ",\"timestamp\":\"" + getUTCTimestamp() + "\"";
    json += ",\"queue\":" + String(getQueueSize());
    json += ",\"sent\":" + String(getTotalSent());
    json += ",\"failed\":" + String(getTotalFailed());
    json += ",\"calibrated\":" + String(isCTCalibrated() ? "true" : "false");
    json += ",\"readings\":{";
    for (int i = 0; i < NUM_CT_CHANNELS; i++) {
        if (i > 0) json += ",";
        json += "\"ct_" + String(i + 1) + "\":{";
        json += "\"amps\":" + String(_lastReadings.ct[i].amps, 3);
        json += ",\"watts\":" + String(_lastReadings.ct[i].watts, 1);
        json += ",\"mv\":" + String(_lastReadings.ct[i].avg_mv);
        json += "}";
    }
    json += "},\"total_watts\":" + String(_lastReadings.total_watts, 1);
    json += "}";

    _statusServer.sendHeader("Access-Control-Allow-Origin", "*");
    _statusServer.send(200, "application/json", json);
}

void _handleStatusPage() {
    String html = R"rawliteral(
<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>LineSights Monitor</title>
<style>
*{box-sizing:border-box}
body{font-family:-apple-system,sans-serif;margin:0;padding:15px;background:#0a0a1a;color:#e0e0e0}
.wrap{max-width:700px;margin:0 auto}
h2{color:#4fc3f7;margin:0 0 2px}
.sub{color:#666;font-size:12px;margin-bottom:15px}
.card{background:#111827;border-radius:12px;padding:15px;margin:10px 0;border:1px solid #1e293b}
canvas{width:100%;border-radius:8px;background:#0f172a}
.legend{display:flex;flex-wrap:wrap;gap:8px;margin:10px 0 5px;font-size:12px}
.legend span{display:flex;align-items:center;gap:4px}
.legend i{width:10px;height:10px;border-radius:2px;display:inline-block}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:10px}
.ct{background:#0f172a;border-radius:8px;padding:10px;text-align:center}
.ct .ch{font-size:11px;color:#666;margin-bottom:2px}
.ct .amp{font-size:20px;font-weight:700;color:#4fc3f7}
.ct .watt{font-size:13px;color:#e94560;margin-top:2px}
.ct .raw{font-size:10px;color:#475569;margin-top:2px}
.stats{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;margin-top:10px}
.stat{background:#0f172a;border-radius:8px;padding:8px;text-align:center}
.stat .lbl{font-size:10px;color:#666;text-transform:uppercase}
.stat .v{font-size:16px;font-weight:700;margin-top:2px}
.ok{color:#22c55e}.err{color:#ef4444}.blue{color:#4fc3f7}.warn{color:#f59e0b}
.total{font-size:28px;font-weight:700;color:#e94560;text-align:center;padding:5px 0}
.total span{font-size:13px;color:#666}
</style>
</head><body>
<div class="wrap">
<h2>LineSights Power Monitor</h2>
<div class="sub" id="info">Connecting...</div>

<div class="card">
<div class="total"><span>Total Power</span><br><span id="total">--</span></div>
</div>

<div class="card">
<canvas id="chart" height="200"></canvas>
<div class="legend" id="legend"></div>
</div>

<div class="card">
<div class="grid" id="cts"></div>
</div>

<div class="card">
<div class="stats">
<div class="stat"><div class="lbl">Queue</div><div class="v blue" id="queue">--</div></div>
<div class="stat"><div class="lbl">Sent</div><div class="v ok" id="sent">--</div></div>
<div class="stat"><div class="lbl">Failed</div><div class="v err" id="failed">--</div></div>
<div class="stat"><div class="lbl">Uptime</div><div class="v" id="uptime">--</div></div>
<div class="stat"><div class="lbl">Heap</div><div class="v" id="heap">--</div></div>
<div class="stat"><div class="lbl">RSSI</div><div class="v" id="rssi">--</div></div>
</div>
</div>
</div>

<script>
const COLORS=['#4fc3f7','#e94560','#22c55e','#f59e0b','#a78bfa','#fb923c'];
const MAX_PTS=120;
let history=[[],[],[],[],[],[]];
let labels=[];

let leg=document.getElementById('legend');
for(let i=0;i<6;i++){
  leg.innerHTML+='<span><i style="background:'+COLORS[i]+'"></i>CT'+(i+1)+'</span>';
}

function drawChart(){
  let c=document.getElementById('chart');
  let ctx=c.getContext('2d');
  let dpr=window.devicePixelRatio||1;
  c.width=c.offsetWidth*dpr;
  c.height=200*dpr;
  ctx.scale(dpr,dpr);
  let W=c.offsetWidth,H=200;
  ctx.clearRect(0,0,W,H);

  let maxW=1;
  for(let ch=0;ch<6;ch++){
    for(let j=0;j<history[ch].length;j++){
      if(history[ch][j]>maxW) maxW=history[ch][j];
    }
  }
  maxW=Math.ceil(maxW/10)*10;
  if(maxW<10) maxW=10;

  let pad={t:10,b:25,l:40,r:10};
  let gW=W-pad.l-pad.r;
  let gH=H-pad.t-pad.b;

  ctx.strokeStyle='#1e293b';ctx.lineWidth=1;
  ctx.font='10px sans-serif';ctx.fillStyle='#475569';ctx.textAlign='right';
  for(let i=0;i<=4;i++){
    let y=pad.t+gH-(gH*i/4);
    ctx.beginPath();ctx.moveTo(pad.l,y);ctx.lineTo(W-pad.r,y);ctx.stroke();
    ctx.fillText((maxW*i/4).toFixed(0)+'W',pad.l-5,y+3);
  }

  ctx.textAlign='center';ctx.fillStyle='#475569';
  let n=labels.length;
  if(n>1){
    let step=Math.max(1,Math.floor(n/5));
    for(let i=0;i<n;i+=step){
      let x=pad.l+(i/(n-1))*gW;
      ctx.fillText(labels[i],x,H-5);
    }
  }

  for(let ch=0;ch<6;ch++){
    let pts=history[ch];
    if(pts.length<2) continue;
    ctx.strokeStyle=COLORS[ch];ctx.lineWidth=2;
    ctx.beginPath();
    for(let j=0;j<pts.length;j++){
      let x=pad.l+(j/(pts.length-1))*gW;
      let y=pad.t+gH-(pts[j]/maxW)*gH;
      if(j===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
    }
    ctx.stroke();
  }
}

function fmt(s){let h=Math.floor(s/3600),m=Math.floor((s%3600)/60),sc=s%60;return h+'h '+m+'m '+sc+'s'}

function update(){
  fetch('/api/status').then(r=>r.json()).then(d=>{
    document.getElementById('info').textContent=d.device_id+' | '+d.location+' | '+d.ip+' | '+d.timestamp;

    let now=new Date(d.timestamp);
    let tStr=now.getUTCHours().toString().padStart(2,'0')+':'+now.getUTCMinutes().toString().padStart(2,'0')+':'+now.getUTCSeconds().toString().padStart(2,'0');
    labels.push(tStr);
    if(labels.length>MAX_PTS) labels.shift();

    for(let i=0;i<6;i++){
      let c=d.readings['ct_'+(i+1)];
      history[i].push(c.watts);
      if(history[i].length>MAX_PTS) history[i].shift();
    }
    drawChart();

    let ct='';
    for(let i=1;i<=6;i++){
      let c=d.readings['ct_'+i];
      ct+='<div class="ct"><div class="ch">CT'+i+'</div><div class="amp">'+c.amps.toFixed(3)+'A</div><div class="watt">'+c.watts.toFixed(1)+'W</div><div class="raw">'+c.mv+'mV</div></div>';
    }
    document.getElementById('cts').innerHTML=ct;

    document.getElementById('total').textContent=d.total_watts.toFixed(1)+'W';
    document.getElementById('queue').textContent=d.queue;
    document.getElementById('sent').textContent=d.sent;
    document.getElementById('failed').textContent=d.failed;
    document.getElementById('uptime').textContent=fmt(d.uptime_s);
    document.getElementById('heap').textContent=(d.free_heap/1024).toFixed(0)+'KB';
    document.getElementById('rssi').textContent=d.rssi+'dBm';
  }).catch(e=>{document.getElementById('info').textContent='Connection error';});
}

update();setInterval(update,2000);
window.addEventListener('resize',drawChart);
</script>
</body></html>
)rawliteral";
    _statusServer.send(200, "text/html", html);
}

void initStatusServer() {
    _statusServer.on("/", _handleStatusPage);
    _statusServer.on("/api/status", _handleStatus);
    _statusServer.begin();
    _statusServerRunning = true;
    Serial.printf("[WEB] Status server started at http://%s/\n", WiFi.localIP().toString().c_str());
}

void handleStatusServer() {
    if (_statusServerRunning) {
        _statusServer.handleClient();
    }
}
