
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

const char* AP_SSID = "RobotArm";
const char* AP_PASSWORD = "12345678";
const char* AP_DOMAIN = "RAC";

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// PCA9685通道分配
const int SERVO_CH[4] = {0, 1, 2, 3};
const int ANGLE_MIN[4] = {0, 40, 70, 80};
const int ANGLE_MAX[4] = {180, 95, 115, 120};

int currentAngles[4] = {90, 80, 90, 90};
int targetAngles[4] = {90, 80, 90, 90};

// 【修改】脉宽改用 µs 直接驱动（单位：微秒）
// 0.5ms ~ 2.5ms 是典型 180° 舵机行程；请按你的舵机手册/实测校准这两个值
int PULSE_US_MIN = 500;    // 0°   → 500µs
int PULSE_US_MAX = 2500;   // 180° → 2500µs

// 【修改】记录上次写入的脉宽(µs)，相同则跳过 I2C 写
int lastUs[4] = {-1, -1, -1, -1};

// 步进计时
unsigned long lastStepMs[4] = {0, 0, 0, 0};
const unsigned long STEP_INTERVAL_MS = 20;

// 【新增】PCA9685 OE 输出使能引脚：上电先禁输出，初始化完成后再使能
// 模块没引出 OE 就删掉下面这行 + setup 中对应三行
const int PCA_OE = 4;

// 【修改】I2C 引脚：ESP32-S3 默认 GPIO8(SDA)/GPIO9(SCL)
// 注意：8/9 是 strapping 引脚，模块 SDA/SCL 需带 4.7k~10k 上拉到 3.3V
const int I2C_SDA = 8;
const int I2C_SCL = 9;

WebServer server(80);
DNSServer dnsServer;

struct KeyFrame {
  int angles[4];
  int delayMs;
};
const KeyFrame seqHome[] = {{{90, 80, 90, 90}, 10}};
const int seqHomeCount = 1;
const KeyFrame seqWave[] = {
  {{60, 75, 95, 90}, 500},
  {{110, 75, 95, 90}, 500},
  {{60, 75, 95, 90}, 500},
  {{110, 75, 95, 90}, 500},
  {{60, 75, 95, 90}, 500},
  {{110, 75, 95, 90}, 500},
  {{60, 75, 95, 90}, 500},
  {{110, 75, 95, 90}, 500},
  {{90, 80, 90, 90}, 10}
};
const int seqWaveCount = 9;
const KeyFrame seqGrab[] = {
  {{90, 80, 110, 90}, 500},
  {{90, 80, 110, 82}, 500},
  {{90, 85, 105, 82}, 500}
};
const int seqGrabCount = 3;
const KeyFrame seqRelease[] = {
  {{90, 85, 105, 118}, 500},
  {{90, 80, 90, 118}, 500}
};
const int seqReleaseCount = 2;

bool seqRunning = false;
const KeyFrame* seqFrames = NULL;
int seqCount = 0;
int seqIndex = 0;
unsigned long seqTimer = 0;

void startSequence(const KeyFrame* frames, int count) {
  seqRunning = true;
  seqFrames = frames;
  seqCount = count;
  seqIndex = 0;
  seqTimer = 0;
  for (int i = 0; i < 4; i++) {
    targetAngles[i] = constrain(frames[0].angles[i], ANGLE_MIN[i], ANGLE_MAX[i]);
  }
}
void stopSequence() {
  seqRunning = false;
  seqFrames = NULL;
  seqCount = 0;
  seqIndex = 0;
}
void runSequence() {
  if (!seqRunning || !seqFrames) return;
  unsigned long now = millis();
  if (now - seqTimer < (unsigned long)seqFrames[seqIndex].delayMs) return;
  seqIndex++;
  seqTimer = now;
  if (seqIndex >= seqCount) { stopSequence(); return; }
  for (int i = 0; i < 4; i++) {
    int angle = seqFrames[seqIndex].angles[i];
    targetAngles[i] = constrain(angle, ANGLE_MIN[i], ANGLE_MAX[i]);
  }
}

// 【修改】角度 → 脉宽(µs)
inline int angleToUs(int angle) {
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, PULSE_US_MIN, PULSE_US_MAX);
}

// 【修改】写入舵机脉宽：相同 µs 跳过 I2C 写，减少总线负载
void writeServo(int idx, int ch, int angle) {
  int us = angleToUs(angle);
  if (us == lastUs[idx]) return;
  pwm.writeMicroseconds(ch, us);
  lastUs[idx] = us;
}

const char PAGE_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0,maximum-scale=1.0,user-scalable=no">
<title>机械臂控制</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:'PingFang SC','Microsoft YaHei','Helvetica Neue',sans-serif;background:#0b0b18;color:#ddd;padding:0;overflow-x:hidden;-webkit-user-select:none;user-select:none}
.wrap{max-width:420px;margin:0 auto;padding:8px 8px 16px}
.hd{text-align:center;padding:8px 0 6px}
.hd h1{font-size:18px;color:#00d2ff;letter-spacing:1px}
.hd .sub{font-size:11px;color:#555;margin-top:1px}
.conn{text-align:center;background:rgba(255,255,255,0.04);border-radius:6px;padding:5px 10px;margin-bottom:8px;font-size:11px;color:#777;overflow:hidden;white-space:nowrap}
.conn .dot{display:inline-block;width:6px;height:6px;border-radius:50%;background:#ffd740;vertical-align:middle;margin-right:4px}
.conn .dot.ok{background:#00e676}
.conn .dot.move{background:#ffd740;animation:blink 0.6s infinite}
@keyframes blink{0%,100%{opacity:1}50%{opacity:0.2}}
.cd{margin-bottom:6px}
.c{background:rgba(255,255,255,0.04);border-radius:10px;padding:10px 12px;margin-bottom:6px;border:1px solid rgba(255,255,255,0.04)}
.ct{display:-webkit-flex;display:flex;-webkit-justify-content:space-between;justify-content:space-between;-webkit-align-items:center;align-items:center;margin-bottom:4px}
.cl{font-size:13px;font-weight:600}
.ct .sn{font-size:10px;color:#666;background:rgba(255,255,255,0.06);padding:0 6px;border-radius:3px;margin-left:5px}
.cv{font-size:20px;font-weight:700;min-width:48px;text-align:right;position:relative}
.cv .mv{display:none}
.cv.moving .mv{display:inline-block;width:5px;height:5px;border-radius:50%;background:#ffd740;margin-left:3px;vertical-align:middle;animation:blink 0.5s infinite}
.h0 .cl{color:#00e5ff}.h0 .cv{color:#00e5ff}
.h1 .cl{color:#76ff03}.h1 .cv{color:#76ff03}
.h2 .cl{color:#ffd740}.h2 .cv{color:#ffd740}
.h3 .cl{color:#ff4081}.h3 .cv{color:#ff4081}
.sl{position:relative;height:32px;margin-bottom:1px}
.sl .tk{position:absolute;top:15px;left:0;right:0;height:4px;border-radius:2px;background:rgba(255,255,255,0.08)}
.sl .fl{position:absolute;top:15px;left:0;height:4px;border-radius:2px;background:#00d2ff;pointer-events:none}
.h0 .sl .fl{background:#00e5ff}
.h1 .sl .fl{background:#76ff03}
.h2 .sl .fl{background:#ffd740}
.h3 .sl .fl{background:#ff4081}
.sl input{position:absolute;top:0;left:-4px;width:calc(100% + 8px);height:32px;-webkit-appearance:none;appearance:none;background:transparent;outline:none;z-index:2;margin:0;padding:0}
.sl input::-webkit-slider-thumb{-webkit-appearance:none;width:36px;height:36px;border-radius:50%;background:transparent;cursor:pointer;border:none}
.sl input::-moz-range-thumb{width:36px;height:36px;border-radius:50%;background:transparent;cursor:pointer;border:none}
.tk-lb{display:-webkit-flex;display:flex;-webkit-justify-content:space-between;justify-content:space-between;font-size:10px;color:#555;margin-bottom:4px}
.bt{display:-webkit-flex;display:flex;margin-left:-2px;margin-right:-2px}
.bt button{-webkit-flex:1;flex:1;margin:0 2px;padding:7px 0;border:none;border-radius:6px;font-size:11px;background:rgba(255,255,255,0.07);color:#aaa;cursor:pointer}
.bt button:active{background:rgba(255,255,255,0.16);color:#ddd}
.bt button.dz{background:rgba(0,210,255,0.12);color:#00d2ff}
.bt button.dz:active{background:rgba(0,210,255,0.22)}
.pr{display:-webkit-flex;display:flex;margin-left:-3px;margin-right:-3px;margin-bottom:6px}
.pr button{-webkit-flex:1;flex:1;margin:0 3px;padding:10px 0;border:none;border-radius:8px;font-size:13px;background:rgba(255,255,255,0.05);color:#bbb;cursor:pointer}
.pr button:active{background:rgba(0,210,255,0.12);color:#fff}
.pr button.do{background:rgba(255,215,64,0.1);color:#ffd740}
.pn{background:rgba(255,255,255,0.03);border-radius:8px;padding:8px 10px;border:1px solid rgba(255,255,255,0.03)}
.pn .pt{display:-webkit-flex;display:flex;-webkit-justify-content:space-between;justify-content:space-between;font-size:11px;color:#666;margin-bottom:4px}
.pn .pt .stt{font-size:10px;color:#555}
.pn .pt .stt .sp{display:inline-block;width:5px;height:5px;border-radius:50%;background:#555;margin-right:3px;vertical-align:middle}
.pn .pt .stt .sp.go{background:#76ff03;animation:blink 0.6s infinite}
.pn .rw{display:-webkit-flex;display:flex;-webkit-justify-content:space-between;justify-content:space-between;padding:2px 0;font-size:11px;border-bottom:1px solid rgba(255,255,255,0.02)}
.pn .rw:last-child{border-bottom:none}
.pn .rw .rv{color:#00d2ff;font-weight:600}
.ft{text-align:center;padding:12px 0 2px;font-size:10px;color:#333}
</style>
</head>
<body>
<div class="wrap">
<div class="hd">
<h1>机械臂控制台</h1>
<div class="sub">四舵机 · PCA9685</div>
</div>
<div class="conn"><span class="dot" id="dot"></span><span id="ctxt">连接中...</span></div>
<div class="cd" id="cd"></div>
<div class="pr">
<button id="bthome" onclick="dp('home')">归零</button>
<button id="btwave" onclick="dp('wave')">挥手</button>
<button id="btgrab" onclick="dp('grab')">抓取</button>
<button id="btrelease" onclick="dp('release')">松开</button>
</div>
<div class="pn">
<div class="pt"><span>实时状态</span><span class="stt"><span class="sp" id="sp"></span><span id="sttl">就绪</span></span></div>
<div class="rw"><span>舵机1 基座</span><span class="rv" id="st0">90°</span></div>
<div class="rw"><span>舵机2 肩部</span><span class="rv" id="st1">-10°</span></div>
<div class="rw"><span>舵机3 肘部</span><span class="rv" id="st2">0°</span></div>
<div class="rw"><span>舵机4 夹爪</span><span class="rv" id="st3">0°</span></div>
</div>
<div class="ft">RobotArm · RAC</div>
</div>
<script>
var L=[{a:0,b:180},{a:40,b:95},{a:70,b:115},{a:80,b:120}];
var NM=["基座旋转","肩部关节","肘部关节","夹爪"];
var ST=[90,80,90,90];
var S=[90,80,90,90];
var T=[90,80,90,90];
var P=false;
var AC=false;
function r2(i,v){return i===0?v:v-90}
function p2(i,v){return((v-L[i].a)/(L[i].b-L[i].a)*100).toFixed(1)}
function build(){
  var h='';
  for(var i=0;i<4;i++){
    var lo=L[i].a,hi=L[i].b;
    var r0=r2(i,lo),r1=r2(i,hi);
    var lb=i===0?lo+'°':(r0<0?'':r0==0?'0':'')+r0+'°';
    var rb=i===0?hi+'°':(r1>0?'+':r1==0?'':r1<0?'':'')+r1+'°';
    var ms=(i===0?'90°':i===1?'-10°':i===2?'0°':'0°');
    h+='<div class="c h'+i+'">'+
      '<div class="ct">'+
        '<span class="cl">'+NM[i]+'<span class="sn">舵机'+(i+1)+'</span></span>'+
        '<span class="cv" id="v'+i+'">0°<span class="mv" id="mv'+i+'"></span></span>'+
      '</div>'+
      '<div class="sl">'+
        '<div class="tk"></div>'+
        '<div class="fl" id="f'+i+'" style="width:50%"></div>'+
        '<input type="range" id="s'+i+'" min="'+lo+'" max="'+hi+'" value="'+ST[i]+'">'+
      '</div>'+
      '<div class="tk-lb"><span>'+lb+'</span><span>'+ms+'</span><span>'+rb+'</span></div>'+
      '<div class="bt">'+
        '<button onclick="go('+i+','+lo+')">'+lb+'</button>'+
        '<button class="dz" onclick="go('+i+','+ST[i]+')">'+ms+'</button>'+
        '<button onclick="go('+i+','+hi+')">'+rb+'</button>'+
      '</div></div>';
  }
  document.getElementById('cd').innerHTML=h;
  for(var i=0;i<4;i++){(function(j){
    var e=document.getElementById('s'+j);
    e.addEventListener('input',function(){slide(j)});
    e.addEventListener('change',function(){commit(j)});
  })(i);}
  ref();
}
function slide(i){
  var v=parseInt(document.getElementById('s'+i).value);
  show(i,v);
}
function commit(i){
  var v=parseInt(document.getElementById('s'+i).value);
  AC=true;
  for(var j=0;j<4;j++)document.getElementById('bt'+['home','wave','grab','release'][j]).className='';
  sd(i,v);
}
function go(i,v){
  v=Math.max(L[i].a,Math.min(L[i].b,v));
  document.getElementById('s'+i).value=v;
  AC=true;
  for(var j=0;j<4;j++)document.getElementById('bt'+['home','wave','grab','release'][j]).className='';
  sd(i,v);
}
function show(i,v){
  var r=r2(i,v);
  var rs=i===0?r+'°':(r>0?'+':r==0?'0':'')+r+'°';
  document.getElementById('v'+i).innerHTML=rs+'<span class="mv" id="mv'+i+'"></span>';
  document.getElementById('st'+i).textContent=rs;
  var pc=p2(i,v);
  document.getElementById('f'+i).style.width=pc+'%';
}
function ref(){
  for(var i=0;i<4;i++){
    show(i,S[i]);
    var el=document.getElementById('v'+i);
    if(S[i]!==T[i]){el.className='cv moving'}else{el.className='cv'}
  }
  var mo=false;for(var i=0;i<4;i++){if(S[i]!==T[i]){mo=true;break}}
  var sp=document.getElementById('sp');var stl=document.getElementById('sttl');
  if(mo){sp.className='sp go';stl.textContent='运动中'}else{sp.className='sp';stl.textContent='就绪'}
}
async function sd(i,v){
  if(P)return;P=true;
  try{
    var r=await fetch('/api?'+i+'='+v);var d=await r.json();
    if(d&&d.t){for(var j=0;j<4;j++){T[j]=parseInt(d.t[j]);S[j]=parseInt(d.c[j])}ref()}
  }catch(e){}P=false;
}
async function poll(){
  try{
    var r=await fetch('/status');var d=await r.json();
    if(d&&d.c&&d.t){
      for(var i=0;i<4;i++){S[i]=parseInt(d.c[i]);T[i]=parseInt(d.t[i])}
      for(var i=0;i<4;i++){
        var el=document.getElementById('s'+i);
        if(el && document.activeElement!==el) el.value=S[i];
      }
      ref();
      var mo=false;for(var i=0;i<4;i++){if(S[i]!==T[i]){mo=true;break}}
      document.getElementById('dot').className=mo?'dot move':'dot ok';
    }
    document.getElementById('ctxt').textContent='已连接';
  }catch(e){
    document.getElementById('dot').className='dot';document.getElementById('ctxt').textContent='等待连接...';
  }
}
async function dp(name){
  if(P)return;
  AC=true;
  for(var j=0;j<4;j++)document.getElementById('bt'+['home','wave','grab','release'][j]).className='';
  document.getElementById('bt'+name).className='do';
  try{
    var r=await fetch('/action?name='+name);var d=await r.json();
    if(d&&d.t){for(var j=0;j<4;j++){T[j]=parseInt(d.t[j])}ref()}
  }catch(e){}
}
build();poll();setInterval(poll,250);
</script>
</body>
</html>
)rawhtml";

void handleRoot() {
  server.send(200, "text/html; charset=utf-8", PAGE_HTML);
}
void handleStatus() {
  String json = "{\"c\":[";
  for (int i = 0; i < 4; i++) { if (i > 0) json += ","; json += String(currentAngles[i]); }
  json += "],\"t\":[";
  for (int i = 0; i < 4; i++) { if (i > 0) json += ","; json += String(targetAngles[i]); }
  json += "]}";
  server.send(200, "application/json", json);
}
void handleApi() {
  if (seqRunning) stopSequence();
  for (int i = 0; i < server.args(); i++) {
    String key = server.argName(i);
    bool isNum = key.length() > 0;
    for (size_t k = 0; k < key.length(); k++) {
      if (!isDigit(key[k])) { isNum = false; break; }
    }
    if (!isNum) continue;
    int servoIndex = key.toInt();
    if (servoIndex >= 0 && servoIndex < 4) {
      int rawAngle = server.arg(i).toInt();
      targetAngles[servoIndex] = constrain(rawAngle, ANGLE_MIN[servoIndex], ANGLE_MAX[servoIndex]);
    }
  }
  String json = "{\"c\":[";
  for (int i = 0; i < 4; i++) { if (i > 0) json += ","; json += String(currentAngles[i]); }
  json += "],\"t\":[";
  for (int i = 0; i < 4; i++) { if (i > 0) json += ","; json += String(targetAngles[i]); }
  json += "]}";
  server.send(200, "application/json", json);
}
void handleAction() {
  String name = "";
  for (int i = 0; i < server.args(); i++) {
    if (server.argName(i) == "name") { name = server.arg(i); break; }
  }
  if (name == "home") startSequence(seqHome, seqHomeCount);
  else if (name == "wave") startSequence(seqWave, seqWaveCount);
  else if (name == "grab") startSequence(seqGrab, seqGrabCount);
  else if (name == "release") startSequence(seqRelease, seqReleaseCount);

  String json = "{\"t\":[";
  for (int i = 0; i < 4; i++) { if (i > 0) json += ","; json += String(targetAngles[i]); }
  json += "]}";
  server.send(200, "application/json", json);
}

// 【修改】校准接口：min/max 单位为 µs（例：/calib?min=500&max=2400）
void handleCalib() {
  for (int i = 0; i < server.args(); i++) {
    if (server.argName(i) == "min") PULSE_US_MIN = server.arg(i).toInt();
    if (server.argName(i) == "max") PULSE_US_MAX = server.arg(i).toInt();
  }
  String json = "{\"min\":" + String(PULSE_US_MIN) + ",\"max\":" + String(PULSE_US_MAX) + "}";
  server.send(200, "application/json", json);
}
void handleNotFound() {
  server.send(404, "text/plain", "404");
}

void stepServos() {
  unsigned long now = millis();
  for (int i = 0; i < 4; i++) {
    if (currentAngles[i] == targetAngles[i]) continue;
    if (now - lastStepMs[i] < STEP_INTERVAL_MS) continue;

    if (currentAngles[i] < targetAngles[i]) currentAngles[i]++;
    else                                    currentAngles[i]--;

    writeServo(i, SERVO_CH[i], currentAngles[i]);
    lastStepMs[i] = now;
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n机械臂 启动");

  // 【新增】先禁用 PCA9685 输出，防止初始化期间舵机收到跳变信号
  pinMode(PCA_OE, OUTPUT);
  digitalWrite(PCA_OE, HIGH);

  // 【修改】I2C 显式 400kHz（GPIO8=SDA / GPIO9=SCL）
  Wire.begin(I2C_SDA, I2C_SCL, 400000);

  pwm.begin();
  pwm.setPWMFreq(50);
  // 若角度整体偏移（不抖但角度错），模块用的是 27MHz 晶振时取消下面注释：
  // pwm.setOscillatorFrequency(27000000);

  // 输出仍禁用状态下，给所有舵机写入 90° 安全角度（同时更新 lastUs 缓存）
  for (int i = 0; i < 4; i++) {
    writeServo(i, SERVO_CH[i], 90);
  }
  // 等舵机电源稳定后使能输出，舵机直接停在 90°，无跳变
  delay(300);
  digitalWrite(PCA_OE, LOW);

  Serial.println("PCA9685 初始化完成");

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.print("热点: "); Serial.println(AP_SSID);
  Serial.print("IP:   "); Serial.println(WiFi.softAPIP());
  dnsServer.start(53, "*", WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/api", handleApi);
  server.on("/status", handleStatus);
  server.on("/action", handleAction);
  server.on("/calib", handleCalib);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("Web服务器已启动");
}

void loop() {
  stepServos();
  runSequence();
  server.handleClient();
  dnsServer.processNextRequest();

  // 【新增】每 5s 探测 PCA9685，I2C 总线掉线时打印报警，不静默丢更新
  static unsigned long lastI2cCheck = 0;
  if (millis() - lastI2cCheck > 5000) {
    lastI2cCheck = millis();
    Wire.beginTransmission(0x40);
    if (Wire.endTransmission() != 0) {
      Serial.println("[警告] PCA9685 掉线！检查 I2C 接线/上拉电阻/供电");
    }
  }
}
