/*
 *      web_debug.c
 *      Tiny HTTP server (port 80) for bench debugging from a browser.
 *
 *        GET /          the dashboard page (buttons + live status cards)
 *        GET /s         JSON snapshot of every subsystem, polled twice a second
 *        GET /c?d=<k>   run one command, reply with a one-line status
 *
 *      Threading: lwIP is built NO_SYS=1, so every lwIP call and every
 *      callback below runs in the single WiFi service task
 *      (cyw43_utk_task).  Nothing here may block.  Commands go to
 *      subsystem 2 through web_debug_bridge.c (queue send, TMO_POL).
 *
 *      Safety: the page re-sends the held command every 200 ms.  If the
 *      browser, the phone or the WiFi goes away the refreshes stop and
 *      S2_CMD_TIMEOUT_MS (subsystem 2) brings the robot to a stop.
 *      Only enable this on the bench: there is no password.
 */

#include "web_debug.h"

#if S2_WEBDEBUG && TM_NET_LWIP

#include <stdint.h>
#include <string.h>

#include "lwip/err.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include <tm/tmonitor.h>

#include "web_debug_bridge.h"

#define WD_PORT      80
#define WD_REQ_MAX   64  /* only the request line is needed */
#define WD_REPLY_MAX 240
#define WD_SNAP_MAX  1900 /* JSON for the whole dashboard */

static const char k_hdr_html[] =
    "HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n";
static const char k_hdr_text[] =
    "HTTP/1.0 200 OK\r\nContent-Type: text/plain\r\nCache-Control: no-store\r\n"
    "Connection: close\r\n\r\n";
static const char k_404[] =
    "HTTP/1.0 404 Not Found\r\nConnection: close\r\n\r\nnot found";

/* Kept in flash and sent without copying (tcp_write with flags = 0).
   Two files because one TCP send buffer holds about 11 KB. */
static const char k_page[] =
    "<!doctype html><html><head><meta charset=utf-8>\n"
    "<meta name=viewport content='width=device-width,initial-scale=1'>\n"
    "<title>Robot dashboard</title><style>\n"
    "body{font-family:sans-serif;margin:10px;touch-action:manipulation;-webkit-user-select:none;user-select:none}\n"
    "h2{text-align:center;margin:6px}\n"
    ".pad{text-align:center}\n"
    "button{font-size:1.2em;width:30%;height:3.2em;margin:3px;border-radius:10px}\n"
    "button.s{height:2.4em;width:auto;padding:0 .7em;font-size:1em}\n"
    "button.s.on{outline:3px solid #06c}\n"
    "#s{background:#c00;color:#fff;font-weight:bold;width:90%;height:2.6em}\n"
    "#st{font:12px monospace;margin:8px 0;word-break:break-word;text-align:center}\n"
    ".gr{display:grid;grid-template-columns:repeat(auto-fit,minmax(230px,1fr));gap:8px;margin-top:8px}\n"
    ".c{border:1px solid #888;border-radius:8px;padding:6px 10px}\n"
    ".c b{display:block;margin-bottom:4px}\n"
    ".c div{display:flex;justify-content:space-between;font:13px monospace;padding:1px 0}\n"
    ".c i{font-style:normal;font-weight:bold}\n"
    ".n{color:#888;font-weight:normal;font-size:12px}\n"
    ".tab{display:none}.tab.on{display:block}\n"
    ".bar{position:relative;width:60%;height:12px;background:#ddd;border-radius:6px}\n"
    ".bar u{position:absolute;top:-2px;width:6px;height:16px;background:#06c;border-radius:3px}\n"
    "@media(prefers-color-scheme:dark){body{background:#111;color:#eee}button{background:#333;color:#eee}#s{background:#c00}.c{border-color:#555}.bar{background:#444}}\n"
    "</style></head><body><h2>Robot dashboard <small id=me class=n></small></h2>\n"
    "<div class=pad id=nav><button class=s data-t=m>Motor</button><button class=s data-t=o>Obstacle</button><button class=s data-t=i>IMU</button><button class=s data-t=l>Line</button><button class=s data-t=v>Mission</button><button class=s data-t=a>All data</button><button class=s data-t=g>Guide</button></div>\n"
    "<div class=pad><button id=s data-c=s>STOP (works for everyone)</button></div>\n"
    "<div id=st>ready</div>\n"
    "<div class=tab id=t-g><div id=card-g class=n>loading...</div></div>\n"
    "<div class=tab id=t-m><div class=pad id=ctl-m>\n"
    "<div><button data-c=f>FORWARD</button></div>\n"
    "<div><button data-c=l>SPIN LEFT</button><button data-c=b>BACK</button><button data-c=r>SPIN RIGHT</button></div>\n"
    "<div><button data-k=L>Turn 90 L</button><button data-k=x>Reset</button><button data-k=R>Turn 90 R</button></div>\n"
    "<div>Trim L <button class=s data-k=t>-</button><button class=s data-k=T>+</button> R <button class=s data-k=u>-</button><button class=s data-k=U>+</button> <button class=s data-k=h>Straight on/off</button></div>\n"
    "<div>Speed <button class=s data-k=v>-</button><button class=s data-k=V>+</button></div>\n"
    "<div>Speed correction strength (Kp) <button class=s data-k=p>-</button><button class=s data-k=P>+</button> Drift fix (Ki) <button class=s data-k=i>-</button><button class=s data-k=I>+</button></div>\n"
    "<p class=n>Kp: how hard the motor reacts when a wheel is slower or faster than wanted. Too low = sluggish, too high = shaking. Ki: slowly removes a steady speed error. Too high = overshoot. Change one, press + or - a few times, watch the speed card.</p>\n"
    "<p class=n>Needs Manual mode (Mission tab). Keys: arrows, space = stop.</p></div><div class=gr id=card-m></div></div>\n"
    "<div class=tab id=t-o><div class=pad id=ctl-o>\n"
    "<div>Servo (turns the ultrasonic sensor): <button class=s data-k=Q>Left 45</button><button class=s data-k=W>Centre</button><button class=s data-k=E>Right 45</button></div>\n"
    "<div><button class=s data-k=e>Obstacle monitoring on</button> Mission: <button class=s data-k=g>Ignore obstacles</button><button class=s data-k=G>Use obstacles</button></div>\n"
    "<p class=n>First press switches monitoring off, press again to move.</p></div><div class=gr id=card-o></div></div>\n"
    "<div class=tab id=t-i><div class=pad id=ctl-i>\n"
    "<div><button class=s data-k=j>Calibrate (still and level)</button><button class=s data-k=J>Reset peak hump</button></div>\n"
    "<div>Mission: <button class=s data-k=q>Ignore IMU</button><button class=s data-k=H>Use IMU</button></div>\n"
    "<p class=n>Ignore IMU makes the mission skip hump, impact and tilt events, so you can test line following on its own.</p></div><div class=gr id=card-i></div></div>\n"
    "<div class=tab id=t-l><div class=pad id=ctl-l>\n"
    "<div><button class=s data-k=y>Calibrate sensors</button><button class=s data-k=Y>Reset line search</button></div>\n"
    "<div>Barcodes <button class=s data-k=z>On</button><button class=s data-k=Z>Off</button> Steering strength (Kp) <button class=s data-k=k>-</button><button class=s data-k=K>+</button> Wobble damping (Kd) <button class=s data-k=d>-</button><button class=s data-k=D>+</button></div>\n"
    "<div>Junction detection: <button class=s data-k=5>Off</button><button class=s data-k=6>C+R</button><button class=s data-k=7>L+C+R</button></div>\n"
    "<div>Junction threshold <button class=s data-k=3>-</button><button class=s data-k=4>+</button> <span class=n>0-1000 scale, default 950; C and R must both read above it</span></div>\n"
    "<div>Wait before LOST (ms) <button class=s data-k=1>-</button><button class=s data-k=2>+</button> <span class=n>steps of 100 ms; default 1500 ms. With the line between C and R, both-white is normal, so this only decides when the robot is really off the track</span></div>\n"
    "<div>Steering direction: <button class=s data-k=X>Normal</button><button class=s data-k=w>Swapped</button></div>\n"
    "<div>Mission: <button class=s data-k=a>Ignore barcodes</button><button class=s data-k=B>Use barcodes</button></div>\n"
    "<div>Target position <button class=s data-k=o>-</button><button class=s data-k=O>+</button> Sensors that steer: <button class=s data-k=m>C+R</button><button class=s data-k=n>L+C+R</button></div>\n"
    "<p class=n>Line layout (C+R): the line runs BETWEEN the centre and right sensors. Both white = straight. Centre sees black = steer left. Right sees black = steer right. Both black = junction. The left sensor only reads barcodes. Keep Target position at 0. L+C+R is the old layout where the line sits under the sensors.</p>\n"
    "<p class=n>Kp: how sharply the robot turns back to the line. Too low = leaves the line on curves, too high = zig-zags. Kd: calms the zig-zag. Too high = twitchy. Raise Kp until it just starts to wobble, then add Kd.</p></div><div class=gr id=card-l></div></div>\n"
    "<div class=tab id=t-v><div class=pad id=ctl-v>\n"
    "<div>Mode: <button class=s data-k=M>Manual</button><button class=s data-k=A>Unlock autonomous</button><button class=s data-k=F>Line follow</button><button class=s data-k=N>Monitor only</button></div>\n"
    "<div>Mission speed % <button class=s data-k=c>-</button><button class=s data-k=C>+</button></div></div><div class=gr id=card-v></div>\n"
    "<div class=c style='margin-top:8px'><b>Settings to copy into the config files</b><pre id=cp style='margin:0;font-size:12px;white-space:pre-wrap'></pre></div></div>\n"
    "<div class=tab id=t-a><div class=gr id=card-a></div></div>\n"
    "<div class=c style='margin-top:8px'><div style='display:flex;gap:8px;justify-content:center;flex-wrap:wrap'><div style='text-align:center'><canvas id=gL width=200 height=125></canvas><div id=nL style='font-size:24px;font-weight:bold;color:#4a9eff'>-</div></div><div style='text-align:center'><canvas id=gR width=200 height=125></canvas><div id=nR style='font-size:24px;font-weight:bold;color:#ff9a3c'>-</div></div></div><div id=rp>Wheel RPM monitor: waiting for data (open the Motor tab)</div></div>\n"
    "<div class=c style='margin-top:8px'><b>Speed graph</b> (Motor tab open: target = grey, left = blue, right = orange; about 1 point per 0.6 s, last 2 min)<br><canvas id=gr width=600 height=140 style='width:100%;max-width:600px'></canvas></div>\n"
    "<div class=c style='margin-top:8px'><b>Debug modes</b><pre id=md style='margin:4px 0 0;font-size:12px;white-space:pre-wrap'></pre></div>\n"
    "<div class=c style='margin-top:8px'><b>Log (like a serial monitor)</b> <button id=lc>Clear</button><pre id=lg style='margin:4px 0 0;font-size:12px;height:150px;overflow:auto;white-space:pre-wrap'></pre></div>\n"
    "<script src=/a.js></script></body></html>\n";

static const char k_guide[] =
    "<div class=c><b>Quick start</b><p>1. Motor tab: check each direction first (STOP is always on top). 2. Obstacle, IMU and Line tabs: one person per tab, the tab shows (in use) when someone else has it. 3. Mission tab: Line follow stays locked until you press Unlock autonomous, after the manual values are good.</p></div>\n"
    "<div class=c><b>What are Kp, Ki, Kd?</b><p><b>Kp</b> = how hard it reacts to the error right now. <b>Ki</b> = slowly removes a steady error that never goes away. <b>Kd</b> = brakes fast changes, so it does not overshoot. Change one at a time, a few small steps, then watch the graph or the Log.</p></div>\n"
    "<div class=c><b>Motor speed loop (Motor tab)</b><p>Uses Kp and Ki. Hold forward 8-10 s and watch the Speed graph: blue (left) and orange (right) should rise to the grey target in about 1 s, overshoot under 10%, then go flat. Steady gap below grey: raise Ki a little. Ringing or overshoot: lower Ki, then Kp. Repeat at a slow and a fast speed.</p></div>\n"
    "<div class=c><b>Damping</b><p>Underdamped = overshoots and wobbles (Kp or Ki too high). Overdamped = slow creep (Kp too low). Critically damped = fast rise, at most one small overshoot, then flat: the goal. Unstable = swings that do not shrink: lower the gains. A gentler ramp or a lower speed also adds damping.</p></div>\n"
    "<div class=c><b>Is it stable?</b><p>Same result three runs in a row at race speed, no overshoot over 10%, no sustained swing, no faults, no drift after 30 s, and it recovers after a bump or a held wheel. Leave a safety margin: back Kp off one or two steps from the edge.</p></div>\n"
    "<div class=c><b>Line steering (Line tab)</b><p>Position = weighted average of the sensor readings (-1000 left to +1000 right). error = position - centre. steer = (Kp x error + Kd x change in error) / 10. Raise Kp until it follows curves, add Kd to stop the zig-zag, lower the mission speed if it still wobbles. Press Calibrate sensors over the line and the floor first.</p></div>\n"
    "<div class=c><b>Digital or analogue sensors</b><p>Digital: each sensor is black or not, so the position jumps in steps and steering is coarse. Analogue (S3_IR_USE_ADC 1): each sensor reads 0-1000, so the position moves smoothly and curves and circles follow better, and Kd can be larger. Only the centre and right sensors can be analogue (GP26, GP27); the left one stays digital. With centre and right only, a line far to the left is guessed from the last side seen.</p></div>\n"
    "<div class=c><b>IMU states (IMU tab)</b><p>INIT, CALIBRATING (keep still), MONITORING (normal), CLIMBING, AT_PEAK, DESCENDING, ERROR. Motion: STATIONARY, CRUISING, ACCELERATING, BRAKING, TURNING_LEFT, TURNING_RIGHT. Calibrate flat and still, then Reset peak hump before each test.</p></div>\n"
    "<div class=c><b>Open items</b><p>1. Rebuild and flash once to get this page. 2. Values here reset on reboot: copy the final ones from the Mission tab box into the config files. 3. Line follow tuning is still untested on the track. 4. To use analogue, set S3_IR_USE_ADC 1 and the ADC pins in the shared adc_cnf_sysdep.h (agree with the team first). 5. Set S2_WEBDEBUG to 0 for race runs. 6. Nothing is committed yet.</p></div>\n"
    ;

static const char k_js[] =
    "var cid=Math.random().toString(16).slice(2,10),t=null,mode=0,cur='m',busy=0,st=document.getElementById('st');\n"
    "try{if(sessionStorage.cid)cid=sessionStorage.cid;else sessionStorage.cid=cid}catch(e){}\n"
    "document.getElementById('me').textContent='you: '+cid;\n"
    "var lg=document.getElementById('lg'),lc=null,lr='';\n"
    "function log(m){var d=new Date(),p=function(n){return ('0'+n).slice(-2)};lg.textContent+=p(d.getHours())+':'+p(d.getMinutes())+':'+p(d.getSeconds())+'  '+m+String.fromCharCode(10);var a=lg.textContent.split(String.fromCharCode(10));if(a.length>80)lg.textContent=a.slice(-60).join(String.fromCharCode(10));lg.scrollTop=lg.scrollHeight}\n"
    "document.getElementById('lc').onclick=function(){lg.textContent=''};\n"
    "function send(c){var n=c!=lc;lc=c;if(n)log('> '+c);fetch('/c?d='+c+'&id='+cid).then(function(r){return r.text()}).then(function(x){st.textContent=x;var k=x.split(' ')[0];if(n||k!=lr)log('< '+x);lr=k}).catch(function(){st.textContent='NO LINK - robot will stop';log('NO LINK')})}\n"
    "function hold(c){if(mode)return;send(c);clearInterval(t);t=setInterval(function(){send(c)},200)}\n"
    "function rel(f){if(t===null&&f!==true)return;clearInterval(t);t=null;send('s')}\n"
    "document.querySelectorAll('button[data-c]').forEach(function(b){var c=b.dataset.c;\n"
    "if(c=='s'){b.onclick=function(){rel(true)};return}\n"
    "b.onpointerdown=function(){hold(c)};\n"
    "['onpointerup','onpointerleave','onpointercancel'].forEach(function(e){b[e]=rel})});\n"
    "document.querySelectorAll('button[data-k]').forEach(function(b){b.onclick=function(){send(b.dataset.k)}});\n"
    "var km={ArrowUp:'f',ArrowDown:'b',ArrowLeft:'l',ArrowRight:'r'},kd=null;\n"
    "onkeydown=function(e){if(km[e.key]&&kd!=e.key&&cur=='m'){kd=e.key;hold(km[e.key])}if(e.key==' '){e.preventDefault();rel(true)}};\n"
    "onkeyup=function(e){if(km[e.key]){kd=null;rel()}};\n"
    "onblur=rel;\n"
    "var N={m:['IDLE','DRIVING','DRIVE_DIST','TURNING','STOPPING','SAFE_STOP','FAULT'],\n"
    "l:['INIT','CALIBRATE','SEARCH','FOLLOW','LOST','JUNCTION'],\n"
    "i:['INIT','CALIBRATING','MONITORING','CLIMBING','AT_PEAK','DESCENDING','ERROR'],\n"
    "mo:['UNKNOWN','STATIONARY','CRUISING','ACCELERATING','BRAKING','TURN_LEFT','TURN_RIGHT'],\n"
    "o:['IDLE','MONITORING','DETECTED','COARSE_SCAN','FINE_SCAN','PLANNING','BYPASSING','SEARCH_LINE','LINE_RECOVERY','ERROR'],\n"
    "v:['IDLE','MISSION_START','FOLLOW_LINE','JUNCTION_STOP','LOST_LINE','AVOID_OBSTACLE','DECODE_BARCODE','EXECUTE_ACTION','TERRAIN_CHECK','FAULT','MISSION_COMPLETE'],\n"
    "b:['IDLE','DETECT','READ','DECODE','RETRY','COMPLETE']};\n"
    "function nm(a,i){return a[i]!==undefined?a[i]:i}\n"
    "function card(t,rows){var h='<div class=c><b>'+t+'</b>';\n"
    "if(!rows)return h+'<div class=n>not running</div></div>';\n"
    "rows.forEach(function(r){h+='<div><span>'+r[0]+'</span><i>'+r[1]+'</i></div>'});return h+'</div>'}\n"
    "function f1(x,d){return (x/d).toFixed(1)}\n"
    "var B={\n"
    "m:function(a,c){return card('Motor (S2)',a&&[['state',nm(N.m,a.state)+(a.fault?' fault '+a.fault:'')],['speed L / R rpm',rpm(a.vL)+' / '+rpm(a.vR)],['target L / R rpm',rpm(a.tgtL)+' / '+rpm(a.tgtR)],['duty L / R',a.dutyL+' / '+a.dutyR],['ticks L / R',a.ticksL+' / '+a.ticksR],['distance mm',a.dist],['heading deg',f1(a.head,10)],['loop overruns',a.over],['trim L / R',c.trimL+' / '+c.trimR],['test speed mm/s (rpm) / straight',c.speed+' ('+rpm(c.speed)+') / '+(c.straight?'ON':'OFF')],['Kp reaction strength / Ki drift fix',(c.pidKp100/100).toFixed(2)+' / '+(c.pidKi100/100).toFixed(2)]])},\n"
    "l:function(l,c){return card('Line sensors (S3)',l&&[['state',nm(N.l,l.state)],['position',l.pos+'<br><div class=bar><u style=left:'+((l.pos+1000)/20)+'%></u></div>'],['error / steer',l.err+' / '+l.steer],['raw L C R',l.r0+' '+l.r1+' '+l.r2],['black mask',l.mask],['calibrated',l.cal?'yes':'no'],['barcode',nm(N.b,l.bc)+' ok '+l.bcOk+' fail '+l.bcFail],['Kp steering strength / Kd wobble damping',c.lineKp+' / '+c.lineKd],['target position / sensors steering',c.lsp+' / '+(c.lall?'L C R':'C R')],['steering direction',c.sinv?'SWAPPED':'normal'],['wait before LOST (ms)',c.lost*10],['junction threshold',c.jthr],['junction uses',c.jmode<0?'config default':(c.jmode==0?'OFF':(c.jmode==1?'C R':'L C R'))]])},\n"
    "i:function(i){return card('IMU (S4)',i&&[['state',nm(N.i,i.state)+(i.fault?' FAULT':'')],['motion',nm(N.mo,i.motion)],['pitch / roll deg',f1(i.pitch,100)+' / '+f1(i.roll,100)],['heading deg',f1(i.head,100)],['turn rate deg/s',f1(i.turn,100)],['humps / impacts',i.humps+' / '+i.impacts],['max hump mm',f1(i.maxPeak,1000)],['calibrated',i.cal?'yes':'no']])},\n"
    "o:function(o){return card('Obstacle + servo (S5)',o&&[['state',nm(N.o,o.state)],['distance mm',o.dist?o.dist:'no echo'],['servo deg (+left)',o.servo],['pings / timeouts',o.pings+' / '+o.timeouts],['obstacles',o.obst],['last fault',o.fault]])},\n"
    "v:function(v,c){return card('Mission (VC)',v&&[['state',nm(N.v,v.state)],['previous',nm(N.v,v.prev)],['in state (s)',f1(v.sinceMs,1000)],['barcodes read',v.barcodes],['events handled',v.events],['wifi',v.wifi?'up':'down'],['last fault',v.fault],['dashboard mode',['MANUAL','LINE FOLLOW','MONITOR'][c.mode]],['autonomous',c.arm?'UNLOCKED':'LOCKED (manual only)'],['obstacle checking',c.obst?'ON':'OFF (ignored)'],['IMU checking',c.imu?'ON':'OFF (ignored)'],['barcode reading',c.bc?'ON':'OFF (ignored)'],['mission speed',c.pct+' %']])}};\n"
    "var W={m:'m',o:'o',i:'i',l:'l',v:'v',a:'miolv'};\n"
    "var G={t:[],l:[],r:[]};\n"
    "function rpm(v){return Math.round(v*60/204.2)}\n"
    "var GM=60;function meter(id,nm,col,v,t){var c=document.getElementById(id),x=c.getContext('2d'),w=c.width,h=c.height,cx=w/2,cy=h-18,R=80,P=Math.PI,m=Math.max(Math.abs(v),Math.abs(t));if(m>GM*0.9)GM=Math.ceil(m*1.25/10)*10;var an=function(q){q=Math.max(-GM,Math.min(GM,q));return P+(q+GM)/(2*GM)*P};x.clearRect(0,0,w,h);x.lineWidth=10;x.strokeStyle='#8884';x.beginPath();x.arc(cx,cy,R,P,2*P);x.stroke();x.strokeStyle=col;x.beginPath();x.arc(cx,cy,R,an(0),an(v),v<0);x.stroke();x.lineWidth=2;x.strokeStyle='#888';x.beginPath();x.moveTo(cx+(R-14)*Math.cos(an(t)),cy+(R-14)*Math.sin(an(t)));x.lineTo(cx+(R+8)*Math.cos(an(t)),cy+(R+8)*Math.sin(an(t)));x.stroke();x.strokeStyle=col;x.lineWidth=3;x.beginPath();x.moveTo(cx,cy);x.lineTo(cx+(R-18)*Math.cos(an(v)),cy+(R-18)*Math.sin(an(v)));x.stroke();x.fillStyle='#888';x.font='11px sans-serif';x.fillText('-'+GM,cx-R-8,cy+14);x.fillText('+'+GM,cx+R-14,cy+14);x.fillText('0',cx-3,cy-R-12);x.font='bold 14px sans-serif';x.fillStyle=col;x.textAlign='center';x.fillText(nm==='L'?'LEFT':'RIGHT',cx,cy+14);x.textAlign='left';document.getElementById('n'+nm).textContent=v+' rpm (target '+t+')'}\n"
    "function rpmbox(a){var l=rpm(a.vL),r=rpm(a.vR),tl=rpm(a.tgtL),tr=rpm(a.tgtR);meter('gL','L','#4a9eff',l,tl);meter('gR','R','#ff9a3c',r,tr);var e=function(v,t){var d=v-t;return (d>0?'+':'')+d};var h='<b>Wheel RPM monitor</b><table style=\"width:100%;text-align:center\"><tr><td></td><td><b style=\"color:#4a9eff\">LEFT</b></td><td><b style=\"color:#ff9a3c\">RIGHT</b></td></tr><tr><td>actual rpm</td><td style=\"font-size:26px\">'+l+'</td><td style=\"font-size:26px\">'+r+'</td></tr><tr><td>target rpm</td><td>'+tl+'</td><td>'+tr+'</td></tr><tr><td>error (actual-target)</td><td>'+e(l,tl)+'</td><td>'+e(r,tr)+'</td></tr><tr><td>duty (permille)</td><td>'+a.dutyL+'</td><td>'+a.dutyR+'</td></tr></table>Left minus right: '+e(l,r)+' rpm'+(Math.abs(tl-tr)<2&&Math.abs(l-r)>3?' (driving straight but wheels differ: adjust trim)':'');document.getElementById('rp').innerHTML=h}\n"
    "function plot(a){rpmbox(a);G.t.push(rpm((a.tgtL+a.tgtR)/2));G.l.push(rpm(a.vL));G.r.push(rpm(a.vR));for(var k in G)if(G[k].length>200)G[k].shift();\n"
    "var cv=document.getElementById('gr'),x=cv.getContext('2d'),W2=cv.width,H=cv.height,m=30;G.t.concat(G.l,G.r).forEach(function(v){if(Math.abs(v)>m)m=Math.abs(v)});m*=1.1;x.clearRect(0,0,W2,H);x.strokeStyle='#888';x.beginPath();x.moveTo(0,H/2);x.lineTo(W2,H/2);x.stroke();x.fillStyle='#888';x.fillText('+'+Math.round(m)+' rpm',2,10);\n"
    "[['t','#999'],['l','#4a9eff'],['r','#ff9a3c']].forEach(function(s){x.strokeStyle=s[1];x.beginPath();G[s[0]].forEach(function(v,i){var px=i*W2/200,py=H/2-v/m*H/2;i?x.lineTo(px,py):x.moveTo(px,py)});x.stroke()})}\n"
    "var ps={};\n"
    "function chg(k,v,t){if(v===undefined)return;if(ps[k]!==undefined&&ps[k]!==v)log(t+': '+ps[k]+' -> '+v);ps[k]=v}\n"
    "function render(d){var c=d.cfg,h;\n"
    "var M=['Manual: buttons drive the robot (default)','Line follow: the real mission runs, buttons locked (needs Unlock autonomous)','Monitor only: no motion, just watch the sensors'],nl=String.fromCharCode(10),t2='';M.forEach(function(m,i){t2+=(i==c.mode?'>> ':'   ')+m+nl});t2+='Autonomous: '+(c.arm?'UNLOCKED':'LOCKED')+'   Obstacle checking: '+(c.obst?'ON':'OFF')+'   IMU checking: '+(c.imu?'ON':'OFF')+'   Barcodes: '+(c.bc?'ON':'OFF');document.getElementById('md').textContent=t2;\n"
    "if(d.s2)plot(d.s2);if(d.s2)chg('m',d.s2.state,'motor state');if(d.s2)chg('mf',d.s2.fault,'motor fault');if(d.s3)chg('l',d.s3.state,'line state');if(d.s4)chg('i',d.s4.humps,'humps');if(d.s5)chg('o',d.s5.state,'obstacle state');if(d.vc)chg('v',d.vc.state,'mission state');chg('md',c.mode,'mode');\n"
    "mode=c.mode;\n"
    "['m','l','i','o','v'].forEach(function(a){var s=c['lk'+a],e=document.getElementById('ctl-'+a);e.style.opacity=s==2?0.35:1;\n"
    "document.querySelector('button[data-t='+a+']').textContent=document.querySelector('button[data-t='+a+']').dataset.n+(s==1?' *':s==2?' (in use)':'')});\n"
    "if(cur=='a'){h='';['m','o','i','l','v'].forEach(function(a){h+=B[a](d[{m:'s2',l:'s3',i:'s4',o:'s5',v:'vc'}[a]],c)});document.getElementById('card-a').innerHTML=h}\n"
    "else{document.getElementById('card-'+cur).innerHTML=B[cur](d[{m:'s2',l:'s3',i:'s4',o:'s5',v:'vc'}[cur]],c)}\n"
    "var cp=['s2_config.h:  S2_TRIM_L_PERMILLE '+c.trimL+'   S2_TRIM_R_PERMILLE '+c.trimR,'s2_config.h:  S2_PID_KP '+(c.pidKp100/100).toFixed(2)+'f   S2_PID_KI '+(c.pidKi100/100).toFixed(2)+'f','s3_config.h:  S3_KP_NUM '+c.lineKp+'   S3_KD_NUM '+c.lineKd+'   S3_LINE_SETPOINT '+c.lsp+'   S3_JUNCTION_THRESHOLD '+c.jthr+(c.lall?'   S3_LINE_MASK L|C|R':''),'vc_sm.h:      speeds x '+c.pct+'%  (cruise '+Math.round(200*c.pct/100)+', slow '+Math.round(100*c.pct/100)+' mm/s)'].join(String.fromCharCode(10));\n"
    "var e=document.getElementById('cp');if(e.textContent!=cp)e.textContent=cp}\n"
    "function show(x){cur=x;['m','o','i','l','v','a','g'].forEach(function(k){document.getElementById('t-'+k).className='tab'+(k==x?' on':'')});\n"
    "document.querySelectorAll('button[data-t]').forEach(function(b){b.className='s'+(b.dataset.t==x?' on':'')})}\n"
    "document.querySelectorAll('button[data-t]').forEach(function(b){b.dataset.n=b.textContent;b.onclick=function(){show(b.dataset.t)}});\n"
    "show('m');\n"
    "fetch('/g').then(function(r){return r.text()}).then(function(x){document.getElementById('card-g').innerHTML=x}).catch(function(){});\n"
    "setInterval(function(){if(busy||document.hidden||cur=='g')return;busy=1;\n"
    "fetch('/s?x='+W[cur]+'&id='+cid).then(function(r){return r.json()}).then(render).catch(function(){}).then(function(){busy=0})},600);\n"
    "\n";

static const char k_hdr_js[] =
    "HTTP/1.0 200 OK\r\nContent-Type: application/javascript\r\n"
    "Cache-Control: no-store\r\nConnection: close\r\n\r\n";

/* "&id=<hex>" in the request line -> client id (0 if missing) */
static uint32_t parse_id(const char * p_req)
{
    const char * p = strstr(p_req, "id=");
    uint32_t     v = 0U;

    if (NULL == p)
    {
        return 0U;
    }
    for (p += 3; ; p++)
    {
        if ((*p >= '0') && (*p <= '9')) { v = (v << 4) | (uint32_t)(*p - '0'); }
        else if ((*p >= 'a') && (*p <= 'f')) { v = (v << 4) | (uint32_t)(*p - 'a' + 10); }
        else { break; }
    }
    return v;
}

/* "x=<letters>" -> section filter string ("" if missing = all) */
static const char * parse_x(const char * p_req, char * p_out, int32_t max)
{
    const char * p = strstr(p_req, "x=");
    int32_t      n = 0;

    if (NULL != p)
    {
        for (p += 2; (n < (max - 1)) && (*p >= 'a') && (*p <= 'z'); p++)
        {
            p_out[n++] = *p;
        }
    }
    p_out[n] = '\0';
    return p_out;
}

static struct tcp_pcb * g_listen_pcb;
static char             g_snap[WD_SNAP_MAX]; /* static: keeps the task stack small */

/* Send the reply and close.  Data already queued is still delivered. */
static void reply_and_close(struct tcp_pcb * p_pcb, const char * p_hdr,
                            const char * p_body, uint16_t body_len,
                            uint8_t body_copy)
{
    (void)tcp_write(p_pcb, p_hdr, (uint16_t)strlen(p_hdr), TCP_WRITE_FLAG_COPY);
    if (body_len > 0)
    {
        (void)tcp_write(p_pcb, p_body, body_len,
                        body_copy ? TCP_WRITE_FLAG_COPY : 0);
    }
    (void)tcp_output(p_pcb);
    (void)tcp_close(p_pcb);
}

static err_t on_recv(void * p_arg, struct tcp_pcb * p_pcb, struct pbuf * p_buf,
                     err_t err)
{
    char     req[WD_REQ_MAX];
    char     status[WD_REPLY_MAX];
    char     xs[8];
    uint16_t n;

    (void)p_arg;
    if ((NULL == p_buf) || (ERR_OK != err))
    {
        /* peer closed (or error): finish our side */
        if (NULL != p_buf)
        {
            pbuf_free(p_buf);
        }
        (void)tcp_close(p_pcb);
        return ERR_OK;
    }

    n = (uint16_t)((p_buf->tot_len < (WD_REQ_MAX - 1)) ? p_buf->tot_len
                                                       : (WD_REQ_MAX - 1));
    (void)pbuf_copy_partial(p_buf, req, n, 0);
    req[n] = '\0';
    tcp_recved(p_pcb, p_buf->tot_len);
    pbuf_free(p_buf);

    if (0 == strncmp(req, "GET /c?d=", 9))
    {
        (void)web_debug_motion(req[9], parse_id(req), status, (int32_t)sizeof(status));
        reply_and_close(p_pcb, k_hdr_text, status, (uint16_t)strlen(status), 1);
    }
    else if (0 == strncmp(req, "GET /s", 6))
    {
        (void)web_debug_snapshot(g_snap, (int32_t)sizeof(g_snap), parse_id(req),
                                 parse_x(req, xs, (int32_t)sizeof(xs)));
        reply_and_close(p_pcb, k_hdr_text, g_snap, (uint16_t)strlen(g_snap), 1);
    }
    else if (0 == strncmp(req, "GET /g", 6))
    {
        reply_and_close(p_pcb, k_hdr_html, k_guide, (uint16_t)(sizeof(k_guide) - 1U), 0);
    }
    else if (0 == strncmp(req, "GET /a.js", 9))
    {
        reply_and_close(p_pcb, k_hdr_js, k_js, (uint16_t)(sizeof(k_js) - 1U), 0);
    }
    else if ((0 == strncmp(req, "GET / ", 6)) ||
             (0 == strncmp(req, "GET /index", 10)))
    {
        reply_and_close(p_pcb, k_hdr_html, k_page,
                        (uint16_t)(sizeof(k_page) - 1U), 0);
    }
    else
    {
        reply_and_close(p_pcb, k_404, NULL, 0, 0);
    }
    return ERR_OK;
}

static void on_err(void * p_arg, err_t err)
{
    (void)p_arg;
    (void)err; /* pcb is already freed by lwIP */
}

static err_t on_accept(void * p_arg, struct tcp_pcb * p_pcb, err_t err)
{
    (void)p_arg;
    if ((ERR_OK != err) || (NULL == p_pcb))
    {
        return ERR_VAL;
    }
    tcp_recv(p_pcb, on_recv);
    tcp_err(p_pcb, on_err);
    return ERR_OK;
}

void web_debug_start(void)
{
    struct tcp_pcb * p_pcb;

    if (NULL != g_listen_pcb)
    {
        return; /* already running */
    }
    p_pcb = tcp_new();
    if ((NULL == p_pcb) || (ERR_OK != tcp_bind(p_pcb, IP_ADDR_ANY, WD_PORT)))
    {
        tm_printf((UB *)"webdebug: cannot bind port %d\n", WD_PORT);
        if (NULL != p_pcb)
        {
            (void)tcp_close(p_pcb);
        }
        return;
    }
    g_listen_pcb = tcp_listen(p_pcb); /* frees p_pcb, returns the listener */
    if (NULL == g_listen_pcb)
    {
        tm_printf((UB *)"webdebug: listen failed (out of memory)\n");
        return;
    }
    tcp_accept(g_listen_pcb, on_accept);
    tm_printf((UB *)"webdebug: open http://<robot ip>/ in a browser\n");
}

#endif /* S2_WEBDEBUG && TM_NET_LWIP */

/*** end of file ***/
