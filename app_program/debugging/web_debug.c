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
#define WD_SNAP_MAX  1600 /* JSON for the whole dashboard */

static const char k_hdr_html[] =
    "HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n";
static const char k_hdr_text[] =
    "HTTP/1.0 200 OK\r\nContent-Type: text/plain\r\nCache-Control: no-store\r\n"
    "Connection: close\r\n\r\n";
static const char k_404[] =
    "HTTP/1.0 404 Not Found\r\nConnection: close\r\n\r\nnot found";

/* Kept in flash and sent without copying (tcp_write with flags = 0) */
static const char k_page[] =
    "<!doctype html><html><head><meta charset=utf-8>\n"
    "<meta name=viewport content='width=device-width,initial-scale=1'>\n"
    "<title>Robot dashboard</title><style>\n"
    "body{font-family:sans-serif;margin:10px;touch-action:manipulation;-webkit-user-select:none;user-select:none}\n"
    "h2{text-align:center;margin:6px}\n"
    ".pad{text-align:center}\n"
    "button{font-size:1.3em;width:30%;height:3.6em;margin:3px;border-radius:10px}\n"
    "button.s{height:2.4em;width:auto;padding:0 .7em;font-size:1em}\n"
    "#s{background:#c00;color:#fff;font-weight:bold}\n"
    "#st{font:12px monospace;margin:8px 0;word-break:break-word;text-align:center}\n"
    "#g{display:grid;grid-template-columns:repeat(auto-fit,minmax(230px,1fr));gap:8px}\n"
    ".c{border:1px solid #888;border-radius:8px;padding:6px 10px}\n"
    ".c b{display:block;margin-bottom:4px}\n"
    ".c div{display:flex;justify-content:space-between;font:13px monospace;padding:1px 0}\n"
    ".c i{font-style:normal;font-weight:bold}\n"
    ".n{color:#888}\n"
    ".bar{position:relative;width:60%;height:12px;background:#ddd;border-radius:6px}\n"
    ".bar u{position:absolute;top:-2px;width:6px;height:16px;background:#06c;border-radius:3px}\n"
    "@media(prefers-color-scheme:dark){body{background:#111;color:#eee}button{background:#333;color:#eee}.c{border-color:#555}.bar{background:#444}}\n"
    "</style></head><body><h2>Robot dashboard</h2>\n"
    "<div class=pad>\n"
    "<div><button data-c=f>FORWARD</button></div>\n"
    "<div><button data-c=l>SPIN LEFT</button><button id=s data-c=s>STOP</button><button data-c=r>SPIN RIGHT</button></div>\n"
    "<div><button data-c=b>BACK</button></div>\n"
    "<div><button data-k=L>Turn 90 L</button><button data-k=x>Reset</button><button data-k=R>Turn 90 R</button></div>\n"
    "<div>Trim L <button class=s data-k=t>-</button><button class=s data-k=T>+</button>\n"
    " Trim R <button class=s data-k=u>-</button><button class=s data-k=U>+</button>\n"
    " <button class=s data-k=h>Straight on/off</button>\n"
    " Speed <button class=s data-k=v>-</button><button class=s data-k=V>+</button></div>\n"
    "</div>\n"
    "<div id=st>ready</div>\n"
    "<div id=g></div>\n"
    "<p class=n style='text-align:center'>Keys: arrows = move, space = stop. Dashboard refreshes about 2 times a second.</p>\n"
    "<script>\n"
    "var t=null,st=document.getElementById('st');\n"
    "function send(c){fetch('/c?d='+c).then(function(r){return r.text()}).then(function(x){st.textContent=x}).catch(function(){st.textContent='NO LINK - robot will stop'})}\n"
    "function hold(c){send(c);clearInterval(t);t=setInterval(function(){send(c)},200)}\n"
    "function rel(){clearInterval(t);t=null;send('s')}\n"
    "document.querySelectorAll('button[data-c]').forEach(function(b){var c=b.dataset.c;\n"
    "if(c=='s'){b.onclick=rel;return}\n"
    "b.onpointerdown=function(){hold(c)};\n"
    "['onpointerup','onpointerleave','onpointercancel'].forEach(function(e){b[e]=rel})});\n"
    "document.querySelectorAll('button[data-k]').forEach(function(b){b.onclick=function(){send(b.dataset.k)}});\n"
    "var km={ArrowUp:'f',ArrowDown:'b',ArrowLeft:'l',ArrowRight:'r'},kd=null;\n"
    "onkeydown=function(e){if(km[e.key]&&kd!=e.key){kd=e.key;hold(km[e.key])}if(e.key==' '){e.preventDefault();rel()}};\n"
    "onkeyup=function(e){if(km[e.key]){kd=null;rel()}};\n"
    "onblur=rel;\n"
    "var N={m:['IDLE','DRIVING','DRIVE_DIST','TURNING','STOPPING','SAFE_STOP','FAULT'],\n"
    "l:['INIT','CALIBRATE','SEARCH','FOLLOW','LOST','JUNCTION'],\n"
    "i:['INIT','CALIBRATING','MONITORING','CLIMBING','AT_PEAK','DESCENDING','ERROR'],\n"
    "mo:['UNKNOWN','STATIONARY','CRUISING','ACCELERATING','BRAKING','TURN_LEFT','TURN_RIGHT'],\n"
    "o:['IDLE','MONITORING','DETECTED','COARSE_SCAN','FINE_SCAN','PLANNING','BYPASSING','SEARCH_LINE','LINE_RECOVERY','ERROR'],\n"
    "v:['IDLE','MISSION_START','FOLLOW_LINE','LOST_LINE','AVOID_OBSTACLE','DECODE_BARCODE','EXECUTE_ACTION','TERRAIN_CHECK','FAULT','MISSION_COMPLETE'],\n"
    "b:['IDLE','DETECT','READ','DECODE','RETRY','COMPLETE']};\n"
    "function nm(a,i){return a[i]!==undefined?a[i]:i}\n"
    "function card(t,rows){var h='<div class=c><b>'+t+'</b>';\n"
    "if(!rows)return h+'<div class=n>not running</div></div>';\n"
    "rows.forEach(function(r){h+='<div><span>'+r[0]+'</span><i>'+r[1]+'</i></div>'});return h+'</div>'}\n"
    "function render(d){var a=d.s2,l=d.s3,i=d.s4,o=d.s5,v=d.vc,c=d.cfg,h='';\n"
    "h+=card('Motion',a&&[['state',nm(N.m,a.state)+(a.fault?' fault '+a.fault:'')],['speed L / R mm/s',a.vL+' / '+a.vR],['target L / R',a.tgtL+' / '+a.tgtR],['duty L / R',a.dutyL+' / '+a.dutyR],['ticks L / R',a.ticksL+' / '+a.ticksR],['distance mm',a.dist],['heading deg',(a.head/10).toFixed(1)],['loop overruns',a.over]]);\n"
    "h+=card('Line sensors',l&&[['state',nm(N.l,l.state)],['position',l.pos+'<br><div class=bar><u style=left:'+((l.pos+1000)/20)+'%></u></div>'],['error / steer',l.err+' / '+l.steer],['raw L C R',l.r0+' '+l.r1+' '+l.r2],['black mask',l.mask],['calibrated',l.cal?'yes':'no'],['barcode',nm(N.b,l.bc)+' ok '+l.bcOk+' fail '+l.bcFail]]);\n"
    "h+=card('IMU',i&&[['state',nm(N.i,i.state)+(i.fault?' FAULT':'')],['motion',nm(N.mo,i.motion)],['pitch / roll deg',(i.pitch/100).toFixed(1)+' / '+(i.roll/100).toFixed(1)],['heading deg',(i.head/100).toFixed(1)],['turn rate deg/s',(i.turn/100).toFixed(1)],['humps / impacts',i.humps+' / '+i.impacts],['max hump mm',(i.maxPeak/1000).toFixed(1)],['calibrated',i.cal?'yes':'no']]);\n"
    "h+=card('Obstacle (ultrasonic)',o&&[['state',nm(N.o,o.state)],['distance mm',o.dist?o.dist:'no echo'],['pings / timeouts',o.pings+' / '+o.timeouts],['obstacles',o.obst],['last fault',o.fault]]);\n"
    "h+=card('Vehicle controller',v&&[['state',nm(N.v,v.state)],['previous',nm(N.v,v.prev)],['in state (s)',(v.sinceMs/1000).toFixed(1)],['barcodes read',v.barcodes],['events handled',v.events],['wifi',v.wifi?'up':'down'],['last fault',v.fault]]);\n"
    "h+=card('Debug settings',[['test speed mm/s',c.speed],['trim L / R',c.trimL+' / '+c.trimR],['straight hold',c.straight?'ON':'OFF']]);\n"
    "document.getElementById('g').innerHTML=h}\n"
    "var busy=0;\n"
    "setInterval(function(){if(busy)return;busy=1;\n"
    "fetch('/s').then(function(r){return r.json()}).then(render).catch(function(){}).then(function(){busy=0})},500);\n"
    "</script></body></html>\n";

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
        (void)web_debug_motion(req[9], status, (int32_t)sizeof(status));
        reply_and_close(p_pcb, k_hdr_text, status, (uint16_t)strlen(status), 1);
    }
    else if (0 == strncmp(req, "GET /s", 6))
    {
        (void)web_debug_snapshot(g_snap, (int32_t)sizeof(g_snap));
        reply_and_close(p_pcb, k_hdr_text, g_snap, (uint16_t)strlen(g_snap), 1);
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
