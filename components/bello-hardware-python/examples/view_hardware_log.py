#!/usr/bin/env python3
# ruff: noqa: E501
"""Build a self-contained browser dashboard from a Bello hardware JSONL log."""

from __future__ import annotations

import argparse
import json
import math
import sys
import webbrowser
from html import escape
from pathlib import Path
from typing import Any

MOTOR_METRICS = (
    "position",
    "velocity",
    "torque",
    "temperature",
    "feedback_age_ms",
    "fault_bits",
    "mode_status",
)
IMU_METRICS = (
    "orientation_x",
    "orientation_y",
    "orientation_z",
    "orientation_w",
    "angular_velocity_x",
    "angular_velocity_y",
    "angular_velocity_z",
    "linear_acceleration_x",
    "linear_acceleration_y",
    "linear_acceleration_z",
    "sample_age_ms",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path, help="hardware JSONL file")
    parser.add_argument("--output", type=Path, help="HTML destination (default: LOG.html)")
    parser.add_argument(
        "--max-points",
        type=int,
        default=10_000,
        help="maximum displayed points per trace; extrema are preserved (default: %(default)s)",
    )
    parser.add_argument("--no-open", action="store_true", help="do not open the browser")
    parser.add_argument("--force", action="store_true", help="replace an existing HTML file")
    args = parser.parse_args()
    if args.max_points < 100:
        parser.error("--max-points must be at least 100")
    if not args.log.is_file():
        parser.error(f"log does not exist: {args.log}")
    return args


def downsample_minmax(points: list[list[float]], maximum: int) -> list[list[float]]:
    if len(points) <= maximum:
        return points
    bucket_size = max(1, math.ceil(len(points) / max(1, maximum // 2)))
    sampled = [points[0]]
    for start in range(1, len(points) - 1, bucket_size):
        bucket = points[start : min(start + bucket_size, len(points) - 1)]
        low = min(bucket, key=lambda point: point[1])
        high = max(bucket, key=lambda point: point[1])
        sampled.extend(sorted((low, high), key=lambda point: point[0]))
    sampled.append(points[-1])
    if len(sampled) <= maximum:
        return sampled
    return [sampled[0], *sampled[1 : maximum - 1], sampled[-1]]


def nested(value: dict[str, Any], field: str) -> float:
    section, axis = field.rsplit("_", 1)
    if section == "orientation":
        section = "orientation_xyzw"
    return float(value[section][axis])


def load_log(path: Path, max_points: int) -> dict[str, Any]:
    inventory: dict[str, Any] | None = None
    start_ns: int | None = None
    end_time = 0.0
    sample_count = 0
    motors: dict[str, dict[str, list[list[float]]]] = {}
    imus: dict[str, dict[str, list[list[float]]]] = {}
    groups: dict[str, dict[str, Any]] = {}
    events: list[dict[str, Any]] = []
    previous_group: dict[str, tuple[Any, ...]] = {}

    with path.open(encoding="utf-8") as lines:
        for line_number, line in enumerate(lines, 1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError as error:
                raise ValueError(f"invalid JSON on line {line_number}: {error}") from error
            record = row.get("record")
            if record == "inventory":
                inventory = row
                continue
            if record != "state_frame":
                continue

            logged_ns = int(row["logged_unix_ns"])
            if start_ns is None:
                start_ns = logged_ns
            timestamp = (logged_ns - start_ns) / 1_000_000_000.0
            end_time = max(end_time, timestamp)
            sample_count += 1
            for motor in row.get("motors", []):
                name = str(motor["motor"])
                empty_motor: dict[str, list[list[float]]] = {metric: [] for metric in MOTOR_METRICS}
                empty_motor.update({"fault_bits": [], "mode_status": []})
                traces = motors.setdefault(
                    name,
                    empty_motor,
                )
                for metric in traces:
                    traces[metric].append([timestamp, float(motor[metric])])

            for imu in row.get("imus", []):
                name = str(imu["imu"])
                empty_imu: dict[str, list[list[float]]] = {metric: [] for metric in IMU_METRICS}
                traces = imus.setdefault(name, empty_imu)
                for metric in IMU_METRICS:
                    if metric.startswith(
                        ("orientation_", "angular_velocity_", "linear_acceleration_")
                    ):
                        value = nested(imu, metric)
                    else:
                        value = float(imu[metric])
                    traces[metric].append([timestamp, value])

            for group in row.get("groups", []):
                name = str(group["group"])
                groups[name] = {
                    "state": group["state"],
                    "cause": group["cause"],
                    "detail": group["detail"],
                    "command_count": group["command_count"],
                    "control_tick": group["control_tick"],
                    "jitter_us": group["jitter_us"],
                }
                signature = (
                    group["state"],
                    group["cause"],
                    group["detail"],
                )
                if previous_group.get(name) != signature:
                    events.append(
                        {
                            "time": timestamp,
                            "kind": "group",
                            "component": name,
                            "state": group["state"],
                            "detail": group["detail"] or group["cause"],
                        }
                    )
                    previous_group[name] = signature

    if inventory is None:
        raise ValueError("log has no inventory record")
    if sample_count == 0:
        raise ValueError("log has no state-frame records")

    for device in motors.values():
        for metric, points in device.items():
            device[metric] = downsample_minmax(points, max_points)
    for device in imus.values():
        for metric, points in device.items():
            device[metric] = downsample_minmax(points, max_points)

    initial_faults = sum(
        1
        for collection in ("buses", "serial_devices", "imus", "motors")
        for item in inventory.get(collection, [])
        if (item.get("status") or {}).get("state") in {"fault", "degraded", "reconnecting"}
    )
    return {
        "source": path.name,
        "duration": end_time,
        "samples": sample_count,
        "initial_faults": initial_faults,
        "inventory": inventory,
        "motors": dict(sorted(motors.items())),
        "imus": dict(sorted(imus.items())),
        "groups": dict(sorted(groups.items())),
        "events": events[-1000:],
        "event_count": len(events),
    }


HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>__TITLE__</title>
<style>
:root { color-scheme: dark; --bg:#0b0f14; --panel:#121923; --line:#263244; --text:#e8edf5; --muted:#8fa0b5; --accent:#58a6ff; --danger:#ff6b6b; }
* { box-sizing:border-box; }
body { margin:0; background:var(--bg); color:var(--text); font:14px/1.45 system-ui,sans-serif; }
header { padding:22px 26px 10px; }
h1 { margin:0 0 4px; font-size:22px; } h2 { margin:0 0 14px; font-size:16px; }
.muted { color:var(--muted); }
.cards { display:grid; grid-template-columns:repeat(auto-fit,minmax(145px,1fr)); gap:10px; padding:10px 26px 18px; }
.card,.panel { background:var(--panel); border:1px solid var(--line); border-radius:10px; }
.card { padding:12px 14px; }.card strong { display:block; font-size:22px; }.card span { color:var(--muted); }
main { display:grid; gap:14px; padding:0 26px 30px; }
.panel { padding:16px; overflow:hidden; }
.controls { display:flex; flex-wrap:wrap; gap:10px; align-items:center; margin-bottom:10px; }
select,input,button { color:var(--text); background:#0d141d; border:1px solid var(--line); border-radius:6px; padding:7px 9px; }
button { cursor:pointer; } button:hover { border-color:var(--accent); }
.motor-list { display:flex; flex-wrap:wrap; gap:5px 12px; max-height:110px; overflow:auto; padding:8px 0; }
.motor-list label { white-space:nowrap; color:var(--muted); }
.motor-list input:checked + span { color:var(--text); }
canvas { width:100%; height:360px; display:block; background:#0d141d; border-radius:7px; }
.legend { display:flex; flex-wrap:wrap; gap:8px 14px; margin-top:9px; min-height:20px; color:var(--muted); }
.swatch { display:inline-block; width:9px; height:9px; border-radius:50%; margin-right:5px; }
table { width:100%; border-collapse:collapse; } th,td { padding:8px 9px; text-align:left; border-bottom:1px solid var(--line); } th { color:var(--muted); font-weight:500; }
.table-wrap { overflow:auto; max-height:330px; }.fault { color:var(--danger); }
@media(max-width:700px) { header,.cards,main { padding-left:12px; padding-right:12px; } canvas { height:280px; } }
</style>
</head>
<body>
<header><h1>Bello hardware log</h1><div class="muted" id="source"></div></header>
<section class="cards" id="cards"></section>
<main>
  <section class="panel">
    <h2>Motors</h2>
    <div class="controls"><label>Metric <select id="motorMetric"></select></label><button id="allMotors">All</button><button id="noMotors">None</button></div>
    <div class="motor-list" id="motorList"></div>
    <canvas id="motorPlot"></canvas><div class="legend" id="motorLegend"></div>
  </section>
  <section class="panel" id="imuPanel">
    <h2>IMU</h2>
    <div class="controls"><label>Device <select id="imuName"></select></label><label>Metric <select id="imuMetric"></select></label></div>
    <canvas id="imuPlot"></canvas><div class="legend" id="imuLegend"></div>
  </section>
  <section class="panel"><h2>Hardware inventory at connection</h2><div class="table-wrap"><table><thead><tr><th>Kind</th><th>Name</th><th>Location</th><th>State</th><th>Detail</th></tr></thead><tbody id="inventory"></tbody></table></div></section>
  <section class="panel"><h2>Latest group state</h2><div class="table-wrap"><table><thead><tr><th>Group</th><th>State</th><th>Fault</th><th>Commands</th><th>Tick</th><th>Jitter µs</th></tr></thead><tbody id="groups"></tbody></table></div></section>
  <section class="panel"><h2>Group state changes</h2><div class="muted" id="eventNote"></div><div class="table-wrap"><table><thead><tr><th>Time</th><th>Kind</th><th>Component</th><th>State</th><th>Detail</th></tr></thead><tbody id="events"></tbody></table></div></section>
</main>
<script>
const DATA=__DATA__;
const COLORS=['#58a6ff','#f2cc60','#56d364','#ff7b72','#bc8cff','#39c5cf','#ffa657','#ec6cb9','#8b949e','#7ee787','#d2a8ff','#79c0ff'];
const esc=s=>String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
document.getElementById('source').textContent=DATA.source;
const cards=[['Duration',DATA.duration.toFixed(2)+' s'],['Snapshots',DATA.samples.toLocaleString()],['Motors',Object.keys(DATA.motors).length],['IMUs',Object.keys(DATA.imus).length],['Initial warnings',DATA.initial_faults]];
document.getElementById('cards').innerHTML=cards.map(([k,v])=>`<div class="card"><strong>${esc(v)}</strong><span>${esc(k)}</span></div>`).join('');

function fillSelect(element, values){ element.innerHTML=values.map(v=>`<option value="${esc(v)}">${esc(v.replaceAll('_',' '))}</option>`).join(''); }
function draw(canvas, traces, yLabel, legend){
  const ratio=window.devicePixelRatio||1, rect=canvas.getBoundingClientRect(); canvas.width=rect.width*ratio; canvas.height=rect.height*ratio;
  const ctx=canvas.getContext('2d'); ctx.scale(ratio,ratio); const W=rect.width,H=rect.height,L=62,R=18,T=18,B=42;
  ctx.clearRect(0,0,W,H); legend.innerHTML='';
  let xmin=Infinity,xmax=-Infinity,ymin=Infinity,ymax=-Infinity,count=0;
  traces.forEach(trace=>trace.points.forEach(point=>{xmin=Math.min(xmin,point[0]);xmax=Math.max(xmax,point[0]);ymin=Math.min(ymin,point[1]);ymax=Math.max(ymax,point[1]);count++;}));
  if(!count){ctx.fillStyle='#8fa0b5';ctx.fillText('No data selected',L,T+20);return;}
  if(xmax===xmin)xmax=xmin+1;if(ymax===ymin){const d=Math.abs(ymin)*.05||1;ymin-=d;ymax+=d;} const pad=(ymax-ymin)*.06;ymin-=pad;ymax+=pad;
  const x=v=>L+(v-xmin)/(xmax-xmin)*(W-L-R), y=v=>T+(ymax-v)/(ymax-ymin)*(H-T-B);
  ctx.font='11px system-ui';ctx.strokeStyle='#263244';ctx.fillStyle='#8fa0b5';ctx.lineWidth=1;
  for(let i=0;i<=5;i++){const yy=T+i*(H-T-B)/5,v=ymax-i*(ymax-ymin)/5;ctx.beginPath();ctx.moveTo(L,yy);ctx.lineTo(W-R,yy);ctx.stroke();ctx.fillText(v.toPrecision(4),4,yy+4);}
  for(let i=0;i<=5;i++){const xx=L+i*(W-L-R)/5,v=xmin+i*(xmax-xmin)/5;ctx.beginPath();ctx.moveTo(xx,T);ctx.lineTo(xx,H-B);ctx.stroke();ctx.fillText(v.toFixed(1)+'s',xx-10,H-17);}
  traces.forEach((trace,index)=>{const color=COLORS[index%COLORS.length];ctx.strokeStyle=color;ctx.lineWidth=1.4;ctx.beginPath();trace.points.forEach((p,i)=>{const xx=x(p[0]),yy=y(p[1]);i?ctx.lineTo(xx,yy):ctx.moveTo(xx,yy);});ctx.stroke();legend.insertAdjacentHTML('beforeend',`<span><i class="swatch" style="background:${color}"></i>${esc(trace.name)}</span>`);});
  ctx.save();ctx.translate(14,H/2);ctx.rotate(-Math.PI/2);ctx.fillStyle='#8fa0b5';ctx.fillText(yLabel,0,0);ctx.restore();
}

const motorMetric=document.getElementById('motorMetric'),motorList=document.getElementById('motorList');fillSelect(motorMetric,__MOTOR_METRICS__);
Object.keys(DATA.motors).forEach((name,index)=>motorList.insertAdjacentHTML('beforeend',`<label><input type="checkbox" value="${esc(name)}" ${index<6?'checked':''}><span>${esc(name)}</span></label>`));
function drawMotors(){const metric=motorMetric.value,names=[...motorList.querySelectorAll('input:checked')].map(x=>x.value),traces=names.map(name=>({name,points:DATA.motors[name][metric]}));draw(document.getElementById('motorPlot'),traces,metric,document.getElementById('motorLegend'));}
motorMetric.onchange=drawMotors;motorList.onchange=drawMotors;document.getElementById('allMotors').onclick=()=>{motorList.querySelectorAll('input').forEach(x=>x.checked=true);drawMotors();};document.getElementById('noMotors').onclick=()=>{motorList.querySelectorAll('input').forEach(x=>x.checked=false);drawMotors();};

const imuNames=Object.keys(DATA.imus),imuName=document.getElementById('imuName'),imuMetric=document.getElementById('imuMetric');fillSelect(imuName,imuNames);fillSelect(imuMetric,__IMU_METRICS__);
function drawImu(){if(!imuNames.length)return;const name=imuName.value,metric=imuMetric.value;draw(document.getElementById('imuPlot'),[{name,points:DATA.imus[name][metric]}],metric,document.getElementById('imuLegend'));}
if(!imuNames.length)document.getElementById('imuPanel').style.display='none';imuName.onchange=drawImu;imuMetric.onchange=drawImu;

const inventory=[];
for(const item of DATA.inventory.buses||[])inventory.push(['bus',item.name,item.interface,item.status]);
for(const item of DATA.inventory.serial_devices||[])inventory.push(['serial',item.name,`${item.port} @ ${item.baud_rate}`,item.status]);
for(const item of DATA.inventory.imus||[])inventory.push(['imu',item.name,item.serial_device,item.status]);
for(const item of DATA.inventory.motors||[])inventory.push(['motor',item.name,`${item.group} / ${item.bus}:${item.id}`,item.status]);
document.getElementById('inventory').innerHTML=inventory.map(([kind,name,location,status])=>`<tr><td>${esc(kind)}</td><td>${esc(name)}</td><td>${esc(location)}</td><td class="${status&&['fault','degraded','reconnecting'].includes(status.state)?'fault':''}">${esc(status?status.state:'unknown')}</td><td>${esc(status?status.detail:'')}</td></tr>`).join('');
document.getElementById('groups').innerHTML=Object.entries(DATA.groups).map(([name,g])=>`<tr><td>${esc(name)}</td><td class="${g.state==='fault'?'fault':''}">${esc(g.state)}</td><td>${esc(g.detail||g.cause)}</td><td>${g.command_count}</td><td>${g.control_tick}</td><td>${g.jitter_us}</td></tr>`).join('');
document.getElementById('eventNote').textContent=`Showing ${DATA.events.length.toLocaleString()} of ${DATA.event_count.toLocaleString()} events (most recent first).`;
document.getElementById('events').innerHTML=[...DATA.events].reverse().map(e=>`<tr><td>${e.time.toFixed(3)} s</td><td>${esc(e.kind)}</td><td>${esc(e.component)}</td><td class="${['fault','degraded','reconnecting'].includes(e.state)?'fault':''}">${esc(e.state)}</td><td>${esc(e.detail)}</td></tr>`).join('');
drawMotors();drawImu();let timer;window.onresize=()=>{clearTimeout(timer);timer=setTimeout(()=>{drawMotors();drawImu();},100);};
</script>
</body></html>
"""


def render(data: dict[str, Any]) -> str:
    encoded = json.dumps(data, separators=(",", ":"), allow_nan=False).replace("</", "<\\/")
    return (
        HTML.replace("__TITLE__", escape(f"Bello log — {data['source']}"))
        .replace("__DATA__", encoded)
        .replace("__MOTOR_METRICS__", json.dumps(MOTOR_METRICS))
        .replace("__IMU_METRICS__", json.dumps(IMU_METRICS))
    )


def main() -> int:
    args = parse_args()
    output = args.output or args.log.with_suffix(args.log.suffix + ".html")
    if output.exists() and not args.force:
        print(f"error: output already exists: {output} (use --force)", file=sys.stderr)
        return 2
    try:
        data = load_log(args.log, args.max_points)
        output.write_text(render(data), encoding="utf-8")
    except (OSError, ValueError, KeyError, TypeError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    print(f"wrote {output}")
    if not args.no_open:
        webbrowser.open(output.resolve().as_uri())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
