# server.py
import os, time, sqlite3, math
from datetime import datetime, timedelta
from flask import Flask, request, jsonify, abort, Response, render_template_string, send_file
from flask_cors import CORS

DB_PATH = os.environ.get("AGRIDB", "agri.sqlite3")

app = Flask(__name__)
CORS(app)

SCHEMA = """
CREATE TABLE IF NOT EXISTS records (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  ts INTEGER NOT NULL,
  latitude REAL,
  longitude REAL,
  mq2 INTEGER,
  mq4 INTEGER,
  temperature REAL,
  pressure REAL,
  humidity INTEGER,
  status INTEGER,
  device_id TEXT,
  username TEXT,
  created_at INTEGER NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_records_ts ON records(ts);
"""

def get_db():
  conn = sqlite3.connect(DB_PATH)
  conn.row_factory = sqlite3.Row
  return conn

# init DB
with get_db() as con:
  for stmt in filter(None, (s.strip() for s in SCHEMA.split(";"))):
    con.execute(stmt)

def must(v, name):
  if v is None:
    abort(400, f"missing {name}")
  return v

def is_num(x):
  return isinstance(x, (int, float)) and not (isinstance(x, float) and (math.isnan(x) or math.isinf(x)))

def as_int(x, name):
  if not isinstance(x, int):
    abort(400, f"{name} must be int")
  return x

def as_num(x, name):
  if not is_num(x):
    abort(400, f"{name} must be number")
  return x

@app.route("/")
def home():
  return jsonify({"status":"ok", "records":"POST /pong", "view":"/latest", "range":"/range?from=..&to=..", "dashboard":"/dashboard"})

@app.route("/pong", methods=["POST"])
def pong():
  """
  ESP32 posts one record:
  {
    "timestamp": 1726550000,
    "latitude": 31.52,
    "longitude": 74.35,
    "mq2": 123,
    "mq4": 456,
    "temperature": 28.3,
    "pressure": 1007.4,
    "humidity": 54,
    "status": 1,
    // optional:
    "device_id": "ESP32-ABCD",
    "username": "agri-user"
  }
  """
  try:
    j = request.get_json(force=True, silent=False)
  except Exception:
    abort(400, "invalid JSON")

  ts          = as_num(must(j.get("timestamp"),   "timestamp"),   "timestamp")
  latitude    = as_num(must(j.get("latitude"),    "latitude"),    "latitude")
  longitude   = as_num(must(j.get("longitude"),   "longitude"),   "longitude")
  mq2         = as_int(must(j.get("mq2"),         "mq2"),         "mq2")
  mq4         = as_int(must(j.get("mq4"),         "mq4"),         "mq4")
  temperature = as_num(must(j.get("temperature"), "temperature"), "temperature")
  pressure    = as_num(must(j.get("pressure"),    "pressure"),    "pressure")
  humidity    = as_int(must(j.get("humidity"),    "humidity"),    "humidity")
  status      = as_int(must(j.get("status"),      "status"),      "status")

  device_id   = j.get("device_id")  # optional
  username    = j.get("username")   # optional

  with get_db() as con:
    con.execute(
      "INSERT INTO records (ts, latitude, longitude, mq2, mq4, temperature, pressure, humidity, status, device_id, username, created_at) "
      "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
      (int(ts), float(latitude), float(longitude), int(mq2), int(mq4),
       float(temperature), float(pressure), int(humidity), int(status),
       device_id, username, int(time.time()))
    )
  return jsonify({"ok": True})

@app.route("/latest")
def latest():
  n = int(request.args.get("n", 50))
  with get_db() as con:
    rows = con.execute(
      "SELECT ts, latitude, longitude, mq2, mq4, temperature, pressure, humidity, status, device_id, username "
      "FROM records ORDER BY ts DESC LIMIT ?",
      (n,)
    ).fetchall()
  return jsonify([dict(r) for r in rows])

@app.route("/range")
def range_endpoint():
  try:
    t0 = int(request.args["from"])
    t1 = int(request.args["to"])
  except Exception:
    abort(400, "need from,to epoch seconds")
  with get_db() as con:
    rows = con.execute(
      "SELECT ts, latitude, longitude, mq2, mq4, temperature, pressure, humidity, status, device_id, username "
      "FROM records WHERE ts BETWEEN ? AND ? ORDER BY ts ASC",
      (t0, t1)
    ).fetchall()
  return jsonify([dict(r) for r in rows])

@app.route("/export.csv")
def export_csv():
  # export last N=1000 by default
  n = int(request.args.get("n", 1000))
  def gen():
    yield "ts,datetime,latitude,longitude,mq2,mq4,temperature,pressure,humidity,status,device_id,username\n"
    with get_db() as con:
      for r in con.execute(
        "SELECT ts, latitude, longitude, mq2, mq4, temperature, pressure, humidity, status, device_id, username "
        "FROM records ORDER BY ts DESC LIMIT ?", (n,)
      ):
        ts = r[0]
        dt = datetime.utcfromtimestamp(ts).strftime("%Y-%m-%d %H:%M:%S")
        line = f"{ts},{dt},{r[1]},{r[2]},{r[3]},{r[4]},{r[5]},{r[6]},{r[7]},{r[8]},{r[9] or ''},{r[10] or ''}\n"
        yield line
  return Response(gen(), mimetype="text/csv",
                  headers={"Content-Disposition":"attachment; filename=export.csv"})

# -------------------- HTML Dashboard --------------------
DASHBOARD_HTML = """
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>Agri Dashboard</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body{font-family:system-ui,-apple-system,Segoe UI,Roboto,Helvetica,Arial,sans-serif;margin:20px;background:#0b0f14;color:#dbe1e8}
    .row{display:flex;gap:16px;flex-wrap:wrap}
    .card{background:#131a22;border:1px solid #1f2a36;border-radius:14px;padding:16px;flex:1}
    .controls{display:flex;gap:8px;flex-wrap:wrap;margin-bottom:10px}
    button{background:#1b2937;border:1px solid #2a3a4b;color:#dbe1e8;border-radius:10px;padding:8px 12px;cursor:pointer}
    button:hover{background:#223447}
    table{width:100%;border-collapse:collapse}
    th,td{border-bottom:1px solid #233140;padding:8px;font-size:13px}
    th{position:sticky;top:0;background:#0f141b}
    .pill{padding:2px 8px;border-radius:999px;background:#1e2c3b;border:1px solid #2b3d50;font-size:12px}
    .ok{color:#83e37b}.warn{color:#ffd166}
  </style>
  <script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.1/dist/chart.umd.min.js"></script>
</head>
<body>
  <h2>🌾 Agri Sensor Dashboard</h2>
  <div class="controls">
    <button onclick="loadRange(1)">Last 1h</button>
    <button onclick="loadRange(6)">Last 6h</button>
    <button onclick="loadRange(24)">Last 24h</button>
    <button onclick="loadRange(24*7)">Last 7d</button>
    <button onclick="loadLatest()">Latest 100</button>
    <a href="/export.csv?n=1000"><button>Export CSV</button></a>
  </div>

  <div class="row">
    <div class="card" style="min-width:320px">
      <canvas id="lineChart" height="220"></canvas>
    </div>
    <div class="card" style="min-width:320px;max-height:420px;overflow:auto">
      <table id="dataTable">
        <thead>
          <tr>
            <th>Time (UTC)</th>
            <th>T (°C)</th>
            <th>P (hPa)</th>
            <th>H (%)</th>
            <th>MQ2</th>
            <th>MQ4</th>
            <th>Status</th>
            <th>Device</th>
          </tr>
        </thead>
        <tbody></tbody>
      </table>
    </div>
  </div>

<script>
let chart;
function mkChart(labels, series){
  const ctx = document.getElementById('lineChart');
  if(chart) chart.destroy();
  chart = new Chart(ctx, {
    type: 'line',
    data: {
      labels,
      datasets: [
        {label:'Temperature (°C)', data: series.temp, fill:false},
        {label:'Pressure (hPa)',  data: series.press, fill:false},
        {label:'Humidity (%)',    data: series.hum,  fill:false},
        {label:'MQ2',             data: series.mq2,  fill:false},
        {label:'MQ4',             data: series.mq4,  fill:false},
      ]
    },
    options: {
      responsive:true,
      maintainAspectRatio:false,
      interaction:{mode:'index', intersect:false},
      scales: {
        x: {ticks:{maxRotation:0, autoSkip:true, maxTicksLimit:8}},
        y: {beginAtZero:false}
      }
    }
  });
}

function fmtTs(ts){
  const d = new Date(ts*1000);
  return d.toISOString().replace('T',' ').slice(0,19);
}

function fillTable(rows){
  const tb = document.querySelector('#dataTable tbody');
  tb.innerHTML = '';
  for(const r of rows.slice().reverse()){ // newest at bottom
    const tr = document.createElement('tr');
    tr.innerHTML = `
      <td>${fmtTs(r.ts)}</td>
      <td>${(r.temperature??'').toFixed ? r.temperature.toFixed(1) : r.temperature}</td>
      <td>${(r.pressure??'').toFixed ? r.pressure.toFixed(1) : r.pressure}</td>
      <td>${r.humidity??''}</td>
      <td>${r.mq2??''}</td>
      <td>${r.mq4??''}</td>
      <td><span class="pill ${r.status? 'warn':'ok'}">${r.status? 'OPEN':'OK'}</span></td>
      <td>${r.device_id??''}</td>
    `;
    tb.appendChild(tr);
  }
}

function prepSeries(rows){
  const labels=[], temp=[], press=[], hum=[], mq2=[], mq4=[];
  for (const r of rows){
    labels.push(fmtTs(r.ts));
    temp.push(r.temperature);
    press.push(r.pressure);
    hum.push(r.humidity);
    mq2.push(r.mq2);
    mq4.push(r.mq4);
  }
  return {labels, series:{temp, press, hum, mq2, mq4}};
}

async function loadLatest(n=100){
  const res = await fetch(`/latest?n=${n}`);
  const rows = await res.json();
  fillTable(rows);
  const {labels, series} = prepSeries(rows.slice().reverse());
  mkChart(labels, series);
}

async function loadRange(hours){
  const now = Math.floor(Date.now()/1000);
  const from = now - hours*3600;
  const res = await fetch(`/range?from=${from}&to=${now}`);
  const rows = await res.json();
  fillTable(rows);
  const {labels, series} = prepSeries(rows);
  mkChart(labels, series);
}

loadLatest(100);
// auto-refresh every 10s
setInterval(()=>loadLatest(100), 10000);
</script>
</body>
</html>
"""

@app.route("/dashboard")
def dashboard():
  return render_template_string(DASHBOARD_HTML)

if __name__ == "__main__":
  app.run(host="0.0.0.0", port=5000, debug=False)
