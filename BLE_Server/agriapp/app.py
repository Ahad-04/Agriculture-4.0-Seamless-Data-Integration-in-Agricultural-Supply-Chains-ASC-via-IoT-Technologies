# app.py
from flask import Flask, request, jsonify, send_file, render_template
from pathlib import Path
from datetime import datetime, timezone
import threading, csv

APP_DIR = Path(__file__).parent.resolve()
DATA_DIR = APP_DIR / "data"
DATA_DIR.mkdir(parents=True, exist_ok=True)
CSV_PATH = DATA_DIR / "datalog.csv"

app = Flask(__name__, template_folder="templates", static_folder=None)

# in-memory latest
_latest = None
_lock = threading.Lock()

CSV_HEADER = ["timestamp","datetime_utc","latitude","longitude","mq2","mq4","temperature","pressure","humidity","status"]

def ensure_csv_header():
    if not CSV_PATH.exists() or CSV_PATH.stat().st_size == 0:
        with CSV_PATH.open("w", newline="") as f:
            csv.writer(f).writerow(CSV_HEADER)

def append_csv(rec: dict):
    ts = int(rec.get("timestamp", 0))
    dt = datetime.fromtimestamp(ts, tz=timezone.utc).strftime("%Y-%m-%d %H:%M:%S")
    row = [
        ts, dt,
        float(rec.get("latitude", 0) or 0),
        float(rec.get("longitude", 0) or 0),
        int(rec.get("mq2", 0) or 0),
        int(rec.get("mq4", 0) or 0),
        float(rec.get("temperature", 0) or 0),
        float(rec.get("pressure", 0) or 0),
        int(rec.get("humidity", 0) or 0),
        int(rec.get("status", 0) or 0),
    ]
    with CSV_PATH.open("a", newline="") as f:
        w = csv.writer(f)
        w.writerow(row)
        f.flush()

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/pong", methods=["POST"])
def pong():
    try:
        rec = request.get_json(force=True)
        # minimal validation
        if not isinstance(rec, dict) or "timestamp" not in rec:
            return jsonify({"ok": False, "error": "bad json"}), 400

        ensure_csv_header()
        append_csv(rec)

        global _latest
        with _lock:
            _latest = {
                "timestamp": int(rec.get("timestamp", 0)),
                "latitude": float(rec.get("latitude", 0) or 0),
                "longitude": float(rec.get("longitude", 0) or 0),
                "mq2": int(rec.get("mq2", 0) or 0),
                "mq4": int(rec.get("mq4", 0) or 0),
                "temperature": float(rec.get("temperature", 0) or 0),
                "pressure": float(rec.get("pressure", 0) or 0),
                "humidity": int(rec.get("humidity", 0) or 0),
                "status": int(rec.get("status", 0) or 0),
            }
        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500

@app.route("/api/latest")
def api_latest():
    with _lock:
        return jsonify({"ok": True, "data": _latest})

@app.route("/api/csv")
def api_csv():
    ensure_csv_header()
    return send_file(CSV_PATH, as_attachment=True, download_name="datalog.csv", mimetype="text/csv")

if __name__ == "__main__":
    ensure_csv_header()
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
