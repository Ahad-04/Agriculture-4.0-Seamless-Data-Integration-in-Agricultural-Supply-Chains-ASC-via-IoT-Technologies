from flask import Flask, request, jsonify, send_file, render_template
from pathlib import Path
import threading, time, collections
import requests

# ----------------- CONFIG -----------------
ESP32_HOST = "http://192.168.18.36"   # UPDATE with your ESP32 IP
SERVER_PORT = 5000
# ------------------------------------------

APP_DIR = Path(__file__).parent.resolve()
DATA_DIR = APP_DIR / "data"
DATA_DIR.mkdir(parents=True, exist_ok=True)

# Two separate files:
# CSV_PATH = The full datalog, only updated on "Full Sync"
# TAIL_PATH = A small, temporary file with only the last 15-20 readings for the graph
CSV_PATH = DATA_DIR / "datalog.csv"
TAIL_PATH = DATA_DIR / "tail.csv"

app = Flask(__name__, template_folder="templates", static_folder=None)

# --- Global States ---
_latest = None
_latest_lock = threading.Lock()
csv_state_lock = threading.Lock()
csv_in_progress = False
csv_done = False
# This new variable tells /csv_chunk where to save data
current_sync_target = None 

@app.route("/")
def index(): return render_template("index.html")

# --- LIVE DATA (For the dashboard cards) ---
@app.route("/pong", methods=["POST"])
def pong():
    try:
        rec = request.get_json(force=True)
        global _latest
        with _latest_lock: _latest = rec
        return jsonify({"ok": True})
    except: return jsonify({"ok": False}), 400

@app.route("/api/latest")
def api_latest():
    with _latest_lock: return jsonify({"ok": True, "data": _latest})

# --- CSV SYNC HANDLERS (Used by BOTH sync types) ---
@app.route("/csv_chunk", methods=["POST"])
def csv_chunk():
    global csv_in_progress, current_sync_target
    with csv_state_lock:
         if not csv_in_progress: return "ignored", 200
         # Default to main file if target is somehow lost
         target = current_sync_target if current_sync_target else CSV_PATH
         
    # Append chunk to the correct file (datalog.csv OR tail.csv)
    with target.open("ab") as f: f.write(request.data)
    return "ok"

@app.route("/csv_done", methods=["GET"])
def csv_done_route():
    global csv_done, csv_in_progress
    with csv_state_lock:
        csv_in_progress = False
        csv_done = True
    return "done"

# --- FRONTEND ACTIONS ---

# === 1. FAST SYNC (For Graph) ===
@app.route("/api/start_fast_sync")
def api_start_fast_sync():
    global csv_in_progress, csv_done, current_sync_target
    # Prepare tail file: delete old, create new
    if TAIL_PATH.exists(): TAIL_PATH.unlink()
    TAIL_PATH.touch()
    
    with csv_state_lock:
        csv_in_progress = True
        csv_done = False
        current_sync_target = TAIL_PATH # Set target to tail.csv
        
    try:
        # Call NEW ESP32 endpoint
        requests.get(f"{ESP32_HOST}/pull_fast", timeout=2)
        return jsonify({"ok": True, "mode": "fast"})
    except Exception as e:
        with csv_state_lock: csv_in_progress = False
        return jsonify({"ok": False, "error": str(e)})

# === 2. FULL SYNC (For Download) ===
@app.route("/api/start_sync")
def api_start_sync():
    global csv_in_progress, csv_done, current_sync_target
    # Prepare main file: delete old, create new
    if CSV_PATH.exists(): CSV_PATH.unlink()
    CSV_PATH.touch()
    
    with csv_state_lock:
        csv_in_progress = True
        csv_done = False
        current_sync_target = CSV_PATH # Set target to datalog.csv
        
    try:
        # Call original ESP32 endpoint
        requests.get(f"{ESP32_HOST}/pull_all", timeout=2)
        return jsonify({"ok": True, "mode": "full"})
    except Exception as e:
        with csv_state_lock: csv_in_progress = False
        return jsonify({"ok": False, "error": str(e)})

# === 3. SYNC STATUS (Used by BOTH) ===
@app.route("/api/sync_status")
def api_sync_status():
    with csv_state_lock: ip, d = csv_in_progress, csv_done
    # Report size of the file being written to
    sz = 0
    if current_sync_target and current_sync_target.exists():
        sz = current_sync_target.stat().st_size
    return jsonify({"in_progress": ip, "done": d, "size_bytes": sz})

# === 4. GRAPH DATA (Reads from TAIL file) ===
@app.route("/api/graph_data")
def api_graph_data():
    # Graph ONLY reads from the fast-sync file
    source_file = TAIL_PATH
    
    if not source_file.exists() or source_file.stat().st_size == 0:
         return jsonify({"ok": False, "error": "No graph data. Click 'Update Graph' first."})

    try:
        data_points = []
        with source_file.open("r", encoding="utf-8", errors="ignore") as f:
            # Read all lines (it's a small file)
            for line in f:
                parts = line.strip().split(',')
                # Validation: skip header, skip broken lines (from seek)
                if len(parts) >= 6 and parts[0] != "epoch" and parts[1].count(':') == 2:
                    try:
                        data_points.append({
                            "t": parts[1], # The "YYYY-MM-DD HH:MM:SS" string
                            "mq2": int(parts[4]),
                            "mq4": int(parts[5])
                        })
                    except: continue
        
        # Return only the last 15 valid points found
        return jsonify({"ok": True, "data": data_points[-15:]})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)})

# === 5. DOWNLOAD (Reads from FULL file) ===
@app.route("/api/download_csv")
def api_download_csv():
    # Download ONLY reads from the main datalog file
    if not CSV_PATH.exists(): 
        return "Please perform a 'Full Sync' first.", 404
    return send_file(CSV_PATH, as_attachment=True, download_name=f"datalog_{int(time.time())}.csv", mimetype="text/csv")


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=SERVER_PORT, debug=False, threaded=True)