from flask import Flask, request, jsonify, render_template
from datetime import datetime

app = Flask(__name__)
latest_data = {}

@app.route("/")
def index():
    return render_template("index_new.html")

@app.after_request
def add_cors_headers(response):
    response.headers["Access-Control-Allow-Origin"] = "*"
    response.headers["Access-Control-Allow-Headers"] = "Content-Type"
    response.headers["Access-Control-Allow-Methods"] = "GET, POST, OPTIONS"
    return response

@app.route("/update", methods=["POST", "OPTIONS"])
def update():
    global latest_data

    if request.method == "OPTIONS":
        return ("", 204)

    data = request.get_json(silent=True)
    if not data:
        return jsonify({"ok": False, "error": "No JSON received"}), 400

    latest_data = {
        "lat": data.get("lat"),
        "lon": data.get("lon"),
        "speed_mps": data.get("speed_mps"),
        "speed_kmh": data.get("speed_kmh"),
        "course_deg": data.get("course_deg"),
        "accuracy_m": data.get("accuracy_m"),
        "phone_timestamp": data.get("timestamp"),
        "server_received_at": datetime.now().isoformat()
    }

    print("Received:", latest_data, flush=True)
    return jsonify({"ok": True, "received": latest_data})

@app.route("/latest", methods=["GET"])
def latest():
    return jsonify(latest_data)

@app.route("/", methods=["GET"])
def root():
    return jsonify({
        "message": "Bike telemetry server is running",
        "post_to": "/update",
        "get_latest_from": "/latest"
    })

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)