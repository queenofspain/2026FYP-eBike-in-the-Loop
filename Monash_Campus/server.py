from flask import Flask, request, jsonify, render_template
from datetime import datetime

app = Flask(__name__)
latest_data = {}
latest_feedback = {}

@app.route("/")
def index():
    return render_template("index_new.html")

@app.route("/feedback")
def feedback():
    return render_template("feedback.html")

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

@app.route("/feedback/update", methods=["POST", "OPTIONS"])
@app.route("/sumo_feedback", methods=["POST", "OPTIONS"])
def update_feedback():
    global latest_feedback

    if request.method == "OPTIONS":
        return ("", 204)

    data = request.get_json(silent=True)
    if not data:
        return jsonify({"ok": False, "error": "No JSON received"}), 400

    latest_feedback = dict(data)
    latest_feedback["server_received_at"] = datetime.now().isoformat()

    print("SUMO feedback:", latest_feedback, flush=True)
    return jsonify({"ok": True, "received": latest_feedback})

@app.route("/feedback/latest", methods=["GET"])
@app.route("/sumo_feedback/latest", methods=["GET"])
def latest_sumo_feedback():
    return jsonify({
        "phone": latest_data,
        "feedback": latest_feedback
    })

@app.route("/health", methods=["GET"])
def health():
    return jsonify({
        "message": "Bike telemetry server is running",
        "post_to": "/update",
        "get_latest_from": "/latest",
        "feedback_page": "/feedback",
        "feedback_update": "/feedback/update",
        "feedback_latest": "/feedback/latest"
    })

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True, use_reloader=False)
