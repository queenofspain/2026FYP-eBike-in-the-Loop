"""Contract checks for the add-on without changing the existing bridge."""

import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import feedback_server as feedback
import feedback_live
from rider_advice import select_advice


class AdviceTests(unittest.TestCase):
    def test_speed_warning_wins_over_approaching_light(self):
        sample = {"edge_id": "road", "speed_mps": 10, "speed_limit_mps": 6,
                  "intersection_status": "ahead", "intersection_distance_m": 20,
                  "traffic_light_status": "ahead", "traffic_light_distance_m": 20}
        self.assertEqual(select_advice(sample)["code"], "overspeed")

    def test_missing_road_context_never_claims_clear_road(self):
        sample = {"edge_id": "road", "speed_mps": 3, "speed_limit_mps": 8,
                  "intersection_status": "unknown", "traffic_light_status": "unknown"}
        self.assertEqual(select_advice(sample)["code"], "incomplete")


class FeedbackApiTests(unittest.TestCase):
    def setUp(self):
        self.client = feedback.app.test_client()
        with feedback.lock:
            feedback.latest_sample = None
            feedback.previous_advice = None
            feedback.bridge_state = "stopped"
            feedback.bridge_seen = 0
            feedback.demo_process = None
            feedback.demo_run_id = None
            feedback.demo_state = "idle"
            feedback.demo_stop = False

    def test_old_gps_page_and_new_advice_page_are_both_served(self):
        self.assertEqual(self.client.get("/").status_code, 200)
        page = self.client.get("/feedback")
        self.assertEqual(page.status_code, 200)
        self.assertIn(b"Start GPS tracking", page.data)

    def test_live_sample_produces_advice_then_expires(self):
        payload = {"source": "live", "edge_id": "road", "lane_id": "road_0",
                   "speed_mps": 10, "speed_limit_mps": 6, "angle_deg": 90,
                   "intersection_status": "ahead", "intersection_distance_m": 20,
                   "traffic_light_status": "ahead", "traffic_light_distance_m": 20}
        self.assertEqual(self.client.post("/api/feedback/telemetry", json=payload).status_code, 200)
        current = self.client.get("/api/feedback/status").json
        self.assertEqual(current["advice"]["code"], "overspeed")
        self.assertTrue(current["freshness"]["fresh"])
        with feedback.lock:
            feedback.latest_sample["sampled_at"] = time.time() - 10
            feedback.bridge_seen = time.time() - 10
        stale = self.client.get("/api/feedback/status").json
        self.assertEqual(stale["status"], "stale")
        self.assertIsNone(stale["telemetry"])
        self.assertEqual(stale["advice"]["code"], "stale")

    def test_telemetry_requires_local_producer_and_valid_edge(self):
        self.assertEqual(self.client.post("/api/feedback/telemetry", json={"source":"live"}).status_code, 400)
        self.assertEqual(self.client.post("/api/feedback/telemetry", json={"source":"live", "edge_id":"a"},
                                          environ_base={"REMOTE_ADDR":"203.0.113.5"}).status_code, 403)

    def test_demo_stop_is_shown_immediately(self):
        with feedback.lock:
            feedback.demo_process = SimpleNamespace(poll=lambda: None)
            feedback.demo_state = "stopped"
        status = self.client.get("/api/feedback/status").json
        self.assertEqual(status["status"], "stopped")
        self.assertIsNone(status["telemetry"])


class LiveAdapterTests(unittest.TestCase):
    def test_successful_original_placement_publishes_road_sample(self):
        bridge = feedback_live.bridge
        original_get = bridge.get_latest_phone_data
        original_move = bridge.move_vehicle_to_phone_position
        bridge.METHOD_KEY = "native"
        bridge.VEHICLE_ID = "ebike0"
        published = []

        def fake_placement(*args):
            bridge.traci.vehicle.moveToXY(vehID="ebike0", edgeID="road", laneIndex=0,
                                          x=1, y=2, keepRoute=0)

        def fake_main():
            bridge.move_vehicle_to_phone_position(-37.9, 145.1, 3, 90, 100.0, 5)

        def fake_publish(path, payload, quiet=False):
            published.append((path, payload))
            return True

        try:
            with (patch.object(bridge, "parse_args", return_value=SimpleNamespace(cfg="map.sumocfg")),
                  patch.object(bridge, "parse_sumocfg_for_netfile", return_value="map.net.xml"),
                  patch("sumolib.net.readNet", return_value=object()),
                  patch.object(bridge, "main", side_effect=fake_main),
                  patch.object(bridge.traci.vehicle, "moveToXY"),
                  patch.object(feedback_live, "collect", return_value={"edge_id":"road", "speed_mps":3}),
                  patch.object(feedback_live, "publish", side_effect=fake_publish)):
                bridge.move_vehicle_to_phone_position = fake_placement
                feedback_live.main()
        finally:
            bridge.get_latest_phone_data = original_get
            bridge.move_vehicle_to_phone_position = original_move
        telemetry = [payload for path, payload in published if path == "/api/feedback/telemetry"]
        self.assertEqual(len(telemetry), 1)
        self.assertEqual(telemetry[0]["source"], "live")
        self.assertEqual(telemetry[0]["match_method"], "native")


if __name__ == "__main__":
    unittest.main()
