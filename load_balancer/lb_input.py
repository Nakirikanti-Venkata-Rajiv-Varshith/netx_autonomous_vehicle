import collections
import re
import socket
import subprocess
import threading
import time

import cv2
import numpy as np
import psutil


APPLICATION_TABLE = [
    {"application": "collision_avoidance", "latency_sensitivity": "high", "accuracy_priority": "low"},
    {"application": "collision_detection", "latency_sensitivity": "high", "accuracy_priority": "high"},
    {"application": "pedestrian_avoidance", "latency_sensitivity": "high", "accuracy_priority": "high"},
    {"application": "pedestrian_detection", "latency_sensitivity": "high", "accuracy_priority": "high"},
    {"application": "localization", "latency_sensitivity": "high", "accuracy_priority": "high"},
    {"application": "traffic_light_detection", "latency_sensitivity": "low", "accuracy_priority": "high"},
    {"application": "traffic_sign_detection", "latency_sensitivity": "medium", "accuracy_priority": "high"},
    {"application": "lane_detection", "latency_sensitivity": "high", "accuracy_priority": "high"},
    {"application": "depth_estimation", "latency_sensitivity": "high", "accuracy_priority": "high"},
    {"application": "drowsiness_detection", "latency_sensitivity": "medium", "accuracy_priority": "high"},
    {"application": "drivable_area", "latency_sensitivity": "high", "accuracy_priority": "high"},
    {"application": "infotainment", "latency_sensitivity": "low", "accuracy_priority": "low"},
]


def get_application_table(application_name):
    for app in APPLICATION_TABLE:
        if app["application"] == application_name:
            return {
                "latency_sensitivity": app["latency_sensitivity"],
                "accuracy_priority": app["accuracy_priority"],
            }
    return None


def get_usage_percent():
    try:
        proc = subprocess.Popen(
            ["tegrastats"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            universal_newlines=True,
        )
        line = proc.stdout.readline()
        proc.kill()

        ram_match = re.search(r"RAM (\d+)/(\d+)MB", line)
        ram_percent = round(int(ram_match.group(1)) / int(ram_match.group(2)) * 100, 2) if ram_match else None

        cpu_match = re.search(r"CPU \[([^\]]+)\]", line)
        cpu_percent = None
        if cpu_match:
            usages = []
            for core in cpu_match.group(1).split(","):
                match = re.search(r"(\d+)%", core)
                if match:
                    usages.append(int(match.group(1)))
            if usages:
                cpu_percent = round(sum(usages) / len(usages), 2)

        gpu_match = re.search(r"GR3D_FREQ (\d+)%", line)
        gpu_percent = int(gpu_match.group(1)) if gpu_match else 0

        power_match = re.search(r"POM_5V_IN (\d+)/(\d+)", line)
        power_mw = int(power_match.group(2)) if power_match else 0

        return {
            "cpu_percent": cpu_percent,
            "gpu_percent": gpu_percent,
            "ram_percent": ram_percent,
            "power_mw": power_mw,
        }
    except Exception as exc:
        print("Error reading tegrastats:", exc)
        return {"cpu_percent": None, "gpu_percent": None, "ram_percent": None, "power_mw": None}


class PassiveNetworkProfiler:
    def __init__(self, interface="wlan0", host="192.168.20.16", sample_window=12):
        self.interface = interface
        self.host = host
        self.sample_window = sample_window
        self.lock = threading.Lock()
        self.last_counters = None
        self.last_time = None
        self.rtt_samples = collections.deque(maxlen=sample_window)

    def sample_bandwidth(self):
        try:
            counters = psutil.net_io_counters(pernic=True).get(self.interface)
            now = time.time()
            if counters is None:
                return {"upload_Mbps": 0.0, "download_Mbps": 0.0}

            if self.last_counters is None:
                self.last_counters = counters
                self.last_time = now
                return {"upload_Mbps": 0.0, "download_Mbps": 0.0}

            dt = max(now - self.last_time, 1e-3)
            upload_mbps = ((counters.bytes_sent - self.last_counters.bytes_sent) * 8.0) / (dt * 1e6)
            download_mbps = ((counters.bytes_recv - self.last_counters.bytes_recv) * 8.0) / (dt * 1e6)
            self.last_counters = counters
            self.last_time = now
            return {
                "upload_Mbps": round(max(upload_mbps, 0.0), 2),
                "download_Mbps": round(max(download_mbps, 0.0), 2),
            }
        except Exception:
            return {"upload_Mbps": 0.0, "download_Mbps": 0.0}

    def sample_rtt(self, timeout=0.2):
        try:
            start = time.time()
            with socket.create_connection((self.host, 80), timeout=timeout):
                rtt_ms = (time.time() - start) * 1000.0
        except Exception:
            rtt_ms = None

        with self.lock:
            if rtt_ms is not None:
                self.rtt_samples.append(rtt_ms)
            samples = list(self.rtt_samples)

        jitter_ms = 0.0
        if len(samples) >= 2:
            diffs = [abs(samples[i] - samples[i - 1]) for i in range(1, len(samples))]
            jitter_ms = sum(diffs) / float(len(diffs))

        return {
            "rtt_ms": round(rtt_ms, 2) if rtt_ms is not None else None,
            "jitter_ms": round(jitter_ms, 2),
            "network_ok": rtt_ms is not None,
        }

    def sample(self):
        bandwidth = self.sample_bandwidth()
        latency = self.sample_rtt()
        return {
            "upload_Mbps": bandwidth["upload_Mbps"],
            "download_Mbps": bandwidth["download_Mbps"],
            "rtt_ms": latency["rtt_ms"],
            "jitter_ms": latency["jitter_ms"],
            "network_ok": latency["network_ok"],
        }


class RollingPowerTracker:
    def __init__(self, window_size=15):
        self.samples = collections.deque(maxlen=window_size)

    def update(self, power_mw):
        if power_mw is not None:
            self.samples.append(float(power_mw))
        return self.average()

    def average(self):
        if not self.samples:
            return 0.0
        return sum(self.samples) / float(len(self.samples))


class SceneProfiler:
    def __init__(self, flow_size=(160, 90), embedding_size=(16, 16)):
        self.flow_size = flow_size
        self.embedding_size = embedding_size
        self.prev_gray = None
        self.prev_embedding = None

    def profile(self, frame_rgb):
        gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)
        flow_gray = cv2.resize(gray, self.flow_size)
        embedding = cv2.resize(gray, self.embedding_size).astype(np.float32) / 255.0

        motion = 0.0
        if self.prev_gray is not None:
            flow = cv2.calcOpticalFlowFarneback(
                self.prev_gray,
                flow_gray,
                None,
                0.5,
                3,
                15,
                3,
                5,
                1.2,
                0,
            )
            mag, _ = cv2.cartToPolar(flow[..., 0], flow[..., 1])
            motion = float(np.mean(mag))

        edges = cv2.Canny(gray, 60, 160)
        edge_density = float(np.count_nonzero(edges)) / float(edges.size)
        texture_density = float(cv2.Laplacian(gray, cv2.CV_32F).var())
        scene_complexity = min(1.0, edge_density * 2.5 + texture_density / 2500.0)

        change_score = 0.0
        if self.prev_embedding is not None:
            change_score = float(np.mean(np.abs(embedding - self.prev_embedding)))

        self.prev_gray = flow_gray
        self.prev_embedding = embedding
        return {
            "motion_score": round(motion, 4),
            "scene_complexity": round(scene_complexity, 4),
            "change_score": round(change_score, 4),
        }


_DEFAULT_NETWORK_PROFILER = PassiveNetworkProfiler()


def get_network_bandwidth(interface="wlan0"):
    _DEFAULT_NETWORK_PROFILER.interface = interface
    sample = _DEFAULT_NETWORK_PROFILER.sample_bandwidth()
    return {
        "upload_Mbps": sample["upload_Mbps"],
        "download_Mbps": sample["download_Mbps"],
    }


def get_rtt(host="192.168.20.16"):
    _DEFAULT_NETWORK_PROFILER.host = host
    sample = _DEFAULT_NETWORK_PROFILER.sample_rtt()
    return sample["rtt_ms"]


if __name__ == "__main__":
    network = PassiveNetworkProfiler()
    power = RollingPowerTracker()
    while True:
        usage = get_usage_percent()
        power_avg = power.update(usage.get("power_mw"))
        print("===== System Monitor =====")
        print("CPU Usage:", usage["cpu_percent"], "%")
        print("RAM Usage:", usage["ram_percent"], "%")
        print("GPU Usage:", usage["gpu_percent"], "%")
        print("Power Avg:", round(power_avg, 2), "mW")
        print("Network:", network.sample())
        print("==========================\n")
        time.sleep(2)
