import subprocess
import re
import time
import psutil

# Static application table
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
            return {"latency_sensitivity": app["latency_sensitivity"],
                    "accuracy_priority": app["accuracy_priority"]}
    return None

def get_usage_percent():
    try:
        # Start tegrastats and read one line
        proc = subprocess.Popen(["tegrastats"], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, universal_newlines=True)
        line = proc.stdout.readline()
        proc.kill()  # stop tegrastats after reading one line

        # RAM usage
        ram_match = re.search(r"RAM (\d+)/(\d+)MB", line)
        ram_percent = round(int(ram_match.group(1)) / int(ram_match.group(2)) * 100, 2) if ram_match else None

        # CPU usage
        cpu_match = re.search(r"CPU \[([^\]]+)\]", line)
        cpu_percent = None
        if cpu_match:
            usages = []
            for core in cpu_match.group(1).split(","):
                m = re.search(r"(\d+)%", core)
                if m:
                    usages.append(int(m.group(1)))
            if usages:
                cpu_percent = round(sum(usages) / len(usages), 2)

        # GPU usage
        gpu_match = re.search(r"GR3D_FREQ (\d+)%", line)
        gpu_percent = int(gpu_match.group(1)) if gpu_match else 0

        return {"cpu_percent": cpu_percent, "gpu_percent": gpu_percent, "ram_percent": ram_percent}

    except Exception as e:
        print("Error reading tegrastats:", e)
        return {"cpu_percent": None, "gpu_percent": None, "ram_percent": None}

def get_network_bandwidth(interface="wlan0"):
    """
    Get actual internet bandwidth using speedtest-cli
    This measures your real internet capacity, not just local network traffic
    """
    try:
        # Run speedtest-cli with timeout
        result = subprocess.run([
            'speedtest-cli', '--simple', '--timeout', '10'
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
        
        if result.returncode == 0:
            download_speed = 0
            upload_speed = 0
            
            # Parse output
            for line in result.stdout.split('\n'):
                if 'Download:' in line:
                    download_match = re.search(r'Download:\s+([\d.]+)\s*Mbit/s', line)
                    if download_match:
                        download_speed = float(download_match.group(1))
                elif 'Upload:' in line:
                    upload_match = re.search(r'Upload:\s+([\d.]+)\s*Mbit/s', line)
                    if upload_match:
                        upload_speed = float(upload_match.group(1))
            
            print(f"Speedtest results: Download: {download_speed} Mbps, Upload: {upload_speed} Mbps")
            return {"upload_Mbps": round(upload_speed, 2), "download_Mbps": round(download_speed, 2)}
        else:
            print(f"Speedtest failed: {result.stderr}")
            return {"upload_Mbps": 0, "download_Mbps": 0}
            
    except subprocess.TimeoutExpired:
        print("Speedtest timed out")
        return {"upload_Mbps": 0, "download_Mbps": 0}
    except Exception as e:
        print(f"Error running speedtest: {e}")
        return {"upload_Mbps": 0, "download_Mbps": 0}

def get_rtt(host="192.168.20.16"):
    try:
        output = subprocess.check_output(["ping", "-c", "1", host], stderr=subprocess.STDOUT, universal_newlines=True)
        for line in output.split("\n"):
            if "time=" in line:
                return float(line.split("time=")[1].split(" ")[0])  # ms
    except Exception:
        return None


if __name__ == "__main__":
    while True:
        usage = get_usage_percent()
        print("===== System Monitor =====")
        print("CPU Usage:", usage['cpu_percent'], "%")
        print("RAM Usage:", usage['ram_percent'], "%")
        print("GPU Usage:", usage['gpu_percent'], "%")
        print("Network Bandwidth:", get_network_bandwidth())
        print("RTT (to 192.168.20.16):", get_rtt(), "ms")
        print("==========================\n")
        time.sleep(2)  # refresh interval