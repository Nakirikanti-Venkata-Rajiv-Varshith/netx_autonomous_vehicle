import subprocess
import json
import time
import datetime
import speedtest
import os
import statistics


class WifiBandwidthMonitor:
    def __init__(self, interface="wlp3s0", interval=10):
        self.interface = interface
        self.interval = interval
        self.download_speeds = []
        self.upload_speeds = []
        self.timestamps = []

        os.makedirs("bandwidth_data", exist_ok=True)

    def measure_speed(self):
        try:
            st = speedtest.Speedtest()
            st.get_best_server()
            download = st.download() / 1e6  # Convert to Mbps
            upload = st.upload() / 1e6
        except Exception:
            download = None
            upload = None

        return download, upload


    def save_data(self, download, upload):
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"bandwidth_data/wifi_bandwidth_{timestamp}.json"

        log = {
            "timestamp": timestamp,
            "download_mbps": download,
            "upload_mbps": upload,
        }

        with open(filename, "w") as f:
            json.dump(log, f, indent=4)

    def generate_report(self):
        valid_downloads = [d for d in self.download_speeds if d is not None]
        valid_uploads = [u for u in self.upload_speeds if u is not None]

        print("\n==================================================")
        print("📈 BANDWIDTH MONITORING REPORT")
        print("==================================================")
        print(f"Monitoring duration: {len(self.timestamps) * (self.interval/60):.1f} minutes")
        print(f"Data points collected: {len(self.timestamps)}")

        if valid_downloads:
            print(f"📥 Avg Download: {statistics.mean(valid_downloads):.2f} Mbps")
            print(f"📉 Min Download: {min(valid_downloads):.2f} Mbps")
            print(f"📈 Max Download: {max(valid_downloads):.2f} Mbps")
        else:
            print("No valid download data collected.")

        if valid_uploads:
            print(f"📤 Avg Upload: {statistics.mean(valid_uploads):.2f} Mbps")
            print(f"📉 Min Upload: {min(valid_uploads):.2f} Mbps")
            print(f"📈 Max Upload: {max(valid_uploads):.2f} Mbps")
        else:
            print("No valid upload data collected.")

        print("==================================================\n")

    def run_continuous_monitoring(self):
        print("\nWiFi Bandwidth Monitor (Jetson Nano / Ubuntu / Laptop)")
        print("\n🚀 Starting Continuous Bandwidth Monitoring")
        print("============================================================")
        print(f"• Speed tests every {self.interval // 60} minutes")
        print(f"• WiFi Interface: {self.interface}")
        print("• Press Ctrl+C to stop")
        print("============================================================\n")

        try:
            while True:
                timestamp = datetime.datetime.now().strftime("%H:%M:%S")
                print(f"\n🕒 Iteration @ {timestamp}")
                print("🔄 Measuring internet speed...")

                download, upload = self.measure_speed()

                # Safety: Replace None with 0 for printing
                print(f"📥 Download: {download if download else 0:.2f} Mbps")
                print(f"📤 Upload: {upload if upload else 0:.2f} Mbps")

                # Append values (even None) for reporting
                self.download_speeds.append(download)
                self.upload_speeds.append(upload)
                self.timestamps.append(time.time())

                self.save_data(download, upload)

                time.sleep(self.interval)

        except KeyboardInterrupt:
            print("\n🛑 Monitoring stopped by user.")
            self.generate_report()


def main():
    monitor = WifiBandwidthMonitor(interface="wlp3s0", interval=300)
    monitor.run_continuous_monitoring()


if __name__ == "__main__":
    main()
