#!/usr/bin/env python3

import collections
import socket
import time
import subprocess

import psutil
import rospy
from std_msgs.msg import Float32MultiArray


class BandwidthMonitorNode:
    def __init__(self):
        rospy.init_node("bandwidth_monitor_node_setb")

        # AUTO-DETECT ACTIVE NETWORK INTERFACE
        self.interface = self.get_active_interface()

        self.edge_host = rospy.get_param("~edge_host", "192.168.20.16")
        self.sample_period = rospy.get_param("~sample_period", 1.0)
        self.rtt_window = rospy.get_param("~rtt_window", 12)

        self.pub = rospy.Publisher(
            "/network/bandwidth",
            Float32MultiArray,
            queue_size=1
        )

        self.last_counters = None
        self.last_sample_time = None

        self.rtt_samples = collections.deque(maxlen=self.rtt_window)

        rospy.loginfo(f"[Bandwidth Monitor] Using interface: {self.interface}")

        self.measure_bandwidth()

    def get_active_interface(self):
        try:
            route = subprocess.check_output(
                "ip route | grep default",
                shell=True
            ).decode()

            interface = route.split("dev")[1].split()[0]

            return interface

        except Exception as e:
            rospy.logwarn(f"Failed to auto-detect interface: {e}")
            return "wlan0"

    def _sample_bandwidth(self):

        counters = psutil.net_io_counters(pernic=True).get(self.interface)

        now = time.time()

        if counters is None:
            rospy.logwarn(f"No counters found for interface {self.interface}")
            return 0.0, 0.0

        # FIRST SAMPLE
        if self.last_counters is None:
            self.last_counters = counters
            self.last_sample_time = now
            return 0.0, 0.0

        dt = now - self.last_sample_time

        if dt <= 0:
            return 0.0, 0.0

        bytes_sent_diff = counters.bytes_sent - self.last_counters.bytes_sent
        bytes_recv_diff = counters.bytes_recv - self.last_counters.bytes_recv

        # HANDLE COUNTER RESET
        if bytes_sent_diff < 0:
            bytes_sent_diff = 0

        if bytes_recv_diff < 0:
            bytes_recv_diff = 0

        upload_mbps = (bytes_sent_diff * 8.0) / (dt * 1e6)
        download_mbps = (bytes_recv_diff * 8.0) / (dt * 1e6)

        self.last_counters = counters
        self.last_sample_time = now

        return upload_mbps, download_mbps

    def _sample_latency(self):

        rtt_ms = None

        try:
            start = time.time()

            with socket.create_connection(
                (self.edge_host, 30052),
                timeout=0.5
            ):
                rtt_ms = (time.time() - start) * 1000.0

        except Exception:
            rtt_ms = None

        if rtt_ms is not None:
            self.rtt_samples.append(rtt_ms)

        jitter_ms = 0.0

        if len(self.rtt_samples) >= 2:

            diffs = [
                abs(
                    self.rtt_samples[i] -
                    self.rtt_samples[i - 1]
                )
                for i in range(1, len(self.rtt_samples))
            ]

            jitter_ms = sum(diffs) / len(diffs)

        return rtt_ms, jitter_ms

    def measure_bandwidth(self):

        hz = max(1, int(round(1.0 / max(self.sample_period, 0.1))))

        rate = rospy.Rate(hz)

        while not rospy.is_shutdown():

            try:

                upload, download = self._sample_bandwidth()

                rtt_ms, jitter_ms = self._sample_latency()

                network_ok = 1.0 if rtt_ms is not None else 0.0

                msg = Float32MultiArray()

                msg.data = [
                    round(upload, 3),
                    round(download, 3),
                    round(rtt_ms, 2) if rtt_ms is not None else -1.0,
                    round(jitter_ms, 2),
                    network_ok,
                ]

                self.pub.publish(msg)

                rospy.loginfo_throttle(
                    2.0,
                    (
                        "[Bandwidth] "
                        f"iface={self.interface} "
                        f"up={upload:.3f}Mbps "
                        f"down={download:.3f}Mbps "
                        f"rtt={'NA' if rtt_ms is None else f'{rtt_ms:.2f}ms'} "
                        f"jitter={jitter_ms:.2f} "
                        f"network_ok={bool(network_ok)}"
                    )
                )

            except Exception as exc:

                rospy.logwarn(
                    f"Passive network sampling error: {exc}"
                )

            rate.sleep()


if __name__ == "__main__":
    BandwidthMonitorNode()