#!/usr/bin/env python3
"""
Feedback-Controlled Stress Script
==================================
Reads tegrastats every 500ms and adjusts CPU/GPU/RAM load dynamically
to reach and hold a TARGET percentage — regardless of each system's
baseline. This makes it fair to compare Pure Onboard vs ALB.

Levels:
  70% → hold 30s
  80% → hold 30s
  85% → hold 30s
  90% → hold 90s
"""

import gc
import os
import re
import signal
import subprocess
import threading
import time
import multiprocessing as mp

import numpy as np
import psutil

try:
    import torch
    TORCH_AVAILABLE = torch.cuda.is_available()
except ImportError:
    TORCH_AVAILABLE = False


# ─────────────────────────────────────────────
# STRESS SCHEDULE  (target_percent, hold_secs)
# ─────────────────────────────────────────────
LEVELS = [
    (70, 30),
    (80, 30),
    (85, 30),
    (90, 90),
]

# How often (seconds) we read tegrastats and adjust
FEEDBACK_INTERVAL = 0.5

# Tolerance band: don't adjust if within ±DEAD_BAND % of target
DEAD_BAND = 3.0

# Max RAM we will ever allocate (safety cap, % of total)
RAM_SAFETY_CAP = 88.0


# ─────────────────────────────────────────────
# TEGRASTATS READER  (non-blocking, background)
# ─────────────────────────────────────────────

class TegraStatsReader:
    """
    Runs `sudo tegrastats --interval 500` in background.
    Exposes the latest cpu_percent, gpu_percent, ram_percent.
    Falls back to psutil for CPU/RAM if tegrastats unavailable.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._cpu = 0.0
        self._gpu = 0.0
        self._ram = 0.0
        self._running = True
        self._has_tegrastats = self._check_tegrastats()
        t = threading.Thread(target=self._run, daemon=True)
        t.start()

    def _check_tegrastats(self):
        try:
            r = subprocess.run(
                ["which", "tegrastats"],
                capture_output=True, text=True, timeout=2
            )
            return r.returncode == 0
        except Exception:
            return False

    def _run(self):
        if self._has_tegrastats:
            self._run_tegrastats()
        else:
            self._run_psutil_fallback()

    def _run_tegrastats(self):
        while self._running:
            try:
                proc = subprocess.Popen(
                    ["sudo", "tegrastats", "--interval", "500"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                    universal_newlines=True,
                )
                for line in proc.stdout:
                    if not self._running:
                        break
                    cache = {}

                    ram_m = re.search(r"RAM (\d+)/(\d+)MB", line)
                    if ram_m:
                        cache["ram"] = round(
                            int(ram_m.group(1)) / int(ram_m.group(2)) * 100.0, 1
                        )

                    cpu_m = re.search(r"CPU \[([^\]]+)\]", line)
                    if cpu_m:
                        usages = [
                            int(m.group(1))
                            for core in cpu_m.group(1).split(",")
                            if (m := re.search(r"(\d+)%", core))
                        ]
                        if usages:
                            cache["cpu"] = round(sum(usages) / len(usages), 1)

                    gpu_m = re.search(r"GR3D_FREQ (\d+)%", line)
                    if gpu_m:
                        cache["gpu"] = float(gpu_m.group(1))

                    if cache:
                        with self._lock:
                            if "cpu" in cache:
                                self._cpu = cache["cpu"]
                            if "gpu" in cache:
                                self._gpu = cache["gpu"]
                            if "ram" in cache:
                                self._ram = cache["ram"]

                proc.kill()
            except Exception as e:
                print(f"[tegrastats] error: {e}, retrying in 2s")
                time.sleep(2.0)

    def _run_psutil_fallback(self):
        print("[reader] tegrastats not found — using psutil (no GPU reading)")
        while self._running:
            cpu = psutil.cpu_percent(interval=0.5)
            ram = psutil.virtual_memory().percent
            with self._lock:
                self._cpu = cpu
                self._ram = ram
                self._gpu = 0.0
            time.sleep(FEEDBACK_INTERVAL)

    def read(self):
        with self._lock:
            return self._cpu, self._gpu, self._ram

    def stop(self):
        self._running = False


# ─────────────────────────────────────────────
# CPU WORKER  (runs in separate process)
# ─────────────────────────────────────────────

def _cpu_worker(duty_cycle: "mp.Value", stop_event: "mp.Event"):
    """
    Spin/sleep loop.  duty_cycle (0–1) is updated by the controller.
    """
    period = 0.05   # 50 ms period for smooth control
    while not stop_event.is_set():
        busy = period * max(0.0, min(1.0, duty_cycle.value))
        t0 = time.perf_counter()
        # busy-spin
        while (time.perf_counter() - t0) < busy:
            _ = sum(i * i for i in range(500))
        # sleep remainder
        sleep_t = period - (time.perf_counter() - t0)
        if sleep_t > 0:
            time.sleep(sleep_t)


# ─────────────────────────────────────────────
# GPU STRESSOR  (runs in a thread)
# ─────────────────────────────────────────────

class GPUStressor:
    def __init__(self):
        self._running = False
        self._duty = 0.5          # fraction of time doing matmul
        self._lock = threading.Lock()

    def set_duty(self, duty: float):
        with self._lock:
            self._duty = max(0.0, min(1.0, duty))

    def start(self):
        self._running = True
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        if not TORCH_AVAILABLE:
            return
        size = 1024
        while self._running:
            with self._lock:
                duty = self._duty
            t0 = time.perf_counter()
            busy = 0.1 * duty          # 100 ms period
            try:
                while (time.perf_counter() - t0) < busy:
                    a = torch.randn(size, size, device="cuda")
                    b = torch.randn(size, size, device="cuda")
                    _ = torch.matmul(a, b)
                    torch.cuda.synchronize()
            except Exception:
                pass
            sleep_t = 0.1 - (time.perf_counter() - t0)
            if sleep_t > 0:
                time.sleep(sleep_t)

    def stop(self):
        self._running = False


# ─────────────────────────────────────────────
# FEEDBACK CONTROLLER
# ─────────────────────────────────────────────

class FeedbackController:
    """
    Simple proportional controller.
    error  = target - measured
    output = clamp(prev_output + Kp * error, 0, 1)
    """

    def __init__(self, kp=0.05):
        self._kp = kp
        self._output = 0.3      # start at 30% duty

    def update(self, target_pct: float, measured_pct: float) -> float:
        error = target_pct - measured_pct
        if abs(error) < DEAD_BAND:
            return self._output           # inside dead-band → no change
        self._output = max(0.0, min(1.0, self._output + self._kp * error / 100.0))
        return self._output


# ─────────────────────────────────────────────
# RAM MANAGER
# ─────────────────────────────────────────────

class RAMManager:
    def __init__(self):
        self._blocks: list = []

    def adjust(self, target_pct: float):
        """Add or free 50 MB blocks to approach target_pct."""
        current = psutil.virtual_memory().percent
        safe_target = min(target_pct, RAM_SAFETY_CAP)

        if current < safe_target - DEAD_BAND:
            # Allocate one 50 MB block
            try:
                block = np.ones((50 * 1024 * 1024) // 8, dtype=np.float64)
                self._blocks.append(block)
            except MemoryError:
                print("[RAM] MemoryError — stopping allocation")

        elif current > safe_target + DEAD_BAND and self._blocks:
            # Free one block
            self._blocks.pop()
            gc.collect()

    def release_all(self):
        self._blocks.clear()
        gc.collect()


# ─────────────────────────────────────────────
# MAIN STRESS CONTROLLER
# ─────────────────────────────────────────────

class StressController:

    def __init__(self):
        self._stop_event = mp.Event()

        # CPU
        self._cpu_duty   = mp.Value('d', 0.3)
        self._cpu_procs  = []
        self._cpu_ctrl   = FeedbackController(kp=0.06)

        # GPU
        self._gpu        = GPUStressor()
        self._gpu_ctrl   = FeedbackController(kp=0.06)

        # RAM
        self._ram_mgr    = RAMManager()

        # Tegrastats reader
        self._reader     = TegraStatsReader()

        self._target     = 0.0
        self._start_time = time.time()

    # ── startup ──────────────────────────────

    def _start_cpu_workers(self):
        n = max(1, mp.cpu_count() - 1)
        for _ in range(n):
            p = mp.Process(target=_cpu_worker,
                           args=(self._cpu_duty, self._stop_event),
                           daemon=True)
            p.start()
            self._cpu_procs.append(p)

    # ── feedback loop ────────────────────────

    def _feedback_loop(self, target_pct: float, hold_secs: float):
        """Run feedback control for `hold_secs`, targeting `target_pct`."""
        self._target = target_pct
        deadline = time.time() + hold_secs
        iteration = 0

        while time.time() < deadline:
            cpu, gpu, ram = self._reader.read()

            # ── CPU ──
            new_duty = self._cpu_ctrl.update(target_pct, cpu)
            self._cpu_duty.value = new_duty

            # ── GPU ──
            if TORCH_AVAILABLE:
                gpu_duty = self._gpu_ctrl.update(target_pct, gpu)
                self._gpu.set_duty(gpu_duty)

            # ── RAM ──
            self._ram_mgr.adjust(target_pct)

            # ── console ──
            elapsed = int(time.time() - self._start_time)
            h, rem  = divmod(elapsed, 3600)
            m, s    = divmod(rem, 60)
            remaining = max(0.0, deadline - time.time())

            if iteration % 4 == 0:   # print every ~2 s
                print(
                    f"\r  Target={target_pct:.0f}%  "
                    f"CPU={cpu:5.1f}%  GPU={gpu:5.1f}%  RAM={ram:5.1f}%  "
                    f"duty={new_duty:.2f}  "
                    f"remaining={remaining:5.1f}s  "
                    f"uptime={h:02d}:{m:02d}:{s:02d}",
                    end="", flush=True
                )

            iteration += 1
            time.sleep(FEEDBACK_INTERVAL)

        print()   # newline after level ends

    # ── cleanup ──────────────────────────────

    def cleanup(self):
        print("\n[cleanup] Stopping workers…")
        self._stop_event.set()
        self._gpu.stop()
        for p in self._cpu_procs:
            p.terminate()
            p.join()
        self._cpu_procs.clear()
        self._ram_mgr.release_all()
        self._reader.stop()
        elapsed = int(time.time() - self._start_time)
        h, rem  = divmod(elapsed, 3600)
        m, s    = divmod(rem, 60)
        print(f"[cleanup] Done. Total run time: {h:02d}:{m:02d}:{s:02d}")

    # ── main run ─────────────────────────────

    def run(self):
        self._start_cpu_workers()
        self._gpu.start()

        print(f"\nFeedback-Controlled Stress Test")
        print(f"Tegrastats polling every {FEEDBACK_INTERVAL*1000:.0f} ms")
        print(f"Dead-band ±{DEAD_BAND}%   |   RAM safety cap {RAM_SAFETY_CAP}%")
        print(f"Schedule: {LEVELS}\n")

        for target, hold in LEVELS:
            print(f"\n{'='*60}")
            print(f"  LEVEL: {target}%  for {hold}s")
            print(f"{'='*60}")
            # Let workers spin up before measuring
            time.sleep(1.5)
            self._feedback_loop(target, hold)

        self.cleanup()


# ─────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────

controller = StressController()


def _sigint(sig, frame):
    controller.cleanup()
    os._exit(0)


signal.signal(signal.SIGINT, _sigint)

if __name__ == "__main__":
    controller.run()