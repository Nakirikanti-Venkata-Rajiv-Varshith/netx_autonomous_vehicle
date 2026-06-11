#!/usr/bin/env python3

import os
import gc
import time
import psutil
import signal
import threading
import multiprocessing as mp
import numpy as np

try:
    import torch
    TORCH_AVAILABLE = True
except:
    TORCH_AVAILABLE = False



# For ALB
LEVELS = [
    (50, 30),
    (60, 30),
    (70, 30),
    (80, 90),
]

RELEASE_HOLD = 30  # seconds of cooldown between each stress level

#For onboard
# Just increase the for loop 10000 -> 14000 balnces


# ============================================================
# CPU
# ============================================================

def cpu_worker(target_percent, stop_event):

    period = 1.0

    while not stop_event.is_set():

        busy_time = period * (target_percent.value / 100.0)

        start = time.perf_counter()

        while (
            time.perf_counter() - start
        ) < busy_time and not stop_event.is_set():

            x = 0
            for i in range(10000):
                x += i * i

        sleep_time = period - busy_time

        if sleep_time > 0:
            time.sleep(sleep_time)


# ============================================================
# GPU
# ============================================================

class GPUStressor:

    def __init__(self):

        self.running = False
        self.level = 0

    def set_level(self, level):
        self.level = level

    def run(self):

        if not TORCH_AVAILABLE:
            print("Torch not available. GPU disabled.")
            return

        if not torch.cuda.is_available():
            print("CUDA not available. GPU disabled.")
            return

        self.running = True

        while self.running:

            level = self.level

            # At level 0 (release phase), do minimal work / sleep
            if level == 0:
                time.sleep(0.1)
                continue

            if level <= 55:
                size = 512
            elif level <= 65:
                size = 768
            elif level <= 75:
                size = 1024
            elif level <= 85:
                size = 1280
            else:
                size = 1536

            try:

                a = torch.randn(size, size, device="cuda")
                b = torch.randn(size, size, device="cuda")
                c = torch.matmul(a, b)
                torch.cuda.synchronize()

            except Exception as e:
                print("GPU Error:", e)

    def stop(self):
        self.running = False


# ============================================================
# TIMER
# ============================================================

class LaunchTimer:

    def __init__(self):
        self.start_time = time.time()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._tick, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()

    def elapsed_str(self):
        elapsed = int(time.time() - self.start_time)
        h = elapsed // 3600
        m = (elapsed % 3600) // 60
        s = elapsed % 60
        return f"{h:02d}:{m:02d}:{s:02d}"

    def _tick(self):
        while not self._stop.is_set():
            elapsed = self.elapsed_str()
            # Overwrite same line using carriage return
            print(f"\r  ⏱  Elapsed since launch: {elapsed}", end="", flush=True)
            time.sleep(1)


# ============================================================
# MAIN
# ============================================================

class StressController:

    def __init__(self):

        self.stop_event = mp.Event()

        self.cpu_target = mp.Value('d', 0.0)

        self.cpu_processes = []

        self.ram_blocks = []

        self.gpu = GPUStressor()

        self.gpu_thread = None

        self.timer = LaunchTimer()

    # --------------------------------------------------------

    def start_cpu(self):

        # For ALB
        num_workers = max(1, mp.cpu_count() - 1)

        # FOR Onboard
        # num_workers = mp.cpu_count()

        for _ in range(num_workers):

            p = mp.Process(
                target=cpu_worker,
                args=(self.cpu_target, self.stop_event)
            )

            p.start()
            self.cpu_processes.append(p)

    # --------------------------------------------------------

    def start_gpu(self):

        self.gpu_thread = threading.Thread(
            target=self.gpu.run,
            daemon=True
        )

        self.gpu_thread.start()

    # --------------------------------------------------------

    def allocate_ram(self, target_percent):

        current = psutil.virtual_memory().percent

        if current >= target_percent:
            return

        # Cap RAM target at 85% to avoid OOM
        safe_target = min(target_percent, 85)

        while psutil.virtual_memory().percent < safe_target:

            block_size_mb = 200

            try:

                block = np.ones(
                    (block_size_mb * 1024 * 1024) // 8,
                    dtype=np.float64
                )

                self.ram_blocks.append(block)

            except MemoryError:
                print("\nRAM allocation limit reached")
                break

    # --------------------------------------------------------

    def release_ram(self):
        """Free all allocated RAM blocks."""
        self.ram_blocks.clear()
        gc.collect()

    # --------------------------------------------------------

    def run_release_phase(self):
        """Drop CPU/GPU to 0% and hold for RELEASE_HOLD seconds so the system cools down."""

        print(f"\n\n{'='*32}")
        print(f"  RELEASE PHASE   HOLD = {RELEASE_HOLD}s")
        print(f"{'='*32}")

        # Drop CPU and GPU load to zero
        self.cpu_target.value = 0.0
        self.gpu.set_level(0)

        # Free RAM so we can observe natural cooldown
        self.release_ram()

        # Let readings settle, then monitor
        time.sleep(2)
        self.monitor()

        time.sleep(RELEASE_HOLD - 2)

    # --------------------------------------------------------

    def monitor(self):

        cpu = psutil.cpu_percent(interval=1)
        ram = psutil.virtual_memory().percent

        # Print on new line so it doesn't clash with timer line
        print(f"\n  → Live readings: CPU={cpu:.1f}%  RAM={ram:.1f}%  |  Uptime: {self.timer.elapsed_str()}")

    # --------------------------------------------------------

    def cleanup(self):

        self.timer.stop()

        print(f"\n\nTotal run time: {self.timer.elapsed_str()}")
        print("Releasing resources...")

        self.stop_event.set()

        for p in self.cpu_processes:
            p.terminate()
            p.join()

        self.cpu_processes.clear()

        self.gpu.stop()

        self.ram_blocks.clear()

        gc.collect()

        print("Cleanup complete.")

    # --------------------------------------------------------

    def run(self):

        self.timer.start()

        self.start_cpu()
        self.start_gpu()

        for i, (level, hold) in enumerate(LEVELS):

            print(f"\n\n{'='*32}")
            print(f"  TARGET = {level}%   HOLD = {hold}s")
            print(f"{'='*32}")

            self.cpu_target.value = level
            self.gpu.set_level(level)
            self.allocate_ram(level)

            # Let load settle for 2s before reading
            time.sleep(2)
            self.monitor()

            # Hold for remaining duration
            time.sleep(hold - 2)

            # ── Release phase after every stress level ──────────
            # (skip only after the very last level to go straight to cleanup)
            if i < len(LEVELS) - 1:
                self.run_release_phase()

        self.cleanup()


# ============================================================
# ENTRY
# ============================================================

controller = StressController()

def signal_handler(sig, frame):
    controller.cleanup()
    os._exit(0)

signal.signal(signal.SIGINT, signal_handler)

if __name__ == "__main__":
    controller.run()