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


LEVELS = list(range(20, 100, 5))
STEP_TIME = 5
HOLD_TIME = 80


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
        self.level = 20

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

            if level <= 30:
                size = 512
            elif level <= 50:
                size = 768
            elif level <= 70:
                size = 1024
            elif level <= 85:
                size = 1536
            else:
                size = 2048

            try:

                a = torch.randn(
                    size,
                    size,
                    device="cuda"
                )

                b = torch.randn(
                    size,
                    size,
                    device="cuda"
                )

                c = torch.matmul(a, b)

                torch.cuda.synchronize()

            except Exception as e:
                print("GPU Error:", e)

    def stop(self):
        self.running = False


## +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# TIMER 


    def timer_worker(self):

        start_time = time.time()

        while not self.stop_event.is_set():

            elapsed = int(time.time() - start_time)

            hrs = elapsed // 3600
            mins = (elapsed % 3600) // 60
            secs = elapsed % 60

            print(
                f"[TIMER] {hrs:02d}:{mins:02d}:{secs:02d}"
            )

            time.sleep(1)




# ============================================================
# MAIN
# ============================================================

class StressController:

    def __init__(self):

        self.timer_thread = None

        self.stop_event = mp.Event()

        self.cpu_target = mp.Value(
            'd',
            20.0
        )

        self.cpu_processes = []

        self.ram_blocks = []

        self.gpu = GPUStressor()

        self.gpu_thread = None

    # --------------------------------------------------------

    def start_cpu(self):

        num_workers = max(
            1,
            mp.cpu_count() - 1
        )

        for _ in range(num_workers):

            p = mp.Process(
                target=cpu_worker,
                args=(
                    self.cpu_target,
                    self.stop_event
                )
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

        block_size_mb = 300

        try:

            block = np.ones(
                (block_size_mb * 1024 * 1024) // 8,
                dtype=np.float64
            )

            self.ram_blocks.append(block)

        except MemoryError:
            print("RAM allocation failed")

    # --------------------------------------------------------

    def monitor(self):

        cpu = psutil.cpu_percent()

        ram = psutil.virtual_memory().percent

        print(
            f"CPU={cpu:.1f}% "
            f"RAM={ram:.1f}%"
        )

    # --------------------------------------------------------

    def cleanup(self):

        print("\nReleasing resources...")

        self.stop_event.set()

        for p in self.cpu_processes:

            p.terminate()
            p.join()

        self.cpu_processes.clear()

        self.gpu.stop()

        self.ram_blocks.clear()

        gc.collect()

        print("Cleanup complete")

    # --------------------------------------------------------

    def run(self):

        self.timer_thread = threading.Thread(
            target=self.timer_worker,
            daemon=True
        )

        self.timer_thread.start()        
        self.start_cpu()

        self.start_gpu()

        for level in LEVELS:

            print(
                f"\n========================"
            )

            print(
                f"TARGET LEVEL = {level}%"
            )

            print(
                f"========================"
            )

            self.cpu_target.value = level

            self.gpu.set_level(level)

            self.allocate_ram(level)

            self.monitor()

            time.sleep(STEP_TIME)

        print(
            f"\nHolding at max load "
            f"for {HOLD_TIME}s..."
        )

        time.sleep(HOLD_TIME)

        self.cleanup()


# ============================================================
# ENTRY
# ============================================================

controller = StressController()

def signal_handler(sig, frame):

    controller.cleanup()
    os._exit(0)

signal.signal(
    signal.SIGINT,
    signal_handler
)

if __name__ == "__main__":

    controller.run()