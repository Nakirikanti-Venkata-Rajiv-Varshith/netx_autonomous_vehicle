import time
import threading
import multiprocessing
import numpy as np
import psutil
import os
import signal
import sys
import gc

class ResourceStressor:
    def __init__(self):
        self.stop_event = threading.Event()
        self.cpu_processes = []
        self.ram_arrays = []
        self.gpu_context = None
        
    def cleanup(self):
        """Clean up all resources"""
        print("\nCleaning up resources...")
        self.stop_event.set()
        
        # Stop CPU processes
        for process in self.cpu_processes:
            if process.is_alive():
                process.terminate()
        for process in self.cpu_processes:
            process.join(timeout=1.0)
        
        # Release RAM
        self.ram_arrays.clear()
        gc.collect()
        
        # Clean up PyCUDA context
        if self.gpu_context is not None:
            try:
                self.gpu_context.pop()
            except:
                pass
            self.gpu_context = None
        
        print("Cleanup completed")

    def stress_cpu_parallel(self, target_utilization=75, duration=10):
        """Stress CPU to target utilization"""
        print(f"Stressing CPU to {target_utilization}%...")
        
        def cpu_worker_controlled():
            """Controlled CPU worker with adjustable utilization"""
            period = 0.1  # 100ms period
            busy_time = period * (target_utilization / 100.0)
            idle_time = period - busy_time
            
            end_time = time.time() + duration
            while time.time() < end_time and not self.stop_event.is_set():
                start_time = time.perf_counter()
                
                # Busy loop for portion of period
                while (time.perf_counter() - start_time) < busy_time:
                    # Mix of different operations
                    x = 0
                    for i in range(1000):
                        x += np.sqrt(i) * np.sin(i)
                    
                    # Matrix operation
                    if i % 100 == 0:
                        a = np.random.rand(100, 100)
                        b = np.random.rand(100, 100)
                        c = np.dot(a, b)
                
                # Sleep for remaining time
                if idle_time > 0:
                    time.sleep(idle_time)
        
        # Create multiple processes
        num_cores = multiprocessing.cpu_count()
        self.cpu_processes = []
        
        for i in range(num_cores):
            process = multiprocessing.Process(target=cpu_worker_controlled)
            process.daemon = True
            process.start()
            self.cpu_processes.append(process)
        
        # Wait for specified duration
        time.sleep(duration)
        
        # Cleanup CPU processes
        for process in self.cpu_processes:
            if process.is_alive():
                process.terminate()
        for process in self.cpu_processes:
            process.join(timeout=1.0)
            
        self.cpu_processes.clear()
        print("CPU stress completed")

    def stress_ram_parallel(self, target_utilization=75, duration=10):
        """Stress RAM to target utilization"""
        print(f"Stressing RAM to {target_utilization}%...")
        
        memory = psutil.virtual_memory()
        available_memory = memory.available
        target_memory = int(available_memory * (target_utilization / 100.0))
        
        # Be more conservative to avoid OOM
        target_memory = int(target_memory * 0.90)  # Use 80% of target
        
        print(f"Target memory: {target_memory / (1024**3):.2f} GB")
        
        chunk_size = 50 * 1024 * 1024  # Smaller 50MB chunks
        allocated = 0
        
        try:
            while allocated < target_memory and not self.stop_event.is_set():
                try:
                    # Alternate between data types
                    if len(self.ram_arrays) % 4 == 0:
                        chunk = np.ones(chunk_size // 8, dtype=np.float64)
                    elif len(self.ram_arrays) % 4 == 1:
                        chunk = np.ones(chunk_size // 4, dtype=np.float32)
                    elif len(self.ram_arrays) % 4 == 2:
                        chunk = bytearray(chunk_size)  # Raw bytes
                    else:
                        chunk = np.ones(chunk_size // 2, dtype=np.int32)
                    
                    self.ram_arrays.append(chunk)
                    allocated += chunk_size
                    
                    # Check more frequently
                    current_memory = psutil.virtual_memory().percent
                    if current_memory >= target_utilization - 2:  # Stop 2% early
                        break
                        
                except MemoryError:
                    print("Reached memory limit")
                    break
            
            print(f"Allocated {allocated / (1024**3):.2f} GB of RAM")
            print(f"Current memory usage: {psutil.virtual_memory().percent}%")
            
            # Keep memory allocated for duration
            end_time = time.time() + duration
            while time.time() < end_time and not self.stop_event.is_set():
                # Occasionally access memory to keep it active
                if len(self.ram_arrays) > 0:
                    arr = self.ram_arrays[0]
                    if hasattr(arr, 'dtype'):
                        arr[0] = arr[0] * 1.000001
                time.sleep(0.1)
            
        except Exception as e:
            print(f"RAM stress error: {e}")
        
        finally:
            # Clean up RAM (will be done later in main cleanup)
            pass

    def stress_gpu_pycuda_parallel(self, target_utilization=75, duration=10):
        """GPU stress using PyCUDA"""
        try:
            import pycuda.driver as cuda
            import pycuda.autoinit
            from pycuda.compiler import SourceModule
            import pycuda.gpuarray as gpuarray
            
            print(f"Stressing GPU using PyCUDA to {target_utilization}%...")
            
            # Store context for proper cleanup
            self.gpu_context = cuda.Context.get_current()
            
            # Simpler kernel
            kernel_code = """
            __global__ void simple_stress_kernel(float *data, int size, int iterations) {
                int idx = threadIdx.x + blockIdx.x * blockDim.x;
                if (idx < size) {
                    float x = data[idx];
                    for (int i = 0; i < iterations; i++) {
                        x = sinf(x) * cosf(x);
                    }
                    data[idx] = x;
                }
            }
            """
            
            # Use smaller arrays to avoid timeouts
            array_size = 1000000
            block_size = 256
            grid_size = (array_size + block_size - 1) // block_size
            
            # Compile kernel
            mod = SourceModule(kernel_code)
            kernel = mod.get_function("simple_stress_kernel")
            
            # Create data
            input_data = np.random.rand(array_size).astype(np.float32)
            
            # Allocate GPU memory
            input_gpu = gpuarray.to_gpu(input_data)
            
            end_time = time.time() + duration
            iteration = 0
            
            while time.time() < end_time and not self.stop_event.is_set():
                try:
                    # Use smaller iterations to prevent timeout
                    # iterations = min(10 + (iteration % 20), 30)
                    iterations = 500
                    
                    # Launch kernel
                    kernel(input_gpu, np.int32(array_size), np.int32(iterations),
                          block=(block_size, 1, 1), grid=(grid_size, 1))
                    
                    # Synchronize
                    cuda.Context.synchronize()
                    
                    iteration += 1
                    
                    if iteration % 10 == 0 and self.stop_event.is_set():
                        break
                        
                except cuda.LogicError as e:
                    if "timeout" in str(e).lower():
                        print("GPU operation timed out, continuing...")
                        continue
                    else:
                        raise
                except Exception as e:
                    print(f"GPU kernel error: {e}")
                    break
            
            # Cleanup GPU memory
            try:
                del input_gpu
                cuda.Context.get_current().synchronize()
            except:
                pass
                
            print("GPU stress with PyCUDA completed")
            
        except ImportError:
            print("PyCUDA not available, trying fallback methods...")
            self.stress_gpu_fallback_parallel(duration)
        except Exception as e:
            print(f"PyCUDA stress failed: {e}")
            self.stress_gpu_fallback_parallel(duration)

    def stress_gpu_fallback_parallel(self, duration=10):
        """Fallback GPU stress"""
        print("Using GPU fallback methods...")
        
        # Try TensorFlow first
        if self.try_tensorflow_gpu_parallel(duration):
            return
            
        # Try PyTorch next
        if self.try_pytorch_gpu_parallel(duration):
            return
            
        # Final fallback: CPU-based matrix operations
        self.stress_gpu_cpu_fallback_parallel(duration)

    def try_tensorflow_gpu_parallel(self, duration):
        """Try TensorFlow for GPU stress"""
        try:
            import tensorflow as tf
            if tf.config.list_physical_devices('GPU'):
                print("Using TensorFlow for GPU stress...")
                
                @tf.function
                def gpu_operations():
                    a = tf.random.normal((2000, 2000))
                    b = tf.random.normal((2000, 2000))
                    c = tf.matmul(a, b)
                    return tf.linalg.det(c)
                
                end_time = time.time() + duration
                while time.time() < end_time and not self.stop_event.is_set():
                    _ = gpu_operations().numpy()
                    
                print("TensorFlow GPU stress completed")
                return True
        except:
            pass
        return False

    def try_pytorch_gpu_parallel(self, duration):
        """Try PyTorch for GPU stress"""
        try:
            import torch
            if torch.cuda.is_available():
                print("Using PyTorch for GPU stress...")
                
                device = torch.device('cuda')
                size = 2000
                
                end_time = time.time() + duration
                while time.time() < end_time and not self.stop_event.is_set():
                    a = torch.randn(size, size, device=device)
                    b = torch.randn(size, size, device=device)
                    c = torch.mm(a, b)
                    _ = torch.det(c)
                    torch.cuda.synchronize()
                    
                print("PyTorch GPU stress completed")
                return True
        except:
            pass
        return False

    def stress_gpu_cpu_fallback_parallel(self, duration):
        """Final fallback: simulate GPU workload on CPU"""
        print("Using CPU to simulate GPU workload...")
        
        def matrix_worker():
            size = 1000
            end_time = time.time() + duration
            while time.time() < end_time and not self.stop_event.is_set():
                a = np.random.randn(size, size)
                b = np.random.randn(size, size)
                c = np.dot(a, b)
                _ = np.linalg.det(c)
        
        # Use fewer threads to avoid overwhelming the system
        threads = []
        for i in range(2):
            t = threading.Thread(target=matrix_worker)
            t.daemon = True
            t.start()
            threads.append(t)
        
        # Wait for duration
        time.sleep(duration)
        
        # Threads will automatically stop when function ends
        print("CPU-based GPU simulation completed")

    def monitor_resources_parallel(self, duration=10):
        """Monitor resource usage during parallel stress"""
        print("Monitoring resources during parallel stress...")
        print("Time | CPU% | RAM%")
        print("-" * 25)
        
        start_time = time.time()
        while time.time() - start_time < duration and not self.stop_event.is_set():
            try:
                cpu_percent = psutil.cpu_percent(interval=1)
                memory_percent = psutil.virtual_memory().percent
                elapsed = time.time() - start_time
                print(f"{elapsed:5.1f}s | {cpu_percent:4.1f}% | {memory_percent:4.1f}%")
            except Exception as e:
                print(f"Monitoring error: {e}")
                break

    def run_parallel_stress(self, target_utilization=75, duration=10):
        """Run all three stresses in parallel"""
        print(f"\n=== Starting PARALLEL stress test for {duration} seconds ===")
        
        # Start monitoring
        monitor_thread = threading.Thread(
            target=self.monitor_resources_parallel, 
            args=(duration,)
        )
        monitor_thread.daemon = True
        monitor_thread.start()
        
        # Create threads for each stress type
        stress_threads = []
        
        # CPU stress thread
        cpu_thread = threading.Thread(
            target=self.stress_cpu_parallel,
            args=(target_utilization, duration)
        )
        stress_threads.append(cpu_thread)
        
        # RAM stress thread
        ram_thread = threading.Thread(
            target=self.stress_ram_parallel,
            args=(target_utilization, duration)
        )
        stress_threads.append(ram_thread)
        
        # GPU stress thread
        gpu_thread = threading.Thread(
            target=self.stress_gpu_pycuda_parallel,
            args=(target_utilization, duration)
        )
        stress_threads.append(gpu_thread)
        
        # Start all stress threads
        for thread in stress_threads:
            thread.start()
        
        # Wait for all stress threads to complete
        for thread in stress_threads:
            thread.join()
        
        print("=== Parallel stress test completed ===")

def main():
    stressor = ResourceStressor()
    
    def signal_handler(sig, frame):
        print("\nInterrupt received, shutting down...")
        stressor.cleanup()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        print("=" * 60)
        print("PARALLEL Resource Stress Test for Jetson")
        print("All stresses run simultaneously for maximum load")
        print("=" * 60)
        
        target_utilization = 90
        spike_duration = 200
        interval = 200
        
        cycle_count = 0
        max_cycles = 5  # Safety limit
        
        while cycle_count < max_cycles:
            cycle_count += 1
            print(f"\n*** Parallel Cycle {cycle_count}/{max_cycles} at {time.ctime()} ***")
            
            # Run all stresses in parallel
            stressor.run_parallel_stress(target_utilization, spike_duration)
            
            # Clean up RAM after each cycle (CPU and GPU cleanup happens automatically)
            stressor.ram_arrays.clear()
            gc.collect()
            
            print(f"*** Parallel Cycle {cycle_count} completed ***")
            
            # Wait for next interval
            if cycle_count < max_cycles:
                wait_time = interval - spike_duration
                print(f"Waiting {wait_time} seconds until next cycle...")
                for i in range(wait_time):
                    if stressor.stop_event.is_set():
                        break
                    time.sleep(1)
                    if i % 30 == 0 and i > 0:
                        print(f"Waiting... {wait_time - i}s remaining")
            
    except Exception as e:
        print(f"Error: {e}")
    finally:
        stressor.cleanup()
        print("Test completed successfully!")

if __name__ == "__main__":
    # Check basic dependencies
    try:
        import psutil
        import numpy as np
    except ImportError as e:
        print(f"Missing required package: {e}")
        print("Please install: pip install psutil numpy")
        sys.exit(1)
    
    main()