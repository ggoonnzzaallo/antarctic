import numpy as np
import time

# Define matrix size
N = 1000  # Matrix size (N x N)

# Generate two random matrices
A = np.random.rand(N, N).astype(np.float32)  # Floating-point (FP32) for FLOPS
B = np.random.rand(N, N).astype(np.float32)

# Simulate FLOPS calculation with floating-point operations
start_time = time.time()
C_floats = np.dot(A, B)  # Matrix multiplication
elapsed_time_floats = time.time() - start_time
num_floating_ops = 2 * (N ** 3)  # Approximation: 2N^3 ops for matrix mult

# Calculate FLOPS (floating-point operations per second)
flops = num_floating_ops / elapsed_time_floats

# Now, simulate TOPS calculation with integer operations
A_int = (A * 255).astype(np.int8)  # Convert to INT8 for TOPS
B_int = (B * 255).astype(np.int8)

start_time = time.time()
C_ints = np.dot(A_int, B_int)  # Matrix multiplication with INT8
elapsed_time_ints = time.time() - start_time
num_integer_ops = 2 * (N ** 3)  # Same number of operations

# Calculate TOPS (integer operations per second)
tops = num_integer_ops / elapsed_time_ints / 1e12  # Convert to TeraOps

# Print results
print(f"Floating-point FLOPS: {flops:.2e} FLOPS")
print(f"Integer TOPS: {tops:.2f} TOPS")
