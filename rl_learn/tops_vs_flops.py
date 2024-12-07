import numpy as np
import time

# Define matrix size
N = 1000  # Matrix size (N x N)

# Generate two random matrices for floating-point operations
A_float = np.random.rand(N, N).astype(np.float32)
B_float = np.random.rand(N, N).astype(np.float32)

# Floating-point operations (FLOPS)
start_time = time.time()
C_float = np.dot(A_float, B_float)
elapsed_time_float = time.time() - start_time
num_floating_ops = 2 * (N ** 3)  # Approximation: 2N^3 ops for matrix mult
flops = num_floating_ops / elapsed_time_float

# Generate two random matrices for integer operations
A_int = (A_float * 255).astype(np.int32)
B_int = (B_float * 255).astype(np.int32)

# Integer operations (TOPS)
start_time = time.time()
C_int = np.dot(A_int, B_int)
elapsed_time_int = time.time() - start_time
num_integer_ops = 2 * (N ** 3)  # Same number of operations
tops = num_integer_ops / elapsed_time_int / 1e12  # Convert to TeraOps

# Handle cases where TOPS might still report zero due to scaling
if tops < 1e-6:  # If TOPS is effectively zero, report the raw result in GigaOps
    print(f"Integer Operations: {num_integer_ops / elapsed_time_int / 1e9:.2f} GOPS (Billion Ops/sec)")
else:
    print(f"Integer TOPS: {tops:.2f} TOPS (TeraOps/sec)")

# Print results
print(f"Floating-point FLOPS: {flops:.2e} FLOPS")
