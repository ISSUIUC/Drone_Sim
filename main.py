import matplotlib.pyplot as plt
import numpy as np
import numerical_integration_methods
import drone_sim

# Initialize your variables with sample data for testing
u0_b_mps = np.array([0.0])
v0_b_mps = np.array([0.0])
w0_b_mps = np.array([0.0])
p0_b_rps = np.array([0.0])
q0_b_rps = np.array([0.0])
r0_b_rps = np.array([0.0])
phi0_rad = np.array([0.0])
theta0_rad = np.array([0.0])
psi0_rad = np.array([0.0])
p10_n_m = np.array([0.0])
p20_n_m = np.array([0.0])
p30_n_m = np.array([0.0])

x0 = np.array([u0_b_mps, v0_b_mps, w0_b_mps, p0_b_rps, q0_b_rps, r0_b_rps, phi0_rad, theta0_rad, psi0_rad, p10_n_m, p20_n_m, p30_n_m])
x0 = x0.transpose()
nx0 = x0.size
t0_s = 0.0
tf_s = 10.0
h_s = 0.01
t_s = np.arange(t0_s, tf_s + h_s, h_s)
nt_s = t_s.size
x = np.empty((nx0, nt_s), dtype=float)
x[:,0] = x0.flatten()  # Make sure x0 is 1D
print(x.shape)

# Replace 'main' with the actual function you want to use from drone_sim
t_s, x = numerical_integration_methods.forward_euler(drone_sim.main, t_s, x, h_s)

# Create subplots and set layout
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 6))

# Plot line 1 on first subplot
ax1.plot(t_s, x[0, :], label='Line 1')
ax1.set_xlabel('Time (seconds)')
ax1.set_ylabel('Line 1 Value')
ax1.set_title('Line 1')
ax1.grid(True)

# Plot line 2 on second subplot
ax2.plot(t_s, x[1, :], label='Line 2')
ax2.set_xlabel('Time (seconds)')
ax2.set_ylabel('Line 2 Value')
ax2.set_title('Line 2')
ax2.grid(True)

# 1 row, 2 columns
plt.show()
