import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('p7.txt')
heading, base_dist, h1_dist, h2_dist = data.T

heading[heading > np.pi] -= 2*np.pi
n = len(data)

plt.plot(np.arange(n), heading, label=r'heading (rad)')
plt.plot(np.arange(n), base_dist, label=r'Base (m)')
plt.plot(np.arange(n), h2_dist, label=r'Left hand (m)')
plt.plot(np.arange(n), h1_dist, label=r'Right hand (m)')
plt.legend()

plt.xlabel('Time (ms)')
plt.ylabel('Heading + Distances from ball')

plt.show()