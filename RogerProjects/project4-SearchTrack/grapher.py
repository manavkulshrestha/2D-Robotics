import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('p4.txt')
# data = data[1:1001]

data[data > np.pi] -= 2*np.pi

plt.plot(np.arange(len(data)), data)
plt.legend()

plt.xlabel('Time (ms)')
plt.ylabel('Base Rotation Error (rad)')

plt.show()