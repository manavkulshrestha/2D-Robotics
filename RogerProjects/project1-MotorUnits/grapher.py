import numpy as np
import matplotlib.pyplot as plt

# error = np.loadtxt('eyes.txt')

# plt.plot(np.arange(len(error)), error)
# plt.xlabel('Time (s)')
# plt.ylabel('Error (rad)')

# plt.show()

# data = np.loadtxt('arms.txt')
# q1_err = data[:,1][data[:,0] == 1]
# q2_err = data[:,1][data[:,0] == 2]

# cutoff = 1000

# if len(q1_err) > cutoff:
# 	q1_err = q1_err[:cutoff]

# if len(q2_err) > cutoff:

# 	q2_err = q2_err[:cutoff]

# plt.plot(np.arange(len(q1_err)), q1_err, label='$q_1$')
# plt.plot(np.arange(len(q2_err)), q2_err, label='$q_2$')
# plt.legend()
# plt.xlabel('Time (s)')
# plt.ylabel('Error (rad)')

# plt.show()

data = np.loadtxt('base_b.txt')
data = data[1:1001]
data -= 0.005
data[data < 0] = 0

x, y, t = [data[:,i] for i in range(3)]

plt.plot(np.arange(len(x)), x, label='$x$ (m)')
plt.plot(np.arange(len(y)), y, label='$y$ (m)')
plt.plot(np.arange(len(t)), t, label=r'$\theta$ (rad)')
plt.legend()

plt.xlabel('Time (unit)')
plt.ylabel('Error')

plt.show()