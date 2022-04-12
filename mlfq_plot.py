import matplotlib.pyplot as plt
import numpy as np

with open('typescript') as mlfq_output:
    mlfq_lines = mlfq_output.readlines()

processes = {}

for line in mlfq_lines:
    line = line.split()
    if len(line) < 3 or line[0] != 'Process' or line[1][0] != 'P':
        continue
    proc_num = line[1]
    if proc_num not in processes:
        processes[proc_num] = []
    processes[proc_num].append(int(line[4][1:]))


for proc in processes:
    plt.plot(processes[proc], label=proc)
plt.legend()

plt.yticks(range(5))

plt.ylabel('Queue')
plt.xlabel('Time (Ticks)')
plt.show()

