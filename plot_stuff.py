import matplotlib.pyplot as plt
import numpy as np
import time

def get_encoder_time():
	# open the driver
	driver_file = open("/dev/encoder_driver")
	# read from the driver
	line = driver_file.readline()
	# convert to an int
	line = line.strip()
	value = int(line)
	# close driver
	driver_file.close()
	# return the value
	return value

num_samples = 500
window_len = 10
results = []
avg = []

for i in range(num_samples):
    new_time = 10000 / get_encoder_time()
    results.append(new_time)

    if i == 0:
        window = np.full(window_len, new_time)
    else:
        window[i % window_len] = new_time
    
    avg.append(np.average(window))

    time.sleep(0.01)


fig, ax1 = plt.subplots()
t_ax = np.arange(len(results))
ax1.plot(t_ax, results, '-', label="Samples")
ax2 = ax1.twinx()
ax2.plot(t_ax, avg, '--r', label="Avg")

ax1.set_xlabel("loop")
ax1.set_ylabel("Sample val")
# ax2.set_ylim(-90, 90)
ax2.set_ylabel("Avg val")

plt.title("Samples over loops")
fig.legend()
fig.tight_layout()
plt.savefig("plot_plot.png")

plt.clf()