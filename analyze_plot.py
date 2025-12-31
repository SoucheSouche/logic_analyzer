import matplotlib.pyplot as plt

# Replace 'log.txt' with your file path
filename = "log.txt"

# Lists to store signal data
times = [0]  # cumulative time in microseconds
levels = []

with open("output_1.txt", "r") as f:
    for line in f:
        line = line.strip()
        if not line or ',' not in line:
            continue
        level_str, dt_str = line.split(",")
        level = int(level_str.strip())
        dt = int(dt_str.strip())
        times.append(times[-1] + dt)
        levels.append(level)

# Ensure the levels array aligns with times
# times has one more entry; trim it
times = times[1:]

plt.figure(figsize=(15, 3))
plt.step(times, levels, where='post')
plt.xlabel("Time (µs)")
plt.ylabel("Level")
plt.title("Logic Analyzer Plot")
plt.grid(True)
plt.savefig("logic_plot.png", dpi=300)
print("Plot saved to logic_plot.png")
