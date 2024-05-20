import io, matplotlib.pyplot as plt

file = open("test-output/output.txt", "r")

lines = file.readlines()

ctrl_parsed = []
parsed = []
for line in lines:
  split = line.split(" ")
  if len(split) >= 2:
    if split[0] == "ctrl":
      ctrl_parsed.append([float(split[1]), float(split[2])])
    else:
      parsed.append([float(split[0]), float(split[1])])

spline_xs = []
spline_ys = []
for item in parsed:
  spline_xs.append(item[0])
  spline_ys.append(item[1])

ctrl_xs = []
ctrl_ys = []
for item in ctrl_parsed:
  ctrl_xs.append(item[0])
  ctrl_ys.append(item[1])

fig, ax = plt.subplots()
ax.plot(spline_xs, spline_ys)
# ax.scatter(ctrl_xs, ctrl_ys)

# ax.plot([0.0, 1], [0.0, 1])
# ax.plot([0.5, 1], [0.25, 2])
ax.grid()
plt.show()