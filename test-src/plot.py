import io, matplotlib.pyplot as plt

file = open("test-output/output.txt", "r")

def plot(ax, parsed_list, style = '-'):
  xs = []
  ys = []
  for item in parsed_list:
    xs.append(item[0])
    ys.append(item[1])
  l, = ax.plot(xs, ys)
  l.set_linestyle(style)

def scatter(ax, parsed_list):
  xs = []
  ys = []
  for item in parsed_list:
    xs.append(item[0])
    ys.append(item[1])
  ax.scatter(xs, ys)

lines = file.readlines()

ctrl_parsed = []
spline_parsed = []
pursuit_parsed = []
for line in lines:
  split = line.split(" ")
  if len(split) >= 2:
    if split[0] == "ctrl":
      ctrl_parsed.append([float(split[1]), float(split[2])])
    elif split[0] == "spline":
      spline_parsed.append([float(split[1]), float(split[2])])
    elif split[0] == "pursuit":
      pursuit_parsed.append([float(split[1]), float(split[2])])

fig, ax = plt.subplots()
scatter(ax, ctrl_parsed)
plot(ax, spline_parsed, "-.")
plot(ax, pursuit_parsed)
# ax.scatter(ctrl_xs, ctrl_ys)

# ax.plot([0.0, 1], [0.0, 1])
# ax.plot([0.5, 1], [0.25, 2])
ax.grid()
plt.show()