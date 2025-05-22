import webcolors
import sys
r = int(sys.argv[1])
g = int(sys.argv[2])
b = int(sys.argv[3])
color = webcolors.rgb_to_name((r, g, b))
print(color)