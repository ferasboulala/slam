import json
import sys
import math

import matplotlib.pyplot as plt

def main():
    content = json.load(sys.stdin)
    plt.figure()
    for epoch in content.values():
        real_position = epoch['real_position']
        xs = epoch['x']
        ys = epoch['y']
        thetas = epoch['theta']

        plt.plot(xs, ys, 'k.', alpha=0.25)
        plt.plot(real_position[0], real_position[1], 'r', marker=(3, 0, math.degrees(real_position[2]) - 90), markersize=10)

    plt.title('2D Position prediction step using Particle Filter')
    plt.xlabel('x')
    plt.ylabel('y')

    plt.show()

if __name__ == '__main__':
    main()
