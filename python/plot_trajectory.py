#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
import json

if __name__ == '__main__':
  with open('../build/traj1d_time_1.000.json', 'r') as f:
    j = json.load(f)

    for index, (k, w) in enumerate(j.items()):
      plt.figure(index + 1)
      plt.ion()
      plt.show()
      polys = []
      for i in range(6):
        polys.append(np.polynomial.Polynomial(w[f'func{i}']))
      # traj_polys.append((polys, w['time']))
      t = w['time']

      curr_time = 0.0;
      values = []
      while curr_time < t + 1e-6:
        values.append([
          poly(curr_time) for poly in polys
        ])
        curr_time += 0.02
      values = np.array(values)

      for i in range(6):
        plt.subplot(6, 1, i + 1)
        plt.plot(values[:, 5 - i])
      plt.legend()

    from IPython import embed
    embed()
