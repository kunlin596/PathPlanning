#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
import json

if __name__ == '__main__':
  with open('../build/traj1d_time_exp.json', 'r') as f:
    j = json.load(f)

    all_polys = []
    all_times = []
    accu_time = 0.0
    names = ['position', 'velocity', 'accelaration', 'jerk', 'snap', 'crackle']
    for index, (k, w) in enumerate(j.items()):
      # plt.figure(index + 1)
      # plt.suptitle(f'{index + 1}')
      plt.ion()
      plt.show()
      polys = []
      for i in range(6):
        polys.append(np.polynomial.Polynomial(w[f'func{i}']))
      all_polys.append(polys)

      t = w['time']
      all_times.append(t)

      curr_time = 0.0;
      values = []
      while curr_time < (t + 1e-6):
        values.append([
          poly(curr_time) for poly in polys
        ])
        curr_time += 0.02
      values = np.array(values)

      for i in range(6):
        plt.subplot(3, 2, i + 1)
        plt.title(names[i])
        plt.plot(
          np.arange(
            accu_time,
            accu_time + t + 1e-6,
            0.02),
          values[:, 5 - i],
          linewidth=0.5)

      # plt.legend()
      # accu_time += t


    from IPython import embed
    embed()
