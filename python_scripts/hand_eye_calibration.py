import copy
import logging
import sys

import evo.core.lie_algebra as lie
from evo.core import trajectory
from evo.tools import plot, file_interface, log

import numpy as np
import matplotlib.pyplot as plt

logger = logging.getLogger("evo")
log.configure_logging(verbose=True)

traj_est = file_interface.read_kitti_poses_file("../../dataset/lidar_traj/kitti_traj_left.txt")
traj_ref = file_interface.read_kitti_poses_file(
    "../../dataset/lidar_traj/kitti_traj_front.txt")

logger.info("\nUmeyama alignment without scaling")
traj_est_aligned = copy.deepcopy(traj_est)
traj_est_aligned.align(traj_ref)

logger.info("\nUmeyama alignment with scaling")
traj_est_aligned_scaled = copy.deepcopy(traj_est)
traj_est_aligned_scaled.align(traj_ref, correct_scale=True)

logger.info("\nUmeyama alignment with scaling only")
traj_est_aligned_only_scaled = copy.deepcopy(traj_est)
traj_est_aligned_only_scaled.align(traj_ref, correct_only_scale=True)

fig = plt.figure(figsize=(8, 8))
plot_mode = plot.PlotMode.yz

ax = plot.prepare_axis(fig, plot_mode, subplot_arg=221)
plot.traj(ax, plot_mode, traj_ref, '--', 'gray')
plot.traj(ax, plot_mode, traj_est, '-', 'blue')
fig.axes.append(ax)
plt.title('not aligned')

ax = plot.prepare_axis(fig, plot_mode, subplot_arg=222)
plot.traj(ax, plot_mode, traj_ref, '--', 'gray')
plot.traj(ax, plot_mode, traj_est_aligned, '-', 'blue')
fig.axes.append(ax)
plt.title('$\mathrm{SE}(3)$ alignment')

ax = plot.prepare_axis(fig, plot_mode, subplot_arg=223)
plot.traj(ax, plot_mode, traj_ref, '--', 'gray')
plot.traj(ax, plot_mode, traj_est_aligned_scaled, '-', 'blue')
fig.axes.append(ax)
plt.title('$\mathrm{Sim}(3)$ alignment')

ax = plot.prepare_axis(fig, plot_mode, subplot_arg=224)
plot.traj(ax, plot_mode, traj_ref, '--', 'gray')
plot.traj(ax, plot_mode, traj_est_aligned_only_scaled, '-', 'blue')
fig.axes.append(ax)
plt.title('only scaled')

fig.tight_layout()
plt.show()