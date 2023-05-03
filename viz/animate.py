import matplotlib.animation as animation
from viz.util import *
import os
from models.agent import *

def animate_results(models, agent_types, limits, obstacles, Thetas, goals, ma_segs, name):
    agent_num = len(models)
    dim = len(limits)
    fig, axes = plot_env(limits, obstacles)
    plot_goals(goals)

    interval = 20
    total_frames = 500
    total_time = max([ma_segs[idx][-1][0][-1] for idx in range(agent_num)])

    paths = extract_paths(models, agent_types, Thetas, ma_segs)
    ref_patches = []
    tru_patches = []
    err_patches = []
    for idx in range(agent_num):
        agent_type = agent_types[idx]
        if agent_type == 'unicycle':
            ref_x, ref_y, _, tru_x, tru_y, _, times = paths[idx]
            ref_patch = plt.Rectangle((ref_x[0], ref_y[0]), models[idx].size*2, models[idx].size*2, fc='k')
            tru_patch = plt.Rectangle((tru_x[0], tru_y[0]), models[idx].size*2, models[idx].size*2, fc='navy', alpha = 0.5)
            err_patch = plt.Rectangle((ref_x[0], ref_y[0]), models[idx].size*2, models[idx].size*2, fc='lightsteelblue', alpha = 0.4)
        else:
            ref_x, ref_y, tru_x, tru_y, times = paths[idx]
            ref_patch = plt.Circle((ref_x[0], ref_y[0]), 0, fc='k')
            tru_patch = plt.Circle((tru_x[0], tru_y[0]), models[idx].size, fc='navy', alpha = 0.7)
            err_patch = plt.Circle((ref_x[0], ref_y[0]), models[idx].size, fc='lightsteelblue', alpha = 0.4)

        ref_patches.append(ref_patch)
        tru_patches.append(tru_patch)
        err_patches.append(err_patch)
    # init
    def init():
        for idx in range(agent_num):
            agent_type = agent_types[idx]
            if agent_type == 'unicycle':
                ref_x, ref_y, _, tru_x, tru_y, _, times = paths[idx]
                ref_patches[idx].set_xy((ref_x[0],ref_y[0]))
                tru_patches[idx].set_xy((tru_x[0],tru_y[0]))
                err_patches[idx].set_xy((ref_x[0],ref_y[0]))
            else:
                ref_x, ref_y, tru_x, tru_y, times = paths[idx]
                ref_patches[idx].center = (ref_x[0], ref_y[0])
                tru_patches[idx].center = (tru_x[0], tru_y[0])
                err_patches[idx].center = (ref_x[0], ref_y[0])

        for patch in ref_patches + tru_patches + err_patches: axes.add_patch(patch)
        return ref_patches + tru_patches + err_patches
    # animate
    tpf = total_time / total_frames

    def animate(f):
        for idx in range(agent_num):
            agent_type = agent_types[idx]
            if agent_type == 'unicycle':
                ref_x, ref_y, _, tru_x, tru_y, _, times = paths[idx]
                step = 0
                while (step < len(times) - 1) and (times[step] < tpf * f):
                    step = step + 1
                ref_patches[idx].set_xy((ref_x[step],ref_y[step]))
                tru_patches[idx].set_xy((tru_x[step],tru_y[step]))
                err_patches[idx].set_xy((ref_x[step],ref_y[step]))
            else:
                ref_x, ref_y, tru_x, tru_y, times = paths[idx]
                step = 0
                while (step < len(times) - 1) and (times[step] < tpf * f):
                    step = step + 1
                ref_patches[idx].center = (ref_x[step], ref_y[step])
                tru_patches[idx].center = (tru_x[step], tru_y[step])
                err_patches[idx].center = (ref_x[step], ref_y[step])

            if step == len(ref_x) - 1: error = models[idx].size
            else: error  = (models[idx].size + models[idx].bloating(step))
            err_patches[idx].width = 2 * error
            err_patches[idx].height = 2 * error
        return ref_patches + tru_patches + err_patches


    ani = animation.FuncAnimation(fig, animate, frames = total_frames, init_func=init,
                                    blit=True, interval = interval)

    path = os.path.abspath("results/%s.mp4" % (name))
    # ani.save(path, writer='ffmpeg')
    plt.show()
