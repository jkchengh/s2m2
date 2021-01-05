from algs.decentralized import decentralized_algo
from algs.ref2traj import ref2traj
from algs.xref import get_gb_xref
from problems.util import *
from timeit import *
from util import *
from viz.util import *
import os

# problem file configuration
A_rect = np.array([[-1, 0],
                   [1, 0],
                   [0, -1],
                   [0, 1]])

min_x, max_x, min_y, max_y = 0, 20, 0, 20
limits = [[min_x, max_x], [min_y, max_y]]

b1 = np.array([[0], [5], [-9], [11]])
b2 = np.array([[-7], [9], [-9], [11]])
b3 = np.array([[-11], [13], [-9], [11]])
b4 = np.array([[-15], [20], [-9], [11]])
Obstacles = [(A_rect, b1), (A_rect, b2), (A_rect, b3), (A_rect, b4)]

size = 0.4
velocity = 1
k = [1, 1, 1]
models = [Car(size, velocity, k)]
N_min, N_max = 3, 4

paths = {}
# run pairwise routing
time = 3
r = time * velocity
lG = [1, 1]
for i in range(min_x, max_x, 1):
    for j in range(min_y, max_y, 1):
        Thetas = [[i + 0.5, j + 0.5]]
        min_p = max(min_x, min(max_x, i - r))
        max_p = max(min_x, min(max_x, i + r))
        min_q = max(min_y, min(max_y, j - r))
        max_q = max(min_y, min(max_y, j + r))
        for p in range(min_p, max_p, 1):
            for q in range(min_q, max_q, 1):
                pG = [p + 0.5, q + 0.5]
                b = np.array([[-(pG[0] - lG[0])], [pG[0] + lG[0]], [-(pG[1] - lG[1])], [pG[1] + lG[1]]])
                Goals = [[A_rect, b]]
                path = get_gb_xref(models, Thetas, Goals, limits, Obstacles, [], N_min, N_max, min_dur = 0)
                if path == None: print("Run (%s,%s)->(%s,%s) No Path"%(i, j, p, q))
                else:
                    print("Run (%s,%s)->(%s,%s) Find Path with Makepspan %s"%(i, j, p, q, path[0][-1][0]))
                    if path[0][-1][0] <= time: paths["(%s,%s)->(%s,%s)"%(i, j, p, q)] = path[0]

plt.close('all')
fig, axes = plot_env(limits, Obstacles)

# plot trajs
Theta_cells = [[4,5], [5,2]]
Goal_cells = [[6,4], [4,4]]
Thetas = [[cell[0]+0.5, cell[1]+0.5] for cell in Theta_cells]
models = models * len(Theta_cells)
refs = [paths["(%s,%s)->(%s,%s)"%(Theta_cells[i][0],Theta_cells[i][1], Goal_cells[i][0],Goal_cells[i][1])]
        for i in range(len(Thetas))]
ma_segs = ref2traj(refs)
paths = extract_paths(models, Thetas, ma_segs)
agent_num = len(paths)
print(agent_num)
for idx in range(agent_num):
    ref_x, ref_y, _, tru_x, tru_y, _, _ = paths[idx]
    ref_x, ref_y, _, tru_x, tru_y, _, _ = paths[idx]
    axes.plot(ref_x, ref_y, color='k', linewidth = 1, linestyle = 'dashed')
    axes.plot(tru_x, tru_y, color='purple', linewidth = 1, linestyle='dashed',)
# plot grid
for i in range(min_x, max_x+1, 1): plt.plot([i, i], [min_y, max_y], color = 'gray', linewidth = 0.3, linestyle = 'dashed')
for p in range(min_y, max_y+1, 1): plt.plot([min_x, max_x], [p, p], color = 'gray', linewidth = 0.3, linestyle = 'dashed')

fig.show()
file = os.path.abspath("conges.pdf")
fig.savefig(file, bbox_inches='tight', pad_inches = 0)