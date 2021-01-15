from algs.ref2traj import ref2traj
from algs.xref import get_gb_xref
from problems.util import *
from gurobi import *
from viz.util import *
from networkx import *
import os

def compute_graph(limits, obstacles, models):
    # problem file configuration
    N_min, N_max = 3, 4
    paths = {}
    # run pairwise routing
    time = 3
    r = time * velocity
    lG = [1, 1]
    min_x, max_x = limits[0]
    min_y, max_y = limits[1]
    for i in range(min_x, max_x, 1):
        for j in range(min_y, max_y, 1):
            Thetas = [[i + 0.5, j + 0.5]]
            min_p = max(min_x, min(max_x, i - r))
            max_p = max(min_x, min(max_x, i + r))
            min_q = max(min_y, min(max_y, j - r))
            max_q = max(min_y, min(max_y, j + r))
            for p in range(min_p, max_p, 1):
                for q in range(min_q, max_q, 1):
                    if i != p or j != q:
                        pG = [p + 0.5, q + 0.5]
                        b = np.array([[-(pG[0] - lG[0])], [pG[0] + lG[0]], [-(pG[1] - lG[1])], [pG[1] + lG[1]]])
                        Goals = [[A_rect, b]]
                        path = get_gb_xref(models, Thetas, Goals, limits, Obstacles, [], N_min, N_max, min_dur = 0)
                        # if path == None:
                        #     print("Run (%s,%s)->(%s,%s) No Path"%(i, j, p, q))
                        # else:
                        #     print("Run (%s,%s)->(%s,%s) Find Path with Makepspan %s"%(i, j, p, q, path[0][-1][0]))
                        if path != None and path[0][-1][0] <= time:
                            paths["(%s,%s)->(%s,%s)"%(i, j, p, q)] = path[0]
    return paths



def plot_graph(Theta_cells, Goal_cells, models, paths):
    plt.close('all')
    fig, axes = plot_env(limits, Obstacles)
    # plot trajs
    Thetas = [[cell[0]+0.5, cell[1]+0.5] for cell in Theta_cells]
    models = models * len(Theta_cells)
    refs = [paths["(%s,%s)->(%s,%s)"%(Theta_cells[i][0],Theta_cells[i][1], Goal_cells[i][0],Goal_cells[i][1])]
            for i in range(len(Thetas))]
    ma_segs = ref2traj(refs)
    paths = extract_paths(models, Thetas, ma_segs)
    agent_num = len(paths)
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


def optimize(Theta_cells, Goal_cells, limits, paths):
    min_x, max_x = limits[0]
    min_y, max_y = limits[1]

    model = Model("Model")
    agent_num = len(Theta_cells)
    vars = {}

    # variables
    vars["threshold"] = model.addVar(vtype=GRB.INTEGER, name="threshold", lb = 0, ub = agent_num)
    for k in range(agent_num):
        for e in paths.keys():
            name = "A%s-%s"%(k, e)
            vars[name] = model.addVar(vtype=GRB.BINARY, name=name)

    # objective
    model.setObjective(vars["threshold"], GRB.MINIMIZE)

    # constraints
    for k in range(agent_num):
        theta = Theta_cells[k]
        goal = Goal_cells[k]
        theta_in = incoming_edges(theta, limits, paths)
        theta_out = outcoming_edges(theta, limits, paths)
        goal_in = incoming_edges(goal, limits, paths)
        goal_out = outcoming_edges(goal, limits, paths)
        theta_in_vars = [vars["A%s-%s"%(k,e)] for e in theta_in]
        theta_out_vars = [vars["A%s-%s"%(k,e)] for e in theta_out]
        goal_in_vars = [vars["A%s-%s"%(k,e)] for e in goal_in]
        goal_out_vars = [vars["A%s-%s"%(k,e)] for e in goal_out]
        # constraint incoming and outcoming edges to be one for start and end, respectively
        model.addConstr(LinExpr([1] * len(theta_in_vars), theta_in_vars) == 0, name="A%s-StartIn")
        model.addConstr(LinExpr([1] * len(theta_out_vars), theta_out_vars) == 1, name="A%s-StartOut")
        model.addConstr(LinExpr([1] * len(goal_in_vars), goal_in_vars) == 1, name="A%s-GoalIn")
        model.addConstr(LinExpr([1] * len(goal_out_vars), goal_out_vars) == 0, name="A%s-GoalOut")
        print("theta out", len(theta_out))
        for name in theta_out: print(name)
        print("goal in ", len(goal_in))
        for name in goal_in: print(name)

        # other edges has same numbers of incoming and outcoming edges
        for i in range(min_x, max_x, 1):
            for j in range(min_y, max_y, 1):
                if (not ((i == theta[0]) and (j == theta[1]))) and (not ((i == goal[0]) and (j == goal[1]))):
                    edges_out = outcoming_edges((i,j), limits, paths)
                    edges_in = incoming_edges((i,j), limits, paths)
                    vars_out = [vars["A%s-%s"%(k,e)] for e in edges_out]
                    vars_in = [vars["A%s-%s"%(k,e)] for e in edges_in]
                    model.addConstr(LinExpr([1] * len(vars_out) + [-1] * len(vars_in),
                                            vars_out + vars_in)  == 0)

    # threshold constraint
    for e in paths.keys():
        agents_on_edges_vars = [vars["A%s-%s"%(k,e)] for k in range(agent_num)]
        model.addConstr(LinExpr([1] * agent_num, agents_on_edges_vars) <= vars["threshold"])

    # solve
    model.setParam(GRB.Param.OutputFlag, 0)
    model.optimize()
    print_solution(vars, Theta_cells, Goal_cells, limits, paths)
    model.dispose

def print_solution(vars, starts, goals, limits, paths):
    print("Print Solutions")
    for name in vars.keys(): print(name, " = ", vars[name].x)
    print("Minimized Threshold: %s"%(vars["threshold"].x))
    agent_num = len(starts)
    for k in range(agent_num):
        print("Path for Agent [%s] with Start (%s,%s) and Goal (%s,%s)"%(k,starts[k][0],starts[k][1],
                                                                         goals[k][0],goals[k][1]))
        pos = starts[k]
        chosen_nodes = [pos]
        while (pos[0] != goals[k][0]) or (pos[1] != goals[k][1]):
            nodes = outcoming_nodes(pos, limits, paths)
            # print("Current Position:(%s,%s)"%(pos[0],pos[1]))
            for (i,j) in nodes:
                edge = "(%s,%s)->(%s,%s)"%(pos[0], pos[1], i, j)
                if edge in paths.keys():
                    var = vars["A%s-(%s,%s)->(%s,%s)"%(k,pos[0],pos[1],i,j)]
                    if 0.99 < var.x < 1.01:
                        chosen_nodes.append((i,j))
                        pos = (i,j)
        print("Ｐath:", chosen_nodes)

def incoming_edges(cell, limits, paths):
    min_x, max_x = limits[0]
    min_y, max_y = limits[1]
    edges = []
    for i in range(min_x, max_x, 1):
        for j in range(min_y, max_y, 1):
            edge = "(%s,%s)->(%s,%s)"%(i, j, cell[0], cell[1])
            if edge in paths.keys(): edges.append(edge)
    return edges

def outcoming_edges(cell, limits, paths):
    min_x, max_x = limits[0]
    min_y, max_y = limits[1]
    edges = []
    for i in range(min_x, max_x, 1):
        for j in range(min_y, max_y, 1):
            edge = "(%s,%s)->(%s,%s)"%(cell[0], cell[1], i, j)
            if edge in paths.keys(): edges.append(edge)
    return edges

def incoming_nodes(cell, limits, paths):
    min_x, max_x = limits[0]
    min_y, max_y = limits[1]
    nodes = []
    for i in range(min_x, max_x, 1):
        for j in range(min_y, max_y, 1):
            edge = "(%s,%s)->(%s,%s)"%(i, j, cell[0], cell[1])
            if edge in paths.keys(): nodes.append((i,j))
    return nodes

def outcoming_nodes(cell, limits, paths):
    min_x, max_x = limits[0]
    min_y, max_y = limits[1]
    nodes = []
    for i in range(min_x, max_x, 1):
        for j in range(min_y, max_y, 1):
            edge = "(%s,%s)->(%s,%s)"%(cell[0], cell[1], i, j)
            if edge in paths.keys(): nodes.append((i,j))
    return nodes

# example one
# min_x, max_x, min_y, max_y = 0, 5, 0, 7
min_x, max_x, min_y, max_y = 0, 20, 0, 20
limits = [[min_x, max_x], [min_y, max_y]]
A_rect = np.array([[-1, 0],
                   [1, 0],
                   [0, -1],
                   [0, 1]])

b1 = np.array([[0], [5], [-9], [11]])
b2 = np.array([[-7], [9], [-9], [11]])
b3 = np.array([[-11], [13], [-9], [11]])
b4 = np.array([[-15], [20], [-9], [11]])
Obstacles = [(A_rect, b1), (A_rect, b2), (A_rect, b3), (A_rect, b4)]
size = 0.4
velocity = 1
k = [1, 1, 1]
models = [Car(size, velocity, k)]
# plan path
paths = compute_graph(limits, Obstacles, models)
# plot
Theta_cells = [(3, 3), (2, 2)]
Goal_cells = [(3, 5), (4, 3)]
plot_graph(Theta_cells, Goal_cells, models, paths)
# MILP
Theta_cells = [(3, 3), (2, 2)]
Goal_cells = [(3, 6), (4, 3)]
optimize(Theta_cells, Goal_cells, limits, paths)