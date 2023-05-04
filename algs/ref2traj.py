# Ref2Traj

import numpy as np
from numpy.linalg import norm

def get_xref(agent_type, p1, p2, times):
	qref = [] # append the states to this list
	t1, t2 = times[0], times[-1]
	mx = p2[0] - p1[0]
	bx = p1[0]

	my = p2[1] - p1[1]
	by = p1[1]
	if agent_type == 'unicycle':
		theta = np.arctan2((np.array(p2) - np.array(p1))[1], (np.array(p2) - np.array(p1))[0])
		for time in times:
			xref = mx*((time - t1) / (t2 - t1)) + bx
			yref = my*((time - t1) / (t2 - t1)) + by
			qref.append([xref, yref, theta])
	else:
		for time in times:
			xref = mx*((time - t1) / (t2 - t1)) + bx
			yref = my*((time - t1) / (t2 - t1)) + by
			qref.append([xref, yref])

	return qref

def get_uref(agent_type, p1, p2, times):
	if agent_type == 'unicycle':
		dist = norm(np.array(p2) - np.array(p1))
		v_ref = dist / (times[-1] - times[0])
		uref = []
		for _ in times:
			vref = v_ref
			wref = 0
			uref.append([vref, wref])
	else: # for single integrator, needs to be checked
		dist = norm(np.array(p2) - np.array(p1))
		v_ref = dist / (times[-1] - times[0]) # constant velocity
		uref = []
		for _ in times:
			vxref = v_ref
			vyref = vxref
			uref.append([vxref, vyref])
	return uref

def ref2traj(agent_types,ma_nodes):
	dim = len(ma_nodes[0][0])-1
	agent_num = len(ma_nodes)

	# compute step size
	step_num = 10000
	max_t = max([ma_nodes[idx][-1][0] for idx in range(agent_num)])

	step_size = max_t / step_num

	ref_trajs = []
	# calculate controller
	for idx in range(agent_num):
		agent_type = agent_types[idx]
		ref_traj = []
		nodes = ma_nodes[idx]
		for j in range(len(nodes) - 1):
			s1, s2 = nodes[j:j+2]
			t1, t2 = s1[0], s2[0]
			p1, p2 = s1[1:], s2[1:]
			times = np.arange(t1, t2, step_size)
			if times != []:
				qref = get_xref(agent_type, p1, p2, times)
				uref = get_uref(agent_type, p1, p2, times)
				ref_traj.append([times, qref, uref])
		# keep still
		times = np.arange(ma_nodes[idx][-1][0], max_t, step_size)
		p = ma_nodes[idx][-1][1:]
		if times != []:
			qref = get_xref(agent_type, p, p, times)
			uref = get_uref(agent_type, p, p, times)
			ref_traj.append([times, qref, uref])
		ref_trajs.append(ref_traj)

	return ref_trajs
