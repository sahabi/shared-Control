from math import log
import maxflow
import sys

def list_2D(columns,rows,val=0):
	return [[val for x in range(columns)] for y in range(rows)]

def denoise(evals,param_arg,betaswtr_arg):

	param = param_arg
	betaswtr = betaswtr_arg
	ROW = 2

	w1 = -log(1 - param)
	w2 = -log(param)
	wh = -log(0.5)

	evals.sort()
	#print evals

	evaluations = evals

	nCols = 0
	NEVALS = len(evals)
	g = maxflow.Graph[float](200, 200)

	imageBuilder = list_2D(NEVALS, ROW)
	imageBuilder_ts = list_2D(NEVALS, ROW+1)
	imageBuilder_times = list_2D(NEVALS, ROW)

	for i in range(ROW):
		for j in range(NEVALS):
			imageBuilder[i][j] = -1
			
	for i in range(ROW+1):
		for j in range(NEVALS):
			imageBuilder_ts[i][j] = -1

	for i, row in enumerate(evals):
		state = row[0]
		action = row[1]
		evaluation = row[2]
		ts = row[3]
		time_step = row[4]
		if i == 0:
			currentState = state
			currentCol = 0
			imageBuilder[1-action][currentCol] = evaluation
			imageBuilder_ts[1-action][currentCol] = evaluation
			imageBuilder_times[1-action][currentCol] = [evaluation, time_step]
			imageBuilder_ts[2][currentCol] = ts
			#print imageBuilder_ts
			actionLast = action
		else:
			if state == currentState:
				if action == actionLast:
					action = 1 - action
					evaluation = 1 - evaluation
				if imageBuilder[1-action][currentCol] == -1:
					imageBuilder[1-action][currentCol] = evaluation
					imageBuilder_ts[1-action][currentCol] = evaluation
					imageBuilder_times[1-action][currentCol] = [evaluation, time_step]
					imageBuilder_ts[2][currentCol] = ts
				else:
					currentCol = currentCol + 1
					imageBuilder[1-action][currentCol] = evaluation
					imageBuilder[action][currentCol] = -1
					imageBuilder_ts[1-action][currentCol] = evaluation
					imageBuilder_ts[action][currentCol] = -1
					imageBuilder_times[1-action][currentCol] = [evaluation, time_step]
					imageBuilder_times[action][currentCol] = [-1, time_step]
					imageBuilder_ts[2][currentCol] = ts
			else:
				currentState = state
				currentCol = currentCol + 1
				imageBuilder[1 - action][currentCol] = evaluation
				imageBuilder[action][currentCol] = -1
				imageBuilder_ts[1 - action][currentCol] = evaluation
				imageBuilder_ts[action][currentCol] = -1
				imageBuilder_times[1 - action][currentCol] = [evaluation, time_step]
				imageBuilder_times[action][currentCol] = [-1, time_step]
				imageBuilder_ts[2][currentCol] = ts
			actionLast = action

	for i in range(len(evals)):
		if imageBuilder[0][i] != -1:
			imageBuilder[0][i] = 1 - imageBuilder[0][i]
			#imageBuilder_ts[0][i][0] = 1 - imageBuilder_ts[0][i][0]
######
	currentCol += 1
	deNoiseImage = list_2D(currentCol, ROW)
	finalDeNoiseImage = list_2D(currentCol, ROW)
	count = 0
	nNew = currentCol
	imageBuilder_ts = imageBuilder_ts[:][0:nNew+1]
	imageBuilder = imageBuilder[:][0:nNew+1]	

	# for i in range(ROW):
	# 	for j in range(nNew):
	# 		g.add_nodes(1)
	# 		if imageBuilder[i][j] == -1:
	# 			if imageBuilder[1-i][j] == 0:
	# 				g.add_tedge(j+i*nNew,w1,w2)
	# 			else:
	# 				g.add_tedge(j+i*nNew,w2,w1)
	# 		else:
	# 			if imageBuilder[i][j] == 0:
	# 				g.add_tedge(j+i*nNew,w1,w2)
	# 			else:
	# 				g.add_tedge(j+i*nNew,w2,w1)

	for i in range(ROW):
		for j in range(nNew):
			g.add_nodes(1)
			if imageBuilder[i][j] == 0:
				g.add_tedge(j+i*nNew,w1,w2)
			elif imageBuilder[i][j] == 1:
				g.add_tedge(j+i*nNew,w2,w1)
			else:
				g.add_tedge(j+i*nNew,wh,wh)


	for i in range(nNew - 1):
		g.add_edge( i,i+1, betaswtr, betaswtr );
		g.add_edge( i+nNew,i+1+nNew, betaswtr, betaswtr );
		g.add_edge( i, i+nNew, betaswtr, betaswtr );	

	g.add_edge( nNew-1,2*nNew-1, betaswtr, betaswtr );

	flow = g.maxflow();

	for i in range(ROW):
		for j in range(nNew):

			if (g.get_segment(j+i*nNew)):
				deNoiseImage[i][j] = 0
			else:
				deNoiseImage[i][j] = 1

	for j in range(nNew):
		finalDeNoiseImage[0][j] = 1 - deNoiseImage[0][j]
		finalDeNoiseImage[1][j] = deNoiseImage[1][j]

	return (flow, deNoiseImage, finalDeNoiseImage, imageBuilder_ts, imageBuilder, imageBuilder_times)

if __name__ == "__main__":
	denoise(getEvals(),.3,.5)