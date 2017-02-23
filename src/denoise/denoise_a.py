from math import log
import maxflow
import sys
from Moutils.Moutils import list_2D

def denoise(evals,param_arg,betaswtr_arg):

	param = param_arg
	betaswtr = betaswtr_arg
	ROW = 1

	w1 = -log(1 - param)
	w2 = -log(param)
	wh = -log(0.5)

	evals.sort()
	#print evals
	evals = sorted(evals, key=lambda eval: eval[1],reverse=True)
	evals = sorted(evals, key=lambda eval: eval[0])
	print 'sorted',evals

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

	currentCol = 0
	for i, row in enumerate(evals):
		state = row[0]
		action = row[1]
		evaluation = row[2]
		ts = row[3]
		time_step = row[4]
		imageBuilder[0][currentCol] = action
		imageBuilder_ts[0][currentCol] = action
		imageBuilder_times[0][currentCol] = [action, time_step]
		imageBuilder_ts[1][currentCol] = state		
		currentCol+=1
		
		
		#imageBuilder_ts[0][i][0] = 1 - imageBuilder_ts[0][i][0]
	deNoiseImage = list_2D(currentCol, ROW)
	finalDeNoiseImage = list_2D(currentCol, ROW)
	count = 0
	nNew = currentCol
	imageBuilder_ts = imageBuilder_ts[:][0:nNew+1]
	print 'IB 1st:',imageBuilder_ts[0]
	print 'IB 2nd:',imageBuilder_ts[1]
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
		if imageBuilder_ts[1][i] == imageBuilder_ts[1][i+1]:
			g.add_edge( i,i+1, betaswtr*1.5, betaswtr*1.5 )
		else:
			g.add_edge( i,i+1, betaswtr, betaswtr )

	flow = g.maxflow();

	for i in range(ROW):
		for j in range(nNew):

			if (g.get_segment(j+i*nNew)):
				deNoiseImage[i][j] = 0
			else:
				deNoiseImage[i][j] = 1

	for j in range(nNew):
		finalDeNoiseImage[0][j] = deNoiseImage[0][j]
	print "FDNI",finalDeNoiseImage

	return (flow, deNoiseImage, finalDeNoiseImage, imageBuilder_ts, imageBuilder, imageBuilder_times)

if __name__ == "__main__":
	denoise(getEvals(),.3,.5)