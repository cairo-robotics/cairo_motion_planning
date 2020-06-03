import math

def euclidean(v1, v2):
	return math.sqrt(sum([(a - b)**2 for a, b in zip(v1, v2)]))