import numpy as np
from sklearn.neighbors import KDTree

class NearestNeighbors():

	def __init__(self, X, k=3, model_type="KDTree", model_args=[], model_kwargs={}):
		self.X = X
		print(self.X)
		self.k = k
		self.model_type = model_type
		self.model_args = model_args
		self.model_kwargs = model_kwargs
		self._init_model()

	def _init_model(self):
		if self.model_type == "KDTree":
			self.model = KDTree(self.X, *self.model_args, **self.model_kwargs)

	def append(self, x):
		self.X.append(x)
		self._init_model()

	def query(self, x):
		return self.model.query(x, k=self.k) 

