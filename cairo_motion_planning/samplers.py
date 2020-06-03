"""Summary
"""
import random

class UniformSampler():

    """Summary
    
    Attributes:
        dimension_limits (TYPE): Description
    """
    
    def __init__(self, dimension_limits):
        """Summary
        
        Args:
            dimension_limits (TYPE): Description
        """
        self.dimension_limits = dimension_limits

    def sample(self):
        """Summary
        
        Returns:
            TYPE: Description
        """
        return [random.uniform(limit[0], limit[1]) for limit in self.dimension_limits]

class GaussianSampler():

    """Summary
    
    Attributes:
        covariance (TYPE): Description
        mean (TYPE): Description
    """
    
    def __init__(self, mean, covariance):
        """Summary
        
        Args:
            mean (TYPE): Description
            covariance (TYPE): Description
        """
        self.mean = mean
        self.covariance = covariance

    def sample():
        """Summary
        """
        pass




