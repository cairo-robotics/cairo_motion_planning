"""
Class for different sampling strategies.
"""
import random


class UniformSampler():

    """
    Uniformly samples at random each dimension given the provided limits.

    Attributes:
        dimension_limits (list): list of tuples indicated limited range of each dimension.
    """

    def __init__(self, dimension_limits):
        """
        Args:
        dimension_limits (list): list of tuples indicated limited range of each dimension.
        """
        self.dimension_limits = dimension_limits

    def sample(self):
        """
        Samples a random sample.

        Returns:
            list: Random sample.
        """
        return [random.uniform(limit[0], limit[1]) for limit in self.dimension_limits]


class GaussianSampler():

    """
    TODO: Still a work in progress. Might require a distance function and awareness of objects.

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
