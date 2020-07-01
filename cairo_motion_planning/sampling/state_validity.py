"""
Interfaces for state validity checking.
"""

class StateValidityChecker(): #TODO: Fix this disgusting typo

    """
    This StateValidityChecker class expects a collision checking function, a self collision checking function, 
    and list of other validity functions (constraints etc,.). It is up to the developer to inject these functions
    as deemed appropriate.

    Attributes:
        col_func (func): External collisions between moving objects and environment object.
        self_col_func (func): Self-collision checking function
        validity_funcs (list): List of other state validating functions.
    """

    def __init__(self, self_col_func, col_func=None, validity_funcs=None):
        """
        Args:
            col_func (func): External collisions between moving objects and environment object.
            self_col_func (func): Self-collision checking function
            validity_funcs (list): List of other state validating functions.
        """
        self.self_col_func = self_col_func
        self.col_func = col_func
        self.validity_funcs = validity_funcs

    def validate(self, sample):
        """
        Validates a given sample.

        Args:
            sample (list): The sample to validate.

        Returns:
            bool: Whether or not the sample is valid.
        """
        if self.validity_funcs is not None:
            if not all([func(sample) for func in self.validity_funcs]):
                return False
        if not self.self_col_func(sample):
            return False
        if self.col_func is not None and not self.col_func(sample):
            return False
        return True
