"""Summary
"""

class StateValidyChecker():

    """Summary

    Attributes:
        col_func (TYPE): Description
        self_col_func (TYPE): Description
        validity_funcs (TYPE): Description
    """

    def __init__(self, self_col_func, col_func=None, validity_funcs=None):
        """Summary

        Args:
            self_col_func (TYPE): Description
            col_func (TYPE): Description
            validity_funcs (None, optional): Description
        """
        self.self_col_func = self_col_func
        self.col_func = col_func
        self.validity_funcs = validity_funcs

    def validate(self, sample):
        if self.validity_funcs is not None:
            if not all([func(sample) for func in self.validity_funcs]):
                return False
        if not self.self_col_func(sample):
            return False
        if self.col_func is not None and not self.col_func(sample):
            return False
        return True
