class SubdivisionPathIterator():

    def __init__(self, local_path):
        self.segment_queue=[local_path]

    def __iter__(self):
        return self

    def __next__(self):
        if len(self.segment_queue) == 0:
            raise StopIteration
        else:
            segment=self.segment_queue.pop(0)
            m_idx=int(len(segment) / 2)
            s1=segment[:m_idx]
            s2=segment[m_idx + 1:]
            if len(s1) > 0:
                self.segment_queue.append(s1)
            if len(s2) > 0:
                self.segment_queue.append(s2)
            if len(segment) > 1:
                return segment[m_idx]
            elif len(segment) == 1:
                return segment[0]

    next=__next__  # python2.x compatibility.


class IncrementalEvaluation():

    def __init__(self, eval_fn):
        self.eval_fn=eval_fn

    def evaluate(self, local_path):
        for point in local_path:
            if not self.eval_fun(point):
                return False
        return True


class SubdividionEvaluation():

    def __init__(self, eval_fn):
        self.eval_fn=eval_fn

    def evaluate(self, local_path):
        for point in SubdivisionPathIterator(local_path):
            if not self.eval_fun(point):
                return False
        return True