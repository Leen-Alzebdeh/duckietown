"""
This file manages the exploration of the map, i.e. tries to record the turn history in each of the intersection so to explore least explored path
"""


class ExplorationHistory:
    def __init__(self):
        self.history = {}
        for id in (58, 133, 162, 169, 62, 153):
            self.history[id] = [0, 0, 0]
    
    def getCount(self, id, turn_idx):
        return self.history[id][turn_idx]
    
    def incCount(self, id, turn_idx):
        self.history[id][turn_idx] += 1
    
