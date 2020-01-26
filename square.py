import numpy as np
class square:
    def __init__(self,imageCs,realCs,l,_kc,ID):
        self.imageCorners = imageCs
        self.realCorners = realCs
        self.length = l
        self.kc = _kc
        self.id = ID
    def setImageCorners(self,corners):
        self.imageCorners = corners
    def setRealCorners(self,corners):
        self.realCorners = corners
    