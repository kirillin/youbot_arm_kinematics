import math

PI = math.pi


class Config:

    DH_A = (0.033, 0.155, 0.135, 0, 0)
    DH_ALPHA = (PI/2, 0, 0, PI/2, 0)
    DH_D = (0.147, 0, 0, 0, 0.218)
    DH_THETA = (PI*169.0/180.0, PI*65.0/180.0+PI/2, -PI*146.0/180.0, PI*102.5/180.0+PI/2, PI*167.5/180.0)  #theta = DH_THETA - q

    def __init__(self):
        # TODO init from params?
        print('im do nothing')
