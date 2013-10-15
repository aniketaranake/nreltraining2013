__all__ = ['BEMComponent']

from math import pi, cos, sin, tan
import numpy as np

from openmdao.main.api import Component, Assembly, VariableTree
from openmdao.lib.datatypes.api import Float, Int, Array, VarTree

import sys
sys.path.append('../CCblade/src/')
from ccblade import CCBlade, CCAirfoil

def BEMComponent(Component):
    '''Component to evaluate wind turbine performance for a given twist and taper'''

    r = np.array([2.8667, 5.6000, 8.3333, 11.7500, 15.8500, 19.9500, 24.0500,
                  28.1500, 32.2500, 36.3500, 40.4500, 44.5500, 48.6500, 52.7500,
                  56.1667, 58.9000, 61.6333])
    Rhub = 1.5
    Rtip = 63.

    def __init__(self, n_elements=17):

        # Inputs
        self.add('theta',  Array(np.zeros([n_elements]), size=[n_elements], iotype="in"))
        self.add('chord',  Array(np.zeros([n_elements]), size=[n_elements], iotype="in"))

        # Outputs
        self.add('Cp', Float(), iotype="out")

        # Construct turbine ------------------------------------------------------------------------------------------------
        #    def __init__(self, r, chord, theta, af, Rhub, Rtip, B=3, rho=1.225, mu=1.81206e-5,
        #                 precone=0.0, tilt=0.0, yaw=0.0, shearExp=0.2, hubHt=80.0,
        #                 nSector=8, precurve=None, precurveTip=0.0, presweep=None, presweepTip=0.0,
        #                 tiploss=True, hubloss=True, wakerotation=True, usecd=True, iterRe=1, derivatives=False):
        #-------------------------------------------------------------------------------------------------------------------
        self.blade = CCBlade(r, chord, theta, af, Rhub, Rtip)


if __name__=="__main__"


    chord = np.array([3.542, 3.854, 4.167, 4.557, 4.652, 4.458, 4.249, 4.007, 3.748,
                      3.502, 3.256, 3.010, 2.764, 2.518, 2.313, 2.086, 1.419])
    theta = np.array([13.308, 13.308, 13.308, 13.308, 11.480, 10.162, 9.011, 7.795,
                      6.544, 5.361, 4.188, 3.125, 2.319, 1.526, 0.863, 0.370, 0.106])
