# Python imports
import sys
import numpy as np

# Imports for our model
from BEMComponent import BEMComponent, BEMAssembly
from su2_caller import SU2_CLCD_Fake, SU2_CLCD

# OpenMDAO imports
from openmdao.main.api import Component, Assembly, VariableTree
from openmdao.lib.datatypes.api import Float, Int, Array, VarTree
from openmdao.lib.drivers.api import SLSQPdriver 

#from custom_opt import SLSQPdriver

from openmdao.lib.casehandlers.api import DumpCaseRecorder

# SU^2 imports
from SU2.io import Config

def alpha_dist2():
    return np.array([-4., 6.])

def alpha_dist10():
    return np.array([-10., -4., 0., 6., 8., 10., 12., 14., 45., 60.])

def alpha_dist():
    return np.array(range(-12,16,3))

class blade_opt(Assembly):

    nElements = 17  # Number of BEM sections for CCBlade (BEM code by Andrew Ning)
    nDVvals   = 38  # Number of Hicks-Henne bump functions
    alpha_min = -10
    alpha_max = 80
    optimizeChord = False

    r = np.array([2.8667, 5.6000, 8.3333, 11.7500, 15.8500, 19.9500, 24.0500,
                  28.1500, 32.2500, 36.3500, 40.4500, 44.5500, 48.6500, 52.7500,
                  56.1667, 58.9000, 61.6333]) 

    def __init__(self, fake=False, russianDolls=True):
        self.fake = fake
        self.russianDolls = russianDolls
        super(blade_opt, self).__init__()

    def configure(self):

        print "In blade_opt configure"

        self.alpha_sweep = alpha_dist()
        self.nSweep      = len(self.alpha_sweep)


        # Add SU2 Component
        if self.fake:
            self.add('su2',SU2_CLCD_Fake(self.alpha_sweep,nDVvals=self.nDVvals))
        else:
            self.add('su2',SU2_CLCD(self.alpha_sweep,nDVvals=self.nDVvals))        

        # We are doing russian dolls, so the outer parameters are just the dvvals
        if self.russianDolls:
            # Add BEM Assembly (optimizes for theta given airfoil)
            self.add('bem',BEMAssembly(self.alpha_sweep, self.r, optChord=self.optimizeChord))
        else:
            # Otherwise we just add the BEMComponent
            self.add('bem',BEMComponent(self.alpha_sweep, self.r, optChord=self.optimizeChord))

        # Create driver and add components to its workflow
        # Need to have something about if it's fake and russian dolls
        if not (self.fake and self.russianDolls):
            print "adding driver"
            self.add('driver',SLSQPdriver())
            self.driver.workflow.add(['bem','su2'])   
            # Objective: minimize negative power
            self.driver.add_objective('-bem.power') 
        else:
            print "driver not added"


        # Add Hicks-Henne bump function parameters
        if not self.fake:
            for i in range(self.nDVvals):
                self.driver.add_parameter('su2.dv_vals[%d]' % i, low=-.05, high=.05)

        if self.russianDolls:
            pass
        else:
            # If not nested, we want to add the theta (and maybe chord)
            for i in range(self.nElements):
                self.driver.add_parameter('bem.theta[%d]'%i,low=-80,high=80)
                if self.optimizeChord:
                    self.driver.add_parameter('bem.chord[%d]'%i,low=1e-8,high=10,start=1)
        # Connect outputs from SU^2 wrapper to CCBlade
        for i in range(self.nSweep):
            self.connect('su2.cls[%d]'%i, 'bem.cls[%d]'%i)
            self.connect('su2.cds[%d]'%i, 'bem.cds[%d]'%i)

        # Some additional driver parameters
        self.driver.maxiter = 100000
        self.driver.iprint = 1
        self.driver.accuracy = 1e-8

if __name__=="__main__":
    bo = blade_opt(fake=True, russianDolls=False)
    bo.run()
    print "Recoder dictionary"
    for item in bo.driver.recorders.__dict__:
        print "\n", item
    print bo.driver.error_code
    if bo.driver.error_code != 0:
        print "optimization error:", bo.driver.error_code,": ",bo.driver.error_messages[bo.driver.error_code]
      #print bo.driver.__dict__

