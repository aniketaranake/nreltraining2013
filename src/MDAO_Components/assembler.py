# Python imports
import sys
import numpy as np

# Imports for our model
from BEMComponent import BEMComponent
from su2_caller import SU2_CLCD_Fake

# OpenMDAO imports
from openmdao.main.api import Component, Assembly, VariableTree
from openmdao.lib.datatypes.api import Float, Int, Array, VarTree
from openmdao.lib.drivers.api import SLSQPdriver 

#from custom_opt import SLSQPdriver

from openmdao.lib.casehandlers.api import DumpCaseRecorder

# SU^2 imports
from SU2.io import Config

class blade_opt(Assembly):

    nElements = 17  # Number of BEM sections for CCBlade (BEM code by Andrew Ning)
    nDVvals   = 38  # Number of Hicks-Henne bump functions
    alpha_min = -10
    alpha_max = 80
    optimizeChord = False

    def __init__(self, fake=False):
        self.fake = fake
        super(blade_opt, self).__init__()

    def configure(self):

      # TODO: Improve alpha_sweep
      self.alpha_sweep = np.linspace(self.alpha_min, self.alpha_max,10)
      self.nSweep      = len(self.alpha_sweep)

      # Add components
      if self.fake:
          self.add('su2',SU2_CLCD_Fake(self.alpha_sweep,nDVvals=self.nDVvals))
      else:
          self.add('su2',SU2_CLCD(self.alpha_sweep,nDVvals=self.nDVvals))
      self.add('bem',BEMComponent(self.alpha_sweep, n_elements=self.nElements))

      # Create driver and add components to its workflow
      self.add('driver',SLSQPdriver())
      self.driver.workflow.add(['bem','su2'])

      # Design parameters for CCBlade 
      for i in range(self.nElements):
        self.driver.add_parameter('bem.theta[%d]'%i,low=-80,high=80)
        if self.optimizeChord:
            self.driver.add_parameter('bem.chord[%d]'%i,low=1e-8,high=10,start=1)

      # Design parameters for SU^2
      if not self.fake:
          for i in range(self.nDVvals):
            self.driver.add_parameter('su2.dv_vals[%d]' % i, low=-.05, high=.05)

      # Connect outputs from SU^2 wrapper to CCBlade
      for i in range(self.nSweep):
        self.connect('su2.cls[%d]'%i, 'bem.cls[%d]'%i)
        self.connect('su2.cds[%d]'%i, 'bem.cds[%d]'%i)

      # Objective: minimize negative power
      self.driver.add_objective('-bem.power')

      # Specify max iterations
      self.driver.maxiter = 1000

      # Some additional driver parameters
      self.driver.maxiter = 100000
      self.driver.iprint = 1
      self.driver.accuracy = 1e-8
      for item in  self.driver.__dict__:
          print item

if __name__=="__main__":
    bo = blade_opt(fake=True)
    bo.run()
    print "Recoder dictionary"
    for item in bo.driver.recorders.__dict__:
        print "\n", item
    print bo.driver.error_code
    if bo.driver.error_code != 0:
        print "optimization error:", bo.driver.error_code,": ",bo.driver.error_messages[bo.driver.error_code]
    #print bo.driver.__dict__

