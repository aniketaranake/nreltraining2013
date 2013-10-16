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



class blade_opt_fake(Assembly):

  nSweep = 50
  nElements = 17
  alpha_min = -10
  alpha_max = 80
  optimizeChord = False
  def configure(self):
    self.add('su2',SU2_CLCD_Fake(nSweep=self.nSweep))
    self.add('bem',BEMComponent(n_elements=self.nElements,nSweep=self.nSweep,optChord=optimizeChord))
    self.add('driver',SLSQPdriver())
    self.driver.workflow.add(['bem','su2'])

    for i in range(self.nElements):
      self.driver.add_parameter('bem.theta[%d]'%i,low=self.alpha_min,high=self.alpha_max)
      if optimize_chord:
        self.driver.add_parameter('bem.chord[%d]'%i,low=1e-8,high=10,start=1)
    for i in range(self.nSweep):
      self.connect('su2.cls[%d]'%i, 'bem.cls[%d]'%i)
      self.connect('su2.alphas[%d]'%i, 'bem.alphas[%d]'%i)
      self.connect('su2.cds[%d]'%i, 'bem.cds[%d]'%i)
    self.driver.add_objective('-bem.power')
    self.driver.maxiter = 100000
    self.driver.iprint = 1
    self.driver.accuracy = 1e-14
    for item in  self.driver.__dict__:
      print item

class blade_opt(Assembly):

  nSweep    = 10  # Points in each alpha-sweep
  nElements = 17  # Number of BEM sections for CCBlade (BEM code by Andrew Ning)
  nDVvals   = 38  # Number of Hicks-Henne bump functions

  def configure(self):

    # Add components
    self.add('su2',SU2_CLCD(nSweep=self.nSweep,nDVvals=self.nDVvals))
    self.add('bem',BEMComponent(n_elements=self.nElements,nSweep=self.nSweep))

    # Create driver and add components to its workflow
    self.add('driver',SLSQPdriver())
    self.driver.workflow.add(['bem','su2'])

    # Design parameters for CCBlade 
    for i in range(self.nElements):
      self.driver.add_parameter('bem.theta[%d]'%i,low=-80,high=80)
      self.driver.add_parameter('bem.chord[%d]'%i,low=1e-8,high=10,start=1)

    # Design parameters for SU^2
    for i in range(self.nDVvals):
      self.driver.add_parameter('su2.dv_vals[%d]' % i, low=-.05, high=.05)

    # Connect outputs from SU^2 wrapper to CCBlade
    for i in range(self.nSweep):
      self.connect('su2.cls[%d]'%i, 'bem.cls[%d]'%i)
      self.connect('su2.alphas[%d]'%i, 'bem.alphas[%d]'%i)
      self.connect('su2.cds[%d]'%i, 'bem.cds[%d]'%i)

    # Objective: minimize negative power
    self.driver.add_objective('-bem.power')

    # Specify max iterations
    self.driver.maxiter = 1000

if __name__=="__main__":
  bo = blade_opt_fake()
  bo.run()
  print "Recoder dictionary"
  for item in bo.driver.recorders.__dict__:
    print "\n", item
  print bo.driver.error_code
  if bo.driver.error_code != 0:
    print "optimization error:", bo.driver.error_code,": ",bo.driver.error_messages[bo.driver.error_code]
  #print bo.driver.__dict__

