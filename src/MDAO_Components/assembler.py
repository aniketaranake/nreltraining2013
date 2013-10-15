# Python imports
import sys
import numpy as np

# Imports for our model
from BEMComponent import BEMComponent
from su2_caller import SU2_CLCD_Fake

# OpenMDAO imports
from openmdao.main.api import Component, Assembly, VariableTree
from openmdao.lib.datatypes.api import Float, Int, Array, VarTree
from openmdao.lib.drivers.api import SLSQPdriver, COBYLAdriver, CONMINdriver

#from custom_opt import SLSQPdriver

from openmdao.lib.casehandlers.api import DumpCaseRecorder

# SU^2 imports
from SU2.io import Config



class blade_opt(Assembly):

	nSweep = 10
	nElements = 12
	def configure(self):
		self.add('su2',SU2_CLCD_Fake(nSweep=self.nSweep))
		self.add('bem',BEMComponent(n_elements=self.nElements,nSweep=self.nSweep))
		self.add('driver',SLSQPdriver())
		self.driver.workflow.add(['bem','su2'])

		for i in range(self.nElements):
			self.driver.add_parameter('bem.theta[%d]'%i)
			self.driver.add_parameter('bem.chord[%d]'%i)
			self.connect('su2.cls[%d]'%i, 'bem.cls[%d]'%i)
			self.connect('su2.alphas[%d]'%i, 'bem.alphas[%d]'%i)
			self.connect('su2.cds[%d]'%i, 'bem.cds[%d]'%i)
		self.driver.add_objective('bem.power')

if __name__=="__main__":
	bo = blade_opt()
	bo.run()
	print bo.driver.error_code