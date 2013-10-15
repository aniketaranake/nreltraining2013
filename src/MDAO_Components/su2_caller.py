# Python imports
import numpy as np
from scipy.interpolate import interp1d

# OpenMDAO imports
from openmdao.lib.datatypes.api import Float, Int, Array, VarTree, File
from openmdao.main.api import Component, Assembly, set_as_top
#from SU2_wrapper import Solve, Deform
#from SU2_wrapper.SU2_wrapper import ConfigVar, Config


class SU2_CLCD_Fake(Component):
	def __init__(self, nSweep = 10):
		self.alpha_min = -20
		self.alpha_max = 20
		self.nSweep = nSweep

		self.alpha_sweep = np.linspace(self.alpha_min, self.alpha_max, num=nSweep)

		self.alphas = Array(np.zeros([self.nSweep,]), dtype=np.float,shape=[self.nSweep,],iotype="out")
		self.cls = Array(np.zeros([self.nSweep,]), dtype=np.float,shape=[self.nSweep,],iotype="out")
		self.cds = Array(np.zeros([self.nSweep,]),dtype=np.float, shape=[self.nSweep,],iotype="out")

		alpha_data = np.array([-30, -20, -15, -13, 0., 13., 15, 20, 30])
		cl_data    = np.array([-1.1,-.7, -.8, -1.3,0, 1.3, .8, .7, 1.1])
		cd_data    = np.array([1.,0.6,0.3, 1e-2, 0., 1e-2, 0.3, 0.6, 1.]) + 1e-5

		self.f_cl = interp1d(alpha_data, cl_data, fill_value=0.001, bounds_error=False)
		self.f_cd = interp1d(alpha_data, cd_data, fill_value=0.001, bounds_error=False)

	def execute(self):
		for i in range(self.nSweep):
			self.alphas.default_value[i] = self.alpha_sweep[i]
			self.cls.default_value[i] = self.f_cl(self.alpha_sweep[i])
			self.cds.default_value[i] = self.f_cd(self.alpha_sweep[i])

if __name__ == "__main__":
	var = SU2_CLCD_Fake(nSweep = 15)
	var.execute()
	print var.alphas.default_value
	print var.cls.default_value
	print var.cds.default_value