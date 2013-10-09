# Python imports
from math import pi
import numpy as np
from scipy.interpolate import interp1d

# OpenMDAO imports
from openmdao.lib.datatypes.api import Float, Int, Array, VarTree
from openmdao.main.api import Component, Assembly
from SU2_wrapper import Solve

# SU2 imports
from SU2.io import Config

class SU2_CLCD(Solve):
  
  """Calculate the coefficient of lift and the coefficient of drag from SU2"""

  kind = "Fake_SU2"
  alpha = Float(2., iotype="in",desc="angle of attack of the airfoil section", units="deg")

  coefficientOfLift = Float(iotype="out", desc="coefficient of lift from the airfoil section at that angle of attack")
  coefficientOfDrag = Float(iotype="out", desc="coefficient of lift from the airfoil section at that angle of attack")

  def __init__(self):
    print "SU2_CLCD item created. config_in=", self.config_in

  def execute(self):
    if self.kind == "Fake_SU2":

      alpha_data = np.array([0., 13., 15, 20, 30])
      cl_data    = np.array([0, 1.3, .8, .7, 1.1])
      cd_data    = np.array([0., 1e-2, 0.3, 0.6, 1.])

      f_cl = interp1d(alpha_data, cl_data, fill_value=0.001, bounds_error=False)
      f_cd = interp1d(alpha_data, cd_data, fill_value=0.001, bounds_error=False)
      self.coefficientOfLift = float(f_cl(self.alpha))
      self.coefficientOfDrag = float(f_cd(self.alpha)) + 1e-5

      #self.coefficientOfLift = 2 * pi * self.alpha
      #self.coefficientOfDrag = .00002 * pi * pi * self.alpha


      print "Fake_SU2: alpha = ", self.alpha, ", cl = ", self.coefficientOfLift, ", cd = ", self.coefficientOfDrag
    if self.kind == "SU2":
      super(SU2_CLCD, self).execute()
      coefficientOfLift = self.LIFT
      coefficientOfDrag = self.DRAG      

class SU2_CLCD_Sections(Assembly):

  def __init__(self, nElems = 6):
    super(SU2_CLCD_Sections, self).__init__()
    self.nElems = nElems
    self.add("alphas",Array(np.zeros([self.nElems,]), shape=[self.nElems,],iotype="in"))
    self.add("cls",Array(np.zeros([self.nElems,]), shape=[self.nElems,],iotype="out"))
    self.add("cds",Array(np.zeros([self.nElems,]), shape=[self.nElems,],iotype="out"))

  def configure(self):
    for i in range(self.nElems):
      su2comp = "SU2_%d"%i
      self.add(su2comp, SU2_CLCD())
      self.connect("alphas[%d]"%i, su2comp+".alpha")
      self.connect(su2comp+".coefficientOfLift","cls[%d]"%i)
      self.connect(su2comp+".coefficientOfDrag", "cds[%d]"%i)
      self.driver.workflow.add(su2comp)

  def set_su2_config(self, filename):
    '''Routine to specify an SU^2 config file'''
    for i in range(self.nElems):

      # Name of the i'th su2 object
      su2comp = "SU2_%d"%i

      # Create a new config option and read the file into it
      new_config = Config()
      new_config.read(filename)

      # Using eval to deal with su2comp variable
      evalcmd = 'self.%s.config_in=%s'%(su2comp,new_config)
      print evalcmd
      eval(evalcmd)



if __name__ == "__main__":
  print "Hello, world!"

