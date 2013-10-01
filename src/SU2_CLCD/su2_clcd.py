from openmdao.lib.datatypes.api import Float, Int, Array, VarTree
from math import pi


class SU2_CLCD(Component):
	
	"""Calculate the coefficient of lift and the coefficient of drag from SU2"""

	kind = "Fake_SU2"
	alpha = Float(2, iotype="in",desc="angle of attack of the airfoil section", units="degrees")

	coefficientOfLift = Float(iotype="out", desc="coefficient of lift from the airfoil section at that angle of attack")
	coefficientOfDrag = Float(iotype="out", desc="coefficient of lift from the airfoil section at that angle of attack")

	def execute(self):
		if kind == "Fake_SU2":
			self.coefficientOfLift = 2 * pi * self.alpha
			self.coefficientOfDrag = .00002 * pi * pi * self.alpha

