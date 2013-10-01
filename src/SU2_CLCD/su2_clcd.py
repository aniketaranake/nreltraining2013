from openmdao.lib.datatypes.api import Float, Int, Array, VarTree
from math import pi
from SU2_wrapper import Solve



class SU2_CLCD(Solve):
	
	"""Calculate the coefficient of lift and the coefficient of drag from SU2"""

	kind = "Fake_SU2"
	alpha = Float(2, iotype="in",desc="angle of attack of the airfoil section", units="deg")

	coefficientOfLift = Float(iotype="out", desc="coefficient of lift from the airfoil section at that angle of attack")
	coefficientOfDrag = Float(iotype="out", desc="coefficient of lift from the airfoil section at that angle of attack")

	def execute(self):
		if kind == "Fake_SU2":
			self.coefficientOfLift = 2 * pi * self.alpha
			self.coefficientOfDrag = .00002 * pi * pi * self.alpha
		if kind == "SU2":
			super(SU2_CLCD, self).execute()
			coefficientOfLift = self.LIFT
			coefficientOfDrag = self.DRAG			

if __name__ == "__main__":
	print "Hello, world!"

