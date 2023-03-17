import numpy as np

class thrustCE:
  def __init__(self, cdist):
    self._ceilingDist = cdist
    self._inputFlowVel = 5.0  ## to run thrust stand CE experiment and update, probably a curvefit model 
    self._radius = 5*0.0254  ## 5 inches to m
    self._alpha0 = 1.6
    self._alpha1 = 0
    self._rho = 1.293  ## kg/m3
    self.A = np.pi*(np.power(self._radius,2))
    self.update()

  def delta(self):
    self._del = self._radius / self._ceilingDist

  def gamma(self):
    self._gamma = 0.5*(1-(self._alpha1*(np.power(self._del,2)))) + 0.5*(np.sqrt(np.power(1-(self._alpha1*(np.power(self._del,2))),2)+(self._alpha0/8*(np.power(self._del,2)))))

  def getThrust(self):
    thrust = 2*(self._rho)*(self.A)*(np.power(self._gamma,2))*(np.power(self._inputFlowVel,2))
    return thrust

  def update(self):
    """ Update self variables for getThrust function """
    self.delta()
    self.gamma()
    # print(self._radius)
    # print(self._del)
    # print(self.A)
    # print(self._gamma)

def main():
  run = thrustCE(0.1)
  print(run.getThrust())


if __name__ == "__main__":
  main()