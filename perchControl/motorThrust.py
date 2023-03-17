import rospy
import numpy as np

from std_msgs.msg import Float32

"""
Thrust to motor cap % for Sunnysky 2212-13 980kv 10x4.5 germfan nylon/carbon props
"""

class motorThrust():
  def __init__(self):
    # self.thrust = 378   # 1.5/4
    self.thrust = 1335.3/4   # 1.335/4
    # self.thrust = 168.22
    self.bounds = [0.33, 0.47]
    rospy.Subscriber('/thrust', Float32, self.thrust_cb)

  def thrust_cb(self, msg):
    self.thrust = msg.data*1000/4
    # print(self.thrust)

  def getThrustRatio(self):
    a = 2e-9*np.power(self.thrust,3)
    b = 3e-6*np.power(self.thrust,2)
    c = 0.0021*self.thrust
    d = 6e-14 
    return (a-b+c+d)

  def getThrust(self, val_ratio):
    coeff = [2e-9, -3e-6, 0.0021, 6e-14-val_ratio]
    return np.roots(coeff)

  # def getThrust(self):
  #   coeff = [2e-9, -3e-6, 0.0021, 6e-14-self.thrust]
  #   # return np.roots(coeff)
  #   roots = np.roots(coeff)
  #   # print(np.linalg.norm(roots[np.isreal(roots)]))
  #   return print(np.linalg.norm(roots[np.isreal(roots)]))
  
  def getCompThrustSP(self, crispVal, ce=0):
    # compThrustSP = np.mean([np.mean([ce,crispVal]), crispVal])
    bounds = [0.35, 0.47]
    compThrustSP = np.mean([(crispVal-ce), bounds[0], crispVal])   ## Taking the mean of 3 thrust values
    print("Compensated Thrust: ", compThrustSP)
    return compThrustSP


def main():
  rospy.init_node('motorThrust', anonymous=True)
  run = motorThrust()
  print(round(run.getThrustRatio(),5))

  # run.getThrust()
  # # return print(np.linalg.norm(run.getThrust(run.thrust)[np.isreal(run.getThrust(run.thrust))]))
  
  # thrustVal = 0.315
  # return print(np.linalg.norm(run.getThrust(thrustVal)[np.isreal(run.getThrust(thrustVal))]))

  print("check: ", run.getCompThrustSP(0.22, 4/9.81))

  # run.getCompThrustSP(0.444, 0.1701)



if __name__ == "__main__":
  try:
    while not rospy.is_shutdown():
      main()
  except rospy.ROSInterruptException():
    pass
