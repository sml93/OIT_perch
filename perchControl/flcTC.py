import time
import rospy
import numpy as np

## importing rosmsg functions
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from mavros_msgs.msg import AttitudeTarget

## importing helper functions
from ceilingEffect import thrustCE
# from thrustMapper import getThrustSP
from motorThrust import motorThrust
from matplotlib import pyplot as plt


class flc():
    def __init__(self):
        ## Defining fuzzy input variables
        self._ceiling_dist = 2.5
        self._vel_uav = 0

        ## Initialise all ceiling distance linguistic variables
        self.NLCD = 0
        self.NSCD = 0
        self.ZCD = 0
        self.PSCD = 0
        self.PLCD = 0

        ## Initialise all speed linguistic variables
        self.NLSD = 0
        self.NSSD = 0
        self.ZSD = 0
        self.PSSD = 0
        self.PLSD = 0

        ## Initialise all throttle/thrust setpoint linguistic variables
        self.NLTC = 0
        self.NSTC = 0
        self.ZTC = 0
        self.PSTC = 0
        self.PLTC = 0

        ## Defining default fuzzy output variables (thrust setpoint)
        self._throttle = 0.438

        ## Initialising thrust value from ceiling effect
        self.thrustCE = 0.0
        self.thrust_val = 0.438
        self.thrust_bound = [0.33, 0.47]   ## Set bounds for thrust

        ## Initialising UAV z pose
        self.zuav = 0.0

        self.init_pubsubs()
        self.update()

    def init_pubsubs(self):
      self.ranger_sub = rospy.Subscriber('/lw20_ranger', LaserScan, self.ranger_callback)
      self.odom_sub = rospy.Subscriber('/mavros/local_position/odom', Odometry, self.odom_callback)
      self.atti_sub = rospy.Subscriber('/mavros/setpoint_raw/target_attitude', AttitudeTarget, self.atti_callback)

    def ranger_callback(self, msg):
      range = msg.ranges[0]
      if np.isinf(range):
        # print("range in inf")
        range = 0
        self._ceiling_dist = 2.5 - range
        # print("dist_CE: ", self._ceiling_dist)
      else:
        self._ceiling_dist = 2.5 - range
        # print("dist_CE: ", self._ceiling_dist)
    #   print("ok: ", self._ceiling_dist)

    def odom_callback(self, msg):
      self._vel_uav = msg.twist.twist.linear.z
      self.zuav = msg.pose.pose.position.z
      # print(self._vel_uav)

    def atti_callback(self, msg):
      self._ori_y = msg.orientation.y
      if self._ceiling_dist < 0.4:
          if (self._ori_y >= -1e-5) and (self._ori_y <= 1e-5):
              self.thrust_val = 0.44     ## To change to subscribed value from motorThrust
          else:
              self.thrust_val = 0.33     ## This is bare minimum for UAV in gazebo to pitch.

    """ Functions for calculating open left-right fuzzification for Membership Functions (MF) """
    def openLeft(self, curr_value, alpha, beta):
        ## x is the current value, alpha and beta are the extreme left and right values of the range respectively.
        if curr_value < alpha:
            return 1
        elif alpha < curr_value and curr_value <= beta:
            return (beta - curr_value) / (beta - alpha)
        else:
            return 0

    def openRight(self, curr_value, alpha, beta):
        if curr_value < alpha:
            return 0
        elif alpha < curr_value and curr_value <= beta:
            return (curr_value - alpha) / (beta - alpha)
        else:
            return 0

    """ Functions for calculating triangular fuzzification"""
    def triangular(self, curr_value, left, center, right):
        return max(min((curr_value-left)/(center-left), (right-curr_value)/(right-center)), 0)

    def halftriangular(self, curr_value, left, right):
        return ((right-curr_value)/(right-left))
    
    """ Fuzzy Membership Functions for Descriptors """
    def partitionCD(self, curr_value):
        NL = 0; NS = 0; Z = 0; PS = 0; PL = 0
        """ Need to check on this triangle values. Might need to reduce granularity """  ## <------------ TO CHECK
        if curr_value > 0 and curr_value < 0.1:
            NL = self.halftriangular(curr_value, 0.0, 0.1)
        # elif curr_value > 0.1 and curr_value < 0.3:
        #     NM = self.triangular(curr_value, 0.1, 0.2, 0.3)
        if curr_value > 0.0 and curr_value < 0.2:
            NS = self.triangular(curr_value, 0.0, 0.1, 0.2)
        if curr_value > 0.2 and curr_value < 0.3:
            Z = self.triangular(curr_value, 0.1, 0.2, 0.3)
        if curr_value > 0.4 and curr_value < 0.4:
            PS = self.triangular(curr_value, 0.2, 0.3, 0.4)
        # elif curr_value > 0.5 and curr_value < 0.7:
        #     PM = self.triangular(curr_value, 0.5, 0.6, 0.7)
        if curr_value > 0.3 and curr_value < 0.6:
            PL = self.openRight(curr_value, 0.3, 0.4)

        return NL, NS, Z, PS, PL

        # NL = 0; NS = 0; Z = 0; PS = 0; PL = 0
        # if curr_value > 0 and curr_value < 0.1:
        #     NL = self.halftriangular(curr_value, 0.0, 0.1)
        # # elif curr_value > 0.1 and curr_value < 0.3:
        # #     NM = self.triangular(curr_value, 0.1, 0.2, 0.3)
        # if curr_value > 0.1 and curr_value < 0.3:
        #     NS = self.triangular(curr_value, 0.1, 0.2, 0.3)
        # if curr_value > 0.2 and curr_value < 0.4:
        #     Z = self.triangular(curr_value, 0.2, 0.3, 0.4)
        # if curr_value > 0.3 and curr_value < 0.5:
        #     PS = self.triangular(curr_value, 0.3, 0.4, 0.5)
        # # elif curr_value > 0.5 and curr_value < 0.7:
        # #     PM = self.triangular(curr_value, 0.5, 0.6, 0.7)
        # if curr_value > 0.4 and curr_value < 0.6:
        #     PL = self.openRight(curr_value, 0.4, 0.5)
        
        # return NL, NS, Z, PS, PL


    def partitionSD(self, curr_value):
        NL = 0; NS = 0; Z = 0; PS = 0; PL = 0
        if curr_value > 0 and curr_value < 0.1:
            NL = self.halftriangular(curr_value, 0.0, 0.1)
        # elif curr_value > 0.1 and curr_value < 0.3:
        #     NM = self.triangular(curr_value, 0.1, 0.2, 0.3)
        if curr_value > 0.1 and curr_value < 0.3:
            NS = self.triangular(curr_value, 0.1, 0.2, 0.3)
        if curr_value > 0.2 and curr_value < 0.4:
            Z = self.triangular(curr_value, 0.2, 0.3, 0.4)
        if curr_value > 0.3 and curr_value < 0.5:
            PS = self.triangular(curr_value, 0.3, 0.4, 0.5)
        # elif curr_value > 0.5 and curr_value < 0.7:
        #     PM = self.triangular(curr_value, 0.5, 0.6, 0.7)
        if curr_value > 0.4 and curr_value < 0.6:
            PL = self.openRight(curr_value, 0.4, 0.5)
        
        return NL, NS, Z, PS, PL

    def partitionNegSD(self, curr_value):
        NL = 0; NS = 0; Z = 0; PS = 0; PL = 0
        if curr_value > -0.2 and curr_value < 0:
            NL = self.halftriangular(curr_value, -0.2, 0.0)
        # elif curr_value > 0.1 and curr_value < 0.3:
        #     NM = self.triangular(curr_value, 0.1, 0.2, 0.3)
        if curr_value > -0.4 and curr_value < 0.0:
            NS = self.triangular(curr_value, -0.4, -0.2, 0.0)
        if curr_value > -0.6 and curr_value < -0.2:
            Z = self.triangular(curr_value, -0.6, -0.4, -0.2)
        if curr_value > -0.8 and curr_value < -0.4:
            PS = self.triangular(curr_value, -0.8, -0.6, -0.4)
        # elif curr_value > 0.5 and curr_value < 0.7:
        #     PM = self.triangular(curr_value, 0.5, 0.6, 0.7)
        if curr_value > -1.0 and curr_value < -0.8:
            PL = self.openLeft(curr_value, -0.8, -0.6)

        return NL, NS, Z, PS, PL

    """ Rules implementation """
    def compare(self, TC1, TC2, TC3, TC4, TC5, TC6, TC7, TC8, TC9):
        TC = 0.0

        ## Method for comparison for more than 2 items:
        check_list = np.array([TC1, TC2, TC3, TC4, TC5, TC6, TC7, TC8, TC9])
        if not not check_list.any():
            TC = min(check_list[check_list != 0]) ## Take min item that is not zero.
        else:
            TC = 0.0 ## If array contains only zero element items, TC = 0
        return TC


    """ 
    Input rules here 
    With reference to the FLC rule table, only consider those that returns PL
    Very Near is NL, Near is NS, Norm is Z, Far is PS, Very Far is PL
    """
    def perchRules(self):
        ## Rules for PL
        PLTC1 = min(self.PLCD, self.NLSD)
        PLTC2 = min(self.PLCD, self.NSSD)
        PLTC3 = min(self.PLCD, self.ZSD)
        PLTC4 = min(self.PSCD, self.NLSD)
        PLTC5 = min(self.PSCD, self.NSSD)
        PLTC6 = 0.0
        PLTC7 = 0.0
        PLTC8 = 0.0
        PLTC9 = 0.0
        self.PLTC = self.compare(PLTC1, PLTC2, PLTC3, PLTC4, PLTC5, PLTC6, PLTC7, PLTC8, PLTC9)

        ## Rules for PS
        PSTC1 = min(self.PLCD, self.PSSD)
        PSTC2 = min(self.PLCD, self.PLSD)
        PSTC3 = min(self.PSCD, self.ZSD)
        PSTC4 = min(self.PSCD, self.PSSD)
        PSTC5 = min(self.PSCD, self.PLSD)
        PSTC6 = min(self.ZCD, self.NSSD)
        PSTC7 = min(self.NSCD, self.NLSD)
        PSTC8 = min(self.NSCD, self.NSSD)
        PSTC9 = min(self.ZCD, self.NLSD)
        self.PSTC = self.compare(PSTC1, PSTC2, PSTC3, PSTC4, PSTC5, PSTC6, PSTC7, PSTC8, PSTC9)
        
        ## Rules for Z
        ZTC1 = min(self.NLCD, self.NLSD)
        ZTC2 = min(self.NLCD, self.NSSD)
        ZTC3 = min(self.NLCD, self.ZSD)
        ZTC4 = min(self.ZCD, self.ZSD)
        ZTC5 = min(self.NSCD, self.ZSD)
        ZTC6 = 0.0
        ZTC7 = 0.0
        ZTC8 = 0.0
        ZTC9 = 0.0
        self.ZTC = self.compare(ZTC1, ZTC2, ZTC3, ZTC4, ZTC5, ZTC6, ZTC7, ZTC8, ZTC9)

        ## Rules for NS
        NSTC1 = min(self.ZCD, self.PSSD)
        NSTC2 = min(self.ZCD, self.ZSD)
        NSTC3 = min(self.NSCD, self.PSSD)
        NSTC4 = 0.0
        NSTC5 = 0.0
        NSTC6 = 0.0
        NSTC7 = 0.0
        NSTC8 = 0.0
        NSTC9 = 0.0
        self.NSTC = self.compare(NSTC1, NSTC2, NSTC3, NSTC4, NSTC5, NSTC6, NSTC7, NSTC8, NSTC9)

        ## Rules for NL
        NLTC1 = min(self.ZCD, self.PLSD)
        NLTC2 = min(self.NSCD, self.PSSD)
        NLTC3 = min(self.NSCD, self.PLSD)
        NLTC4 = min(self.NLCD, self.ZSD)
        NLTC5 = min(self.NLCD, self.PSSD)
        NLTC6 = min(self.NLCD, self.PLSD)
        NLTC7 = 0.0
        NLTC8 = 0.0
        NLTC9 = 0.0 
        self.NLTC = self.compare(NLTC1, NLTC2, NLTC3, NLTC4, NLTC5, NLTC6, NLTC7, NLTC8, NLTC9)


    """ Rules for detaching from ceiling """
    def detachRules(self):
        ## Rules for PL
        PLTC1 = min(self.ZCD, self.PSSD)
        PLTC2 = min(self.ZCD, self.PLSD)
        PLTC3 = 0.0
        PLTC4 = 0.0
        PLTC5 = 0.0
        PLTC6 = 0.0
        PLTC7 = 0.0
        PLTC8 = 0.0
        PLTC9 = 0.0
        self.PLTC = self.compare(PLTC1, PLTC2, PLTC3, PLTC4, PLTC5, PLTC6, PLTC7, PLTC8, PLTC9)

        ## Rules for PS
        PSTC1 = min(self.ZCD, self.NLSD)
        PSTC2 = min(self.ZCD, self.NSSD)
        PSTC3 = min(self.ZCD, self.ZSD)
        PSTC4 = min(self.PSCD, self.ZSD)
        PSTC5 = min(self.PSCD, self.PSSD)
        PSTC6 = min(self.PSCD, self.PLSD)
        # PSTC7 = 0.0
        # PSTC8 = 0.0
        # PSTC9 = 0.0
        PSTC7 = min(self.PLCD, self.ZSD)
        PSTC8 = min(self.PLCD, self.PSSD)
        PSTC9 = min(self.PLCD, self.PLSD)
        self.PSTC = self.compare(PSTC1, PSTC2, PSTC3, PSTC4, PSTC5, PSTC6, PSTC7, PSTC8, PSTC9)
        
        ## Rules for Z
        ZTC1 = min(self.NLCD, self.PSSD)
        ZTC2 = min(self.NLCD, self.PLSD)
        ZTC3 = min(self.NSCD, self.NSSD)
        ZTC4 = min(self.NSCD, self.ZSD)
        ZTC5 = min(self.NSCD, self.PSSD)
        ZTC6 = min(self.NSCD, self.PLSD)
        ZTC7 = min(self.PSCD, self.NLSD)
        ZTC8 = min(self.PSCD, self.NSSD)
        ZTC9 = 0.0
        self.ZTC = self.compare(ZTC1, ZTC2, ZTC3, ZTC4, ZTC5, ZTC6, ZTC7, ZTC8, ZTC9)

        ## Rules for NS
        NSTC1 = min(self.NLCD, self.NSSD)
        NSTC2 = min(self.NLCD, self.ZSD)
        NSTC3 = min(self.NSCD, self.NLSD)
        NSTC4 = min(self.PLCD, self.NLSD)
        NSTC5 = min(self.PLCD, self.NSSD)
        # NSTC6 = min(self.PLCD, self.ZSD)
        # NSTC7 = min(self.PLCD, self.PSSD)
        # NSTC8 = min(self.PLCD, self.PLSD)
        NSTC6 = 0.0
        NSTC7 = 0.0
        NSTC8 = 0.0
        NSTC9 = 0.0
        self.NSTC = self.compare(NSTC1, NSTC2, NSTC3, NSTC4, NSTC5, NSTC6, NSTC7, NSTC8, NSTC9)

        ## Rules for NL
        NLTC1 = min(self.NLCD, self.NLSD)
        NLTC2 = 0.0
        NLTC3 = 0.0
        NLTC4 = 0.0
        NLTC5 = 0.0
        NLTC6 = 0.0
        NLTC7 = 0.0
        NLTC8 = 0.0
        NLTC9 = 9.0
        self.NLTC = self.compare(NLTC1, NLTC2, NLTC3, NLTC4, NLTC5, NLTC6, NLTC7, NLTC8, NLTC9)


    """ Defuzzification """
    def areaTR(self, mu, left, center, right):
        x1 = mu*(center-left)+left
        x2 = right-mu*(right-center)
        d1 = (right-left)  # getting the base length
        d2 = x2-x1
        area = 0.5*mu*(d1+d2)  # mu here is the height of the region of interest (roi)
        return area

    def areahalfTR(self, mu, left, right):
        area = 0.5*mu*(right-left)
        return area 

    def areaOL(self, mu, left, right):
        extremeL = 0
        # xOL = right*mu(right-left)
        aOL = (0.5*mu*(right-left))+((left-extremeL)*mu)
        return aOL, (right-extremeL)/2

    def areaOR(self, mu, left, right):
        extremeR = 0.5
        # xOR = (right-left)*mu+left  # xOR is x OpenRight
        # aOR = 0.5*mu*((0.6-left)+(0.6-xOR))  # 0.5 here is the max, aOR is area OpenRight
        aOR = (0.5*mu*(right-left))+(mu*(extremeR-right))
        print(mu, left, right)
        return aOR, (extremeR-left)/2 + left
    
    def defuzzification(self):
        areaNL = 0
        areaNS = 0
        areaZ = 0
        areaPS = 0
        areaPL = 0
        
        cNL = 0
        cNS = 0
        cZ = 0
        cPS = 0
        cPL = 0

        """ 
        To Check:
          Probably need to update this table variables
          UAV thrust cannot be lesser than 0.33 when perched,
          UAV thrust cannot be more than x value at any point, might damage prismatic joints
        """
        # if self.NLTC != 0:
        #     areaNL, cNL = self.areaOL(self.NLTC, 0.365, 0.375)
        
        # if self.NSTC != 0:
        #     areaNS = self.areaTR(self.NSTC, 0.37, 0.375, 0.38)
        #     cNS = 0.375
        
        # if self.ZTC != 0:
        #     areaZ = self.areaTR(self.ZTC, 0.375, 0.38, 0.385)
        #     cZ = 0.38
        
        # if self.PSTC != 0:
        #     areaPS = self.areaTR(self.PSTC, 0.38, 0.385, 0.385)
        #     cPS = 0.385
        
        # if self.PLTC != 0:
        #     areaPL, cPL = self.areaOR(self.PLTC, 0.385, 0.395)

        if self.NLTC != 0:
            areaNL, cNL = self.areaOL(self.NLTC, self.thrust_val-0.015, self.thrust_val-0.005)
        
        if self.NSTC != 0:
            areaNS = self.areaTR(self.NSTC, self.thrust_val-0.01, self.thrust_val-0.005, self.thrust_val)
            cNS = self.thrust_val-0.005
        
        if self.ZTC != 0:
            areaZ = self.areaTR(self.ZTC, self.thrust_val-0.005, self.thrust_val, self.thrust_val+0.005)
            cZ = self.thrust_val
        
        if self.PSTC != 0:
            areaPS = self.areaTR(self.PSTC, self.thrust_val, self.thrust_val+0.005, self.thrust_val+0.01)
            cPS = self.thrust_val+0.005
        
        if self.PLTC != 0:
            areaPL, cPL = self.areaOR(self.PLTC, self.thrust_val+0.01, self.thrust_val+0.015)

        numerator = (areaNL*cNL) + (areaNS*cNS) + (areaZ*cZ) + (areaPS*cPS) + (areaPL*cPL)
        # print("num: ", numerator)
        denominator = areaNL + areaNS + areaZ + areaPS + areaPL
        # print("denom: ", denominator)
        if denominator == 0:
            # print("No rules exist to give the results")
            print("Default thrust setpoint at: ", self._throttle)
            return (self._throttle)     ## if FLC not operating, want UAV to perch
        else:
            crispOp = np.round(numerator,5)/np.round(denominator,5)
            return (crispOp)


    def update(self):
        """ Update thrustCE """
        self.init_pubsubs()
        # if self._ceiling_dist < 1.0:
        runCE = thrustCE(self._ceiling_dist)
        self.thrustCE = runCE.getThrust()/9.81  ## Need to map thrust to thrust setpoint
        print("Thrust from CE per motor (kg): ", self.thrustCE)


        """ Update all fuzzy values for all inputs of the fuzzy sets """
        self.NLCD, self.NSCD, self.ZCD, self.PSCD, self.PLCD = self.partitionCD(self._ceiling_dist)
        print(self._vel_uav)
        if self._vel_uav > 0.0:
            self.NLSD, self.NSSD, self.ZSD, self.PSSD, self.PLSD = self.partitionSD(self._vel_uav)
            self.perchRules()
        # else:         ## uncomment this part out for now
        #     print("\ndescending\n")
        #     self._throttle = 0.305
        #     self.NLSD, self.NSSD, self.ZSD, self.PSSD, self.PLSD = self.partitionNegSD(self._vel_uav)
        #     self.detachRules()

        OP = [[self.NLCD, self.NSCD, self.ZCD, self.PSCD, self.PLCD],
              [self.NLSD, self.NSSD, self.ZSD, self.PSSD, self.PLSD]]
        print("The fuzzy values of the crisp inputs are: \n", np.round(OP, 4))


        OpRules = [[self.NLTC, self.NSTC, self.ZTC, self.PSTC, self.PLTC]]
        # print("Output Rules: ", np.round(OpRules, 4))

        crispOpFinal = self.defuzzification()
        print("\n The crisp TC value is: ", crispOpFinal)

        if self._ceiling_dist <= 0.4:
            fce = motorThrust.getThrust(motorThrust, self.thrustCE/1000)
            F_ce = np.linalg.norm(fce[np.isreal(fce)])
            thrustSP = motorThrust.getCompThrustSP(motorThrust, crispOpFinal, F_ce)
            print("F_CE: ", F_ce)
            # print("CRISPOP: ", crispOpFinal)
            print("HERE: ", thrustSP)
        else: thrustSP = crispOpFinal
        return thrustSP
        # return crispOpFinal

        # if (crispOpFinal < self.thrust_bound[0]) and (self._ceiling_dist <= 0.5):
        #     return self.thrust_bound[0]
        # elif (self._ceiling_dist > 0.5):
        #     return self.thrust_val
        # else:
        #     # thrustSP = getThrustSP(crispOpFinal, self.thrustCE)
        #     fce = motorThrust.getThrust(motorThrust, self.thrustCE/1000)
        #     F_ce = np.linalg.norm(fce[np.isreal(fce)])
        #     crispOpFinal = 0.33
        #     thrustSP = motorThrust.getCompThrustSP(motorThrust, crispOpFinal, F_ce)
        #     print("F_CE: ", F_ce)
        #     print("CRISPOP: ", crispOpFinal)
        #     print("HERE: ", thrustSP)
        #     return thrustSP



def main():
    my_list = []
    time_list = []
    poseZ = []
    velZ = []
    ce_list = []
    cd_list = []
    rospy.init_node("flc", anonymous=True)
    run = flc()
    time_start = time.time()

    ## To send thrust to topic
    atti = rospy.Publisher('/desired_atti', AttitudeTarget, queue_size=10)
    # rospy.init_node('topicSender', anonymous=True)
    rate = rospy.Rate(10)
    atti_msg = AttitudeTarget()
    
    while not rospy.is_shutdown():
      my_list.append(run.update())
      time_list.append(time.time()-time_start)
      poseZ.append(run.zuav)
      velZ.append(run._vel_uav)
      ce_list.append(run.thrustCE*9.81)
      cd_list.append(run._ceiling_dist)
      
      atti_msg.header = Header()
      atti_msg.header.stamp = rospy.Time.now()
      atti_msg.orientation.x = 0
      atti_msg.orientation.y = 0
      atti_msg.orientation.z = 0
      atti_msg.orientation.w = 1
      atti_msg.thrust = run.update()
      atti.publish(atti_msg)
    #   rate.sleep()

    plt.figure()
    plt.plot(time_list, my_list, label="thrustSP")
    plt.plot(time_list, poseZ, label="poseZ (m)")
    plt.plot(time_list, velZ, label="velZ (m/s)")
    plt.plot(time_list, ce_list, label="thrust_ce (N)")
    plt.plot(time_list, cd_list, label="ceiling_dist (m)")
    plt.legend()
    plt.show()
    # return my_list, time_list   



if __name__ == "__main__":
    main()
    # try:
    #     main()
    # except rospy.ROSInterruptException:
    #     pass