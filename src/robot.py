from utils import *


class Robot(object):

    def __init__(self):
        self.l1 = 0.2
        self.l2 = 0.2
        self.ltx = 0.25
        self.lty = 0.2
        self.ltz = 0.05

    def update_config(self, q):
        self.q = q

    def fkine_fright_base(self):
        """
        Forward kinematics of the front right leg with respect to the base
        
        """
        d = 7
        q1 = self.q[d+0]; q2 = self.q[d+1]; q3 = self.q[d+2]
        T01 = np.dot(Ttransl([self.ltx, -self.lty, -self.ltz]),
                     Trotx(q1))
        T12 = Troty(q2)
        T23 = np.dot(Ttransl([0., 0., -self.l1]),
                     Troty(q3))
        T34 = Ttransl([0., 0., -self.l2])
        T = np.dot(np.dot(np.dot(T01, T12), T23), T34)
        return T


    def fkine_fleft_base(self):
        """
        Forward kinematics of the front left leg with respect to the base
        
        """
        d = 7+3
        q1 = self.q[d+0]; q2 = self.q[d+1]; q3 = self.q[d+2]
        T01 = np.dot(Ttransl([self.ltx, self.lty, -self.ltz]),
                     Trotx(q1))
        T12 = Troty(q2)
        T23 = np.dot(Ttransl([0., 0., -self.l1]),
                     Troty(q3))
        T34 = Ttransl([0., 0., -self.l2])
        T = np.dot(np.dot(np.dot(T01, T12), T23), T34)
        return T


    def fkine_rright_base(self):
        """
        Forward kinematics of the rear right leg with respect to the base
        
        """
        d = 7+6
        q1 = self.q[d+0]; q2 = self.q[d+1]; q3 = self.q[d+2]
        T01 = np.dot(Ttransl([-self.ltx, -self.lty, -self.ltz]),
                     Trotx(q1))
        T12 = Troty(q2)
        T23 = np.dot(Ttransl([0., 0., -self.l1]),
                     Troty(q3))
        T34 = Ttransl([0., 0., -self.l2])
        T = np.dot(np.dot(np.dot(T01, T12), T23), T34)
        return T


    def fkine_rleft_base(self):
        """
        Forward kinematics of the rear right leg with respect to the base
        
        """
        d = 7+9
        q1 = self.q[d+0]; q2 = self.q[d+1]; q3 = self.q[d+2]
        T01 = np.dot(Ttransl([-self.ltx, self.lty, -self.ltz]),
                     Trotx(q1))
        T12 = Troty(q2)
        T23 = np.dot(Ttransl([0., 0., -self.l1]),
                     Troty(q3))
        T34 = Ttransl([0., 0., -self.l2])
        T = np.dot(np.dot(np.dot(T01, T12), T23), T34)
        return T

    def jacobian_base(self, d, T01):
        """
        Proto generator of the Jacobian (since all of them are the same with
        respect to the base, but the used joints and T01 change)

        """
        q1 = self.q[d+0]; q2 = self.q[d+1]; q3 = self.q[d+2]
        T12 = Troty(q2)
        T23 = np.dot(Ttransl([0., 0., -self.l1]),
                     Troty(q3))
        T34 = Ttransl([0., 0., -self.l2])
        T02 = np.dot(T01, T12)
        T03 = np.dot(T02, T23)
        T04 = np.dot(T03, T34)

        #w1 = np.array([1.,0.,0.])
        w1 = T01[0:3,0]
        p14 = T04[0:3,3]-T01[0:3,3]
        v1 = np.cross(w1, p14)
        w2 = T02[0:3,1]
        p24 = T04[0:3,3]-T02[0:3,3]
        v2 = np.cross(w2, p24)
        w3 = T03[0:3,1]
        p34 = T04[0:3,3]-T03[0:3,3]
        v3 = np.cross(w3, p34)
        J = np.zeros((6,3))
        # J[0:3,0] = w1; J[0:3,1] = w2; J[0:3,2] = w3
        # J[3:6,0] = v1; J[3:6,1] = v2; J[3:6,2] = v3
        J[0:3,0] = v1; J[0:3,1] = v2; J[0:3,2] = v3
        J[3:6,0] = w1; J[3:6,1] = w2; J[3:6,2] = w3
        return J
    
    def jacobian_fright_base(self):
        """
        Geometric Jacobian for the front right with respect to the base
        """
        d = 7
        T01 = np.dot(Ttransl([self.ltx, -self.lty, -self.ltz]),
                     Trotx(self.q[d+0]))
        return self.jacobian_base(d, T01)

    def jacobian_fleft_base(self):
        """
        Geometric Jacobian for the front left with respect to the base
        """
        d = 7+3
        T01 = np.dot(Ttransl([self.ltx, self.lty, -self.ltz]),
                     Trotx(self.q[d+0]))
        return self.jacobian_base(d, T01)

    def jacobian_rright_base(self):
        """
        Geometric Jacobian for the rear right with respect to the base
        """
        d = 7+6
        T01 = np.dot(Ttransl([-self.ltx, -self.lty, -self.ltz]),
                     Trotx(self.q[d+0]))
        return self.jacobian_base(d, T01)

    def jacobian_rleft_base(self):
        """
        Geometric Jacobian for the rear left with respect to the base
        """
        d = 7+9
        T01 = np.dot(Ttransl([-self.ltx, self.lty, -self.ltz]),
                     Trotx(self.q[d+0]))
        return self.jacobian_base(d, T01)

        
    """
    ================================================================

    The methods below consider the floating base
    
    ================================================================
    """

    def fkine_fright(self):
        """
        Forward kinematics of the front right leg with respect to the 
        inertial frame

        """
        T = self.fkine_fright_base()
        Rb = rotationFromQuat(self.q[3:7])
        p = np.dot(Rb, T[0:3,3]) + self.q[0:3]
        o = np.dot(Rb, T[0:3,0:3])
        T[0:3,3] = p
        T[0:3,0:3] = o
        return T

    def fkine_fleft(self):
        """
        Forward kinematics of the front left leg with respect to the 
        inertial frame

        """
        T = self.fkine_fleft_base()
        Rb = rotationFromQuat(self.q[3:7])
        p = np.dot(Rb, T[0:3,3]) + self.q[0:3]
        o = np.dot(Rb, T[0:3,0:3])
        T[0:3,3] = p
        T[0:3,0:3] = o
        return T

    def fkine_rright(self):
        """
        Forward kinematics of the rear right leg with respect to the 
        inertial frame

        """
        T = self.fkine_rright_base()
        Rb = rotationFromQuat(self.q[3:7])
        p = np.dot(Rb, T[0:3,3]) + self.q[0:3]
        o = np.dot(Rb, T[0:3,0:3])
        T[0:3,3] = p
        T[0:3,0:3] = o
        return T


    def fkine_rleft(self):
        """
        Forward kinematics of the rear left leg with respect to the 
        inertial frame

        """
        T = self.fkine_rleft_base()
        Rb = rotationFromQuat(self.q[3:7])
        p = np.dot(Rb, T[0:3,3]) + self.q[0:3]
        o = np.dot(Rb, T[0:3,0:3])
        T[0:3,3] = p
        T[0:3,0:3] = o
        return T

    
    def jacobian_fright(self):
        """
        Jacobian for the front right with respect to the inertial frame

        """
        Jfr = self.jacobian_fright_base()
        Tfr = self.fkine_fright()
        J = np.zeros((6,19))
        R = rotationFromQuat(self.q[3:7])
        Tq = Tmat(self.q)
        J[0:3,7:10] = np.dot(R, Jfr[0:3,:])
        J[3:6,7:10] = np.dot(R, Jfr[3:6,:])
        J[0:3,0:3] = np.eye(3)
        J[0:3,3:7] = np.dot(skew(self.q[0:3]-Tfr[0:3,3]), Tq)
        J[3:6,3:7] = Tq
        return J

    def jacobian_fleft(self):
        """
        Jacobian for the front left with respect to the inertial frame

        """
        Jfl = self.jacobian_fleft_base()
        Tfl = self.fkine_fleft()
        J = np.zeros((6,19))
        R = rotationFromQuat(self.q[3:7])
        Tq = Tmat(self.q)
        J[0:3,10:13] = np.dot(R, Jfl[0:3,:])
        J[3:6,10:13] = np.dot(R, Jfl[3:6,:])
        J[0:3,0:3] = np.eye(3)
        J[0:3,3:7] = np.dot(skew(self.q[0:3]-Tfl[0:3,3]), Tq)
        J[3:6,3:7] = Tq
        return J

    
    def jacobian_rright(self):
        """
        Jacobian for the rear right with respect to the inertial frame

        """
        Jrr = self.jacobian_rright_base()
        Trr = self.fkine_rright()
        J = np.zeros((6,19))
        R = rotationFromQuat(self.q[3:7])
        Tq = Tmat(self.q)
        J[0:3,13:16] = np.dot(R, Jrr[0:3,:])
        J[3:6,13:16] = np.dot(R, Jrr[3:6,:])
        J[0:3,0:3] = np.eye(3)
        J[0:3,3:7] = np.dot(skew(self.q[0:3]-Trr[0:3,3]), Tq)
        J[3:6,3:7] = Tq
        return J

    def jacobian_rleft(self):
        """
        Jacobian for the rear left with respect to the inertial frame

        """
        Jrl = self.jacobian_rleft_base()
        Trl = self.fkine_rleft()
        J = np.zeros((6,19))
        R = rotationFromQuat(self.q[3:7])
        Tq = Tmat(self.q)
        J[0:3,16:19] = np.dot(R, Jrl[0:3,:])
        J[3:6,16:19] = np.dot(R, Jrl[3:6,:])
        J[0:3,0:3] = np.eye(3)
        J[0:3,3:7] = np.dot(skew(self.q[0:3]-Trl[0:3,3]), Tq)
        J[3:6,3:7] = Tq
        return J

    """
    ================================================================

    Tasks for position, orientation and pose
    
    ================================================================
    """

    def error_position_fright(self, pdes):
        e = pdes - self.fkine_fright()[0:3,3]
        return e

    def error_position_fleft(self, pdes):
        e = pdes - self.fkine_fleft()[0:3,3]
        return e

    def error_position_rright(self, pdes):
        e = pdes - self.fkine_rright()[0:3,3]
        return e

    def error_position_rleft(self, pdes):
        e = pdes - self.fkine_rleft()[0:3,3]
        return e

    def taskj_position_fright(self):
        J = self.jacobian_fright()
        return J[0:3,:]

    def taskj_position_fleft(self):
        J = self.jacobian_fleft()
        return J[0:3,:]

    def taskj_position_rright(self):
        J = self.jacobian_rright()
        return J[0:3,:]

    def taskj_position_rleft(self):
        J = self.jacobian_rleft()
        return J[0:3,:]
    

    def fkine_base(self):
        return self.q[0:7]

    def error_position_base(self, pdes):
        e = pdes - self.fkine_base()[0:3]
        return e

    def taskj_position_base(self):
        # Trr = self.fkine_rright()
        J = np.zeros((3,19))
        # R = rotationFromQuat(self.q[3:7])
        # Tq = Tmat(self.q)
        # J[0:3,13:16] = np.dot(R, Jrr[0:3,:])
        # J[3:6,13:16] = np.dot(R, Jrr[3:6,:])
        J[0:3,0:3] = np.eye(3)
        # J[0:3,3:7] = np.dot(skew(self.q[0:3]-Trr[0:3,3]), Tq)
        # J[3:6,3:7] = Tq
        return J

    def error_pose_base(self, pdes):
        e = np.zeros(7)
        e[0:3] = pdes[0:3] - self.fkine_base()[0:3]
        Q = self.fkine_base()[3:7]
        Qe = quaternionMult(pdes[3:7], np.array([Q[0],-Q[1],-Q[2],-Q[3]]) )
        e[3:7] = np.array([Qe[0]-1.0, Qe[1], Qe[2], Qe[3]])
        return e
    
    def taskj_pose_base(self):
        J = np.zeros((7,19))
        # Tq = Tmat(self.q)
        J[0:3,0:3] = np.eye(3)
        J[3:7,3:7] = np.eye(4)
        # J[3:6,3:7] = Tq
        return J
