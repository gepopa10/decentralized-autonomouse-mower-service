from math import atan, pi, cos, sin

class NonLinearDifferentialController(object):
    def __init__(self, dt, r, L, kp_pos = 0.8, kp_theta = 2, dist_precision = 0.3, yaw_precision = 1*pi / 180, turning_speed = 2):
        self.dt = dt

        self.kp_pos = kp_pos #0.5 #1
        self.kp_theta = kp_theta #0.1 #10 with rectangular one #5
        self.t = 0
        self.r = r # rayon roue (in m)
        self.L = L # distance between wheels in m

        self.yaw_precision = yaw_precision # +/- 5 degree allowed
        self.dist_precision = dist_precision
        self.turning_speed = turning_speed
        self.straigth_speed = 10 # currently not used

    def update_params(self, kp_pos, kp_theta):
        self.kp_pos = kp_pos
        self.kp_theta = kp_theta

    def get_action(self, state, state_target): # state should be x, y, theta
        x, y, theta = state
        x_t, y_t, theta_t = state_target
        # increment time step
        self.t += self.dt

        d = ((y_t - y)**2 + (x_t - x)**2)**(0.5)
        error_pos = d

        error_theta = theta_t - theta
        # if abs(error_theta) > 3.1416:
        #     error_theta = theta - theta_t

        if abs(error_theta) > pi:
            if error_theta > 0:
                error_theta = error_theta - 2*pi
            else:
                error_theta = error_theta + 2*pi

        # fix heading
        if abs(error_theta) > self.yaw_precision and abs(error_theta) > 25*pi / 180:
            # print("aaaaaaaaaaaaaaaaa", abs(error_theta))
            # turn left
            if error_theta > 0:
                vl = -self.turning_speed
                vr = self.turning_speed
            # turn right
            else:
                vl = self.turning_speed
                vr = -self.turning_speed
        # go straight
        # print(error_pos)

        # with user
        if (error_pos>self.dist_precision):
            # if abs(error_theta) > pi/2:
            #     d = -d
            #     if error_theta < 0:
            #         error_theta = pi + error_theta
            #     else:
            #         error_theta = pi - error_theta

            v = self.kp_pos*d*cos(error_theta)
            w = self.kp_pos*sin(error_theta)*cos(error_theta) + self.kp_theta*error_theta
            # print('iciiiii', v, ' ', w, ' ', error_theta, ' ', error_pos)
            vr = 0.5*w*self.L/self.r + v/self.r
            vl = v/self.r - 0.5*w*self.L/self.r
        # stop
        else:
            vl = 0
            vr = 0


        return -vl, -vr
