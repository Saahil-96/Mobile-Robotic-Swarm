import numpy as np
import pybullet as p
import itertools
import math

class Robot():
    """
    The class is the interface to a single robot
    """

    def __init__(self, init_pos, robot_id, dt):
        self.id = robot_id
        self.dt = dt
        self.pybullet_id = p.loadSDF("../models/robot.sdf")[0]
        self.joint_ids = list(range(p.getNumJoints(self.pybullet_id)))
        self.initial_position = init_pos
        self.reset()

        # No friction between bbody and surface.
        p.changeDynamics(self.pybullet_id, -1, lateralFriction=5., rollingFriction=0.)

        # Friction between joint links and surface.
        for i in range(p.getNumJoints(self.pybullet_id)):
            p.changeDynamics(self.pybullet_id, i, lateralFriction=5., rollingFriction=0.)

        self.messages_received = []
        self.messages_to_send = []
        self.neighbors = []
        self.L = np.array(([3, -1, -1, -1, 0, 0],
                      [-1, 3, -1, -1, 0, 0],
                      [-1, -1, 5, -1, -1, -1],
                      [-1, -1, -1, 5, -1, -1],
                      [0, 0, -1, -1, 3, -1],
                      [0, 0, -1, -1, -1, 3]))

        self.E = np.array(([-1, 1, 0, 0, 0, 1, -1, 0, 0, 0, 0],
                      [1, -1, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 1, -1, 0, 1, -1, 0, -1],
                      [0, 1, -1, 0, 0, 0, 1, -1, 0, -1, 0],
                      [0, 0, 0, 1, -1, 0, 0, 0, 0, 1, 0],
                      [0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 1]))
        # Variable For Control Law.
        self.K = 4

        # Variable For Potential Fields.
        self.sigma = 0.1

        # Variables For State Change.
        self.Stable = 0
        self.Stable2 = 0
        self.Stable3 = 0
        self.Stable4 = 0
        self.Stable5 = 0
        self.Stable6 = 0
        self.Stable7 = 0
        self.Stable8 = 0
        self.Stable9 = 0
        self.Stable10 = 0
        self.Stable11 = 0
        self.Stable12 = 0
        self.Stable13 = 0
        self.Stable14 = 0
        self.Stable15 = 0
        self.Stable16 = 0
        self.Stable17 = 0
        self.Stable18 = 0
        self.lead = 0

    def reset(self):
        """
        Moves the robot back to its initial position
        """
        p.resetBasePositionAndOrientation(self.pybullet_id, self.initial_position, (0., 0., 0., 1.))

    def set_wheel_velocity(self, vel):
        """
        Sets the wheel velocity,expects an array containing two numbers (left and right wheel vel)
        """
        assert len(vel) == 2, "Expect velocity to be array of size two"
        p.setJointMotorControlArray(self.pybullet_id, self.joint_ids, p.VELOCITY_CONTROL,
                                    targetVelocities=vel)

    def get_pos_and_orientation(self):
        """
        Returns the position and orientation (as Yaw angle) of the robot.
        """
        pos, rot = p.getBasePositionAndOrientation(self.pybullet_id)
        euler = p.getEulerFromQuaternion(rot)
        return np.array(pos), euler[2]

    def get_messages(self):
        """
        returns a list of received messages, each element of the list is a tuple (a,b)
        where a= id of the sending robot and b= message (can be any object, list, etc chosen by user)
        Note that the message will only be received if the robot is a neighbor (i.e. is close enough)
        """
        return self.messages_received

    def send_message(self, robot_id, message):
        """
        sends a message to robot with id number robot_id, the message can be any object, list, etc
        """
        self.messages_to_send.append([robot_id, message])

    def get_neighbors(self):
        """
        returns a list of neighbors (i.e. robots within 2m distance) to which messages can be sent
        """
        return self.neighbors

    def shape(self, Shape):

        if Shape=="Line":
            P_des_x = np.array(([0], [0.5], [0], [0.5], [0], [0.5]))
            P_des_y = np.array(([0], [0], [-0.5], [-0.5], [-1], [-1]))
        elif Shape == "Square":
            P_des_x = np.array(([0], [0], [0.5], [0.5], [1], [1]))
            P_des_y = np.array(([0], [1], [0], [1], [0], [1]))
        elif Shape == "Circle":
            P_des_x = np.array(([1], [1.75], [1.5], [2], [2], [2.5]))
            P_des_y = np.array(([0], [0.5], [-0.5], [0.5], [-0.5], [0]))
        elif Shape == "Diamond":
            P_des_x = np.array(([0], [0.5], [0.25], [0.75], [0.5], [1]))
            P_des_y = np.array(([0], [0.5], [-0.25], [0.25], [-0.5], [0]))
        elif Shape == "Gripper":
            P_des_x = np.array(([0], [0.5], [1], [1.5], [2], [2.5]))
            P_des_y = np.array(([0.5], [0.25], [0], [0], [0.5], [1]))
        elif Shape == "UGripper":
            P_des_x = np.array(([0], [0.25], [0.5], [0.75], [1], [1.25]))
            P_des_y = np.array(([0.5], [0.25], [0], [0], [0.25], [0.5]))

        return P_des_x, P_des_y

    def formation(self, P_des_x, P_des_y, x, y, dx, dy):
        z_ref_x = np.matmul(np.transpose(self.E), P_des_x)
        z_ref_y = np.matmul(np.transpose(self.E), P_des_y)

        # region Obstacle and Target Variables
        robo_obs_x = 0
        robo_obs_y = 0
        obs_x = 0
        obs_y = 0
        #
        # i = 0
        #
        # for i in range(x.shape[0]):
        #     if i==self.id:
        #         continue
        #     else:
        #         robo_obs_x += (0 * math.exp(-math.pow((x[self.id, 0] - x[i, 0]), 2)/self.sigma)*(x[self.id, 0] - x[i, 0]))/self.sigma
        #         robo_obs_y += (0 * math.exp(-math.pow((y[self.id, 0] - y[i, 0]), 2)/self.sigma)*(y[self.id, 0] - y[i, 0]))/self.sigma
        # endregion

        for i in range(x.shape[0]):
            obs_x += (0 * math.exp(-math.pow((x[self.id, 0] - x[i, 0]), 2)/self.sigma)*(x[self.id, 0] - x[i, 0]))/self.sigma
            obs_y += (0 * math.exp(-math.pow((y[self.id, 0] - y[i, 0]), 2)/self.sigma)*(y[self.id, 0] - y[i, 0]))/self.sigma

        U_x = robo_obs_x-self.K * np.matmul(self.L, x) + self.K * np.matmul(self.E, z_ref_x)
        U_y = robo_obs_y-self.K * np.matmul(self.L, y) + self.K * np.matmul(self.E, z_ref_y)

        dx += U_x[self.id]
        dy += U_y[self.id]

        if self.id == 0:
            dx = 0
            dy = 0

        dx = 5 * dx
        dy = 5 * dy
        return dx, dy



    def compute_controller(self):
        """
        function that will be called each control cycle which implements the control law
        TO BE MODIFIED

        we expect this function to read sensors (built-in functions from the class)
        and at the end to call set_wheel_velocity to set the appropriate velocity of the robots
        """

        # here we implement an example for a consensus algorithm
        neig = self.get_neighbors()
        messages = self.get_messages()
        pos, rot = self.get_pos_and_orientation()

        # send message of positions to all neighbors indicating our position
        for n in neig:
            self.send_message(n, pos)

        # region Variables
        dx = 0.
        dy = 0.
        x = np.zeros((6, 1))
        y = np.zeros((6, 1))
        curr_dis = np.zeros((6, 1))
        check = np.zeros((6, 1))
        des_dis = np.zeros((6, 1))
        # endregion

        # region Create Message Array
        if messages:
            for m in messages:
                x[m[0], 0] = m[1][0]
                y[m[0], 0] = m[1][1]
            x[self.id, 0] = pos[0]
            y[self.id, 0] = pos[1]
        # endregion

        # region Create Lead Variable For Times When It Is Important For Leader To Move First
        if abs(x[0, 0] - 0.5) <= 0.2 and abs(y[0, 0] + 0.5) <= 0.1 and self.Stable==0:
            self.lead = 1
            print("State1")
        elif abs(x[0, 0] - 1) <= 0.02 and abs(y[0, 0] + 0.5) >= 0.4 and self.Stable2==0:
            self.lead = 2
            print("State2")
        elif abs(x[0, 0]-0.5) <= 0.02 and abs(y[0, 0]-0.2) <= 0.2 and self.Stable3==0:
            self.lead = 3
            print("State3")
        elif abs(x[0, 0] - 2.15) <= 0.02 and abs(y[0, 0] - 1.8) <= 0.2 and self.Stable4==0:
            self.lead = 4
            print("State4")
        elif abs(x[0, 0] - 1.62) <= 0.02 and abs(y[0, 0] - 3.5) <= 0.2 and self.Stable6==0:
            self.lead = 7
            print("State7")
        # endregion

        # region Move Leaders
        if self.lead == 1 and self.id == 0 and self.Stable == 0:
            dx += 2 - x[0, 0] + self.dt * dx
            dy += 1 - y[0, 0] + self.dt * dx
            dx = 5*dx
            dy = 5*dy
            print("Self Lead 1")
        elif self.lead == 2 and self.id == 0 and self.Stable2 == 0:
            dx += -0.5 - x[0, 0] + self.dt * dx
            dy += 0.5 - y[0, 0] + self.dt * dx
            dx=5*dx
            dy=5*dy
            print("Self Lead 2")
        elif self.lead == 3 and self.id == 0 and self.Stable3 == 0:
            print("Self Lead 3")
            # No Need To Do Anything.
        elif self.lead == 4 and self.id == 0 and self.Stable4 == 0:
            print("Self Lead 4")
            # No Need To Do Anything.
        elif self.lead == 5 and self.id == 0 and self.Stable5 == 0:
            print("Self Lead 5")
            # No Need To Do Anything.
        elif self.lead == 6 and self.id == 0 and self.Stable6 == 0:
            print("Self Lead 6")
            # No Need To Do Anything.
        elif self.lead == 7 and self.id == 0 and self.Stable7 == 0:
            dx += -1.1 - x[0, 0] + self.dt * dx
            dy += 4 - y[0, 0] + self.dt * dx

            print(x[0,0],y[0,0])
            print("Self Lead 7")
            # No Need To Do Anything.
        elif self.lead == 8 and self.id == 0 and self.Stable8 == 0:
            print("Self Lead 8")
            # No Need To Do Anything.
        elif self.lead == 9 and self.id == 0 and self.Stable9 == 0:
            print("Self Lead 9")
            # No Need To Do Anything.
        elif self.lead == 10 and self.id == 0 and self.Stable10 == 0:
            print("Self Lead 9")
            # No Need To Do Anything.
        elif self.lead == 11 and self.id == 0 and self.Stable11 == 0:
            print("Self Lead 9")
            # No Need To Do Anything.
        elif self.lead == 12 and self.id == 0 and self.Stable12 == 0:
            print("Self Lead 12")
            # No Need To Do Anything.
        elif self.lead == 13 and self.id == 0 and self.Stable13 == 0:
            print("Self Lead 13")
            # No Need To Do Anything.
        elif self.lead == 14 and self.id == 0 and self.Stable14 == 0:
            print("Self Lead 14")
            # No Need To Do Anything.
        # endregion

        if self.Stable == 0:
            if abs(x[0, 0]-1) <= 0.02 and abs(y[0, 0]+0.5) >= 0.4 :
                P_des_x, P_des_y = self.shape("Square")

                for i in range(x.shape[0] - 1):
                    curr_dis[i] = abs(math.pow((math.pow(x[i]-x[i-1], 2)+math.pow(y[i]-y[i-1], 2)), 0.5))
                    des_dis[i] = abs(math.pow((math.pow(P_des_x[i]-P_des_x[i-1], 2)+math.pow(P_des_y[i]-P_des_y[i-1], 2)), 0.5))

                for j in range(x.shape[0]):
                    check[j]=abs(curr_dis[j]-des_dis[j])

                if np.mean(check) <= 0.008:
                    print("This State Complete")
                    self.Stable=1

                dx, dy = self.formation(P_des_x, P_des_y, x, y, dx, dy)
        elif self.Stable2 == 0:
            if abs(x[0, 0]-0.5) <= 0.02 and abs(y[0, 0]-0.2) <= 0.2 :
                P_des_x, P_des_y = self.shape("Line")

                for i in range(x.shape[0] - 1):
                    curr_dis[i] = abs(math.pow((math.pow(x[i]-x[i-1], 2)+math.pow(y[i]-y[i-1], 2)), 0.5))
                    des_dis[i] = abs(math.pow((math.pow(P_des_x[i]-P_des_x[i-1], 2)+math.pow(P_des_y[i]-P_des_y[i-1], 2)), 0.5))

                for j in range(x.shape[0]):
                    check[j]=abs(curr_dis[j]-des_dis[j])

                if np.mean(check) <= 0.008:
                    print("This State Complete")
                    self.Stable2=1

                dx, dy = self.formation(P_des_x, P_des_y, x, y, dx, dy)
        elif self.Stable3 == 0:
            if abs(x[0, 0]-2.4) >= 0.02 and abs(y[0, 0]-2) >= 0.2 :
                if self.id==0:
                    dx += 3.5 - x[0, 0]
                    dy += 3 - y[0, 0]
                    dx=2*dx
                    dy=2*dy
                else:
                    P_des_x, P_des_y = self.shape("Line")
                    dx, dy = self.formation(P_des_x, P_des_y, x, y, dx, dy)
            else:
                print("Point Reached")
                self.Stable3=1
        elif self.Stable4 == 0:
            if abs(y[0, 0]-3) >= 0.2 :
                if self.id==0:
                    dy += 8 - y[0, 0]
                    print(x[0,0],y[0,0])
                else:
                    P_des_x, P_des_y = self.shape("Line")
                    dx, dy = self.formation(P_des_x, P_des_y, x, y, dx, dy)
            else:
                print("Point Reached")
                self.Stable4=1
        elif self.Stable5 == 0:
            if abs(y[0, 0] - 3.6) >= 0.2:
                if self.id == 0:
                    dy += 3.9 - y[0, 0]
                    print(x[0, 0], y[0, 0])
                    print("Stable 6")
                else:
                    P_des_x, P_des_y = self.shape("Line")
                    dx, dy = self.formation(P_des_x, P_des_y, x, y, dx, dy)
            else:
                print("Point Reached")
                self.Stable5 = 1
        elif self.Stable6 == 0:
            if abs(x[0, 0] - 1.6) >= 0.02:
                if self.id == 0:
                    dx += 1 - x[0, 0]
                    print("Stable 6")
                    dx = 2 * dx

                else:
                    P_des_x, P_des_y = self.shape("Line")
                    dx, dy = self.formation(P_des_x, P_des_y, x, y, dx, dy)
            else:
                print("Point Reached")
                self.Stable6 = 1
        elif self.Stable7 == 0:

            if abs(x[0, 0]-1.035) <= 0.02 and abs(y[0, 0]-3.6) <= 0.2 :
                P_des_x, P_des_y = self.shape("Gripper")

                for i in range(x.shape[0] - 1):
                    curr_dis[i] = abs(math.pow((math.pow(x[i]-x[i-1], 2)+math.pow(y[i]-y[i-1], 2)), 0.5))
                    des_dis[i] = abs(math.pow((math.pow(P_des_x[i]-P_des_x[i-1], 2)+math.pow(P_des_y[i]-P_des_y[i-1], 2)), 0.5))

                for j in range(x.shape[0]):
                    check[j]=abs(curr_dis[j]-des_dis[j])

                if np.mean(check) <= 0.30:
                    print("This State Complete")
                    self.Stable7=1

                dx, dy = self.formation(P_des_x, P_des_y, x, y, dx, dy)
        elif self.Stable8 == 0:
            if abs(y[0, 0] - 5) >= 0.02:
                if self.id == 0:
                    dy += 5.5 - y[0, 0]
                    dy = 2 * dy

                else:
                    P_des_x, P_des_y = self.shape("Gripper")
                    dx, dy = self.formation(P_des_x, P_des_y, x, y, dx, dy)
            else:
                print("Point Reached")
                self.Stable8 = 1
        elif self.Stable9 == 0:
            if abs(x[0, 0] - 1) >= 0.02 and abs(y[0, 0] - 3.6) >= 0.02:
                if self.id == 0:
                    dy += 3 - y[0, 0]
                    dx += 1 - x[0, 0]
                    dx = 2 * dx
                    dy = 2 * dy

                else:
                    P_des_x, P_des_y = self.shape("Gripper")
                    dx, dy = self.formation(P_des_x, P_des_y, x, y, dx, dy)
            else:
                print("Point Reached")
                self.Stable9 = 1
        elif self.Stable10 == 0:
            if abs(x[0, 0] - 4.5) >= 0.02:
                if self.id == 0:
                    dx += 5 - x[0, 0]
                    dx = 2 * dx

                else:
                    P_des_x, P_des_y = self.shape("Gripper")
                    dx, dy = self.formation(P_des_x, P_des_y, x, y, dx, dy)
            else:
                print("Point Reached")
                self.Stable10 = 1
        elif self.Stable11 == 0:
            if abs(y[0, 0]-0.7) >= 0.02:
                if self.id == 0:
                    dy += y[0, 0] - 6
                    dy = 2 * dy

                else:
                    P_des_x, P_des_y = self.shape("Gripper")
                    dx, dy = self.formation(P_des_x, P_des_y, x, y, dx, dy)
            else:
                print("Point Reached")
                self.Stable11 = 1
        elif self.Stable12 == 0:
            if abs(x[0, 0] - 3.4) >= 0.02:
                if self.id == 0:
                    dx += 2 - x[0, 0]
                    dx = 2 * dx

                else:
                    P_des_x, P_des_y = self.shape("Gripper")
                    dx, dy = self.formation(P_des_x, P_des_y, x, y, dx, dy)

            else:
                print("Point Reached")
                self.Stable12 = 1
        elif self.Stable13 == 0:
            if abs(3-y[0, 0]) >= 0.02:
                if self.id == 0:
                    dy += 3.2 - y[0, 0]
                    dy = 2 * dy

                else:
                    P_des_x, P_des_y = self.shape("Gripper")
                    dx, dy = self.formation(P_des_x, P_des_y, x, y, dx, dy)
            else:
                print("Point Reached")
                self.Stable13 = 1
        elif self.Stable14 == 0:
            if abs(-1-x[0, 0]) >= 0.2:
                if self.id == 0:
                    dx = - 0.5 + x[0, 0]

                else:
                    P_des_x, P_des_y = self.shape("Gripper")
                    dx, dy = self.formation(P_des_x, P_des_y, x, y, dx, dy)
            else:
                print("Point Reached")
                self.Stable14 = 1
        elif self.Stable15 == 0:
            if abs(y[0, 0]-5.5) >= 0.2:
                if self.id == 0:
                    dy = 5.5 - y[0, 0]
                    dy = 0.2*dy

                else:
                    P_des_x, P_des_y = self.shape("Gripper")
                    dx, dy = self.formation(P_des_x, P_des_y, x, y, dx, dy)
            else:
                print("Point Reached")
                self.Stable15 = 1

        # region Give Velocity To Robots
        vel_norm = np.linalg.norm([dx, dy])  # norm of desired velocity
        if vel_norm < 0.01:
            vel_norm = 0.01
        des_theta = np.arctan2(dy / vel_norm, dx / vel_norm)
        right_wheel = np.sin(des_theta - rot) * vel_norm + np.cos(des_theta - rot) * vel_norm
        left_wheel = -np.sin(des_theta - rot) * vel_norm + np.cos(des_theta - rot) * vel_norm
        self.set_wheel_velocity([left_wheel, right_wheel])
        # endregion







