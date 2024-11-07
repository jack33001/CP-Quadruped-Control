import numpy as np

class leg:
    def __init__(self):
        self.l1 = 0.15
        self.l2 = 0.15
        self.step_height = 0.15
        
    # Generate the keypoints and times for a joint trajectory to be passed to ros2_control
    def generate_joint_trajectory(self,end_pos,end_time):
        # Define the keypoints in cartesian space
        mid_pos = [0.0,self.step_height]
        midpoint_time = end_time/2
        
        # convert cartesian values to joint space
        mid_pos = self.cartesian_to_joint(mid_pos)
        end_pos = self.cartesian_to_joint(end_pos)
        
        return [[mid_pos,midpoint_time],[end_pos,end_time]]

    # System jacobian
    def jacobian(self, th_vector):
            th1 = th_vector[0]
            th2 = th_vector[1]
            j_11 = self.l1 * np.cos(th1) + self.l2 * np.cos(th1 + th2)
            j_12 = self.l2 * np.cos(th1 + th2)
            j_21 = -self.l1 * np.sin(th1) - self.l2 * np.sin(th1 + th2)
            j_22 = -self.l2 * np.sin(th1 + th2)
            J = np.array([[j_11, j_12], [j_21, j_22]])
            return J

    # Inverse Kinematics - Solution
    def cartesian_to_joint(self, cartesian): 
        solution = [0, 0]
        solution[1] = np.arccos(
            (cartesian[0]**2 + cartesian[1]**2 - self.l1**2 - self.l2**2) /
            (2 * self.l1 * self.l2))
        
        if cartesian[1] != 0:
            solution[0] = np.arctan(cartesian[0] / cartesian[1]) - np.arctan(
                (self.l2 * np.sin(solution[1])) /
                (self.l1 + self.l2 * np.cos(solution[1])))
        else:
            solution[0] = np.arctan(
                (self.l2 * np.sin(solution[1])) /
                (self.l1 + self.l2 * np.cos(solution[1])))
            
        return solution

    # Forward kinematics - joint space to cartesian space
    def joint_to_cartesian(self, th1, th2):
        # calculate x and y positions
        x = self.l2 * np.sin(th1 + th2) + self.l1 * np.sin(th1)
        y = self.l2 * np.cos(th1 + th2) + self.l1 * np.cos(th1)

        return np.array([x, y])

    # grf to joint torques - grf space to joint space
    def force_map(self, F):
        transformed_curr_pos = self.curr_pos - np.array([np.pi / 2, 0])
        # calculate the Jacobian
        j = self.jacobian(transformed_curr_pos)

        # Transform from grf space to joint space
        torque = -np.dot(np.transpose(j), F)

        return torque