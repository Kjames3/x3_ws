import numpy as np
from scipy.optimize import minimize
import math

class HolonomicCBFFilter:
    def __init__(self, safe_distance=0.3, gamma=1.0):
        """
        Args:
            safe_distance (float): The minimum allowable distance to any obstacle (meters).
                                   Set to 0.3 for the X3 robot chassis + padding.
            gamma (float): How aggressively the CBF intervenes. 
        """
        self.safe_distance = safe_distance
        self.gamma = gamma

    def filter_velocity(self, v_nom_x, v_nom_y, robot_pos, obstacles):
        """
        Filters the requested velocities to guarantee collision avoidance.
        
        Args:
            v_nom_x (float): Commanded velocity in X (m/s)
            v_nom_y (float): Commanded velocity in Y (m/s)
            robot_pos (tuple): Current (x, y) position of the robot in global/local frame
            obstacles (list of tuples): List of (x, y) coordinates of obstacles in the SAME frame
            
        Returns:
            tuple: (safe_v_x, safe_v_y)
        """
        if not obstacles:
            return v_nom_x, v_nom_y

        rx, ry = robot_pos
        u_nom = np.array([v_nom_x, v_nom_y])

        # Objective Function: Minimize || u - u_nom ||^2
        def objective(u):
            return 0.5 * np.sum((u - u_nom)**2)

        def jacobian(u):
            return u - u_nom

        constraints = []
        
        # Build a CBF constraint for EVERY obstacle
        for (ox, oy) in obstacles:
            dist_sq = (rx - ox)**2 + (ry - oy)**2
            h = dist_sq - self.safe_distance**2
            
            # Allow h to be negative. If the robot breaches the safe boundary (h < 0), 
            # the constraint dhx*u[0] + dhy*u[1] + gamma*h >= 0 will naturally force
            # the velocity to point away from the obstacle, acting as an automatic backup.
            
            # The gradient of h with respect to the robot's position
            dh_dx = 2 * (rx - ox)
            dh_dy = 2 * (ry - oy)
            
            constraints.append({
                'type': 'ineq',
                'fun': lambda u, dhx=dh_dx, dhy=dh_dy, h_val=h: dhx * u[0] + dhy * u[1] + self.gamma * h_val,
                'jac': lambda u, dhx=dh_dx, dhy=dh_dy: np.array([dhx, dhy])
            })

        u0 = u_nom
        
        max_speed = 1.5
        bounds = [(-max_speed, max_speed), (-max_speed, max_speed)]

        # Optimize the velocity safely
        res = minimize(objective, u0, method='SLSQP', jac=jacobian, bounds=bounds, constraints=constraints)

        if res.success:
            return float(res.x[0]), float(res.x[1])
        else:
            return 0.0, 0.0
