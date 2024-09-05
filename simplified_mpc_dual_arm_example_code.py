import numpy as np
from scipy.optimize import minimize

# Define the robot dynamics
def robot_dynamics(x, u, dt=0.1):
    # Simple linear model: x(t+1) = x(t) + u * dt
    return x + u * dt

# Cost function for MPC
def cost_function(u, *args):
    x, x_target, N, Q, R = args  # Current state, target state, horizon, weights
    cost = 0
    for k in range(N):
        x = robot_dynamics(x, u[k*14:(k+1)*14])
        cost += np.sum(Q * (x - x_target)**2)  # Tracking error
        cost += np.sum(R * u[k*14:(k+1)*14]**2)  # Control effort
    return cost

# MPC controller
def mpc_control(x, x_target, N=10, dt=0.1):
    # Weights for the cost function
    Q = np.ones(17)  # State error penalty (joint angles + object position)
    R = np.ones(14) * 0.01  # Control effort penalty

    # Initial guess for control inputs (small movements)
    u0 = np.zeros(N * 14)

    # Bounds on control inputs (joint angle changes)
    bounds = [(-0.1, 0.1)] * (N * 14)

    # Optimize control inputs
    result = minimize(cost_function, u0, args=(x, x_target, N, Q, R), bounds=bounds)

    # Return the first control input in the optimal sequence
    return result.x[:14]

# Simulate the task
def simulate_mpc():
    # Initial joint positions and object position
    x = np.concatenate([np.random.uniform(-1.0, 1.0, 14), [5.0, 0.0, 0.0]])  # 14 joint angles + object position
    x_target = np.concatenate([np.zeros(14), [10.0, 0.0, 0.0]])  # Target joint angles + object target position

    for t in range(50):
        # Get the control input from MPC
        u = mpc_control(x, x_target)

        # Apply control input and simulate the system
        x = robot_dynamics(x, u)

        print(f"Time step {t}, State: {x[:14]} (joint angles), Object Position: {x[14:]}")

# Run the simulation
simulate_mpc()
