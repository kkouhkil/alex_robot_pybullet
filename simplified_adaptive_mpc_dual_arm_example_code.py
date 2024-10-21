import numpy as np
from scipy.optimize import minimize

# Define the robot dynamics with a model parameter that can change
def robot_dynamics(x, u, dt=0.1, model_params=None):
    if model_params is None:
        model_params = np.ones(14)  # Default linear model parameters for each joint
    # Modify the dynamics based on the model parameters (e.g., joint stiffness or load)
    return x + u * dt * model_params

# Cost function for Adaptive MPC
def cost_function(u, *args):
    x, x_target, N, Q, R, model_params = args  # Current state, target state, horizon, weights, model parameters
    cost = 0
    for k in range(N):
        x = robot_dynamics(x, u[k*14:(k+1)*14], model_params=model_params)
        cost += np.sum(Q * (x - x_target)**2)  # Tracking error
        cost += np.sum(R * u[k*14:(k+1)*14]**2)  # Control effort
    return cost

# Adaptive MPC controller
def adaptive_mpc_control(x, x_target, model_params, N=10, dt=0.1):
    # Weights for the cost function
    Q = np.ones(17)  # State error penalty (joint angles + object position)
    R = np.ones(14) * 0.01  # Control effort penalty

    # Initial guess for control inputs (small movements)
    u0 = np.zeros(N * 14)

    # Bounds on control inputs (joint angle changes)
    bounds = [(-0.1, 0.1)] * (N * 14)

    # Optimize control inputs using the current model parameters
    result = minimize(cost_function, u0, args=(x, x_target, N, Q, R, model_params), bounds=bounds)

    # Return the first control input in the optimal sequence
    return result.x[:14]

# Simulate the task with adaptive MPC
def simulate_adaptive_mpc():
    # Initial joint positions and object position
    x = np.concatenate([np.random.uniform(-1.0, 1.0, 14), [5.0, 0.0, 0.0]])  # 14 joint angles + object position
    x_target = np.concatenate([np.zeros(14), [10.0, 0.0, 0.0]])  # Target joint angles + object target position

    # Initial model parameters (e.g., assume normal joint behavior)
    model_params = np.ones(14)  # Default parameters for each joint

    for t in range(50):
        # Update model parameters based on observations (adaptive)
        # Example: If the object is heavy, increase the control effort needed
        if np.linalg.norm(x[14:] - x_target[14:]) < 2.0:  # Object is being moved
            model_params += 0.01  # Increase the effort needed to move the object

        # Get the control input from Adaptive MPC
        u = adaptive_mpc_control(x, x_target, model_params)

        # Apply control input and simulate the system with updated dynamics
        x = robot_dynamics(x, u, model_params=model_params)

        print(f"Time step {t}, State: {x[:14]} (joint angles), Object Position: {x[14:]}, Model Params: {model_params}")

# Run the simulation
simulate_adaptive_mpc()
