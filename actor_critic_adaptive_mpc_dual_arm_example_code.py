import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
from scipy.optimize import minimize

# Actor-Critic model
class ActorCritic(nn.Module):
    def __init__(self, input_dim, action_dim):
        super(ActorCritic, self).__init__()
        self.fc1 = nn.Linear(input_dim, 128)
        self.fc2 = nn.Linear(128, 128)

        # Actor: Outputs mean of actions (for continuous control)
        self.actor = nn.Linear(128, action_dim)
        
        # Critic: Outputs state-value
        self.critic = nn.Linear(128, 1)

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        
        action_mean = self.actor(x)
        value = self.critic(x)
        return action_mean, value

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

# Hybrid control combining Actor-Critic with Adaptive MPC
def hybrid_actor_critic_mpc(env, actor_critic, optimizer, num_episodes=1000, gamma=0.99, model_params=None):
    for episode in range(num_episodes):
        state = env.reset()
        log_probs = []
        values = []
        rewards = []
        
        done = False
        while not done:
            state_tensor = torch.FloatTensor(state)
            action_mean, value = actor_critic(state_tensor)
            
            # Sample high-level action from the actor's policy
            action_dist = torch.distributions.Normal(action_mean, torch.ones_like(action_mean) * 0.1)
            high_level_action = action_dist.sample()
            log_prob = action_dist.log_prob(high_level_action).sum()

            # Use Adaptive MPC to optimize the low-level control actions
            low_level_action = adaptive_mpc_control(state, high_level_action.detach().numpy(), model_params)

            # Apply the low-level action and get the new state from the environment
            next_state, reward, done, _ = env.step(low_level_action)

            log_probs.append(log_prob)
            values.append(value)
            rewards.append(reward)

            state = next_state

        # Compute returns and advantages
        returns = []
        R = 0
        for r in reversed(rewards):
            R = r + gamma * R
            returns.insert(0, R)

        returns = torch.FloatTensor(returns)
        values = torch.cat(values)
        log_probs = torch.cat(log_probs)

        advantages = returns - values.detach()

        # Calculate loss: Policy loss + Value loss
        actor_loss = -(log_probs * advantages).mean()
        critic_loss = (returns - values).pow(2).mean()

        loss = actor_loss + critic_loss

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        if episode % 10 == 0:
            print(f"Episode {episode}, Loss: {loss.item()}")

# Simulate the task using the hybrid approach
def simulate_hybrid_actor_critic_mpc():
    # Initialize environment and model
    env = DualArmRobotEnv()
    input_dim = env.observation_space.shape[0]
    action_dim = env.action_space.shape[0]
    actor_critic = ActorCritic(input_dim, action_dim)
    optimizer = optim.Adam(actor_critic.parameters(), lr=1e-3)
    
    # Initial model parameters
    model_params = np.ones(14)

    # Run the hybrid Actor-Critic + Adaptive MPC approach
    hybrid_actor_critic_mpc(env, actor_critic, optimizer, model_params=model_params)

# Run the hybrid simulation
simulate_hybrid_actor_critic_mpc()
