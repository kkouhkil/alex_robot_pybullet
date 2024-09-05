import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import gym

# Define a simple continuous environment
class DualArmRobotEnv(gym.Env):
    def __init__(self):
        super(DualArmRobotEnv, self).__init__()
        # Define action and observation space
        # Action: Change in joint angles for both arms
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(14,), dtype=np.float32)  # 7 joints per arm

        # Observation: Joint angles + object position
        self.observation_space = gym.spaces.Box(low=-10.0, high=10.0, shape=(17,), dtype=np.float32)  # 14 for joints + 3 for object position

        # Initial positions
        self.initial_joint_positions = np.zeros(14)  # Both arms start with all joints at 0
        self.object_position = np.array([5.0, 0.0, 0.0])  # Object's initial position
        self.target_position = np.array([10.0, 0.0, 0.0])  # Target position to place the object

        self.reset()

    def reset(self):
        # Reset joint positions and object position
        self.joint_positions = np.random.uniform(-1.0, 1.0, size=(14,))
        self.object_held = False
        return np.concatenate((self.joint_positions, self.object_position))

    def step(self, action):
        # Apply action to move joint positions
        self.joint_positions += action
        reward = 0
        done = False

        # Check if arms are near the object
        if np.linalg.norm(self.joint_positions[:3] - self.object_position) < 0.1:
            self.object_held = True  # Grasp the object
            reward += 10  # Reward for grasping

        # If object is held, move it
        if self.object_held:
            self.object_position = self.joint_positions[:3]  # Move object with arm

        # Check if the object is moved to the target position
        if self.object_held and np.linalg.norm(self.object_position - self.target_position) < 0.1:
            reward += 50  # Reward for successfully placing the object
            done = True

        # Observation: Joint angles + object position
        state = np.concatenate((self.joint_positions, self.object_position))
        return state, reward, done, {}

    def render(self):
        # For simplicity, we won’t render in this example
        pass

# Actor-Critic Network
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

# Training function
def train(env, model, optimizer, num_episodes=1000, gamma=0.99):
    for episode in range(num_episodes):
        state = env.reset()
        log_probs = []
        values = []
        rewards = []
        
        done = False
        while not done:
            state = torch.FloatTensor(state)
            action_mean, value = model(state)
            
            # Sample action from normal distribution with the mean predicted by the actor
            action_dist = torch.distributions.Normal(action_mean, torch.ones_like(action_mean) * 0.1)
            action = action_dist.sample()
            log_prob = action_dist.log_prob(action).sum()

            next_state, reward, done, _ = env.step(action.detach().numpy())

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

# Initialize environment and model
env = DualArmRobotEnv()
input_dim = env.observation_space.shape[0]
action_dim = env.action_space.shape[0]
model = ActorCritic(input_dim, action_dim)
optimizer = optim.Adam(model.parameters(), lr=1e-3)

# Train the model
train(env, model)
