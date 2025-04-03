import numpy as np
import matplotlib.pyplot as plt

goal_point = np.array([100, 90], dtype=np.float64)
robot_initial_position = np.array([0, 0], dtype=np.float64)
obstacle_locations = np.array([[10, 30],[50, 60],[70, 80], [77, 79], [90,50]], dtype=np.float64)
k = 10
virtual_expansion = 5
step_size = 0.5

def attractive_force(position, k):
    if np.linalg.norm(position - goal_point) < 1e-6:
        return np.array([0, 0], dtype=np.float64)
    
    f = k*(position - goal_point)/np.linalg.norm(position - goal_point)
    return f

def repulsive_potential(position, k, r):
    U_rep = np.exp(k * (r - np.linalg.norm(position - goal_point)))
    return U_rep

def repulsive_force(position, k, r):
    U_rep = repulsive_potential(position,k,r)

    norm_of_vector = np.linalg.norm(position - goal_point)
    
    if norm_of_vector < 1e-6:
        norm_of_vector = 1e-6
    
    f = U_rep * (position - goal_point)/norm_of_vector
    return f

def simulate():
    position = robot_initial_position
    path = [position.copy()]

    for _ in range(1000):
        velocity = repulsive_force(position, k, virtual_expansion) - attractive_force(position, k)
        # print(velocity)
        velocity_norm = np.linalg.norm(velocity)
        
        if velocity_norm < 1e-6:
            velocity_norm = 1e-6
        
        position += step_size * velocity/velocity_norm

        path.append(position.copy())

        if np.linalg.norm(position - goal_point) < 0.03:
            break

    return np.array(path)

path = simulate()


plt.scatter(goal_point[0], goal_point[1], marker="*", color="g", s=200, label="Goal")
plt.plot(path[:,0], path[:,1], '-b', label="Robot Path")

N, M = obstacle_locations.shape

for i in range(N):
    plt.scatter(obstacle_locations[i,0], obstacle_locations[i,1], marker="o", color="r", s=100, label="Obstacle")

plt.legend()
plt.xlim(0, 150)
plt.ylim(0, 150)
plt.grid()
plt.title("Vector Field Path Planning")
plt.show()