import numpy as np

# RRT
trials_obs2 = np.array([2.718, 1.704, 1.211, 1.532, 1.277, 1.171, 1.823, 1.273, 1.759, 2.140, 1.732, 1.834, 1.020, 1.359, 2.111])
trials_obs3 = np.array([4.321, 4.565, 2.078, 3.355, 2.088, 2.883, 4.084, 3.757, 2.970, 3.905, 2.464, 3.817, 2.411, 3.534, 3.248])
print(f"mean and std of computational time of map 2 {np.mean(trials_obs2)} and {np.std(trials_obs2)}, respectively.")
print(f"mean and std of computational time of map 3 {np.mean(trials_obs3)} and {np.std(trials_obs3)}, respectively.")

# Goal Oriented RRT
# trials_obs3 = np.array([2.788, 3.160, 4.0219, 3.676, 3.620, 2.966, 3.007, 2.405, 2.396, 3.757, 2.525, 2.890, 1.905, 2.578, 2.776])
# trials_obs2 = np.array([1.174, 2.219, 1.546, 1.055, 1.119, 1.302, 1.082, 0.908, 0.943, 1.823, 1.018, 1.392, 1.932, 0.920, 1.449])
# print(f"mean and std of computational time of map 3 {np.mean(trials_obs3)} and {np.std(trials_obs3)}, respectively.")
# print(f"mean and std of computational time of map 2 {np.mean(trials_obs2)} and {np.std(trials_obs2)}, respectively.")