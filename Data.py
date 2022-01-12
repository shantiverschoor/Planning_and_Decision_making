import numpy as np

print("RRT")
time_obs2 = np.array([2.718, 1.704, 1.211, 1.532, 1.277, 1.171, 1.823, 1.273, 1.759, 2.140, 1.732, 1.834, 1.020, 1.359, 2.111])
time_obs3 = np.array([4.321, 4.565, 2.078, 3.355, 2.088, 2.883, 4.084, 3.757, 2.970, 3.905, 2.464, 3.817, 2.411, 3.534, 3.248])
print(f"mean and std of computational time of map 2 {np.mean(time_obs2)} and {np.std(time_obs2)}, respectively.")
print(f"mean and std of computational time of map 3 {np.mean(time_obs3)} and {np.std(time_obs3)}, respectively.")
path_obs3 = np.array([2847, 2546, 2644, 2848, 2685, 2971, 2862, 2891, 2693, 2701, 2814, 2712, 3098, 2799, 2678])
path_obs2 = np.array([2228, 2231, 2167, 2264, 2394, 2285, 2286, 2293, 2451, 2273, 2259, 2433, 2298, 2179, 2291])

print(f"mean and std of computational time of map 3 {np.mean(path_obs3)} and {np.std(path_obs3)}, respectively.")
print(f"mean and std of computational time of map 2 {np.mean(path_obs2)} and {np.std(path_obs2)}, respectively.")

print("Goal Oriented RRT")
time_obs3 = np.array([2.788, 3.160, 4.0219, 3.676, 3.620, 2.966, 3.007, 2.405, 2.396, 3.757, 2.525, 2.890, 1.905, 2.578, 2.776])
time_obs2 = np.array([1.174, 2.219, 1.546, 1.055, 1.119, 1.302, 1.082, 0.908, 0.943, 1.823, 1.018, 1.392, 1.932, 0.920, 1.449])
print(f"mean and std of computational time of map 3 {np.mean(time_obs3)} and {np.std(time_obs3)}, respectively.")
print(f"mean and std of computational time of map 2 {np.mean(time_obs2)} and {np.std(time_obs2)}, respectively.")

path_obs3 = np.array([2664, 2778, 2734, 2590, 2701, 2654, 2829, 2624, 2854, 2617, 2784, 2716, 2803, 2950, 2693])
path_obs2 = np.array([2239, 2296, 2182, 2448, 2282, 2343, 2281, 2332, 2150, 2318, 2323, 2158, 2161, 2462, 2403])
print(f"mean and std of computational time of map 3 {np.mean(path_obs3)} and {np.std(path_obs3)}, respectively.")
print(f"mean and std of computational time of map 2 {np.mean(path_obs2)} and {np.std(path_obs2)}, respectively.")