import pandas as pd
import matplotlib.pyplot as plt

# Load data
z_std = pd.read_csv('../../z_std_cvat.csv')
z_noise = pd.read_csv('../../z_noise_cvat.csv')
z_filter = pd.read_csv('../../z_filt_cvat_cpp_normal.csv')
u_prob = pd.read_csv('../../prob_cvat_cpp_normal.csv')

# Calculate position and speed errors
position_error_noise_x = z_noise['x'] - z_std['x']
position_error_noise_y = z_noise['y'] - z_std['y']
position_error_filter_x = z_filter['x'] - z_std['x']
position_error_filter_y = z_filter['y'] - z_std['y']
speed_error_noise_x = z_noise['vx'] - z_std['vx']
speed_error_noise_y = z_noise['vy'] - z_std['vy']
speed_error_filter_vx = z_filter['vx'] - z_std['vx']
speed_error_filter_vy = z_filter['vy'] - z_std['vy']

# Create a single figure with subplots
plt.figure(figsize=(15, 20))

# Plot 1: Position comparison
# plt.subplot(321)
plt.subplot(511)
plt.plot(z_std['x'], z_std['y'], label='pos_std', color='red')
plt.plot(z_noise['x'], z_noise['y'], label='pos_noise', color='green')
plt.plot(z_filter['x'], z_filter['y'], label='pos_filter', color='blue')
plt.title('Position Comparison')
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()

# Plot 2: Speed comparison
plt.subplot(512)
plt.plot(z_std['vx'], z_std['vy'], label='spd_std', color='red')
plt.plot(z_noise['vx'], z_noise['vy'], label='spd_noise', color='green')
plt.plot(z_filter['vx'], z_filter['vy'], label='spd_filter', color='blue')
plt.title('Speed Comparison')
plt.xlabel('Speed X')
plt.ylabel('Speed Y')
plt.legend()

# Plot 3: Position error over frames
plt.subplot(513)
frame_numbers = range(len(position_error_noise_x))
plt.plot(frame_numbers, position_error_noise_x, label='Noise Position Error X', color='red')
plt.plot(frame_numbers, position_error_noise_y, label='Noise Position Error Y', color='green')
plt.plot(frame_numbers, position_error_filter_x, label='Filter Position Error X', color='blue')
plt.plot(frame_numbers, position_error_filter_y, label='Filter Position Error Y', color='purple')
plt.title('Position Error over Frames')
plt.xlabel('Frame Number')
plt.ylabel('Position Error')
plt.legend()

# Plot 4: Speed error over frames
plt.subplot(514)
plt.plot(frame_numbers, speed_error_noise_x, label='Noise Speed Error X', color='red')
plt.plot(frame_numbers, speed_error_noise_y, label='Noise Speed Error Y', color='green')
plt.plot(frame_numbers, speed_error_filter_vx, label='Filter Speed Error DX', color='blue')
plt.plot(frame_numbers, speed_error_filter_vy, label='Filter Speed Error DY', color='purple')
plt.title('Speed Error over Frames')
plt.xlabel('Frame Number')
plt.ylabel('Speed Error')
plt.legend()

# Plot 5: Probability of CV & CT Models over Frames
plt.subplot(515)
for column in u_prob.columns:
    plt.plot(u_prob.index, u_prob[column], label=f'Prob {column}')

# Set title and labels
plt.title('Probability of Models over Frames')
plt.xlabel('Frame Number')
plt.ylabel('Probability')
plt.legend()

# Adjust layout
plt.tight_layout()
plt.show()
