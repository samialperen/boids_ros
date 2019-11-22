import matplotlib.pyplot as plt
import numpy as np

"""
Research Question 1 --> Relation between metrics and leader speed
Controlled Variables: separation threshold = 0.7m , cohesion threshold = 2.25m,
alignment threshold = 45 degree, total number of robots = 13, delay = 1.8s
all boids weights (4 of them) are 1.0
Independent variable: leader velocity 0.5 or 0.55 or 0.6 m/s
"""
led_05_sep = [7.418773, 6.908654, 6.589588, 6.962874, 7.393245]
led_05_coh = [0.105897, 0.176683, 0.181598, 0.101796, 0.108565]
led_05_align = [2.547533, 2.451923, 2.340194, 2.402395, 2.572979]

led_055_sep = [0.174263, 0.052209, 0.395442, 0.0, 0.197315]
led_055_coh = [0.225201, 0.360107, 0.222520, 0.538255, 0.233557]
led_055_align = [2.076408, 2.228916, 2.268097, 2.249664, 2.146309]

led_06_sep = [0.0, 0.0, 0.0, 0.0, 0.0]
led_06_coh = [2.441691, 1.906841, 2.676385, 2.150146, 2.572886]
led_06_align = [2.864431, 2.799127, 2.750729, 2.857143, 2.839650]

bar_width = 0.25
 
# set height of bar
bars_sep = [np.mean(led_05_sep),np.mean(led_055_sep),np.mean(led_06_sep)]
bars_coh = [np.mean(led_05_coh),np.mean(led_055_coh),np.mean(led_06_coh)]
bars_align = [np.mean(led_05_align),np.mean(led_055_align),np.mean(led_06_align)]

# set standard deviation of data for error bars
sep_std = [np.std(led_05_sep), np.std(led_055_sep), np.std(led_06_sep)]
coh_std = [np.std(led_05_coh), np.std(led_055_coh), np.std(led_06_coh)]
align_std = [np.std(led_05_align), np.std(led_055_align), np.std(led_06_align)]


# Debug
print("Separation:")
print(bars_sep)
print("Cohesion:")
print(bars_coh)
print("Alignment:")
print(bars_align)

# Set position of bar on X axis
r1 = np.arange(len(bars_sep))
r2 = [x + bar_width for x in r1]
r3 = [x + bar_width for x in r2]
 
# Make the plot
plt.bar(r1, bars_sep, yerr=sep_std, error_kw=dict(lw=3, capsize=5, capthick=2), color='#7f6d5f', width=bar_width, edgecolor='white', label='seperation')
plt.bar(r2, bars_coh, yerr=coh_std, error_kw=dict(lw=3, capsize=5, capthick=2), color='#557f2d', width=bar_width, edgecolor='white', label='cohesion')
plt.bar(r3, bars_align, yerr=align_std, error_kw=dict(lw=3, capsize=5, capthick=2), color='#2d7f5e', width=bar_width, edgecolor='white', label='alignment')
 
# Add xticks on the middle of the group bars
plt.xlabel('Leader Velocity (m/s)', fontweight='bold')
plt.ylabel('Total Violation Amount', fontweight='bold')
plt.xticks([r + bar_width for r in range(len(bars_sep))], ['0.5','0.55','0.6'])
plt.title('Relationship between Leader Velocity and Total Violation Amount')
# Create legend & Show graphic
plt.legend()
#plt.savefig('leader_velocity.png')
plt.show()

"""
Research Question 2 --> Relation between metrics and leader weight metric/other metrics
Controlled Variables: separation threshold = 0.7m , cohesion threshold = 2.25m,
alignment threshold = 45 degree, total number of robots = 13, delay = 1.8s
leader_velocity = 0.55 m/s
separation_weight = 1.0 , cohesion_weight = 1.0 and alignment_weight = 1.0
Independent variable: leader_weight = 1.0 or 1.1 or 1.2
"""
weight_10_sep = [0.174263, 0.052209, 0.395442, 0.0, 0.197315]
weight_10_coh = [0.225201, 0.360107, 0.222520, 0.538255, 0.233557]
weight_10_align = [2.076408, 2.228916, 2.268097, 2.249664, 2.146309]

weight_11_sep = [0.099063, 0.113788, 0.080429, 0.053619, 0.095174]
weight_11_coh = [0.313253, 0.195448, 0.276139, 0.237265, 0.325737]
weight_11_align = [2.275770, 2.099063, 2.298928, 2.095174, 2.064343]

weight_12_sep = [0.058981, 0.041287, 0.044177, 0.065684, 0.070415]
weight_12_coh = [0.317694, 0.197051, 0.239625, 0.241287, 0.265060]
weight_12_align = [2.353887, 2.148794, 2.167336, 2.147453, 2.271754]

bar_width = 0.25
 
# set height of bar
bars_sep2 = [np.mean(weight_10_sep),np.mean(weight_11_sep),np.mean(weight_12_sep)]
bars_coh2 = [np.mean(weight_10_coh),np.mean(weight_11_coh),np.mean(weight_12_coh)]
bars_align2 = [np.mean(weight_10_align),np.mean(weight_11_align),np.mean(weight_12_align)]

# set standard deviation of data for error bars
sep_std2 = [np.std(weight_10_sep), np.std(weight_11_sep), np.std(weight_12_sep)]
coh_std2 = [np.std(weight_10_coh), np.std(weight_11_coh), np.std(weight_12_coh)]
align_std2 = [np.std(weight_10_align), np.std(weight_11_align), np.std(weight_12_align)]


# Debug
print("Separation:")
print(bars_sep2)
print("Cohesion:")
print(bars_coh2)
print("Alignment:")
print(bars_align2)

# Set position of bar on X axis
r1 = np.arange(len(bars_sep2))
r2 = [x + bar_width for x in r1]
r3 = [x + bar_width for x in r2]
 
# Make the plot
plt.bar(r1, bars_sep2, yerr=sep_std2, error_kw=dict(lw=3, capsize=5, capthick=2), color='#7f6d5f', width=bar_width, edgecolor='white', label='seperation')
plt.bar(r2, bars_coh2, yerr=coh_std2, error_kw=dict(lw=3, capsize=5, capthick=2), color='#557f2d', width=bar_width, edgecolor='white', label='cohesion')
plt.bar(r3, bars_align2, yerr=align_std2, error_kw=dict(lw=3, capsize=5, capthick=2), color='#2d7f5e', width=bar_width, edgecolor='white', label='alignment')
 
# Add xticks on the middle of the group bars
plt.xlabel('Leader Weight over Other Boids Weights', fontweight='bold')
plt.ylabel('Total Violation Amount', fontweight='bold')
plt.xticks([r + bar_width for r in range(len(bars_sep2))], ['1.0','1.1','1.2'])
plt.title('Relationship Between Relative Leader Weight and Total Violation Amount')
# Create legend & Show graphic
plt.legend(loc=2, prop={'size': 8})
#plt.savefig('leader_weight.png')
plt.show()