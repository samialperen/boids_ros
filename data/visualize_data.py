import matplotlib.pyplot as plt
import numpy as np
import scipy.stats as stats
import pandas as pd
from statsmodels.stats.multicomp import (pairwise_tukeyhsd,MultiComparison)

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
plt.ylabel('Violation Metrics', fontweight='bold')
plt.xticks([r + bar_width for r in range(len(bars_sep))], ['0.5','0.55','0.6'])
plt.title('Relationship between Leader Velocity and Violation Metrics')
# Create legend & Show graphic
plt.legend()
#plt.savefig('leader_velocity.png')
plt.show()

# One way ANOVA analysis for statistical analysis
fvalue_sep, pvalue_sep = stats.f_oneway(led_05_sep,led_055_sep,led_06_sep)
fvalue_coh, pvalue_coh = stats.f_oneway(led_05_coh,led_055_coh,led_06_coh)
fvalue_align, pvalue_align = stats.f_oneway(led_05_align,led_055_align,led_06_align)
print("Research Question 1 --> Seperation F and P Value:")
print(fvalue_sep,pvalue_sep)
print("Research Question 1 --> Cohesion F and P Value:")
print(fvalue_coh,pvalue_coh)
print("Research Question 1 --> Alignment F and P Value:")
print(fvalue_align,pvalue_align)

# If pvalue < 0.05 --> Apply Tukey's Multi-Comparison Method to
# find out between which subgroups there is a significant difference

df1 = pd.DataFrame()
df1['led_05_sep'] = led_05_sep
df1['led_055_sep'] = led_055_sep
df1['led_06_sep'] = led_06_sep

df2 = pd.DataFrame()
df2['led_05_coh'] = led_05_coh
df2['led_055_coh'] = led_055_coh
df2['led_06_coh'] = led_06_coh

df3 = pd.DataFrame()
df3['led_05_align'] = led_05_align
df3['led_055_align'] = led_055_align
df3['led_06_align'] = led_06_align

# Stack the data (and rename columns):
stacked_data1 = df1.stack().reset_index()
stacked_data1 = stacked_data1.rename(columns={'level_0': 'index',
                                            'level_1': 'seperation',
                                            0:'violation metric'})
stacked_data2 = df2.stack().reset_index()
stacked_data2 = stacked_data2.rename(columns={'level_0': 'index',
                                            'level_1': 'cohesion',
                                            0:'violation metric'})

stacked_data3 = df3.stack().reset_index()
stacked_data3 = stacked_data3.rename(columns={'level_0': 'index',
                                            'level_1': 'alignment',
                                            0:'violation metric'})
                                    
#print(stacked_data1)
MultiComp1 = MultiComparison(stacked_data1['violation metric'],stacked_data1['seperation'])
MultiComp2 = MultiComparison(stacked_data2['violation metric'],stacked_data2['cohesion'])
MultiComp3 = MultiComparison(stacked_data3['violation metric'],stacked_data3['alignment'])

#print(MultiComp1.tukeyhsd().summary())
statistic_txt = open("statistical_test.txt","w") 
statistic_txt.write(str(MultiComp1.tukeyhsd().summary()))
statistic_txt.write("\n")
statistic_txt.write(str(MultiComp2.tukeyhsd().summary()))
statistic_txt.write("\n") 
statistic_txt.write(str(MultiComp3.tukeyhsd().summary()))
statistic_txt.write("\n")  
#statistic_txt.close() #to change file access modes 
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
plt.ylabel('Violation Metrics', fontweight='bold')
plt.xticks([r + bar_width for r in range(len(bars_sep2))], ['1.0','1.1','1.2'])
plt.title('Relationship Between Relative Leader Weight and Violation Metrics')
# Create legend & Show graphic
plt.legend(loc=2, prop={'size': 8})
#plt.savefig('leader_weight.png')
plt.show()


# One way ANOVA analysis for statistical analysis
fvalue_sep2, pvalue_sep2 = stats.f_oneway(weight_10_sep,weight_11_sep,weight_12_sep)
fvalue_coh2, pvalue_coh2 = stats.f_oneway(weight_10_coh,weight_11_coh,weight_12_coh)
fvalue_align2, pvalue_align2 = stats.f_oneway(weight_10_align,weight_11_align,weight_12_align)
print("Research Question 2 --> Seperation F and P Value:")
print(fvalue_sep2,pvalue_sep2)
print("Research Question 2 --> Cohesion F and P Value:")
print(fvalue_coh2,pvalue_coh2)
print("Research Question 2 --> Alignment F and P Value:")
print(fvalue_align2,pvalue_align2)

# If pvalue < 0.05 --> Apply Tukey's Multi-Comparison Method to
# find out between which subgroups there is a significant difference

df4 = pd.DataFrame()
df4['weight_10_sep'] = weight_10_sep
df4['weight_11_sep'] = weight_11_sep
df4['weight_12_sep'] = weight_12_sep

df5 = pd.DataFrame()
df5['weight_10_coh'] = weight_10_coh
df5['weight_11_coh'] = weight_11_coh
df5['weight_12_coh'] = weight_12_coh

df6 = pd.DataFrame()
df6['weight_10_align'] = weight_10_align
df6['weight_11_align'] = weight_11_align
df6['weight_12_align'] = weight_12_align

# Stack the data (and rename columns):
stacked_data4 = df4.stack().reset_index()
stacked_data4 = stacked_data4.rename(columns={'level_0': 'index',
                                            'level_1': 'seperation',
                                            0:'violation metric'})
stacked_data5 = df5.stack().reset_index()
stacked_data5 = stacked_data5.rename(columns={'level_0': 'index',
                                            'level_1': 'cohesion',
                                            0:'violation metric'})

stacked_data6 = df6.stack().reset_index()
stacked_data6 = stacked_data6.rename(columns={'level_0': 'index',
                                            'level_1': 'alignment',
                                            0:'violation metric'})
                                    
#print(stacked_data1)
MultiComp4 = MultiComparison(stacked_data4['violation metric'],stacked_data4['seperation'])
MultiComp5 = MultiComparison(stacked_data5['violation metric'],stacked_data5['cohesion'])
MultiComp6 = MultiComparison(stacked_data6['violation metric'],stacked_data6['alignment'])

statistic_txt.write(str(MultiComp4.tukeyhsd().summary()))
statistic_txt.write("\n")
statistic_txt.write(str(MultiComp5.tukeyhsd().summary()))
statistic_txt.write("\n") 
statistic_txt.write(str(MultiComp6.tukeyhsd().summary()))
statistic_txt.write("\n")  
#statistic_txt.close() #to change file access modes 
"""
Research Question 3 --> Relation between violation count and total number of agents
Controlled Variables: separation threshold = 0.7m , cohesion threshold = 2.25m,
alignment threshold = 45 degree, delay = 1.8s, leader_velocity = 0.55 m/s
separation_weight = 1.0 , cohesion_weight = 1.0, alignment_weight = 1.0
leader_weight = 1.0 
Independent variable: total number of robots = 7 or 13 or 19
"""

total_7_sep_robot1 = [0, 0, 0, 0, 0]
total_7_sep_robot2 = [37, 83, 15, 23, 10]
total_7_sep_robot3 = [47, 69, 62, 44, 56]
total_7_sep_robot4 = [0, 20, 14, 5, 0]
total_7_sep_robot5 = [33, 92, 47, 60, 46]
total_7_sep_robot6 = [50, 226, 89, 215, 79]

total_7_coh_robot1 = [0, 0, 0, 0, 0]
total_7_coh_robot2 = [0, 0, 0, 0, 0]
total_7_coh_robot3 = [0, 0, 0, 0, 0]
total_7_coh_robot4 = [0, 0, 0, 0, 0]
total_7_coh_robot5 = [0, 0, 0, 0, 0]
total_7_coh_robot6 = [0, 0, 0, 0, 0]

total_7_align_robot1 = [120, 121, 118, 121, 123]
total_7_align_robot2 = [120, 115, 119, 120, 120]
total_7_align_robot3 = [116, 112, 116, 116, 115]
total_7_align_robot4 = [117, 121, 116, 124, 124]
total_7_align_robot5 = [118, 118, 117, 116, 116]
total_7_align_robot6 = [116, 118, 118, 117, 117]


bar_width = 0.25
 
# set height of bar
bars_sep3 = [np.mean(total_7_sep_robot1),np.mean(total_7_sep_robot2),np.mean(total_7_sep_robot3),
            np.mean(total_7_sep_robot4), np.mean(total_7_sep_robot5), np.mean(total_7_sep_robot6)]

bars_coh3 = [np.mean(total_7_coh_robot1),np.mean(total_7_coh_robot2),np.mean(total_7_coh_robot3),
            np.mean(total_7_coh_robot4), np.mean(total_7_coh_robot5), np.mean(total_7_coh_robot6)]

bars_align3 = [np.mean(total_7_align_robot1),np.mean(total_7_align_robot2),np.mean(total_7_align_robot3),
            np.mean(total_7_align_robot4), np.mean(total_7_align_robot5), np.mean(total_7_align_robot6)]

# set standard deviation of data for error bars
sep_std3 = [np.std(total_7_sep_robot1), np.std(total_7_sep_robot2), np.std(total_7_sep_robot3), 
            np.std(total_7_sep_robot4), np.std(total_7_sep_robot5), np.std(total_7_sep_robot6)]

coh_std3 = [np.std(total_7_coh_robot1), np.std(total_7_coh_robot2), np.std(total_7_coh_robot3), 
            np.std(total_7_coh_robot4), np.std(total_7_coh_robot5), np.std(total_7_coh_robot6)]

align_std3 = [np.std(total_7_align_robot1), np.std(total_7_align_robot2), np.std(total_7_align_robot3), 
            np.std(total_7_align_robot4), np.std(total_7_align_robot5), np.std(total_7_align_robot6)]


# Set position of bar on X axis
r1 = np.arange(len(bars_sep3))
r2 = [x + bar_width for x in r1]
r3 = [x + bar_width for x in r2]
 
# Make the plot
plt.bar(r1, bars_sep3, yerr=sep_std3, error_kw=dict(lw=3, capsize=5, capthick=2), color='#7f6d5f', width=bar_width, edgecolor='white', label='seperation')
plt.bar(r2, bars_coh3, yerr=coh_std3, error_kw=dict(lw=3, capsize=5, capthick=2), color='#557f2d', width=bar_width, edgecolor='white', label='cohesion')
plt.bar(r3, bars_align3, yerr=align_std3, error_kw=dict(lw=3, capsize=5, capthick=2), color='#2d7f5e', width=bar_width, edgecolor='white', label='alignment')
 
# Add xticks on the middle of the group bars
plt.xlabel('Robots', fontweight='bold')
plt.ylabel('Total Violation Amount During Entire Simulation', fontweight='bold')
plt.xticks([r + bar_width for r in range(len(bars_sep3))], ['Robot1','Robot2','Robot3','Robot4','Robot5','Robot6'])
plt.title('Total Violation Amounts for Simulation with 7 Robots')
# Create legend & Show graphic
x1,x2,y1,y2 = plt.axis()
plt.axis((x1,x2,0,y2))
plt.legend(loc=9) #(loc=2, prop={'size': 8})
#plt.savefig('7robots.png')
plt.show()

#####################################
total_13_sep_robot1 = [0, 0, 0, 0, 0]
total_13_sep_robot2 = [14, 0, 0, 0, 0]
total_13_sep_robot3 = [0, 0, 25, 0, 0]
total_13_sep_robot4 = [0, 0, 0, 0, 0]
total_13_sep_robot5 = [24, 16, 25, 0, 24]
total_13_sep_robot6 = [27, 12, 86, 0, 25]
total_13_sep_robot7 = [0, 0, 0, 0, 0]
total_13_sep_robot8 = [0, 0, 27, 0, 0]
total_13_sep_robot9 = [0, 0, 0, 0, 0]
total_13_sep_robot10 = [0, 0, 59, 0, 0]
total_13_sep_robot11 = [15, 0, 50, 0, 0]
total_13_sep_robot12 = [50, 11, 23, 0, 98]

total_13_coh_robot1 = [0, 0, 0, 0, 0]
total_13_coh_robot2 = [0, 0, 0, 0, 0]
total_13_coh_robot3 = [0, 0, 0, 0, 0]
total_13_coh_robot4 = [0, 0, 0, 0, 0]
total_13_coh_robot5 = [31, 61, 32, 67, 32]
total_13_coh_robot6 = [70, 71, 65, 88, 70]
total_13_coh_robot7 = [0, 0, 0, 0, 0]
total_13_coh_robot8 = [0, 0, 0, 0, 0]
total_13_coh_robot9 = [0, 0, 0, 19, 0]
total_13_coh_robot10 = [0, 0, 0, 60, 0]
total_13_coh_robot11 = [0, 64, 0, 86, 30]
total_13_coh_robot12 = [67, 73, 69, 81, 42]

total_13_align_robot1 = [107, 124, 152, 136, 129]
total_13_align_robot2 = [105, 125, 157, 107, 129]
total_13_align_robot3 = [159, 158, 168, 153, 139]
total_13_align_robot4 = [136, 155, 157, 167, 137]
total_13_align_robot5 = [127, 127, 128, 128, 133]
total_13_align_robot6 = [135, 136, 121, 134, 138]
total_13_align_robot7 = [111, 128, 128, 128, 130]
total_13_align_robot8 = [108, 129, 127, 126, 129]
total_13_align_robot9 = [164, 171, 171, 174, 131]
total_13_align_robot10 = [133, 151, 115, 137, 138]
total_13_align_robot11 = [132, 130, 137, 153, 134]
total_13_align_robot12 = [132, 131, 131, 153, 132]


bar_width = 0.25
 
# set height of bar
bars_sep4 = [np.mean(total_13_sep_robot1),np.mean(total_13_sep_robot2),np.mean(total_13_sep_robot3),
            np.mean(total_13_sep_robot4), np.mean(total_13_sep_robot5), np.mean(total_13_sep_robot6)]

bars_coh4 = [np.mean(total_13_coh_robot1),np.mean(total_13_coh_robot2),np.mean(total_13_coh_robot3),
            np.mean(total_13_coh_robot4), np.mean(total_13_coh_robot5), np.mean(total_13_coh_robot6)]

bars_align4 = [np.mean(total_13_align_robot1),np.mean(total_13_align_robot2),np.mean(total_13_align_robot3),
            np.mean(total_13_align_robot4), np.mean(total_13_align_robot5), np.mean(total_13_align_robot6)]

# set standard deviation of data for error bars
sep_std4 = [np.std(total_13_sep_robot1), np.std(total_13_sep_robot2), np.std(total_13_sep_robot3), 
            np.std(total_13_sep_robot4), np.std(total_13_sep_robot5), np.std(total_13_sep_robot6)]

coh_std4 = [np.std(total_13_coh_robot1), np.std(total_13_coh_robot2), np.std(total_13_coh_robot3), 
            np.std(total_13_coh_robot4), np.std(total_13_coh_robot5), np.std(total_13_coh_robot6)]

align_std4 = [np.std(total_13_align_robot1), np.std(total_13_align_robot2), np.std(total_13_align_robot3), 
            np.std(total_13_align_robot4), np.std(total_13_align_robot5), np.std(total_13_align_robot6)]


# Set position of bar on X axis
r1 = np.arange(len(bars_sep4))
r2 = [x + bar_width for x in r1]
r3 = [x + bar_width for x in r2]
 
# Make the plot
plt.bar(r1, bars_sep4, yerr=sep_std4, error_kw=dict(lw=3, capsize=5, capthick=2), color='#7f6d5f', width=bar_width, edgecolor='white', label='seperation')
plt.bar(r2, bars_coh4, yerr=coh_std4, error_kw=dict(lw=3, capsize=5, capthick=2), color='#557f2d', width=bar_width, edgecolor='white', label='cohesion')
plt.bar(r3, bars_align4, yerr=align_std4, error_kw=dict(lw=3, capsize=5, capthick=2), color='#2d7f5e', width=bar_width, edgecolor='white', label='alignment')
 
# Add xticks on the middle of the group bars
plt.xlabel('Robots', fontweight='bold')
plt.ylabel('Total Violation Amount During Entire Simulation', fontweight='bold')
plt.xticks([r + bar_width for r in range(len(bars_sep3))], ['Robot1','Robot2','Robot3','Robot4','Robot5','Robot6'])
plt.title('Total Violation Amounts for Simulation with 13 Robots')
# Create legend & Show graphic
x1,x2,y1,y2 = plt.axis()
plt.axis((x1,x2,0,y2))
plt.legend() #(loc=2, prop={'size': 8})
#plt.savefig('13robots.png')
plt.show()

####################################
total_19_sep_robot1 = [0, 0, 0, 0, 0]
total_19_sep_robot2 = [0, 0, 0, 0, 130]
total_19_sep_robot3 = [0, 0, 0, 0, 0]
total_19_sep_robot4 = [0, 0, 0, 0, 69]
total_19_sep_robot5 = [14, 10, 19, 9, 64]
total_19_sep_robot6 = [0, 0, 0, 35, 18]
total_19_sep_robot7 = [0, 0, 0, 0, 119]
total_19_sep_robot8 = [0, 177, 0, 8, 100]
total_19_sep_robot9 = [0, 0, 0, 0, 0]
total_19_sep_robot10 = [0, 200, 0, 0, 57]
total_19_sep_robot11 = [0, 0, 0, 0, 0]
total_19_sep_robot12 = [9, 0, 0, 12, 5]
total_19_sep_robot13 = [0, 0, 0, 30, 4]
total_19_sep_robot14 = [0, 0, 0, 0, 0]
total_19_sep_robot15 = [0, 0, 0, 43, 0]
total_19_sep_robot16 = [0, 0, 0, 27, 0]
total_19_sep_robot17 = [18, 46, 31, 40, 22]
total_19_sep_robot18 = [0, 140, 0, 64, 64]

total_19_coh_robot1 = [187, 0, 226, 0, 0]
total_19_coh_robot2 = [0, 0, 47, 0, 0]
total_19_coh_robot3 = [17, 8, 19, 6, 9]
total_19_coh_robot4 = [28, 30, 53, 51, 0]
total_19_coh_robot5 = [64, 64, 72, 64, 0]
total_19_coh_robot6 = [269, 74, 229, 75, 74]
total_19_coh_robot7 = [177, 0, 186, 0, 0]
total_19_coh_robot8 = [0, 0, 0, 0, 0]
total_19_coh_robot9 = [175, 21, 191, 0, 0]
total_19_coh_robot10 = [53, 0, 16, 30, 0]
total_19_coh_robot11 = [278, 70, 232, 68, 68]
total_19_coh_robot12 = [82, 85, 87, 82, 83]
total_19_coh_robot13 = [216, 0, 128, 0, 0]
total_19_coh_robot14 = [187, 0, 238, 0, 0]
total_19_coh_robot15 = [159, 73, 165, 66, 71]
total_19_coh_robot16 = [221, 75, 243, 69, 73]
total_19_coh_robot17 = [81, 80, 85, 79, 81]
total_19_coh_robot18 = [154, 142, 151, 95, 112]

total_19_align_robot1 = [137, 140, 151, 128, 143]
total_19_align_robot2 = [137, 148, 152, 141, 137]
total_19_align_robot3 = [143, 175, 148, 112, 182]
total_19_align_robot4 = [175, 179, 174, 177, 165]
total_19_align_robot5 = [143, 149, 136, 165, 168]
total_19_align_robot6 = [146, 193, 145, 143, 145]
total_19_align_robot7 = [138, 144, 140, 115, 164]
total_19_align_robot8 = [135, 156, 166, 137, 131]
total_19_align_robot9 = [172, 178, 167, 180, 173]
total_19_align_robot10 = [166, 141, 160, 182, 175]
total_19_align_robot11 = [141, 139, 186, 118, 141]
total_19_align_robot12 = [141, 142, 142, 119, 141]
total_19_align_robot13 = [147, 136, 134, 143, 166]
total_19_align_robot14 = [143, 133, 176, 159, 140]
total_19_align_robot15 = [189, 164, 190, 153, 166]
total_19_align_robot16 = [169, 154, 158, 145, 150]
total_19_align_robot17 = [189, 166, 202, 142, 145]
total_19_align_robot18 = [147, 140, 189, 141, 142]


bar_width = 0.25
 
# set height of bar
bars_sep5 = [np.mean(total_19_sep_robot1),np.mean(total_19_sep_robot2),np.mean(total_19_sep_robot3),
            np.mean(total_19_sep_robot4), np.mean(total_19_sep_robot5), np.mean(total_19_sep_robot6)]

bars_coh5 = [np.mean(total_19_coh_robot1),np.mean(total_19_coh_robot2),np.mean(total_19_coh_robot3),
            np.mean(total_19_coh_robot4), np.mean(total_19_coh_robot5), np.mean(total_19_coh_robot6)]

bars_align5 = [np.mean(total_19_align_robot1),np.mean(total_19_align_robot2),np.mean(total_19_align_robot3),
            np.mean(total_19_align_robot4), np.mean(total_19_align_robot5), np.mean(total_19_align_robot6)]

# set standard deviation of data for error bars
sep_std5 = [np.std(total_19_sep_robot1), np.std(total_19_sep_robot2), np.std(total_19_sep_robot3), 
            np.std(total_19_sep_robot4), np.std(total_19_sep_robot5), np.std(total_19_sep_robot6)]

coh_std5 = [np.std(total_19_coh_robot1), np.std(total_19_coh_robot2), np.std(total_19_coh_robot3), 
            np.std(total_19_coh_robot4), np.std(total_19_coh_robot5), np.std(total_19_coh_robot6)]

align_std5 = [np.std(total_19_align_robot1), np.std(total_19_align_robot2), np.std(total_19_align_robot3), 
            np.std(total_19_align_robot4), np.std(total_19_align_robot5), np.std(total_19_align_robot6)]


# Set position of bar on X axis
r1 = np.arange(len(bars_sep5))
r2 = [x + bar_width for x in r1]
r3 = [x + bar_width for x in r2]
 
# Make the plot
plt.bar(r1, bars_sep5, yerr=sep_std5, error_kw=dict(lw=3, capsize=5, capthick=2), color='#7f6d5f', width=bar_width, edgecolor='white', label='seperation')
plt.bar(r2, bars_coh5, yerr=coh_std5, error_kw=dict(lw=3, capsize=5, capthick=2), color='#557f2d', width=bar_width, edgecolor='white', label='cohesion')
plt.bar(r3, bars_align5, yerr=align_std5, error_kw=dict(lw=3, capsize=5, capthick=2), color='#2d7f5e', width=bar_width, edgecolor='white', label='alignment')
 
# Add xticks on the middle of the group bars
plt.xlabel('Robots', fontweight='bold')
plt.ylabel('Total Violation Amount During Entire Simulation', fontweight='bold')
plt.xticks([r + bar_width for r in range(len(bars_sep5))], ['Robot1','Robot2','Robot3','Robot4','Robot5','Robot6'])
plt.title('Total Violation Amounts for Simulation with 19 Robots')
# Create legend & Show graphic
x1,x2,y1,y2 = plt.axis()
plt.axis((x1,x2,0,y2))
plt.legend() #(loc=2, prop={'size': 8})
#plt.savefig('19robots.png')
plt.show()