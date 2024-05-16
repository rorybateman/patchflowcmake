import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

df = pd.read_csv('date_3.csv')


df.columns = ['z', 'ang', 'flow', 'time']
df['delflow'] = df['flow'].diff()
print(df)
# Add a new column



df['Cy'] =  0.122 * np.sin(np.deg2rad(df['ang']))
df['Cx'] =  0.122 * np.cos(np.deg2rad(df['ang']))
df['Cz'] =  10*0.00016*(df['z'])

#df = df[~(df['delflow'] == 0)]

#df = df[~(df['delflow'] > 20)]
#df = df[~(df['delflow'] < -20)]

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot data
ax.scatter(df['Cx'], df['Cy'], df['Cz'], c=df['delflow'], cmap='viridis')

#plt.colorbar()  # To show the color scale

# Set labels
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

df.to_csv("iteration.csv", index=False)
# Show the plot
plt.show()