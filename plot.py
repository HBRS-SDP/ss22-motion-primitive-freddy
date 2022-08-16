import pathlib
import matplotlib.pyplot as plt
from pathlib import Path
import pandas as pd
import numpy as np
import os

base = Path(__file__).parent
file = os.path.join(base, 'ramp_log.csv')


data = pd.read_csv(file)

print(data.columns)
drive0 = data.loc[data['drive id'] == 0]
drive1 = data.loc[data['drive id'] == 1]
drive2 = data.loc[data['drive id'] == 2]
drive3 = data.loc[data['drive id'] == 3]


for drive_data, name in zip([drive0, drive1, drive2, drive3],['Wheel unit - 0','Wheel unit - 1','Wheel unit - 2','Wheel unit - 3']):
    fig, axd = plt.subplot_mosaic([['left', 'right'],['left1', 'right1'],['bottom', 'bottom']],
                                constrained_layout=True)
    fig.suptitle(name, fontsize=12)

    axd['left'].plot(np.arange(250),drive_data['wheel_velocity1'].values, label = 'wheel_velocity1')
    axd['left'].plot(np.arange(250),drive_data['wheel_velocity2'].values, label = 'wheel_velocity2')
    axd['left'].legend (['wheel_velocity1','wheel_velocity2'])

    axd['right'].plot(np.arange(250),drive_data['wheel_current1'].values)
    axd['right'].plot(np.arange(250),drive_data['wheel_current2'].values)
    axd['right'].legend (['wheel_current1','wheel_current2'])

    axd['left1'].plot(np.arange(250),drive_data['imu_ang_vel_x'].values)
    axd['left1'].plot(np.arange(250),drive_data['imu_ang_vel_y'].values)
    axd['left1'].plot(np.arange(250),drive_data['imu_ang_vel_z'].values)
    axd['left1'].legend (['Angular velcoity - X','Angular velcoity - Y', 'Angular velcoity - Z'])

    axd['right1'].plot(np.arange(250),drive_data['imu_lin_acc_x'].values)
    axd['right1'].plot(np.arange(250),drive_data['imu_lin_acc_y'].values)
    axd['right1'].plot(np.arange(250),drive_data['imu_lin_acc_z'].values)
    axd['right1'].legend (['Linear acceleration - X','Linear acceleration - Y', 'Linear acceleration - Z'])

    axd['bottom'].plot(np.arange(250),drive_data['pivot_angle'].values)
    axd['bottom'].legend (['Pivot angle'])

    plt.savefig(name+'.png', format='png')

    plt.show()

