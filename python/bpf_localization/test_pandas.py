import pandas as pd

df = pd.read_csv (r'data/calibrations/boat_imu.csv', index_col=['vector','component', 'data']).T

print (df.iloc[:, df.columns.get_level_values(2)=='lb'])
