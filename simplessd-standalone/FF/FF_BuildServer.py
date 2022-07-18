import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
from sklearn.linear_model import LinearRegression
import tailer as tl
from io import StringIO


ecLogFile = "/home/zjiao04/simplessd-standalone/256GB_BuildServer.ambertrace.ec"

data = open(ecLogFile)
lastLines = tl.tail(data, 2)  # to read last 2 lines, change it  to any value.
data.close()
df = pd.read_csv(StringIO('\n'.join(lastLines)), delimiter=' ', header=None)

# df = pd.read_csv(ecLogFile, delimiter=' ', header=None)


blks = len(df.iloc[0, :]) - 1
obs = len(df.iloc[:, 0])
print(ecLogFile)
print("Number of (super)blocks: " + str(len(df.iloc[0, :]) - 1))
print("Number of observations per (super)block: " + str(len(df.iloc[:, 0])))

obs = 2
X = [[i] for i in range(0, obs)]

file = open(ecLogFile, "a")
for index in range(0, blks):
    # y = df.iloc[-2:, index]
    # lin_reg = LinearRegression()
    # lin_reg.fit(X, y)
    # prediction = int(np.round(lin_reg.predict([[obs]])[0] + 0.49))
    prediction = df.iloc[-1, index] + df.iloc[-1, index] - df.iloc[-2, index]
    file.write(str(prediction) + " ")
file.write("\n")
file.close()
