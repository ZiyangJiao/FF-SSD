import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
from sklearn.linear_model import LinearRegression
import math

ecLogFile = "/home/zjiao04/simplessd-standalone/256GB_BuildServer.ambertrace.ec2"

df = pd.read_csv(ecLogFile, delimiter=' ', header=None)

blks = len(df.iloc[0, :]) - 1
obs = len(df.iloc[:, 0])
print("Number of (super)blocks: " + str(len(df.iloc[0, :]) - 1))
print("Number of observations per (super)block: " + str(len(df.iloc[:, 0])))

obs = 2
X = [[i] for i in range(0, obs)]

file = open(ecLogFile, "a")
for index in range(0, blks):
    y = df.iloc[-2:, index] # use k (2) most recent observations to train model (may have stale data)
    lin_reg = LinearRegression()
    lin_reg.fit(X, y)
    prediction = math.ceil(lin_reg.predict([[obs]]))
    file.write(str(prediction) + " ")
file.write("\n")
file.close()
