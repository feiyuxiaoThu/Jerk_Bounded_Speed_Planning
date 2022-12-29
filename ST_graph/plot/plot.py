#!/usr/bin/env python3
import math
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os
pwd = os.getcwd()


fig = plt.figure()
ax = fig.add_subplot(121)
st_data = pd.read_csv(pwd+"/st_boundaries.csv")
t = st_data["t"].values
s = st_data["s"].values
plt.scatter(t, s, color='b')
# plt.scatter(t, reference_s, color='g')
ax.set_title("st_test.csv")
plt.xlabel('t')
plt.ylabel('s')
plt.savefig("plot.png")


