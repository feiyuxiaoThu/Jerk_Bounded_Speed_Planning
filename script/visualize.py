import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches



if __name__ =='__main__':

    width = 6
    height = 4

    as_data = pd.read_csv("../result_as.csv")
    has_data = pd.read_csv("../result_has.csv")

    fig = plt.figure()
    ax = plt.axes()
    r = patches.Rectangle(xy=(4, 10), width=width, height=height, color='black')
    ax.add_patch(r)
    ax.plot(as_data['t'], as_data['s'], color="red", label="astar search")
    ax.plot(has_data['t'], has_data['s'], color="blue", label="hybrid astar search")
    plt.xlabel('t[s]')
    plt.ylabel('s[m]')
    plt.title('A star Speed Planning')
    plt.legend()
    plt.savefig('../media/result.png')
    plt.show()
