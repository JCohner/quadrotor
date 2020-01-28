from matplotlib import pyplot as plt
import pandas as pd
import numpy as np
import sys
if __name__ == '__main__':
	name = sys.argv[1]
	data = pd.read_csv("{}.csv".format(name), header=None).to_numpy()
	tvec = np.arange(data.shape[0])
	plt.plot(tvec,data[:,0])
	plt.plot(tvec,data[:,1])
	plt.plot(tvec,data[:,2]*100)
	plt.legend(["pitch+", "pitch-", "filtered"])
	plt.ylabel("effort | angle")
	plt.xlabel("itter count")
	plt.savefig("{}.png".format(name))
	plt.show()