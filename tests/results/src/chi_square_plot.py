import matplotlib.pyplot as plt
import numpy as np

#optimzier n = 40, max frames = 180
# iterative_1 = [(123.009940,33847), (289.532348, 86074), (440.213807, 138410), (571.515348, 181728), (661.998198, 229393)   ]
# single_1 = [(722.860211, 242119)]

# iterative_2 = [(115.249030,31710), (284.395561, 84176), (430.501494, 131781), (541.550783, 175038), (613.868294, 222620)   ]
# single_2 = [(744.188005, 241105)]



# single_3 = [(699.448373, 239548)]
# iterative_3 = [(113.144611,30769), (283.613092, 82746), (418.259296, 130619), (509.283492, 172800), (583.139983, 220734)   ]

#optimzier n = 100, max frames = 500
iterative_1 = [(550.986424, 160050), (1242.085628, 349596), (1863.683004, 569486), (2502.517324, 760375), (3232.027229, 929300)   ]
single_1 = [(3701.907183, 935315)]


# frames = np.array([40, 80, 120, 160, 180])
frames = np.arange(len(iterative_1))
labels = ["100", "200", "300", "400", "500"]

# fig = plt.figure()

fig, ax = plt.subplots()

ax2 = ax.twinx()

chi_error_it = []
no_edges_it = []
for it in iterative_1:
    chi_error_it.append(it[0])
    no_edges_it.append(it[1])

chi_error_single = [0] * len(chi_error_it)
no_edges_single = [0] * len(chi_error_it)

chi_error_single[-1] = single_1[0][0]
no_edges_single[-1] = single_1[0][1]

ax.set_ylabel('Chi-Squared Error')
ax.set_title('Incremental vs. Non-Incremental Optimization')
ax.set_xticks(frames)
ax.set_xticklabels(labels)
ax.set_xlabel("Number of Frames")


ax2.set_ylabel("Edges in Factor Graph")


ax2.plot(frames,no_edges_it, linestyle='dashed', color='r')


incremental_size = ax2.scatter(frames,no_edges_it, linestyle='dashed', marker='o', color='r')
single_size = ax2.scatter(frames[-1] + 0.5 ,no_edges_single[-1], linestyle='dashed', marker='o', color='black')
incremental_chi = ax.bar(frames + 0.00, chi_error_it, color = 'b', width = 0.5)
single_chi = ax.bar(frames + 0.5, chi_error_single, color = 'g', width = 0.5)

plt.legend((incremental_size, single_size, incremental_chi, single_chi),
           ("Incremental (graph-size)", "Single (graph-size)","Incremental (chi-error)", "Single (chi-error)"),
           scatterpoints=1,
           loc='upper left',
           ncol=2,
           fontsize=10)
ax.legend()
# ax.legend()# ax.bar(frames,chi_error_it)
plt.show()