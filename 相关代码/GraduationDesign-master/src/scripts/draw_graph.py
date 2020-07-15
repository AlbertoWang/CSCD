import matplotlib.pyplot as plt
plt.rcParams['font.sans-serif']=['SimHei']

mve_mean_distance_data = """
0	0.0163	0.0163
0.05	0.017	0.0164
0.1	0.0188	0.0171
0.15	0.0214	0.0189
0.2	0.0243	0.0212
0.25	0.0285	0.024
0	0.0129	0.0129
0.05	0.0138	0.0133
0.1	0.0152	0.0135
0.15	0.0174	0.0146
0.2	0.022	0.0154
0.25	0.0261	0.0179
0	0.0078	0.0078
0.05	0.0081	0.0081
0.1	0.0087	0.0091
0.15	0.0093	0.0092
0.2	0.0103	0.0098
0.25	0.0119	0.0107
    """

raw_mean_distance_data = """
0	0.0194	0.0194
0.05	0.0227	0.0226
0.1	0.0238	0.0236
0.15	0.0263	0.0255
0.2	0.0307	0.0282
0.25	0.0336	0.03
0	0.0198	0.0198
0.05	0.0253	0.0246
0.1	0.0277	0.0247
0.15	0.0308	0.0267
0.2	0.0355	0.0269
0.25	0.0439	0.0363
0	0.0351	0.0351
0.05	0.0368	0.037
0.1	0.0387	0.0388
0.15	0.0422	0.041
0.2	0.0449	0.0424
0.25	0.0484	0.0465
    """

def draw_graph(percentages, data):
    #colors = ['red', 'red', 'blue', 'blue', 'brown', 'brown']
    colors = ['orange', 'orange', 'black', 'black', 'cyan', 'cyan']
    linestyles = ['-', '--', '-', '--', '-', '--']
    markers = ['*', '*', 'o', 'o', '>', '>']
    #labels = ['4-未修补', '4-修补后', '5-未修补', '5-修补后', '6-未修补', '6-修补后']
    labels = ['1-未修补', '1-修补后', '2-未修补', '2-修补后', '3-未修补', '3-修补后']

    for index in range(len(data)):
        subdata = data[index]
        plt.plot(percentages, subdata, color=colors[index], linestyle=linestyles[index], marker=markers[index], label=labels[index])
        pass
    #plt.plot(x, y)
    plt.xlabel('缺失比例')
    plt.ylabel('到Ground Truth的平均距离')
    plt.legend()
    plt.show()
    pass

if __name__ == '__main__':
    percentages = [0.0, 0.05, 0.10, 0.15, 0.20, 0.25]
    data = [] # [ [mve-2-wor],  [mve-2-rep], [mve-5-wor], ... ]
    # distances_str = mve_mean_distance_data
    distances_str = raw_mean_distance_data
    lines = distances_str.splitlines()
    line_index = 1
    for _ in range(3):
        data_rep = []
        data_wor = []
        for percentage in percentages:
            line = lines[line_index]
            words = line.split()
            data_wor.append(float(words[1]))
            data_rep.append(float(words[2]))
            line_index += 1
            pass 
        data.append(data_wor)
        data.append(data_rep)
        pass
    draw_graph(percentages, data)
    pass