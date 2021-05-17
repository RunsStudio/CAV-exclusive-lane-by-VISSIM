import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
def get_records(pt, p, return_type):
    file_name = ('C:\\Users\\run\\Desktop\\毕设仿真\\travel_time_result.txt')
    file_object = open(file_name, 'r')
    file_context = file_object.read().splitlines()
    file_name = ('C:\\Users\\run\\Desktop\\毕设仿真\\travel_time_result_2.txt')
    save_dir = ('C:\\Users\\run\\Desktop\\毕设\\结果-图\\')
    file_object = open(file_name, 'r')
    file_context += file_object.read().splitlines()
    result_dict_type_NVeh = {}
    result_dict_type_TravelTm = {}

    for line in file_context:
        ls = line.split(',')
        ml_type = ls[0]
        penetrate_ratio = ls[1]
        peak_type = ls[2]
        simu_times = ls[3]
        evaluation_times = ls[4]
        veh_type = ls[5]
        Nveh = ls[6]
        travelTm = ls[7]
        if travelTm == None:
            continue
        key = ml_type + '_' + peak_type + '_' + penetrate_ratio

        if int(penetrate_ratio) == p and int(evaluation_times) < 6900 and veh_type == 'All' and peak_type == pt:
            if key not in result_dict_type_NVeh:
                result_dict_type_NVeh[key] = []
                result_dict_type_NVeh[key].append(int(Nveh))
            else:
                result_dict_type_NVeh[key].append(int(Nveh))
            if key not in result_dict_type_TravelTm:
                result_dict_type_TravelTm[key] = []
                result_dict_type_TravelTm[key].append(float(travelTm))
            else:
                result_dict_type_TravelTm[key].append(float(travelTm))
    if return_type == 'NVeh':
        return result_dict_type_NVeh
    else:
        return result_dict_type_TravelTm
from matplotlib.ticker import FuncFormatter
def to_percent(temp, position):
    return '%1.0f'%(100*temp) + '%'
def get_mean_traveltm(pt, p, return_type):
    UNIQUE_FONT_SITE = 12
    plt.rcParams['font.sans-serif'] = ['Microsoft YaHei']
    plt.rcParams['axes.unicode_minus'] = False
    save_dir = ('C:\\Users\\run\\Desktop\\毕设\\结果-图\\')
    manage_lane_type = ['无专用道', '内侧软隔离', '内侧硬隔离', '外侧硬隔离', '外侧软隔离']
    print(get_records(pt, p, return_type))
    travelTm_all_res = np.array(list(get_records(pt, p, return_type).values()))
    travelTm_mean=[]
    for travelTm_res in travelTm_all_res:
        travelTm_mean.append(np.mean(travelTm_res))
    # print(travelTm_mean)
    travelTm_mean[2],travelTm_mean[1]=travelTm_mean[1],travelTm_mean[2] ## 软隔离和硬隔离转换一下（交换第一行和第二行 因为原始文件顺序反了）
    print(travelTm_mean)
    return travelTm_mean
    # df = pd.DataFrame(travelTm_mean, columns=[x for x in range(0, 5)])
def print_dict(dict_to_print):
    for key in dict_to_print:
        print(key)
        print(dict_to_print)
def get_overall_performance(travelTm_all_mean):
    overall_performance=[]
    for p in range(0,6):
        sub_overall_performance=[]
        for i in range(1,5):
            sub_overall_performance.append(-(travelTm_all_mean[p][i]-travelTm_all_mean[p][0])/travelTm_all_mean[p][0])
        overall_performance.append(sub_overall_performance)
    return overall_performance
def draw_overall_bar_travelTm(travelTm_all_mean,pt):
    UNIQUE_FONT_SITE = 12
    overall_performance = get_overall_performance(travelTm_all_mean)
    WIDTH_PLT = 0.25
    RANGE_PLT = np.arange(0, 8, 2)
    texts = []
    for p in range(0, 6):
        plt.bar(x=RANGE_PLT + p * WIDTH_PLT, width=WIDTH_PLT, alpha=0.8, height=overall_performance[p], edgecolor='black')
        texts.append([RANGE_PLT + p * WIDTH_PLT, overall_performance[p]])
    plt.xticks(RANGE_PLT + WIDTH_PLT * 3, ['内侧硬隔离', '内侧软隔离', '外侧硬隔离', '外侧软隔离'])
    plt.xlabel('专用道类型')
    plt.ylabel('优化比例（%）')
    plt.ylim(-0.05, 0.15)
    plt.legend(['5%', '10%', '15%', '20%', '30%', '40%'], fontsize=UNIQUE_FONT_SITE - 4, title='渗透率')
    plt.gca().yaxis.set_major_formatter(FuncFormatter(to_percent))
    plt.plot([-0.2, 7.7], [0, 0], color='k', linewidth=1.0)
    for text in texts:
        for j in range(0, 4):
            print(text[1][j])
            if text[1][j] >= 0:
                plt.text(text[0][j], text[1][j] + 0.005, "%.0f%%" % (100 * text[1][j]), ha='center', va='bottom',
                         fontsize=7)
            else:
                plt.text(text[0][j], text[1][j] - 0.010, "%.0f%%" % (100 * text[1][j]), ha='center', va='bottom',
                         fontsize=7)
    for i in range(0, 6):
        strline = pt + ',' + str([5, 10, 15, 20, 30, 40][i]) + str(texts[i][1])
        print(strline.replace('[', ',').replace(']', ','))
    save_dir = ('C:\\Users\\run\\Desktop\\毕设\\结果-图\\综合_')
    plt.savefig(save_dir + pt + '_'  + 'travelTm.png', dpi=300, bbox_inches='tight', pad_inches=0.05)

    plt.show()
if __name__ == '__main__':
    # for p in [5,10,15,20,30,40]:
        # heatmap_drawpic('sub-peak', p, 'TravelTm')
    travelTm_all_mean=[]
    for p in [5, 10, 15, 20, 30, 40]:
        travelTm_all_mean.append(get_mean_traveltm('sub-peak',p,'TravelTm'))
    print_dict(travelTm_all_mean)
    draw_overall_bar_travelTm(travelTm_all_mean,'sub-peak')
