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
def heatmap_drawpic(pt, p, return_type):
    UNIQUE_FONT_SITE = 12
    plt.rcParams['font.sans-serif'] = ['Microsoft YaHei']
    plt.rcParams['axes.unicode_minus'] = False
    save_dir = ('C:\\Users\\run\\Desktop\\毕设\\结果-图\\')
    manage_lane_type = ['无专用道','内侧硬隔离', '内侧软隔离',  '外侧硬隔离', '外侧软隔离']
    yy = np.array(list(get_records(pt, p, return_type).values()))
    yy[[1, 2], :] = yy[[2, 1], :]  # 实现了第i行与第j行的互换
    df = pd.DataFrame(yy, columns=[x for x in range(0, 90)])
    plt.xlabel('x_label', fontsize=UNIQUE_FONT_SITE, color='k')  # x轴label的文本和字体大小
    plt.ylabel('y_label', fontsize=UNIQUE_FONT_SITE, color='k')  # y轴label的文本和字体大小
    plt.xticks(fontsize=UNIQUE_FONT_SITE)  # x轴刻度的字体大小（文本包含在pd_data中了）
    plt.yticks(fontsize=UNIQUE_FONT_SITE)  # y轴刻度的字体大小（文本包含在pd_data中了）
    # 设置图框线粗细
    bwith = 1.5  # 边框宽度设置为2


    sns.heatmap(df, annot=False, yticklabels=manage_lane_type,cmap='YlGnBu')

    plt.savefig(save_dir + pt + '_' + str(p) +'_' +return_type+ '_HEATMAP.png', bbox_inches='tight', dpi=300, pad_inches=0.05)
    plt.show()

if __name__ == '__main__':
    for p in [5,10,15,20,30,40]:
        heatmap_drawpic('peak', p, 'TravelTm')
