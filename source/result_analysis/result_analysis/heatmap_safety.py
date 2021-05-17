import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
def get_safety_res(file_name,sr,record_nums):
    result_list = []

    file = open(file_name+sr, 'r', encoding='utf-8')
    # print(file_name+sr)
    lines = file.readlines()[1::2]
    file.close()
    line = lines[-record_nums:]
    for l in line:
        one_evaluation_res = l.replace('\n', '').split(',')
        result_list.append(one_evaluation_res[1:])
    return result_list
def merge_CAV_MV(s1,s2,index):
    result_list=[]
    for i in range(0,len(s1)):
        for j in range(0,len(s1[i])):
            s = []
            for k in range(0,len(s1[i][j])):
                if index=='All':
                    s.append(float(s1[i][j][k])+float(s2[i][j][k]))
                if index=='CAV':
                    s.append(float(s2[i][j][k]))
                if index=='HDV':
                    s.append(float(s1[i][j][k]))
            result_list.append(s)
    return result_list

def get_column(list,index):
    res=[]
    for i in range(0,len(list)):
        res.append(list[i][index])
    return res
def get_all_record_safety(pt,p,return_type):
    penetrate_ratio = [5, 10, 15, 20, 30, 40]
    manage_lane_type = ['base - fair','strongMLinLine','softMLinline','strongMLoutline','softMLoutline']
    safety_result_list = ['Safety_Total_100.txt','Safety_Total_101.txt']
    file_name_base = 'C:\\Users\\run\\Desktop\\毕设仿真\\veryFair\\'
    file_name_base_2 = 'C:\\Users\\run\\Desktop\\毕设仿真\\veryFair2\\'
    save_dir=('C:\\Users\\run\\Desktop\\毕设\\结果-图\\')
    result_dict_type_TET = {}
    result_dict_type_TIT = {}
    result_dict_type_SAD = {}
    eveluation_type = ['TET', 'TIT', 'SAD']
    for item_ml in manage_lane_type:
        safety_res_100 = []
        safety_res_101 = []
        safety_res = []
        file_name = file_name_base + item_ml + '\\'+pt+'\\' + str(p) + '\\'
        file_name_2 = file_name_base_2 + item_ml + '\\'+pt+'\\' + str(p) + '\\'
        # print(file_name)
        if item_ml != 'base - fair-NoPlat':
            safety_res_100.append(get_safety_res(file_name,'Safety_Total_100.txt',5))
            safety_res_100.append(get_safety_res(file_name_2,'Safety_Total_100.txt',5))
            safety_res_101.append(get_safety_res(file_name,'Safety_Total_101.txt',5))
            safety_res_101.append(get_safety_res(file_name_2,'Safety_Total_101.txt',5))
        else:
            safety_res_100.append(get_safety_res(file_name_2,'Safety_Total_100.txt',10))
            safety_res_101.append(get_safety_res(file_name_2,'Safety_Total_101.txt',10))
        key = item_ml + '_' + pt + '_' + str(p)
        safety_res.append(merge_CAV_MV(safety_res_100,safety_res_101,'All'))
        print(key,get_column(safety_res[0],1))
        result_dict_type_TET[key]=get_column(safety_res[0],0)
        result_dict_type_TIT[key]=get_column(safety_res[0],1)
        result_dict_type_SAD[key]=get_column(safety_res[0],3)
    if return_type=='TET':
        return result_dict_type_TET
    elif return_type=='TIT':
        return result_dict_type_TIT
    else:
        return result_dict_type_SAD

def heatmap_drawpic(pt, p, return_type):
    UNIQUE_FONT_SITE = 12
    plt.rcParams['font.sans-serif'] = ['Microsoft YaHei']
    plt.rcParams['axes.unicode_minus'] = False
    save_dir = ('C:\\Users\\run\\Desktop\\毕设\\结果-图\\')
    manage_lane_type = ['无专用道','内侧硬隔离', '内侧软隔离',  '外侧硬隔离', '外侧软隔离']
    yy = np.array(list(get_all_record_safety(pt, p, return_type).values()))
    df = pd.DataFrame(yy, columns=[x for x in range(0, 10)])
    plt.xlabel('x_label', fontsize=UNIQUE_FONT_SITE, color='k')  # x轴label的文本和字体大小
    plt.ylabel('y_label', fontsize=UNIQUE_FONT_SITE, color='k')  # y轴label的文本和字体大小
    plt.xticks(fontsize=UNIQUE_FONT_SITE)  # x轴刻度的字体大小（文本包含在pd_data中了）
    plt.yticks(fontsize=UNIQUE_FONT_SITE)  # y轴刻度的字体大小（文本包含在pd_data中了）
    # 设置图框线粗细
    sns.heatmap(df, annot=False, yticklabels=manage_lane_type,cmap='YlGnBu')
    # plt.savefig(save_dir + pt + '_' + str(p) +'_' +return_type+ '_HEATMAP.png', bbox_inches='tight', dpi=300, pad_inches=0.05)
    plt.show()

if __name__=='__main__':
    for p in [5,10,15,20,30,40]:
        heatmap_drawpic('peak',p,'SAD')
        heatmap_drawpic('sub-peak',p,'SAD')
        # heatmap_drawpic('peak',p,'TIT')
        # heatmap_drawpic('peak',p,'SAD')