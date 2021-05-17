import numpy as np
class SafetyEvaluation:
    TET = 0.0
    TIT = 0.0
    TERCRI = 0.0
    SAD = 0.0
    TET_err=0.0
    TIT_err=0.0
    TERCRI_err=0.0
    SAD_err=0.0
    ML_type = ''
    penetrate_ratio = 0

    def __init__(self, safety_res, ML, p):
        TET_list = []
        TIT_list = []
        TERCRI_list = []
        SAD_list = []
        for type_veh in safety_res:
            for t in type_veh:
                TET_list.append(t[0])
                TIT_list.append(t[1])
                TERCRI_list.append(t[2])
                SAD_list.append(t[3])
        self.TET =np.mean(TET_list)
        self.TIT =np.mean(TIT_list)
        self.TERCRI =np.mean(TERCRI_list)
        self.SAD=np.mean(SAD_list)
        self.TET_err=np.std(TET_list)
        self.TIT_err=np.std(TIT_list)
        self.TERCRI_err=np.std(TERCRI_list)
        self.SAD_err=np.std(SAD_list)
        self.ML_type = ML
        self.penetrate_ratio = p


import csv


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
def merge_CAV_MV_isolated(s1,s2,index):
    result_list=[]
    for i in range(0,len(s1)):
        for j in range(0,len(s1[i])):
            s = []
            for k in range(0,len(s1[i][j])):
                if index==1:
                    s.append(float(s1[i][j][k]))
            result_list.append(s)
    return result_list
def get_safety_res_mean(safety_res,flag):
    TET=[]
    TIT=[]
    TERCRI=[]
    SAD=[]
    # a=safety_res[:,0]
    for i  in range (0,10):
        for j in range(0,4):
            TET.append(safety_res[i][0])
            TIT.append(safety_res[i][1])
            TERCRI.append(safety_res[i][2])
            SAD.append(safety_res[i][3])
    if flag=='mean':
        return [np.mean(TET),np.mean(TIT),np.mean(TERCRI),np.mean(SAD)]
    return [np.std(TET),np.std(TIT),np.std(TERCRI),np.std(SAD)]

def get_column(list,index):
    res=[]
    for i in range(0,len(list)):
        res.append(list[i][index])
    return res
def put_key(dict,list,key,index):
    if key not in dict:
        dict[key] = []
        dict[key].append(get_column(list, index))
    else:
        dict[key].append(get_column(list, index))
    return dict
import matplotlib.pyplot as plt
def auto_safety_evaluation(veh_type):
    # 中文乱码的处理
    plt.rcParams['font.sans-serif'] = ['Microsoft YaHei']
    plt.rcParams['axes.unicode_minus'] = False
    penetrate_ratio = [5, 10, 15, 20, 30, 40]
    manage_lane_type = ['base - fair','strongMLinLine','softMLinline','strongMLoutline','softMLoutline']
    safety_result_list = ['Safety_Total_100.txt','Safety_Total_101.txt']
    file_name_base = 'C:\\Users\\run\\Desktop\\毕设仿真\\veryFair\\'
    file_name_base_2 = 'C:\\Users\\run\\Desktop\\毕设仿真\\veryFair2\\'
    save_dir=('C:\\Users\\run\\Desktop\\毕设\\结果-图\\')

    eveluation_type = ['TET', 'TIT', 'SAD']
    peak_type=['peak','sub-peak']
    for pt in peak_type:
        total_result = {}
        total_result_err = {}
        for item_ml in manage_lane_type:
            overall_res=[]
            overall_res_err=[]
            for p in penetrate_ratio:
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
                    print(1)
                # print(len(safety_res_101),len(safety_res_100))
                safety_res.append(merge_CAV_MV(safety_res_100,safety_res_101,veh_type))
                # print(safety_res)
                safety_res_mean=get_safety_res_mean(safety_res[0],'mean')
                safety_res_std=get_safety_res_mean(safety_res[0],'std')
                overall_res.append(safety_res_mean)
                overall_res_err.append(safety_res_std)
            for i in range(0,4):
                print(get_column(overall_res,i))
                print(get_column(overall_res_err,i))
            total_result=put_key(total_result,overall_res,'TET',0)
            total_result_err=put_key(total_result_err,overall_res_err,'TET',0)
            total_result=put_key(total_result,overall_res,'TIT',1)
            total_result_err=put_key(total_result_err,overall_res_err,'TIT',1)
            total_result=put_key(total_result,overall_res,'TERCRI',2)
            total_result_err=put_key(total_result_err,overall_res_err,'TERCRI',2)
            total_result=put_key(total_result,overall_res,'SAD',3)
            total_result_err=put_key(total_result_err,overall_res_err,'SAD',3)
        # print(total_result)
        # print(total_result_err)
        UNIQUE_FONT_SITE=12
        line_types=['o-','v:','<--','s:','*--']
        for e in eveluation_type:
            print(penetrate_ratio)
            print(total_result[e])
            for item_ml in range(0,5):
                plt.errorbar(x=penetrate_ratio, y=total_result[e][item_ml], yerr=total_result_err[e][item_ml],fmt=line_types[item_ml],markersize = 4, elinewidth=1.5, capsize=3)
                # plt.scatter(x=penetrate_ratio,y=total_result[e][item_ml],marker=)
            plt.legend(['无专用道','内侧硬隔离','内侧软隔离','外侧硬隔离','外侧软隔离'],fontsize=UNIQUE_FONT_SITE-1)
            plt.xlabel('渗透率（%）',fontsize=UNIQUE_FONT_SITE)
            plt.ylabel(e+'(s)',fontsize=UNIQUE_FONT_SITE)
            # plt.ylim(2000,30000)
            # 设置刻度字体大小
            plt.xticks(fontsize=UNIQUE_FONT_SITE)
            plt.yticks(fontsize=UNIQUE_FONT_SITE)
            plt.savefig(save_dir + pt+'_'+e+'_'+veh_type+'.png',bbox_inches='tight',dpi=300,pad_inches=0.05)
            plt.show()

if __name__=='__main__':
    # auto_safety_evaluation('CAV')
    # auto_safety_evaluation('HDV')
    auto_safety_evaluation('All')