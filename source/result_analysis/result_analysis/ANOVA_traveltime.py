import pandas as pd
import numpy as np
import scipy.stats as stats
def ANOVA_cal_by_stat(data_value):
    da = pd.DataFrame(data_value)

    da.columns.name = '场景'
    df1 = da.melt().dropna()
    from statsmodels.formula.api import ols
    from statsmodels.stats.anova import anova_lm

    model = ols('value ~ C(场景)', df1).fit()
    print(anova_lm(model))
def ttest(dat,dat2,key):
    stat_val, p_val = stats.levene(dat,dat2)

    if p_val<0.05: # 方差有显著性差异
        stat_val, p_val = stats.ttest_ind(dat,dat2,equal_var=False)
        print(replacekey(key),'\t否\t%6.3f\t%6.4f' % (stat_val, p_val))
    else:
        stat_val, p_val = stats.ttest_ind(dat, dat2)
        print(replacekey(key),'\t是\t%6.3f\t%6.4f' % (stat_val, p_val))

def replacekey(key):
    key_proceed=str(key).replace('_','\t')
    key_proceed=key_proceed.replace('sub-peak','平峰')
    key_proceed=key_proceed.replace('peak','高峰')
    key_proceed=key_proceed.replace('All','所有车辆')
    key_proceed=key_proceed.replace('softMLinline','内侧软隔离')
    key_proceed=key_proceed.replace('strongMLinLine','内侧硬隔离')
    key_proceed=key_proceed.replace('strongMLoutline','外侧硬隔离')
    key_proceed=key_proceed.replace('softMLoutline','外侧软隔离')
    key_proceed=key_proceed.replace('NVEH','车辆数')
    key_proceed=key_proceed.replace('TravalTm','行程时间')
    return key_proceed

def ttest_travelTm_and_NVeh(vt,pt):
    file_name=('C:\\Users\\Administrator\\Desktop\\毕设仿真\\travel_time_result.txt')
    file_object=open(file_name,'r')
    file_context = file_object.read().splitlines()
    file_name=('C:\\Users\\Administrator\\Desktop\\毕设仿真\\travel_time_result_2.txt')
    save_dir=('C:\\Users\\Administrator\\Desktop\\毕设\\结果-图\\')
    file_object=open(file_name,'r')
    file_context += file_object.read().splitlines()

    manage_lane_type = ['无专用道','内侧软隔离','内侧硬隔离','外侧硬隔离','外侧软隔离']
    penetrate_ratios=[5,10,15,20,30,40]

    for p in penetrate_ratios:
        result_dict_type_NVeh = {}
        result_dict_type_TravelTm = {}
        for line in file_context:
            ls=line.split(',')
            ml_type=ls[0]
            penetrate_ratio=ls[1]
            peak_type=ls[2]
            simu_times=ls[3]
            evaluation_times=ls[4]
            veh_type=ls[5]
            Nveh=ls[6]
            travelTm=ls[7]
            key=ml_type+'_'+peak_type+'_'+penetrate_ratio

            if int(evaluation_times)<6900 and veh_type==vt and peak_type==pt and int(penetrate_ratio)==p:
                if key not in result_dict_type_NVeh:
                    result_dict_type_NVeh[key]=[]
                    result_dict_type_NVeh[key].append(int(Nveh))
                else:
                    result_dict_type_NVeh[key].append(int(Nveh))
                if key not in result_dict_type_TravelTm:
                    result_dict_type_TravelTm[key]=[]
                    result_dict_type_TravelTm[key].append(float(travelTm))
                else:
                    result_dict_type_TravelTm[key].append(float(travelTm))
        for key in result_dict_type_NVeh:
            if 'base - fair' not in key:
                base_key = 'base - fair_' + pt + '_' + str(p)
                ttest(result_dict_type_NVeh[key], result_dict_type_NVeh[base_key],key+'_NVEH_所有车辆')
                # ttest(result_dict_type_TravelTm[key], result_dict_type_TravelTm[base_key],key+'_TravalTm_所有车辆')
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
def get_column(list,index):
    res=[]
    for i in range(0,len(list)):
        res.append(list[i][index])
    return res
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
def ttest_safety(pt,p):
    penetrate_ratio = [5, 10, 15, 20, 30, 40]
    manage_lane_type = ['base - fair','strongMLinLine','softMLinline','strongMLoutline','softMLoutline']
    safety_result_list = ['Safety_Total_100.txt','Safety_Total_101.txt']
    file_name_base = 'C:\\Users\\Administrator\\Desktop\\毕设仿真\\veryFair\\'
    file_name_base_2 = 'C:\\Users\\Administrator\\Desktop\\毕设仿真\\veryFair2\\'
    save_dir=('C:\\Users\\Administrator\\Desktop\\毕设\\结果-图\\')
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
        # print(key,get_column(safety_res[0],1))
        result_dict_type_TET[key]=get_column(safety_res[0],0)
        result_dict_type_TIT[key]=get_column(safety_res[0],1)
        result_dict_type_SAD[key]=get_column(safety_res[0],3)
    for key in result_dict_type_TET:
        if 'base - fair' not in key:
            base_key = 'base - fair_' + pt + '_' + str(p)
            ttest(result_dict_type_SAD[key], result_dict_type_SAD[base_key], key + '_SAD_所有车辆')
            # ttest(result_dict_type_TIT[base_key], result_dict_type_TIT[key], key + '_TIT_自动驾驶车辆')
if __name__=='__main__':
    ttest_travelTm_and_NVeh('All','peak')
    ttest_travelTm_and_NVeh('All','sub-peak')
    # for p in [5,10,15,20,30,40]:
    #     ttest_safety('peak',p)
    #     ttest_safety('sub-peak',p)
