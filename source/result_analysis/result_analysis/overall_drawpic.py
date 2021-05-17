import numpy as np
from matplotlib import rcParams
import pandas as pd
import matplotlib.pyplot as plt


def ANOVA_cal_by_stat(data_value):
    da = pd.DataFrame(data_value)

    da.columns.name = '场景'
    df1 = da.melt().dropna()
    from statsmodels.formula.api import ols
    from statsmodels.stats.anova import anova_lm

    model = ols('value ~ C(场景)', df1).fit()
    print(anova_lm(model))
def convert_dict(original_dict,i,j):
    key_list=[]
    value_list=[]
    for key in original_dict:
        key_list.append(key)
        value_list.append(original_dict[key])
    key_list[i],key_list[j]=key_list[j],key_list[i]
    value_list[i],value_list[j]=value_list[j],value_list[i]
    new_dict={}
    i=0
    for item in key_list:
        new_dict[item]=value_list[i]
        i+=1
    return new_dict

def overall_evaluation(pt):
    UNIQUE_FONT_SITE = 12
    plt.rcParams['font.sans-serif'] = ['Microsoft YaHei']
    plt.rcParams['axes.unicode_minus'] = False
    file_name=('C:\\Users\\run\\Desktop\\毕设仿真\\travel_time_result.txt')
    file_object=open(file_name,'r')
    file_context = file_object.read().splitlines()
    file_name=('C:\\Users\\run\\Desktop\\毕设仿真\\travel_time_result_2.txt')
    save_dir=('C:\\Users\\run\\Desktop\\毕设\\结果-图\\')
    file_object=open(file_name,'r')
    file_context += file_object.read().splitlines()
    # file_name=('C:\\Users\\run\\Desktop\\毕设仿真\\travel_time_result_platExam.txt')
    # file_object=open(file_name,'r')
    # file_context += file_object.read().splitlines()
    result_dict_type_NVeh={}
    result_dict_type_TravelTm={}
    manage_lane_type = ['无专用道','内侧硬隔离','内侧软隔离','外侧硬隔离','外侧软隔离','无专用道-hCAV']
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
        if travelTm==None:
            continue

        key=ml_type+'_'+peak_type+'_'+penetrate_ratio

        if int(penetrate_ratio)<=40 and int(penetrate_ratio)>=5and int(evaluation_times)<6900 and veh_type=='All' and peak_type==pt :
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
    overall_dict_NVeh={}
    overall_dict_TravelTm={}
    overall_dict_NVeh_err={}
    overall_dict_TravelTm_err={}
    for key in result_dict_type_NVeh:
        print(key,',', result_dict_type_NVeh[key])
        # print(len(result_dict_type_NVeh[key]))
        # print(np.mean(result_dict_type_NVeh[key]),np.std(result_dict_type_NVeh[key]))
        if key.split('_')[0] not in overall_dict_NVeh:
            overall_dict_NVeh[key.split('_')[0]]=[]
            overall_dict_NVeh_err[key.split('_')[0]]=[]
            overall_dict_NVeh[key.split('_')[0]].append(np.mean(result_dict_type_NVeh[key]))
            overall_dict_NVeh_err[key.split('_')[0]].append(np.std(result_dict_type_NVeh[key]))
        else:
            overall_dict_NVeh[key.split('_')[0]].append(np.mean(result_dict_type_NVeh[key]))
            overall_dict_NVeh_err[key.split('_')[0]].append(np.std(result_dict_type_NVeh[key]))

        # print(len(result_dict_type_NVeh[key]))
    ANOVA_cal_by_stat(result_dict_type_NVeh)
    for key in result_dict_type_TravelTm:

        print(key,',',result_dict_type_TravelTm[key])
        # print(len(result_dict_type_TravelTm[key]))
        # print(np.mean(result_dict_type_TravelTm[key]),np.std(result_dict_type_TravelTm[key]))
        if key.split('_')[0] not in overall_dict_TravelTm:
            overall_dict_TravelTm[key.split('_')[0]]=[]
            overall_dict_TravelTm_err[key.split('_')[0]]=[]
            overall_dict_TravelTm[key.split('_')[0]].append(np.mean(result_dict_type_TravelTm[key]))
            overall_dict_TravelTm_err[key.split('_')[0]].append(np.std(result_dict_type_TravelTm[key]))
        else:
            overall_dict_TravelTm[key.split('_')[0]].append(np.mean(result_dict_type_TravelTm[key]))
            overall_dict_TravelTm_err[key.split('_')[0]].append(np.std(result_dict_type_TravelTm[key]))
    line_types = ['o-',  '<--', 'v:','s:', '*--']
    i=0
    for t in overall_dict_NVeh:
        Xs=[5,10,15,20,30,40]
        plt.errorbar(x=Xs, y=overall_dict_NVeh[t], yerr=overall_dict_NVeh_err[t], fmt=line_types[i],markersize = 4, elinewidth=1.5, capsize=3)
        i+=1
        # print(overall_dict_NVeh[t])

    plt.legend(manage_lane_type,fontsize=UNIQUE_FONT_SITE-2)
    plt.xlabel('渗透率（%）',fontsize=UNIQUE_FONT_SITE)
    plt.ylabel('通行车辆流率（辆/10分钟）',fontsize=UNIQUE_FONT_SITE)
    # plt.ylim(400,540)
    plt.ylim(400,550)
    plt.xticks(fontsize=UNIQUE_FONT_SITE)
    plt.yticks(fontsize=UNIQUE_FONT_SITE)
    plt.savefig(save_dir+pt+'_NVEH.png',dpi=300,bbox_inches='tight',pad_inches=0.05)
    plt.show()
    i=0
    # print(overall_dict_TravelTm)
    overall_dict_TravelTm=convert_dict(overall_dict_TravelTm,1,2)
    for t in overall_dict_TravelTm:
        Xs=[5,10,15,20,30,40]
        plt.errorbar(x=Xs, y=overall_dict_TravelTm[t], yerr=overall_dict_TravelTm_err[t],fmt=line_types[i],markersize = 4, elinewidth=1.5, capsize=3)
        i+=1
        print(overall_dict_TravelTm[t])
    plt.legend(manage_lane_type,fontsize=UNIQUE_FONT_SITE-2)

    plt.xlabel('渗透率（%）',fontsize=UNIQUE_FONT_SITE)
    plt.ylabel('通行时间（s）',fontsize=UNIQUE_FONT_SITE)
    plt.xticks(fontsize=UNIQUE_FONT_SITE)
    plt.yticks(fontsize=UNIQUE_FONT_SITE)
    # plt.ylim(280,340)
    plt.savefig(save_dir+pt+'_TravelTm.png',dpi=300,bbox_inches='tight',pad_inches=0.05)
    plt.show()
if __name__=='__main__':
    overall_evaluation('sub-peak')