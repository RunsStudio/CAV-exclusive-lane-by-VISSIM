import numpy as np
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
def get_overall_performance(total_result,type_e):
    overall_performance=[]
    for p in range(0,6):
        sub_overall_performance=[]
        for i in range(1,5):
            sub_overall_performance.append(-(total_result[type_e][i][p]-total_result[type_e][0][p])/total_result[type_e][0][p])
        overall_performance.append(sub_overall_performance)
    return overall_performance
def to_percent(temp, position):
    return '%1.0f'%(100*temp) + '%'
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter
def auto_safety_evaluation(veh_type,e_type):
    # 中文乱码的处理
    plt.rcParams['font.sans-serif'] = ['Microsoft YaHei']
    plt.rcParams['axes.unicode_minus'] = False
    penetrate_ratio = [5, 10, 15, 20, 30, 40]
    manage_lane_type = ['base - fair','strongMLinLine','softMLinline','strongMLoutline','softMLoutline']
    safety_result_list = ['Safety_Total_100.txt','Safety_Total_101.txt']
    file_name_base = 'C:\\Users\\run\\Desktop\\毕设仿真\\veryFair\\'
    file_name_base_2 = 'C:\\Users\\run\\Desktop\\毕设仿真\\veryFair2\\'
    save_dir=('C:\\Users\\run\\Desktop\\毕设\\结果-图\\综合_')

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
            # for i in range(0,4):
            #     print(get_column(overall_res,i))
            #     print(get_column(overall_res_err,i))
            total_result=put_key(total_result,overall_res,'TET',0)
            total_result_err=put_key(total_result_err,overall_res_err,'TET',0)
            total_result=put_key(total_result,overall_res,'TIT',1)
            total_result_err=put_key(total_result_err,overall_res_err,'TIT',1)
            total_result=put_key(total_result,overall_res,'TERCRI',2)
            total_result_err=put_key(total_result_err,overall_res_err,'TERCRI',2)
            total_result=put_key(total_result,overall_res,'SAD',3)
            total_result_err=put_key(total_result_err,overall_res_err,'SAD',3)

        UNIQUE_FONT_SITE=12
        overall_performance=get_overall_performance(total_result, e_type)
        WIDTH_PLT=0.25
        RANGE_PLT=np.arange(0,8,2)
        texts=[]
        for p in range(0,6):
            plt.bar(x=RANGE_PLT+p*WIDTH_PLT,width=WIDTH_PLT,alpha = 0.8,height=overall_performance[p],edgecolor='black')
            texts.append([RANGE_PLT+p*WIDTH_PLT,overall_performance[p]])
        plt.xticks(RANGE_PLT + WIDTH_PLT*3, ['内侧硬隔离','内侧软隔离','外侧硬隔离','外侧软隔离'])

        plt.xlabel('专用道类型')
        plt.ylabel('优化比例（%）')
        plt.ylim(-0.3,0.5)
        plt.legend(['5%','10%','15%','20%','30%','40%'], fontsize=UNIQUE_FONT_SITE - 4,title='渗透率')
        plt.gca().yaxis.set_major_formatter(FuncFormatter(to_percent))
        plt.plot([-0.2,7.7],[0,0],color='k', linewidth=1.0)

        for text in texts:
            for j in range(0,4):
                # print(text[1][j])

                if text[1][j]>=0:
                    plt.text(text[0][j], text[1][j] + 0.005, "%.0f%%" % (100*text[1][j]), ha='center', va='bottom', fontsize=7)
                else:
                    plt.text(text[0][j], text[1][j] - 0.035, "%.0f%%" % (100*text[1][j]), ha='center', va='bottom', fontsize=7)
            # print(text)
        for i in range(0,6):
            strline=pt+','+str([5,10,15,20,30,40][i])+str(texts[i][1])
            print(strline.replace('[',',').replace(']',','))

        plt.savefig(save_dir+pt+'_'+e_type+'.png',dpi=300,bbox_inches='tight',pad_inches=0.05)
        plt.show()
        print(1)

if __name__=='__main__':
    # auto_safety_evaluation('CAV')
    # auto_safety_evaluation('HDV')
    auto_safety_evaluation('All','TIT')
    # auto_safety_evaluation('All','TIT')
    # auto_safety_evaluation('All','SAD')
