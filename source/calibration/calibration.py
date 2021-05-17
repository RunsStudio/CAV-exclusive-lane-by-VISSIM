# -*- coding: utf-8 -*-
import numpy as np
import geatpy as ea # import geatpy
from MyProblem import MyProblem # 导入自定义问题接口

"""===============================实例化问题对象==========================="""
problem = MyProblem() # 生成问题对象
"""=================================种群设置==============================="""
Encoding = 'RI'       # 编码方式
NIND = 30             # 种群规模
Field = ea.crtfld(Encoding, problem.varTypes, problem.ranges, problem.borders) # 创建区域描述器
population = ea.Population(Encoding, Field, NIND) # 实例化种群对象（此时种群还没被初始化，仅仅是完成种群对象的实例化）
"""===============================算法参数设置============================="""
myAlgorithm = ea.soea_SEGA_templet(problem, population) # 实例化一个算法模板对象
myAlgorithm.MAXGEN = 25 # 最大进化代数
"""==========================调用算法模板进行种群进化======================="""
[population, obj_trace, var_trace] = myAlgorithm.run() # 执行算法模板
population.save() # 把最后一代种群的信息保存到文件中
# 输出结果
best_gen = np.argmin(problem.maxormins * obj_trace[:, 1]) # 记录最优种群个体是在哪一代
best_ObjV = obj_trace[best_gen, 1]
print('最优的目标函数值为：%s'%(best_ObjV))
print('最优的控制变量值为：')
for i in range(var_trace.shape[1]):
    print(var_trace[best_gen, i])
print('有效进化代数：%s'%(obj_trace.shape[0]))
print('最优的一代是第 %s 代'%(best_gen + 1))
print('评价次数：%s'%(myAlgorithm.evalsNum))
print('时间已过 %s 秒'%(myAlgorithm.passTime))

MyProblem.py ：用来描述问题

# -*- coding: utf-8 -*-
import numpy as np
import geatpy as ea
import xlrd
import win32com.client as com  # VISSIM COM
import time

"""

"""


class MyProblem(ea.Problem):  # 继承Problem父类
    flag = False

    def __init__(self):
        name = 'MyProblem'  # 初始化name（函数名称，可以随意设置）
        M = 1  # 初始化M（目标维数）
        maxormins = [1]  # 初始化maxormins（目标最小最大化标记列表，1：最小化该目标；-1：最大化该目标）
        Dim = 3  # 初始化Dim（决策变量维数）
        varTypes = [0] * Dim  # 初始化varTypes（决策变量的类型，元素为0表示对应的变量是连续的；1表示是离散的）
        lb = [1.2, 0.9, 2]  # 决策变量下界 CC0推荐：1.2-1.7 cc1推荐：0.9-3.0 CC2推荐：2-7
        ub = [1.7, 3.0, 7]  # 决策变量上界
        lbin = [1, 1, 1]  # 决策变量下边界
        ubin = [1, 1, 1]  # 决策变量上边界
        # 调用父类构造方法完成实例化
        ea.Problem.__init__(self, name, M, maxormins, Dim, varTypes, lb, ub, lbin, ubin)

    def aimFunc(self, pop):  # 目标函数
        x = pop.Phen  # 得到决策变量矩阵 x=30
        x0 = x[:, [0]]
        x1 = x[:, [1]]
        x2 = x[:, [2]]
        # x1 = x[:, 1]
        # x2 = x[:, 2]
        # print(x)
        if self.flag == False:
            self.Data = xlrd.open_workbook('resultToCali2.xlsx')
            self.Table = self.Data.sheet_by_name(u'Sheet1')
            self.flag = True
            self.V1 = self.Table.col_values(0)  # spd for segment 1
            self.V2 = self.Table.col_values(1)  # spd for segment 2
            self.V3 = self.Table.col_values(2)  # spd for segment 3
            self.V4 = self.Table.col_values(3)  # spd for segment 4
            self.V5 = self.Table.col_values(6)  # spd for segment 5
            self.V6 = self.Table.col_values(7)  # spd for segment 6
            self.V7 = self.Table.col_values(8)  # spd for segment 7
            self.V8 = self.Table.col_values(9)  # spd for segment 8
            self.relativeFlow = self.Table.col_values(4)  # relative flow
            self.volume = self.Table.col_values(5)  # 流量
        totalResult = []
        for k in range(30):
            # # =================================================================================
            # # VISSIM Configurations
            # # Load VISSIM Network
            self.Vissim = com.Dispatch("Vissim.Vissim");
            dir = "E:\\PycharmProjects\\GA-calibration\\test.inp"
            self.Vissim.LoadNet(dir)
            # Define Simulation Configurations
            graphics = self.Vissim.Graphics
            graphics.SetAttValue("VISUALIZATION", False)  ## 设为 不可见 提高效率
            self.Sim = self.Vissim.Simulation
            self.Net = self.Vissim.Net

            # G = self.Vissim.Graphics
            dbpss = self.Net.DrivingBehaviorParSets  # Driving behavior module
            dbps = dbpss(3)
            # # Set Simulation Parameters
            TotalPeriod = 55802  # Define total simulation period
            WarmPeriod = 900  # Define warm period 10 minutes
            Random_Seed = k  # Define random seed
            step_time = 1  # Define Step Time
            self.Sim.Period = TotalPeriod
            self.Sim.RandomSeed = 42
            # self.Sim.RunIndex= 1
            self.Sim.Resolution = step_time

            # Each scenario run 5 times
            # =================================================================================
            # Data Collection Variables
            t1 = []
            t2 = []
            t3 = []
            t4 = []
            t5 = []
            t6 = []
            t7 = []
            t8 = []
            nVeh = []
            dbps.SetAttValue('CC0', x0[k][0])  ## 天坑！ 不能写x0[k]！！！！！！
            dbps.SetAttValue('CC1', x1[k][0])
            dbps.SetAttValue('CC2', x2[k][0])
            print("第", k, "组参数： CC0=", x0[k][0], ",CC1=", x1[k][0], ",CC2=", x2[k][0])
            eval = self.Vissim.Evaluation
            eval.SetAttValue("TRAVELTIME", True)
            eval.SetAttValue("DATACOLLECTION", True) #打开检测器评价，不然获得的结果是0.00
            TT1 = self.Net.TravelTimes(1)
            TT2 = self.Net.TravelTimes(2)
            dataCollections = self.Vissim.Net.DataCollections
            dt1 = dataCollections(1)   #设置在K0+530断面
            dt2 = dataCollections(2)
            dt3 = dataCollections(3)
            dt4 = dataCollections(4)
            dt5 = dataCollections(5)   #设置在K4+600断面
            dt6 = dataCollections(6)
            dt7 = dataCollections(7)
            dt8 = dataCollections(8)
            composition = self.Net.TrafficCompositions(1)
            vehicleInput = self.Net.VehicleInputs(1)
            # self.Sim.RunContinuous()
            for j in range(1, TotalPeriod):
                if (j % 900 == 0) and (j >= 1801):
                    composition.SetAttValue1("RELATIVEFLOW", 100, 1 - self.relativeFlow[int(j / 900 - 3)])
                    composition.SetAttValue1("RELATIVEFLOW", 200, self.relativeFlow[int(j / 900 - 3)])
                    vehicleInput.SetAttValue("VOLUME", self.volume[int(j / 900 - 3)])
                    t1.append(dt1.GetResult("speed", "mean", 0))  # 车道1
                    t2.append(dt2.GetResult("speed", "mean", 0))  # 车道2
                    t3.append(dt3.GetResult("speed", "mean", 0))  # 车道3
                    t4.append(dt4.GetResult("speed", "mean", 0))  # 车道4
                    t5.append(dt5.GetResult("speed", "mean", 0))  # 车道1
                    t6.append(dt6.GetResult("speed", "mean", 0))  # 车道2
                    t7.append(dt7.GetResult("speed", "mean", 0))  # 车道3
                    t8.append(dt8.GetResult("speed", "mean", 0))  # 车道4  检测器5-8是后来新增的
                    nVeh.append(4 * (dt1.GetResult("NVEHICLES", "sum", 0)
                                     + dt2.GetResult("NVEHICLES", "sum", 0)
                                     + dt3.GetResult("NVEHICLES", "sum", 0)  
                                     + dt4.GetResult("NVEHICLES", "sum", 0))) ## 车辆数的检测
                self.Sim.RunSingleStep()
            spdTotal = np.array(0.125 * sum(abs(np.array(t1) - np.array(self.V1)) / np.array(self.V1)
                                            + abs(np.array(t2) - np.array(self.V2)) / np.array(self.V2)
                                            + abs(np.array(t3) - np.array(self.V3)) / np.array(self.V3)
                                            + abs(np.array(t4) - np.array(self.V4)) / np.array(self.V4)
                                            + abs(np.array(t5) - np.array(self.V5)) / np.array(self.V5)
                                            + abs(np.array(t6) - np.array(self.V6)) / np.array(self.V6)
                                            + abs(np.array(t7) - np.array(self.V7)) / np.array(self.V7)
                                            + abs(np.array(t8) - np.array(self.V8)) / np.array(self.V8)))  
            # NVehTotal = np.array(sum(abs(np.array(nVeh) - np.array(self.volume)) / np.array(self.volume)))  ##
            totalResult.append(spdTotal)  # +NVehTotal)
            # print("spd误差：", spdTotal)
            # print("NVeh误差：", NVehTotal)
            print("误差为：", totalResult[k])
            self.Sim.Stop()
        pop.ObjV = np.vstack(totalResult)  # 计算目标函数值，赋值给pop种群对象的ObjV属性