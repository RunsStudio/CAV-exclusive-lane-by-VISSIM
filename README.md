# CAV-exclusive-lane

基于VISSIM实现在沪宁高速公路低CAV渗透率场景下，设置CAV专用道对高速公路整体行驶安全性和效率的影响，并对该场景进行仿真评估。CAV车辆跟驰使用IDM模型，换道使用自定义的换道逻辑：车辆行驶到高速公路上之后尽快换入CAV专用车道。在专用车道上行驶时CAV不存在换道行为（仅设置1条专用车道）。常规车辆（即人工驾驶车辆，HDV）使用VISSIM 自带的Wiedemann 99模型，使用沪宁高速公路车辆通行的数据对其进行了标定。在不设置专用车道场景下，CAV和HDV混行，在设置专用车道场景下，二者各行其道。本研究面向近未来使用，侧重低渗透率场景：即CAV渗透率低于20%的场景。

详细的建模过程，已公开于本人毕业设计，欢迎阅读、引用：
[1]吴坤润. 沪宁高速自动驾驶专用车道设置策略及仿真研究[D].东南大学,2021.DOI:10.27014/d.cnki.gdnau.2021.003245.

### 运行要求

- VISSIM_6.00-21_win32_Full （必须是大版本，而且支持修改外部驾驶员模型）

### 编译工具

- Visual Studio （编译CAV外部驾驶员模型）
- Pycharm （结果分析、自动运行、参数标定）
  - Python 3.7
  - win32com
  - pandas
  - numpy
  - matplotlib
  - seaborn
  - scipy

### 目录结构

+ dll - 编译好的dll文件

+ result - 结果文件

+ simulation_model - 仿真模型

  + base - fair - 无专用道

    + CACCConf.dat - 设置文件 设置其中的参数，其中各行含义参考out_newdll.dat
    + simu_model.inpx - 仿真的模型，用VISSIM打开
    + Safety_Total_100.txt - 输出的常规车辆的评价结果
    + Safety_Total_101.txt - 输出的CAV的评价结果

    ```
    Tue Nov 17 11:57:05 2020
    ,985804,180329,285688,9155
    分别表示TET ，TIT ，TERCRI（暂不使用）， SAD指标
    ```

    

  + softMLinline 内侧+软隔离专用道

  + softMLoutline 外侧+软隔离专用道

  + strongMLinline 内侧+硬隔离专用道

  + strongMLoutline 外侧+硬隔离专用道

+ source - 源代码

  + auto_running - 自动运行工具
  + calibration - 标定代码，参考https://runsstudio.github.io/2021/07/18/%E4%BD%BF%E7%94%A8%E9%81%97%E4%BC%A0%E7%AE%97%E6%B3%95%E6%A0%87%E5%AE%9AVISSIM-4.3/
  + CAV_microscopic_model - CAV微观模型
  + result_analysis - 结果分析

