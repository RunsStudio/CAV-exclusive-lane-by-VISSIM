# CAV-exclusive-lane

CAV专用道仿真评估，基于VISSIM实现

### 运行要求

- VISSIM_6.00-21_win32_Full （必须是大版本）

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
  + calibration - 标定代码，参考https://kunrunwu.gitee.io/2020/01/09/%E4%BD%BF%E7%94%A8%E9%81%97%E4%BC%A0%E7%AE%97%E6%B3%95%E6%A0%87%E5%AE%9AVISSIM-4.3/
  + CAV_microscopic_model - CAV微观模型
  + result_analysis - 结果分析

