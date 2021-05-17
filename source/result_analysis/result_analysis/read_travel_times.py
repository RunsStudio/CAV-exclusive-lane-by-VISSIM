import win32com.client as com  # VISSIM COM
import numpy as np

# class VehType:
#     CAV = '11'
#     HDV = '10'
#     ALL = 'All'
#
#
# class VehTravelTime(self):
#     No = 0
#     veh_Type = 0
#     veh_counts_list = []
#     travel_time_list = []
#     average_veh_counts = np.mean(veh_counts_list)
#     average_travel_time_counts = np.mean(travel_time_list)


if __name__ == '__main__':
    file_name_base = 'C:\\Users\\run\\Desktop\\毕设仿真\\veryFair-PlatoonExam\\'
    penetrate_ratio = [0,5, 10, 15, 20, 30, 40,60,80,100]
    penetrate_ratio_base = [0, 60, 80, 100]
    manage_lane_type = ['base - fair']
    # manage_lane_type = ['base - fair']
    vol_type = ['peak', 'sub-peak']
    output_file_name = open('C:\\Users\\run\\Desktop\\毕设仿真\\travel_time_result_platExam.txt', 'w')
    ks = ['10', '11', 'All']
    Vissim = com.Dispatch("Vissim.Vissim.600")
    for item_ml in manage_lane_type:
        for p in penetrate_ratio:
            for vt in vol_type:
                Vissim.LoadNet(file_name_base + item_ml + '\\' + vt + '\\' + str(p) + '\\simu_model.inpx')
                smr_cnt = Vissim.Net.SimulationRuns.count
                tc = Vissim.Net.VehicleTravelTimeMeasurements.ItemByKey(1)
                for i in range(smr_cnt - 9, smr_cnt + 1):
                    for j in range(1, 11, 1):
                        for k in ks:
                            key_veh = 'Vehs(' + str(i) + ',' + str(j) + ',' + k + ')'
                            key_trav = 'TravTm(' + str(i) + ',' + str(j) + ',' + k + ')'
                            file_str = item_ml + '-NoPlat,' + str(p) + ',' +vt+','+ str(i) + ',' + str(j * 600 + 900) + ',' +k+','+ str(tc.AttValue(key_veh)) + ',' + str(tc.AttValue(key_trav))
                            print(file_str, file=output_file_name)
                            print(item_ml, p, str(i) + ',' + str(j) + 'Vehs=' + str(tc.AttValue(key_veh)))
                            print(item_ml, p, str(i) + ',' + str(j) + 'TravTm=' + str(tc.AttValue(key_trav)))
