import win32com.client as com  # VISSIM COM

if __name__=='__main__':
    penetrate_ratio = [0,5, 10, 15, 20, 30,40,60,80,100]
    # manage_lane_type = ['base - fair','softMLinline','strongMLinLine','strongMLoutline']
    manage_lane_type = ['base - fair']
    peak_type=['peak','sub-peak']
    Vissim = com.Dispatch("Vissim.Vissim.600")
    for p in penetrate_ratio:
        for item_ml in manage_lane_type:
            for k in peak_type:
                dir = "your_dir  \\"+str(item_ml)+"\\"+k+"\\"+str(p)+"\\simu_model.inpx"
                Vissim.LoadNet(dir)
                dbps = Vissim.Net.DrivingBehaviors.ItemByKey(3)  # Driving behavior module

                # dbps = dbpss(3)
                dbps.SetAttValue('W99cc0', 1.233)
                dbps.SetAttValue('W99cc1', 1.477)
                dbps.SetAttValue('W99cc2', 2.466)
                Vissim.Graphics.CurrentNetworkWindow.SetAttValue('QuickMode',True)
                Vissim.Simulation.SetAttValue('RandSeed',170)
                Vissim.Simulation.SetAttValue('NumRuns',5)
                # input('1')
                Vissim.SaveNet()
                Vissim.Simulation.RunContinuous()


