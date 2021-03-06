# This file is used for performing experiments
# Run by changing path in REPL to scripts folder
# Then `include("experiments.jl")`

using AutomotiveSimulator
using AutomotiveInteraction
using PGFPlots
using JLD

include("helpers.jl")

# Scenarios for upper merge
# List of tuples. Elem 1 of tuple is veh id list, elem 2 is list with ts and te
scenarios_upper = [
        ([6,8,13,19,28,29],[1,120]), #19,29 merge lane
        ([34,37,40,42,43,49,50],[88,230]), #34,37,43,50 merge lane
        ([59,60,62,66,67,71,73],[188,390]), #59,62,66,71 merge lane
        ([66,67,71,73,76,82,88,92,95],[310,440]), #71,76,95 merge lane
        ([187,191,193,194],[940,985]), #187,191,194 merge lane

        ([211,213,215,217,221],[1135,1190]), #211,217,221 merge lane
        ([248,249,250,256],[1323,1420]), # 249 merge lane
        ([272,275,276,277,279],[1440,1615]), #275, 277 merge lane
        ([294,295,300,307,308,311],[1576,1780]), #295,307,308 merge lane
        ([324,327,329,333,335,339,341,346,354,357],[1810,1950]) #327,333,339,346,354
]

# Scenarios for lower merge
scenarios_lower = [
        ([2,7,10,18,25],[1,30]), #7,25 merge vehicles
        ([64,68,70,74,77],[198,230]), #70 merge lane
        ([105,106,108,110],[382,430]), # 110 merge lane
        ([176,177,178],[827,870]), #177 merge lane

        ([252,253,255],[1315,1370]), #253 merge lane
        ([361,363,364,366],[1830,1885]) #363 merge lane
]

#***********Run filtering and store resulting models to jld files****
# s = scenarios_lower
# f = FilteringEnvironment(mergeenv=MergingEnvironmentLower())

s = scenarios_upper
f = FilteringEnvironment()
for i in 1:length(s)
        veh_id_list = s[i][1]
        ts = s[i][2][1]
        te = s[i][2][2]
        m,p,md = obtain_driver_models(f,veh_id_list=veh_id_list,num_p=50,ts=ts,te=te)
        #JLD.save("media/lower_$i.jld","m",m,"p",p,"md",md,"veh_id_list",veh_id_list,"ts",ts,"te",te)
        JLD.save("media/upper_$i.jld","m",m,"p",p,"md",md,"veh_id_list",veh_id_list,"ts",ts,"te",te)
end


#************Docstring example code from multiscenarios_pf in helpers.jl*****
# USEFUL to keep around for making tikz plots later on by rerunning this script
# Select a block of code and press shift+enter to run block selectively

# Lower merge
rmse_pos_mat_idm,rmse_vel_mat_idm,coll_mat_idm = 
        multiscenarios_idm(mergetype="lower",modelmaker=make_IDM_models);
rmse_pos_mat_cidm,rmse_vel_mat_cidm, coll_mat_cidm = 
        multiscenarios_idm(mergetype="lower",modelmaker=make_cidm_models);
rmse_pos_mat_lmidm, rmse_vel_mat_lmidm, coll_mat_lmidm = 
        multiscenarios_idm(mergetype="lower",modelmaker=make_lmidm_models);
rmse_pos_mat_pf,rmse_vel_mat_pf, coll_mat_pf = multiscenarios_pf(mergetype="lower");

rmse_pos_list = [rmse_pos_mat_idm,rmse_pos_mat_cidm,rmse_pos_mat_lmidm,rmse_pos_mat_pf];
rmse_plots_modelscompare(rmse_pos_list,filename = "media/rmse_pos_lower.svg");

coll_mat_list = [coll_mat_idm,coll_mat_cidm,coll_mat_lmidm,coll_mat_pf];
coll_barchart(coll_mat_list,filename = "media/coll_barchart_lower.svg");

# Upper merge
rmse_pos_mat_idm,rmse_vel_mat_idm,coll_mat_idm = 
        multiscenarios_idm(mergetype="upper",modelmaker=make_IDM_models);
rmse_pos_mat_cidm, rmse_vel_mat_cidm, coll_mat_cidm = 
        multiscenarios_idm(mergetype="upper",modelmaker=make_cidm_models);
rmse_pos_mat_lmidm, rmse_vel_mat_lmidm, coll_mat_lmidm = 
        multiscenarios_idm(mergetype="upper",modelmaker=make_lmidm_models);
rmse_pos_mat_pf, rmse_vel_mat_pf, coll_mat_pf = multiscenarios_pf(mergetype="upper");

rmse_pos_list = [rmse_pos_mat_idm,rmse_pos_mat_cidm,rmse_pos_mat_lmidm,rmse_pos_mat_pf];
rmse_plots_modelscompare(rmse_pos_list,filename = "media/rmse_pos_upper.svg");

coll_mat_list = [coll_mat_idm,coll_mat_cidm,coll_mat_lmidm,coll_mat_pf];
coll_barchart(coll_mat_list,filename = "media/coll_barchart_upper.svg");


#********************Train upper test lower******************
# We need to show a variability in the generated scenarios
# So we need to combine the particle sets of different vehicles together
# And then for the same set of vehicles in the lower merge i.e. test domain
# We show significantly different driving behavior by sampling from the particle set


#************LsqFit failed experimentation***************
# model(t, p) = p[1] * exp.(-p[2] * t)
# tdata = range(0,stop=10,length=20)
# ydata = model(tdata, [1.0 2.0]) + 0.01*randn(length(tdata))
# p0 = [0.5,0.5]
# fit = curve_fit(model, tdata, ydata, p0)

# function test_lmfit(d,p)   
#         print("test_lmfit called\n")
#         print("says:d=$d\n") 
#         t = d["t"]
#         return p[1]*exp.(-p[2]*t)
# end

# test_tdata = []
# for t in tdata
#         push!(test_tdata,Dict("t"=>t))
# end

# curve_fit(test_lmfit,test_tdata,ydata,[0.5,0.5])