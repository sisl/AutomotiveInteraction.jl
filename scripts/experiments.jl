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

#*****************Generate idm params for lmfit************
s = scenarios_upper
f = FilteringEnvironment()
name = "upper"

for i in 1:length(s)
        veh_id_list = s[i][1]
        ts = s[i][2][1]
        te = s[i][2][2]
        filename = "media/upper_$i.jld";
        id_list,ts,te = JLD.load(filename,"veh_id_list","ts","te");
        scene_list_true = replay_scenelist(f,id_list=id_list,ts=ts,te=te);

        dirname = "$(name)_$(i)"
        mkdir("lmfit/$(dirname)")
        feat_dict = scenelist2idmfeatures(f,scene_list_true,id_list=id_list,
                        scenario_name=name,scenario_number=i);
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


#***********Histogram plot****************
# Full example in docstring of Filtering/metrics.jl vel_distribution
a = PGFPlots.Axis([veh_hist_true,v_hist_idm,v_hist_cidm,v_hist_lmidm,v_hist_pf,
Plots.Command(raw"\legend{true,idm,cidm,lmidm,pf}")
]);

aidm = PGFPlots.Axis([veh_hist_true,v_hist_idm,Plots.Command(raw"\legend{true,idm}")],
xlabel="velocity",ylabel="density",title="True vs IDM");

acidm = PGFPlots.Axis([veh_hist_true,v_hist_cidm,Plots.Command(raw"\legend{true,c-idm}")],
xlabel="velocity",ylabel="density",title="True vs C-IDM");

almidm = PGFPlots.Axis([veh_hist_true,v_hist_lmidm,Plots.Command(raw"\legend{true,lm-idm}")],
xlabel="velocity",ylabel="density",title="True vs LM-IDM");

apf = PGFPlots.Axis([veh_hist_true,v_hist_pf,Plots.Command(raw"\legend{true,pf}")],
xlabel="velocity",ylabel="density",title="True vs pf");

PGFPlots.save("media/vhist_idm.svg",aidm)
PGFPlots.save("media/vhist_cidm.svg",acidm)
PGFPlots.save("media/vhist_lmidm.svg",almidm)
PGFPlots.save("media/vhist_pf.svg",apf)

#********************Train upper test lower******************
# We need to show a variability in the generated scenarios
# So we need to combine the particle sets of different vehicles together
# And then for the same set of vehicles in the lower merge i.e. test domain
# We show significantly different driving behavior by sampling from the particle set
