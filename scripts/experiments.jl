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
        ([6,8,13,19,28,29],[1,150]), #19,29 merge lane
        ([34,37,40,42,43,49,50],[88,250]), #34,37,43,50 merge lane
        ([59,60,62,66,67,71,73],[188,400]), #59,62,66,71 merge lane
        ([66,67,71,73,76,82,88,92,95],[310,450]), #71,76,95 merge lane
        ([187,191,193,194],[940,995]) #187,191,194 merge lane
]

# Scenarios for lower merge
scenarios_lower = [
        ([2,7,10,18,25],[1,30]), #7,25 merge vehicles
        ([64,68,70,74,77],[198,230]), #70 merge lane
        ([105,106,108,110],[382,430]), # 110 merge lane
        ([176,177,178],[827,870]) #177 merge lane
]

# Run filtering and store resulting models to jld files
s = scenarios_lower
f = FilteringEnvironment(mergeenv=MergingEnvironmentLower())
for i in 1:length(s)
        veh_id_list = s[i][1]
        ts = s[i][2][1]
        te = s[i][2][2]
        m,p,md = obtain_driver_models(f,veh_id_list=veh_id_list,num_p=50,ts=ts,te=te)
        JLD.save("media/lower_$i.jld","m",m,"p",p,"md",md)
end

print("Get the metrics for particle filering\n")
pos_pf,vel_pf,collisions_pf = extract_metrics(f,ts=ts,id_list=veh_id_list,
#filename="media/pf_ts_88.mp4"
)

print("Get the metrics for c-idm\n")
pos_cidm,vel_cidm,collisions_cidm = extract_metrics(f,ts=ts,id_list=veh_id_list,
modelmaker=make_cidm_models,
filename="media/cidm_ts_198.mp4"
)

print("Get the metrics for idm\n")
pos_idm,vel_idm,collisions_idm = extract_metrics(f,ts=ts,id_list=veh_id_list,
modelmaker=make_IDM_models,
filename="media/idm_ts_88.mp4"
)

make_plots=false
if(make_plots)
    # Make rmse_pos plot
    p_pos_pf = pgfplot_vector(pos_pf,leg="pf");
    p_pos_cidm = pgfplot_vector(pos_cidm,leg="cidm");
    p_pos_idm = pgfplot_vector(pos_idm,leg="idm");
    ax_rmse_pos = PGFPlots.Axis([p_pos_pf,p_pos_cidm,p_pos_idm],
            xlabel="timestep",ylabel="rmse pos",title="rmse pos");
    PGFPlots.save("media/rmse_pos.pdf",ax_rmse_pos);

    # Make collisions plot
    coll_pf = pgfplot_vector(collisions_pf,leg="pf");
    coll_idm = pgfplot_vector(collisions_cidm,leg="cidm");
    coll_cidm = pgfplot_vector(collisions_idm,leg="idm");
    ax_coll = PGFPlots.Axis([coll_pf,coll_cidm,coll_idm],
            xlabel="timestep",ylabel="iscolliding",
            title="Collision assessment");
    PGFPlots.save("media/coll.pdf",ax_coll)
end

#***********Train upper merge, test lower merge***********
# Get the final particle set from upper merge i.e a matrix with particles in columns
# Select the mean, and two extremes say min norm and max norm
# On the bottom merge, generate position, velocity, collision traces. And videos as well
# And maybe also successful merges