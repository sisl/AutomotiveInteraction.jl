# This file is used for performing experiments
# Run by changing path in REPL to scripts folder
# Then `include("experiments.jl")`

using AutomotiveSimulator
using AutomotiveInteraction
using PGFPlots

"""
function extract_metrics(f;ts=1,id_list=[],dur=10.,modelmaker=nothing,filename=[])

- Perform driving simulation and extract metrics

# Arguments
- ts: start frame
- id_list: list of vehicles 
- dur: duration
- modelmaker: function that makes the models
- filename: provide optionally to make comparison video

# Example
```julia
f = FilteringEnvironment()
scenes,collisions = extract_metrics(f,id_list=[4,6,8,13,19,28,29],
modelmaker=make_cidm_models,filename="media/cidm_exp.mp4")
```
"""
function extract_metrics(f;ts=1,id_list=[],dur=10.,modelmaker=nothing,filename=[])
    print("Run experiment from scripts being called \n")
    scene_real = deepcopy(f.traj[ts])
    if !isempty(id_list) keep_vehicle_subset!(scene_real,id_list) end

    # If modelmaker argument is provided, use that function to create models
    # Otherwise, obtaine driver models using particle filtering
    models = Dict{Int64,DriverModel}()
    if isnothing(modelmaker)
        print("Let's run particle filtering to create driver models")
        models, = obtain_driver_models(f,id_list,30,1,30)
    else
        print("Lets use idm or c_idm to create driver models\n")
        models = modelmaker(scene_real)
    end

    nticks = Int(ceil(dur/f.timestep))
    scene_list = simulate(scene_real,f.roadway,models,nticks,f.timestep)

    c_array = test_collision(scene_list,id_list)

    truth_list = f.traj[ts:ts+nticks]
    
    # Make a comparison video if filename provided
    if !isempty(filename)
        video_overlay_scenelists(scene_list,truth_list,id_list=id_list,roadway=f.roadway,
            filename=filename)
    end

    # rmse dict with vehicle wise rmse values
    rmse_pos_dict,rmse_vel_dict = compute_rmse(truth_list,scene_list,id_list=id_list)
    
    # Average over the vehicles
    rmse_pos = rmse_dict2mean(rmse_pos_dict)
    rmse_vel = rmse_dict2mean(rmse_vel_dict)

    return rmse_pos,rmse_vel,c_array
end

f = FilteringEnvironment()
veh_id_list = [4,6,8,13,19,28,29]

print("Get the metrics for particle filering\n")
pos_pf,vel_pf,collisions_pf = extract_metrics(f,id_list=veh_id_list)

print("Get the metrics for c-idm\n")
pos_cidm,vel_cidm,collisions_cidm = extract_metrics(f,id_list=veh_id_list,
modelmaker=make_cidm_models)

print("Get the metrics for idm\n")
pos_idm,vel_idm,collisions_idm = extract_metrics(f,id_list=veh_id_list,
modelmaker=make_IDM_models)

make_plots=false
if(make_plots)
    # Make rmse_pos plot
    p_pos_pf = PGFPlots.Plots.Linear(collect(1:length(pos_pf)),pos_cidm,legendentry="pf");
    p_pos_cidm = PGFPlots.Plots.Linear(collect(1:length(pos_cidm)),pos_cidm,legendentry="cidm");
    p_pos_idm = PGFPlots.Plots.Linear(collect(1:length(pos_idm)),pos_idm,legendentry="idm");
    ax_rmse_pos = PGFPlots.Axis([p_pos_pf,p_pos_cidm,p_pos_idm],
            xlabel="timestep",ylabel="rmse pos",title="rmse pos: c-IDM vs IDM");
    PGFPlots.save("media/rmse_pos_cidmvsidm.pdf",ax_rmse_pos);

    # Make collisions plot
    coll_pf = PGFPlots.Plots.Linear(collect(1:length(collisions_pf)),collisions_pf,legendentry="pf")
    coll_idm = PGFPlots.Plots.Linear(collect(1:length(collisions_idm)),collisions_idm,legendentry="idm");
    coll_cidm = PGFPlots.Plots.Linear(collect(1:length(collisions_cidm)),collisions_cidm,legendentry="cidm");
    ax_coll = PGFPlots.Axis([coll_pf,coll_cidm,coll_idm],
            xlabel="timestep",ylabel="iscolliding",title="Collision assessment: c-IDM vs IDM");
    PGFPlots.save("media/coll_cidmvsidm.pdf",ax_coll)
end