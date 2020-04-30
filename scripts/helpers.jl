"""
helpers.jl
This file provides helper functions to experiments.jl, the experiment script
"""

using Distributions # provides `mean`: to compute mean of particle dist over cars
using JLD # to save models
using AutomotiveInteraction

"""
function extract_metrics(f;ts=1,id_list=[],dur=10.,modelmaker=nothing,filename=[])

- Perform driving simulation starting from `ts` for `dur` duration.

# Arguments
- `ts`: start frame
- `id_list`: list of vehicles 
- `dur`: duration
- `modelmaker`: function that makes the models
- `filename`: provide optionally to make comparison video

# Example
```julia
f = FilteringEnvironment()
scenes,collisions = extract_metrics(f,id_list=[4,6,8,13,19,28,29],
modelmaker=make_cidm_models,filename="media/cidm_exp.mp4")
```
"""
function extract_metrics(f::FilteringEnvironment;ts=1,id_list=[],dur=10.,
modelmaker=nothing,filename=[])
    print("Run experiment from scripts being called \n")
    scene_real = deepcopy(f.traj[ts])
    if !isempty(id_list) keep_vehicle_subset!(scene_real,id_list) end

    # If modelmaker argument is provided, use that function to create models
    # Otherwise, obtaine driver models using particle filtering
    models = Dict{Int64,DriverModel}()
    if isnothing(modelmaker)
        print("Let's run particle filtering to create driver models\n")
        models,p,mean_dist_mat = obtain_driver_models(f,veh_id_list=id_list,num_p=50,ts=ts,te=ts+50)
        
        # Particle filtering progress plot
        progressplot=false
        if progressplot
            avg_over_cars = mean(mean_dist_mat,dims=2)
            avg_over_cars = reshape(avg_over_cars,length(avg_over_cars),) # for PGFPlot
            print("Making filtering progress plot\n")
            p = PGFPlots.Plots.Linear(collect(1:length(avg_over_cars)),avg_over_cars)
            ax = PGFPlots.Axis([p],xlabel = "iternum",ylabel = "avg distance",
            title = "Filtering progress")
            PGFPlots.save("media/p_prog.pdf",ax)
        end
        # Save models
        JLD.save("filtered_models.jld","models",models)
    else
        print("Lets use idm or c_idm to create driver models\n")
        models = modelmaker(f,scene_real)
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

# We need a function to read in the jld file and extract metrics based on it
"""
function metrics_from_jld(f::FilteringEnvironment;filename="1.jld")
- Particle filtering based driver models stored in .jld file
- Assumes the .jld file contains driver models, veh id list, ts and te

# Example
```julia
f = FilteringEnvironment(mergeenv=MergingEnvironmentLower())
rmse_pos,rmse_vel,c_array = metrics_from_jld(f,filename="media/lower_4.jld")
```
"""
function metrics_from_jld(f::FilteringEnvironment;filename="1.jld")
    print("Metrics extraction from jld file $(filename) \n")
    # Load scenario information from jld file
    models,id_list,ts,te = JLD.load(filename,"m","veh_id_list","ts","te")
    
    scene_real = deepcopy(f.traj[ts])
    if !isempty(id_list) keep_vehicle_subset!(scene_real,id_list) end

    nticks = te-ts+1
    scene_list = simulate(scene_real,f.roadway,models,nticks,f.timestep)

    c_array = test_collision(scene_list,id_list)

    truth_list = f.traj[ts:ts+nticks]

    # rmse dict with vehicle wise rmse values
    rmse_pos_dict,rmse_vel_dict = compute_rmse(truth_list,scene_list,id_list=id_list)
    
    # Average over the vehicles
    rmse_pos = rmse_dict2mean(rmse_pos_dict)
    rmse_vel = rmse_dict2mean(rmse_vel_dict)

    return rmse_pos,rmse_vel,c_array
end

"""
- Similar function to `metrics_from_jld` but for idm based models
- `metrics_from_jld` was for particle filtering based models
- The jld file is just needed to get scenario information such as id_list,ts,te

# Arguments
- f::FilteringEnvironment
- filename: jld file
- modelmaker: function that assigns idm based models to vehicles in the scene

# Used by
- multiscenarios_idm

# Example
```julia
metrics_from_jld_idmbased(f,filename="media/lower_1.jld",modelmaker=make_IDM_models)
```
"""
function metrics_from_jld_idmbased(f::FilteringEnvironment;filename="1.jld",
modelmaker=make_cidm_models)
    print("idm based: extract veh_id_list and ts, te from $(filename) \n")
    # Load scenario information from jld file
    id_list,ts,te = JLD.load(filename,"veh_id_list","ts","te")
    
    scene_real = deepcopy(f.traj[ts])
    if !isempty(id_list) keep_vehicle_subset!(scene_real,id_list) end

    models = modelmaker(f,scene_real)

    nticks = te-ts+1
    scene_list = simulate(scene_real,f.roadway,models,nticks,f.timestep)

    c_array = test_collision(scene_list,id_list)

    truth_list = f.traj[ts:ts+nticks]

    # rmse dict with vehicle wise rmse values
    rmse_pos_dict,rmse_vel_dict = compute_rmse(truth_list,scene_list,id_list=id_list)
    
    # Average over the vehicles
    rmse_pos = rmse_dict2mean(rmse_pos_dict)
    rmse_vel = rmse_dict2mean(rmse_vel_dict)

    return rmse_pos,rmse_vel,c_array
end

"""
- Run multiple scenarios for the idm based models

# Example
```julia
rmse_pos_mat_idm, = multiscenarios_idm(mergetype="lower",modelmaker=make_IDM_models)
rmse_pos_mat_cidm, = multiscenarios_idm(mergetype="lower",modelmaker=make_cidm_models)
rmse_pos_mat_lmidm, = multiscenarios_idm(mergetype="lower",modelmaker=make_lmidm_models)
rmse_pos_mat_pf, = multiscenarios(mergetype="lower")

rmse_idm = mean(rmse_pos_mat_idm,dims=2)
rmse_cidm = mean(rmse_pos_mat_cidm,dims=2)
rmse_lmidm = mean(rmse_pos_mat_lmidm,dims=2)
rmse_pf = mean(rmse_pos_mat_pf,dims=2)

pi = pgfplot_vector(rmse_idm,leg="idm");
pc = pgfplot_vector(rmse_cidm,leg="cidm");
pl = pgfplot_vector(rmse_lmidm,leg="lmidm");
ppf = pgfplot_vector(rmse_pf,leg="pf");
ax = PGFPlots.Axis([pi,pc,pl,ppf],xlabel="timestep",ylabel="rmse pos",title="multiscenario");
PGFPlots.save("media/rmse_multi4.pdf",ax)
```
"""
function multiscenarios_idm(;mergetype="upper",modelmaker=make_cidm_models)
    if mergetype=="upper"
        print("Upper merge has been selected\n")
        f = FilteringEnvironment()
        num_scenarios = 5
        prefix = "upper"
    elseif mergetype == "lower"
        print("Lower merge has been selected\n")
        f=FilteringEnvironment(mergeenv=MergingEnvironmentLower())
        num_scenarios = 4
        prefix = "lower"
    end
    rmse_pos_scenariowise = []
    rmse_vel_scenariowise = []
    coll_scenariowise = []

    for i in 1:num_scenarios
        print("scenario number = $i\n")
        rmse_pos,rmse_vel,coll_array = metrics_from_jld_idmbased(f,
            filename="media/$(prefix)_$i.jld",modelmaker=modelmaker)
        push!(rmse_pos_scenariowise,rmse_pos)
        push!(rmse_vel_scenariowise,rmse_vel)
        push!(coll_scenariowise,coll_array)
    end

    rmse_pos_matrix = truncate_vecs(rmse_pos_scenariowise)
    rmse_vel_matrix = truncate_vecs(rmse_vel_scenariowise)
    coll_matrix = truncate_vecs(coll_scenariowise)
    return rmse_pos_matrix,rmse_vel_matrix,coll_matrix
end

"""
- Take multiple scenario jld files that contain models obtained using filtering
- Generate simulation trajectories and capture rmse metrics
- Uses `pad_zeros` to make sure that rmse lengths match since scenario lengths not same

# Example
```julia
a1,a2,a3 = multiscenarios(mergetype="lower")
rmse_pos = mean(a1,dims=2)
```
"""
function multiscenarios(;mergetype = "upper")
    if mergetype=="upper"
        f = FilteringEnvironment()
    elseif mergetype == "lower"
        f=FilteringEnvironment(mergeenv=MergingEnvironmentLower())
    end
    num_scenarios = 4
    rmse_pos_scenariowise = []
    rmse_vel_scenariowise = []
    coll_scenariowise = []

    for i in 1:num_scenarios
        rmse_pos,rmse_vel,coll_array = metrics_from_jld(f,filename="media/lower_$i.jld")
        push!(rmse_pos_scenariowise,rmse_pos)
        push!(rmse_vel_scenariowise,rmse_vel)
        push!(coll_scenariowise,coll_array)
    end

    rmse_pos_matrix = truncate_vecs(rmse_pos_scenariowise)
    rmse_vel_matrix = truncate_vecs(rmse_vel_scenariowise)
    coll_matrix = truncate_vecs(coll_scenariowise)
    return rmse_pos_matrix,rmse_vel_matrix,coll_matrix
end
