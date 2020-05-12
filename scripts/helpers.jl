"""
helpers.jl
This file provides helper functions to experiments.jl, the experiment script
Most functions have the script within the doc string as example
"""

using Distributions # provides `mean`: to compute mean of particle dist over cars
using JLD # to save models
using AutomotiveInteraction
using DelimitedFiles # to write to txt file that will be read in by python

# We need a function to read in the jld file and extract metrics based on it
"""
function metrics_from_jld(f::FilteringEnvironment;filename="1.jld")
- Particle filtering based driver models stored in .jld file
- Assumes the .jld file contains driver models, veh id list, ts and te
- Extracts rmse and collision metrics by running simulations

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

# Uses
- metrics_from_jld_idmbased

# Example
```julia
rmse_pos_mat_idm, = multiscenarios_idm(mergetype="lower",modelmaker=make_IDM_models)
rmse_pos_mat_cidm, = multiscenarios_idm(mergetype="lower",modelmaker=make_cidm_models)
rmse_pos_mat_lmidm, = multiscenarios_idm(mergetype="lower",modelmaker=make_lmidm_models)
rmse_pos_mat_pf, = multiscenarios_pf(mergetype="lower")

rmse_idm = mean(rmse_pos_mat_idm,dims=2)
rmse_cidm = mean(rmse_pos_mat_cidm,dims=2)
rmse_lmidm = mean(rmse_pos_mat_lmidm,dims=2)
rmse_pf = mean(rmse_pos_mat_pf,dims=2)

pi = pgfplot_vector(rmse_idm,leg="idm");
pc = pgfplot_vector(rmse_cidm,leg="cidm");
pl = pgfplot_vector(rmse_lmidm,leg="lmidm");
ppf = pgfplot_vector(rmse_pf,leg="pf");
ax = PGFPlots.Axis([pi,pc,pl,ppf],xlabel="timestep",ylabel="rmse pos",title="multiscenario");
PGFPlots.save("media/rmse_multi4.svg",ax)
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
- Uses `truncate_vecs` to make sure that rmse lengths match since scenario lengths not same

# Uses
- metrics_from_jld to extract rmse and collision by running simulations

# Example
```julia
a1,a2,a3 = multiscenarios_pf(mergetype="lower")
rmse_pos = mean(a1,dims=2)
```
"""
function multiscenarios_pf(;mergetype = "upper")
    if mergetype=="upper"
        f = FilteringEnvironment()
        num_scenarios = 5
        prefix = "upper"
    elseif mergetype == "lower"
        f=FilteringEnvironment(mergeenv=MergingEnvironmentLower())
        num_scenarios = 4
        prefix = "lower"
    end

    rmse_pos_scenariowise = []
    rmse_vel_scenariowise = []
    coll_scenariowise = []

    for i in 1:num_scenarios
        rmse_pos,rmse_vel,coll_array = metrics_from_jld(f,
        filename="media/$(prefix)_$i.jld")
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

"""
- Makes a bar plot of the fraction of collision timesteps across all scenarios
- Saves plot to `filename`

# Uses
- `frac_colliding_timesteps`: defined in `utils.jl`

# Arguments
- `coll_mat_list`: Collision matrices i.e. diff scenarios in different columns
- Order is idm,cidm,lmidm,pf

# Example
```julia
rmse_pos_mat_idm,rmse_vel_mat_idm,coll_mat_idm = 
        multiscenarios_idm(mergetype="upper",modelmaker=make_IDM_models)
rmse_pos_mat_cidm, rmse_vel_mat_cidm, coll_mat_cidm = 
        multiscenarios_idm(mergetype="upper",modelmaker=make_cidm_models)
rmse_pos_mat_lmidm, rmse_vel_mat_lmidm, coll_mat_lmidm = 
        multiscenarios_idm(mergetype="upper",modelmaker=make_lmidm_models)
rmse_pos_mat_pf, rmse_vel_mat_pf, coll_mat_pf = multiscenarios_pf(mergetype="upper")

coll_mat_list = [coll_mat_idm,coll_mat_cidm,coll_mat_lmidm,coll_mat_pf]
coll_barchart(coll_mat_list,filename = "media/coll_barchart_upper.svg")
```
"""
function coll_barchart(coll_mat_list;filename="media/test_bar.pdf")
        collfrac_idm = frac_colliding_timesteps(coll_mat_list[1])
        collfrac_cidm = frac_colliding_timesteps(coll_mat_list[2])
        collfrac_lmidm = frac_colliding_timesteps(coll_mat_list[3])
        collfrac_pf = frac_colliding_timesteps(coll_mat_list[4])

        coll_frac_list = [collfrac_idm,collfrac_cidm,collfrac_lmidm,collfrac_pf]
        ax = PGFPlots.Axis(PGFPlots.Plots.BarChart(["idm","cidm","lmidm","pf"],
                coll_frac_list),xlabel="models",ylabel="Collision Fraction")

        PGFPlots.save(filename,ax)
        print("function coll_barchart says: saved $(filename)\n")
        return nothing
end

"""
function rmse_plots_modelscompare(rmse_list;filename="media/test_rmse.pdf")
- Make rmse plots comparing different models after having found the mean rmse over scenarios

# Arguments
- `rmse_mat_list`: List with idm,cidm,lmidm,pf resulting matrices
- Each element of list is a matrix. Each column is a different scenario. Each row is a timestep

# Example
```julia
rmse_pos_mat_idm,rmse_vel_mat_idm,coll_mat_idm = 
        multiscenarios_idm(mergetype="upper",modelmaker=make_IDM_models)
rmse_pos_mat_cidm, rmse_vel_mat_cidm, coll_mat_cidm = 
        multiscenarios_idm(mergetype="upper",modelmaker=make_cidm_models)
rmse_pos_mat_lmidm, rmse_vel_mat_lmidm, coll_mat_lmidm = 
        multiscenarios_idm(mergetype="upper",modelmaker=make_lmidm_models)
rmse_pos_mat_pf, rmse_vel_mat_pf, coll_mat_pf = multiscenarios_pf(mergetype="upper")

rmse_list = [rmse_pos_mat_idm,rmse_pos_mat_cidm,rmse_pos_mat_lmidm,rmse_pos_mat_pf]
rmse_plots_modelscompare(rmse_list,filename = "media/rmse_pos_upper.svg")
```
"""
function rmse_plots_modelscompare(rmse_mat_list;filename="media/test_rmse.pdf")
        rmse_mat_idm = rmse_mat_list[1]
        rmse_mat_cidm = rmse_mat_list[2]
        rmse_mat_lmidm = rmse_mat_list[3]
        rmse_mat_pf = rmse_mat_list[4]

        rmse_idm = mean(rmse_mat_idm,dims=2)
        rmse_cidm = mean(rmse_mat_cidm,dims=2)
        rmse_lmidm = mean(rmse_mat_lmidm,dims=2)
        rmse_pf = mean(rmse_mat_pf,dims=2)
        
        pidm = pgfplot_vector(rmse_idm,leg="idm");
        pcidm = pgfplot_vector(rmse_cidm,leg="cidm");
        plmidm = pgfplot_vector(rmse_lmidm,leg="lmidm");
        ppf = pgfplot_vector(rmse_pf,leg="pf");
        ax = PGFPlots.Axis([pidm,pcidm,plmidm,ppf],xlabel="timestep",
                ylabel="rmse pos",title="Upper scenarios");
        PGFPlots.save(filename,ax)
        print("function rmse_plots_modelscompare says: saved $(filename)\n")
        return nothing
end

"""
function train_one_test_another(;train_filename="media/lower_3.jld",test_filename="media/upper_2.jld",video_filename="media/train_low_test_up.mp4")
- Take final particle (i.e. mean of particles) obtained for one vehicle in train scenario
- Assign cidm model using that particle to all vehicles in test scenario
- Run a simulation, make a video and return collision assessment

# Example 
```julia
# Run this from scripts folder
train_one_test_another(train_filename="media/lower_3.jld",
test_filename="media/upper_4.jld",video_filename="media/train_test_3_4.mp4")
```
"""
function train_one_test_another(;train_filename="media/lower_3.jld",
        test_filename="media/upper_2.jld",video_filename="media/train_low_test_up.mp4")
        # p_dict is a Dict(veh_id=>final_mean_particle)
        p_dict,train_id_list = JLD.load(train_filename,"p","veh_id_list")
        particle = p_dict[train_id_list[1]] # pick the first veh in train list wlog

        f = FilteringEnvironment() # Will need to have lower as mergeenv when testing on lower
        test_id_list,test_ts,test_te = JLD.load(test_filename,"veh_id_list","ts","te")
        models = Dict{Int64,DriverModel}()
        for id in test_id_list
                models[id] = cidm_from_particle(f,particle)
        end
        nticks = test_te-test_ts+1
        scene_real = deepcopy(f.traj[test_ts])
        if !isempty(test_id_list) keep_vehicle_subset!(scene_real,test_id_list) end
        scene_list = simulate(scene_real,f.roadway,models,nticks,f.timestep)

        c_array = test_collision(scene_list,test_id_list)
        scenelist2video(f,scene_list,filename=video_filename)
        return c_array
end

"""
- Compare generated vs true scenario for how real the driving is
- Make bar plot with stopped vehicle fraction and hard deceleration timefraction

# Example
```julia
f = FilteringEnvironment()
compare_realism(f,data_filename = "media/upper_4.jld",plot_filename="media/realism_4.svg")
```
"""
function compare_realism(f::FilteringEnvironment;data_filename="media/upper_1.jld",
    plot_filename="media/realism.svg")
    models,id_list,ts,te = JLD.load(data_filename,"m","veh_id_list","ts","te")

    scene_list_true = replay_scenelist(f,id_list=id_list,ts=ts,te=te)
    stopfrac_true,brakefrac_true = reality_metrics(f,scene_list_true,id_list=id_list)

    scene_real = deepcopy(f.traj[ts])
    if !isempty(id_list) keep_vehicle_subset!(scene_real,id_list) end
    nticks = te-ts+1
    scene_list_generated = simulate(scene_real,f.roadway,models,nticks,f.timestep)
    stopfrac_gen,brakefrac_gen = reality_metrics(f,scene_list_generated,id_list=id_list)

    #TODO: Make bars stacked together
    p = PGFPlots.Plots.BarChart(["stop true","stop gen","brake true","brake gen"],
            [stopfrac_true,stopfrac_gen,brakefrac_true,brakefrac_gen])
    a = PGFPlots.Axis(p,xlabel="Metric",ylabel="Reality value")
    PGFPlots.save(plot_filename,a)
    print("Made plot called $plot_filename\n")
    return nothing
end


#****************Prepare data for idm lm fit*******************
# Goal is to have the data in the form that we can directly use Jeremy's code
"""
# Example
```julia
f = FilteringEnvironment()
scene = f.traj[2]
egoid = 28
extract_idm_features(f,scene,egoid)
```
"""
function extract_idm_features(f::FilteringEnvironment,scene::Scene{Entity{S, D, I}},
    egoid::I) where {S, D, I}
    
    ego = get_by_id(scene, egoid)

    fore = find_neighbor(scene, f.roadway, ego, targetpoint_ego=VehicleTargetPointFront(), targetpoint_neighbor=VehicleTargetPointRear())

    v_ego = vel(ego)
    v_oth = NaN
    headway = NaN

    if fore.ind != nothing
        v_oth = vel(scene[fore.ind].state)
        headway = fore.Δs
    end

    return v_ego,v_oth-v_ego,headway
end

"""
function scenelist2idmfeatures(f,scene_list;id_list=[])
- Loop over a list of scenes and extract idm features
- Returns dict with vehicle id as key and array of idm features as value
- Array has each row a different timestep. Col 1 is v_ego, col2 is delta_v, 3 is headway
- Col4 is the true acceleration

# Example
```julia
using AutomotiveInteraction
using AutomotiveSimulator
using JLD
cd("scripts")
include("helpers.jl")
f = FilteringEnvironment();
filename = "media/upper_4.jld";
models,id_list,ts,te = JLD.load(filename,"m","veh_id_list","ts","te");
scene_list_true = replay_scenelist(f,id_list=id_list,ts=ts,te=te);
feat_dict = scenelist2idmfeatures(f,scene_list_true,id_list=id_list);
```
"""
function scenelist2idmfeatures(f,scene_list;id_list=[])
    numscenes=length(scene_list)
    idmfeat_dict = Dict()
    for vehid in id_list
        data = fill(0.,numscenes,4) # Because 3 idm features and 1 true accl
        acc_trace = acc(f.roadway,scene_list,vehid)
        acc_trace[1] = -1. # Just to get rid of missing. This will be thrown away later
        for (i,scene) in enumerate(scene_list)
            scene = scene_list[i]
            data[i,1],data[i,2],data[i,3] = extract_idm_features(f,scene,vehid)
            data[i,4] = acc_trace[i] # Caution: 1st elem will be missing
        end

        temp = removeNaNrows(data)
        idm_feats = temp[2:end,1:3]
        acc_trace = temp[2:end,4]

        writedlm("pythonscripts/$(vehid)_idmfeats.txt",idm_feats)
        writedlm("pythonscripts/$(vehid)_trueacc.txt",acc_trace)

        idmfeat_dict[vehid] = temp
    end
    return idmfeat_dict
end

"""
function removeNaNrows(a)
- Remove rows with any NaNs in them from the input matrix `a`

# Example
```julia
a = rand(6,3)
a[2,2] = NaN, a[5,3]=NaN
b = removeNaNrows(a)
```
"""
function removeNaNrows(a)
    nan_row_idx = []
    for i in 1:size(a,1)
        curr_row = a[i,:]
        
        if sum(isnan.(curr_row))>0
            push!(nan_row_idx,i)
        end
    end

    b = a[setdiff(1:end,nan_row_idx),:]
    return b
end
