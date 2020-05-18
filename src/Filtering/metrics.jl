"""
metrics.jl

Run by navigating REPL to scripts folder

- Functions to scale up metrics extraction across scenarios and models
- utils.jl provides basic computation such as compute_rmse but this guy scales that up

# List of Functions
- `vel_distribution`
- `metrics_from_jld_idmbased`
- `multiscenarios_idm`
- `metrics_from_jld`
- `multiscenarios_pf`
- `coll_barchart`
- `rmse_plots_modelscompare`
"""

"""
- Make a histogram over the velocities

# Example
```julia
# Extract ground truth velocity distribution for scenario 1 upper
f = FilteringEnvironment()
id_list,ts,te=JLD.load("media/upper_1.jld","veh_id_list","ts","te")
scenelist = replay_scenelist(f,id_list=id_list,ts=ts,te=te)
veh_hist_true = vel_distribution(scenelist);

rmse_pos_mat_idm,rmse_vel_mat_idm,coll_mat_idm,vhist_scenariowise_idm = multiscenarios_idm(mergetype="lower",modelmaker=make_IDM_models)
rmse_pos_mat_pf,rmse_vel_mat_pf,coll_mat_pf,vhist_scenariowise_pf = multiscenarios_pf(mergetype="upper");
rmse_pos_mat_lmidm, rmse_vel_mat_lmidm, coll_mat_lmidm,vhist_scenariowise_lmidm = multiscenarios_lmidm(mergetype="upper")

v_hist_idm = vhist_scenariowise_idm[1]
v_hist_pf = vhist_scenariowise_pf[1]
v_hist_lmidm = vhist_scenariowise_lmidm[1]
a = PGFPlots.Axis([veh_hist_true,v_hist_idm,v_hist_lmidm,v_hist_pf])
```
"""
function vel_distribution(list_of_scenes;id_list=[])
    vel_array = []
    for scene in list_of_scenes
        if !isempty(id_list) keep_vehicle_subset!(scene,id_list) end
        for veh in scene
            vel = veh.state.v
            push!(vel_array,vel)
        end
    end
    h = PGFPlots.Plots.Histogram(Float64.(vel_array),bins=10)
end

"""
- Similar function to `metrics_from_jld` but for idm based models
- `metrics_from_jld` was for particle filtering based models
- The jld file is just needed to get scenario information such as id_list,ts,te

# Arguments
- f::FilteringEnvironment
- filename: jld file
- modelmaker: function that assigns idm based models to vehicles in the scene

# Returns
- `rmse_pos`: array with average over cars rmse_pos at each timestep
- `rmse_vel`:
- `coll_array`: binary array of length number of timesteps. 1 if any collision at that timestep
- `vel_hist`: PGFPlots.Plots.Histogram with velocity distribution over all vehicles over all timesteps

# Used by
- multiscenarios_idm

# Example
```julia
metrics_from_jld_idmbased(f,filename="media/lower_1.jld",modelmaker=make_IDM_models)
```
"""
function metrics_from_jld_idmbased(f::FilteringEnvironment;filename="1.jld",
modelmaker=make_cidm_models)
    #print("idm based: extract veh_id_list and ts, te from $(filename) \n")
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

    vel_hist = vel_distribution(scene_list)

    return rmse_pos,rmse_vel,c_array,vel_hist
end

"""
- Run multiple scenarios for the idm based models
- Scenario number and jld file path are hard coded

# Uses
- `metrics_from_jld_idmbased`

# Example
```julia
rmse_pos_mat_idm,rmse_vel_mat_idm,coll_mat_idm,vhist_scenariowise_idm = multiscenarios_idm(mergetype="lower",modelmaker=make_IDM_models)

PGFPlots.Axis(vhist_scenariowise[1],xlabel="",ylabel="",title="")
```
"""
function multiscenarios_idm(;mergetype="upper",modelmaker=make_cidm_models)
    if mergetype=="upper"
        print("Upper merge has been selected\n")
        f = FilteringEnvironment()
        num_scenarios = 10
        prefix = "upper"
    elseif mergetype == "lower"
        print("Lower merge has been selected\n")
        f=FilteringEnvironment(mergeenv=MergingEnvironmentLower())
        num_scenarios = 4
        prefix = "lower"
    end
    print("multiscenarios_idm: num_scenarios = $(num_scenarios),name=$(prefix)\n")

    rmse_pos_scenariowise = []
    rmse_vel_scenariowise = []
    coll_scenariowise = []
    vhist_scenariowise = []

    for i in 1:num_scenarios
        print("scenario number = $i\n")
        rmse_pos,rmse_vel,coll_array,vhist = metrics_from_jld_idmbased(f,
            filename="media/$(prefix)_$i.jld",modelmaker=modelmaker)
        push!(rmse_pos_scenariowise,rmse_pos)
        push!(rmse_vel_scenariowise,rmse_vel)
        push!(coll_scenariowise,coll_array)
        push!(vhist_scenariowise,vhist)
    end

    rmse_pos_matrix = truncate_vecs(rmse_pos_scenariowise)
    rmse_vel_matrix = truncate_vecs(rmse_vel_scenariowise)
    coll_matrix = truncate_vecs(coll_scenariowise)
    return rmse_pos_matrix,rmse_vel_matrix,coll_matrix,vhist_scenariowise
end

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
    #print("Metrics extraction from jld file $(filename) \n")
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

    vel_hist = vel_distribution(scene_list)

    return rmse_pos,rmse_vel,c_array,vel_hist
end

"""
- Take multiple scenario jld files that contain models obtained using filtering
- Generate simulation trajectories and capture rmse metrics
- Uses `truncate_vecs` to make sure that rmse lengths match since scenario lengths not same

# Uses
- `metrics_from_jld` to extract rmse, collision, and vel histogram by running simulations

# Example
```julia
rmse_pos_mat_pf,rmse_vel_mat_pf,coll_mat_pf,vhist_scenariowise_pf = multiscenarios_pf(mergetype="upper");
```
"""
function multiscenarios_pf(;mergetype = "upper")
    if mergetype=="upper"
        f = FilteringEnvironment()
        num_scenarios = 10
        prefix = "upper"
    elseif mergetype == "lower"
        f=FilteringEnvironment(mergeenv=MergingEnvironmentLower())
        num_scenarios = 4
        prefix = "lower"
    end
    print("multiscenarios_pf: num_scenarios = $(num_scenarios),name=$(prefix)\n")
    
    rmse_pos_scenariowise = []
    rmse_vel_scenariowise = []
    coll_scenariowise = []
    vhist_scenariowise = []

    for i in 1:num_scenarios
        print("i=$i\n")
        print("filename = media/$(prefix)_$i.jld\n")
        rmse_pos,rmse_vel,coll_array,vhist = metrics_from_jld(f,
        filename="media/$(prefix)_$i.jld")
        push!(rmse_pos_scenariowise,rmse_pos)
        push!(rmse_vel_scenariowise,rmse_vel)
        push!(coll_scenariowise,coll_array)
        push!(vhist_scenariowise,vhist)
    end

    rmse_pos_matrix = truncate_vecs(rmse_pos_scenariowise)
    rmse_vel_matrix = truncate_vecs(rmse_vel_scenariowise)
    coll_matrix = truncate_vecs(coll_scenariowise)
    return rmse_pos_matrix,rmse_vel_matrix,coll_matrix,vhist_scenariowise
end

#***************collision and rmse compare***********************
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
cd("scripts")
rmse_pos_mat_idm,rmse_vel_mat_idm,coll_mat_idm = multiscenarios_idm(mergetype="upper",modelmaker=make_IDM_models);
rmse_pos_mat_cidm, rmse_vel_mat_cidm, coll_mat_cidm = multiscenarios_idm(mergetype="upper",modelmaker=make_cidm_models);
rmse_pos_mat_lmidm, rmse_vel_mat_lmidm, coll_mat_lmidm = multiscenarios_lmidm(mergetype="upper");
rmse_pos_mat_pf, rmse_vel_mat_pf, coll_mat_pf = multiscenarios_pf(mergetype="upper");

coll_mat_list = [coll_mat_idm,coll_mat_cidm,coll_mat_lmidm,coll_mat_pf];
coll_barchart(coll_mat_list,filename = "media/coll_barchart_new_cidm.svg");
```
"""
function coll_barchart(coll_mat_list;filename="media/test_bar.svg")
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
cd("scripts")
rmse_pos_mat_idm,rmse_vel_mat_idm,coll_mat_idm = multiscenarios_idm(mergetype="upper",modelmaker=make_IDM_models);
rmse_pos_mat_cidm, rmse_vel_mat_cidm, coll_mat_cidm = multiscenarios_idm(mergetype="upper",modelmaker=make_cidm_models);
rmse_pos_mat_lmidm, rmse_vel_mat_lmidm, coll_mat_lmidm = multiscenarios_lmidm(mergetype="upper");
rmse_pos_mat_pf, rmse_vel_mat_pf, coll_mat_pf = multiscenarios_pf(mergetype="upper");

rmse_list = [rmse_pos_mat_idm,rmse_pos_mat_cidm,rmse_pos_mat_lmidm,rmse_pos_mat_pf];
rmse_plots_modelscompare(rmse_list,filename = "media/rmse_pos_new.svg");
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
