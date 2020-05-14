"""
lmidm.jl

Run functions provided in this file by navigating in the REPL to the scripts folder
That is where the data files that these functions read and write to are kept

- Provides functions to do non-linear least squares fit for IDM
- The actual lmfit is done by the notebook in AutomotiveInteraction/scripts/pythonscripts
"""

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
- Writes the features and accleration trace to .txt files

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
function scenelist2idmfeatures(f,scene_list;id_list=[],
    scenario_name="upper",scenario_number=4)
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

        dirname = "$(scenario_name)_$(scenario_number)"
        
        writedlm("lmfit/$(dirname)/$(vehid)_idmfeats.txt",idm_feats)
        writedlm("lmfit/$(dirname)/$(vehid)_trueacc.txt",acc_trace)

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

"""
function make_lmfit_idm_models(f::FilteringEnvironment,scene)

- Read lmfit params stored in txt file and create associated driver model

# Examples
```julia
cd("scripts")
include("helpers.jl")
f = FilteringEnvironment()
s = scenarios_upper
s_num = 4
ts = s[s_num][2][1]
id_list = s[s_num][1]
scene_real = deepcopy(f.traj[ts])
if !isempty(id_list) keep_vehicle_subset!(scene_real,id_list) end
models = make_lmfit_idm_models(f,scene_real,scenario_name="upper",scenario_number=s_num)
```
"""
function make_lmfit_idm_models(f::FilteringEnvironment,scene;
    scenario_name="upper",scenario_number=1)
    print("Assigning models based on lmfit parameters\n")
    models = Dict{Int64,DriverModel}()

    # Loop over list of vehicles and assign model params based on lmfit values
    for veh in scene
        veh_id = veh.id
        lmfit_param_filename = "lmfit/$(scenario_name)_$(scenario_number)/$(veh_id)_lmfit_params.txt"
        if isfile(lmfit_param_filename)
            lmparams = readdlm(lmfit_param_filename)
            v_des = lmparams[1]
            T = lmparams[2]
            dmin = lmparams[3]
            models[veh.id] = IntelligentDriverModel(v_des=v_des,T=T,s_min=dmin)
        else
            # Default to Jeremy's parameters
            print("Could not find lmfit param file for $(veh_id). Use Jeremy params\n")
            models[veh.id] = IntelligentDriverModel(v_des=17.837,s_min=5.249,
                                T=0.918,a_max=0.758,d_cmf=3.811)
        end
    end
    return models
end

"""
- Similar function to `metrics_from_jld` but for lmidm based models
- `metrics_from_jld` was for particle filtering based models
- The jld file is just needed to get scenario information such as id_list,ts,te

# Arguments
- f::FilteringEnvironment

# Used by
- multiscenarios_lmidm

# Example
```julia
metrics_from_jld_lmidm(f,scenario_name="upper",scenario_number=1)
```
"""
function metrics_from_jld_lmidm(f::FilteringEnvironment;scenario_name="upper",scenario_number=1)
    filename = "media/$(scenario_name)_$(scenario_number).jld"
    print("lmidm metrics: extract veh_id_list and ts, te from $(filename) \n")
    # Load scenario information from jld file
    id_list,ts,te = JLD.load(filename,"veh_id_list","ts","te")
    
    scene_real = deepcopy(f.traj[ts])
    if !isempty(id_list) keep_vehicle_subset!(scene_real,id_list) end

    models = make_lmfit_idm_models(f,scene_real,
    scenario_name=scenario_name,scenario_number=scenario_number)

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
- Run multiple scenarios for the lmidm based models

# Uses
- `metrics_from_jld_lmidm`

# Example
```julia
cd("scripts")
rmse_pos_mat_lmidm, rmse_vel_mat_lmidm, coll_mat_lmidm = multiscenarios_lmidm(mergetype="upper")
```
"""
function multiscenarios_lmidm(;mergetype="upper")
    if mergetype=="upper"
        print("Upper merge has been selected\n")
        f = FilteringEnvironment()
        num_scenarios = 10
        name = "upper"
    elseif mergetype == "lower"
        print("Lower merge has been selected\n")
        f=FilteringEnvironment(mergeenv=MergingEnvironmentLower())
        num_scenarios = 6
        name = "lower"
    end
    print("multiscenarios_lmidm: num_scenarios=$(num_scenarios),name=$(name)\n")

    rmse_pos_scenariowise = []
    rmse_vel_scenariowise = []
    coll_scenariowise = []

    for i in 1:num_scenarios
        print("scenario number = $i\n")
        rmse_pos,rmse_vel,coll_array = metrics_from_jld_lmidm(f,
            scenario_name=name,scenario_number=i)
        push!(rmse_pos_scenariowise,rmse_pos)
        push!(rmse_vel_scenariowise,rmse_vel)
        push!(coll_scenariowise,coll_array)
    end

    rmse_pos_matrix = truncate_vecs(rmse_pos_scenariowise)
    rmse_vel_matrix = truncate_vecs(rmse_vel_scenariowise)
    coll_matrix = truncate_vecs(coll_scenariowise)
    return rmse_pos_matrix,rmse_vel_matrix,coll_matrix
end
