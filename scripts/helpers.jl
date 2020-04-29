"""
helpers.jl
This file provides helper functions to experiments.jl, the experiment script
"""

using Distributions # provides `mean`: to compute mean of particle dist over cars
using JLD # to save models

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
function pgfplot_vector(vec;leg = "nolegend")

- Plot a vector using PGFPlots

# Example
```julia
plot_rmse_pf = pgfplot_vector(pos_pf,leg="pf")
```
"""
function pgfplot_vector(vec;leg = "nolegend")
    return PGFPlots.Plots.Linear(collect(1:length(vec)),vec,legendentry=leg)
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

    rmse_pos_matrix = pad_zeros(rmse_pos_scenariowise)
    rmse_vel_matrix = pad_zeros(rmse_vel_scenariowise)
    coll_matrix = pad_zeros(coll_scenariowise)
    return rmse_pos_matrix,rmse_vel_matrix,coll_matrix
end

"""
- We need to append zeros to the shorter rmse vectors to make all lengths equal

# Example
```julia
intset = collect(1:10) #integer set to draw random numbers from
l = [rand(intset,4,1),rand(intset,5,1),rand(intset,6,1),rand(intset,10,1),rand(intset,3,1)]
a = pad_zeros(l)
```
"""
function pad_zeros(list_of_vectors)
    longest_length = -1
    numvec = length(list_of_vectors)
    for i in 1:numvec
        curr_len = length(list_of_vectors[i])
        if curr_len > longest_length
            longest_length = curr_len
        end
    end
    #print("longest length = $(longest_length)\n")

    new_list = []
    
    for j in 1:numvec
        #print("j = $j\n")
        # pad with zeros
        curr_vec = list_of_vectors[j]
        lengthdiff = longest_length-length(curr_vec)
        padded_vec = vcat(curr_vec,fill(0.,lengthdiff,1))
        #print("padded_vec = $(padded_vec)\n")
        push!(new_list,padded_vec)
    end

    return hcat(new_list...)
end
