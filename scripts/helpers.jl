# This file provides helper functions to run experiments
using Distributions # provides `mean`: to compute mean of particle dist over cars
"""
function extract_metrics(f;ts=1,id_list=[],dur=10.,modelmaker=nothing,filename=[])

- Perform driving simulation starting from `ts` for `dur` duration.

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
        print("Let's run particle filtering to create driver models\n")
        models,p,mean_dist_mat = obtain_driver_models(f,veh_id_list=id_list,num_p=50,ts=ts,te=ts+50)
        avg_over_cars = mean(mean_dist_mat,dims=2)
        avg_over_cars = reshape(avg_over_cars,length(avg_over_cars),) # for PGFPlot
        print("Making filtering progress plot\n")
        p = PGFPlots.Plots.Linear(collect(1:length(avg_over_cars)),avg_over_cars)
        ax = PGFPlots.Axis([p],xlabel = "iternum",ylabel = "avg distance",
        title = "Filtering progress")
        PGFPlots.save("media/p_prog.pdf",ax)
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
