"""
helpers.jl
This file provides helper functions to experiments.jl, the experiment script
Most functions have the script within the doc string as example

# List of functions
- metrics_from_jld
- metrics_from_jld_idmbased
- multiscenarios_idm
- multiscenarios_pf
- collision rmse functions
- scenario generation
"""

using Distributions # provides `mean`: to compute mean of particle dist over cars
using JLD # to save models
using AutomotiveInteraction
using DelimitedFiles # to write to txt file that will be read in by python

#****************scenario_generation************************
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
