"""
helpers.jl
This file provides helper functions to experiments.jl, the experiment script
Most functions have the script within the doc string as example
"""

using Distributions # provides `mean`: to compute mean of particle dist over cars
using JLD # to save models
using AutomotiveInteraction
using DelimitedFiles # to write to txt file that will be read in by python

"""
- Make scatter plot of the lmidm parameters and final particle for a scenario
with each scatter point corresponding to a different car

# Examples
```julia
cd("scripts")
f= FilteringEnvironment()
scatterplot_lmfit_pf(f,scene_real,scenario_name="upper_1")
```
"""
function scatterplot_lmfit_pf(f::FilteringEnvironment;scenario_name="upper_1")
    filename="media/$(scenario_name).jld"
    particle,id_list,ts,te = JLD.load(filename,"p","veh_id_list","ts","te")
    scene = deepcopy(f.traj[ts])
    if !isempty(id_list) keep_vehicle_subset!(scene,id_list) end

    v_des_vec = []
    T_vec = []
    v_des_vec_pf = []
    T_vec_pf=[]
    # Loop over list of vehicles and assign model params based on lmfit values
    for veh in scene
        veh_id = veh.id
        
        # Read the lmfit parameters
        lmfit_param_filename = "lmfit/$(scenario_name)/$(veh_id)_lmfit_params.txt"
        if isfile(lmfit_param_filename)
            lmparams = readdlm(lmfit_param_filename)
            v_des = lmparams[1]
            T = lmparams[2]
            s_min = lmparams[3]
            push!(v_des_vec,v_des)
            push!(T_vec,T)
        else
            # Default to Jeremy's parameters
            print("Could not find lmfit param file for $(veh_id). Use Jeremy params\n")
            v_des=17.837
            T=0.918
            s_min=5.249
            push!(v_des_vec,v_des)
            push!(T_vec,T)
        end

        # Insert the pf parameters
        push!(v_des_vec_pf,particle[veh_id][1])
        push!(T_vec_pf,particle[veh_id][3])
    end

    scatter_lmidm = PGFPlots.Plots.Scatter(Float64.(v_des_vec),Float64.(T_vec),
    legendentry="lmidm")
    scatter_pf = PGFPlots.Plots.Scatter(Float64.(v_des_vec_pf),Float64.(T_vec_pf),
    legendentry="pf")
    a = PGFPlots.Axis([scatter_lmidm,scatter_pf],xlabel="v",ylabel="T",
    title="lmidm vs pf parameters")
    PGFPlots.save("media/lmfit_pf_scatter_$(scenario_name).svg",a)
    return nothing
end

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


# ***************Vehicle track plot****************
struct MyRenderableCircle
    pos::VecE2
    radius::Float64
    color::Colorant
end

function AutomotiveVisualization.add_renderable!(rendermodel::RenderModel, circle::MyRenderableCircle)
    # add the desired render instructions to the rendermodel
    add_instruction!(
        rendermodel, AutomotiveVisualization.render_circle,
        (circle.pos.x, circle.pos.y, circle.radius, circle.color),
        coordinate_system=:scene
    )
    return rendermodel
end

"""
- Make a track of the vehicle positions over time

# Example
```julia
f = FilteringEnvironment()
id_list,ts,te=JLD.load("media/upper_1.jld","veh_id_list","ts","te")
scene_list = replay_scenelist(f,id_list=id_list,ts=ts,te=te)
p6 = veh_track(6,scene_list);
p8 = veh_track(8,scene_list,color=RGB{Float64}(0.20,0.72,0.33));
p13 = veh_track(13,scene_list,color=rand(RGB));
render([f.roadway,p6...,p8...,p13...])
```
"""
function veh_track(vehid::Int64,scene_list;
    color=RGB{Float64}(0.86,0.23,0.49))
    circles = []
    for scene in scene_list
        veh = get_by_id(scene,vehid)
        x = veh.state.posG.x
        y = veh.state.posG.y
        push!(circles,MyRenderableCircle(VecE2(x,y),0.5,color))
    end
    return circles
end
