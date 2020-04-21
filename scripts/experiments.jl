# This file is used for performing experiments
# Run by changing path in REPL to scripts folder
# Then `include("experiments.jl")`

using AutomotiveSimulator
using AutomotiveInteraction

"""
function run_experiment(f;ts=1,id_list=[],dur=10.,modelmaker=make_IDM_models)

- Perform driving simulation and extract metrics

# Arguments
- ts: start frame
- id_list: list of vehicles 
- dur: duration
- modelmaker: function that makes the models

# Example
```julia
f = FilteringEnvironment()
scenes,collisions = run_experiment(f,id_list=[4,6,8,13,19,28,29],
modelmaker=make_cidm_models,filename="media/cidm_exp.mp4")
```
"""
function run_experiment(f;ts=1,id_list=[],dur=10.,
modelmaker=make_IDM_models,filename="media/run_exp.mp4")
    print("Run experiment from scripts being called \n")
    scene_real = deepcopy(f.traj[ts])
    if !isempty(id_list) keep_vehicle_subset!(scene_real,id_list) end
    
    models = modelmaker(scene_real)
    
    nticks = Int(ceil(duration/f.timestep))
    scene_list = simulate(scene_real,f.roadway,models,nticks,f.timestep)
    
    c_array = test_collision(scene_list,id_list)
    
    truth_list = f.traj[start_frame:start_frame+nticks]
    video_overlay_scenelists(scene_list,truth_list,id_list=id_list,roadway=f.roadway,
    filename=filename)

    return scene_list, c_array
end

f = FilteringEnvironment()
scenes,collisions = run_experiment(f,id_list=[4,6,8,13,19,28,29],
modelmaker=make_cidm_models,filename="media/cidm_exp.mp4")