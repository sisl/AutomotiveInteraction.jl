# function: make default parameter driver model given a starting scene of vehicles
"""
    function make_def_models(scene)
- Takes an input `scene` and associates a default param `Tim2DDriver` to all the vehicles in the scene

# Examples
```julia
scene = Scene(500)
scene = get_scene(1,traj_interaction)
models = make_def_models(scene)
```
"""
function make_def_models(scene)
    models = Dict{Int64,DriverModel}()
    for veh in scene
        models[veh.id] = Tim2DDriver(INTERACTION_TIMESTEP,
            mlane=MOBIL(INTERACTION_TIMESTEP,mlon=IntelligentDriverModel()))
    end
    return models
end

# function: get hallucination scenes
"""
    function get_hallucination_scenes
- Hallucinate starting from `start_step` for `nsteps` using `models` and return a list of scenes
- Used by `plot_carwise_pos_vel` to assess position and velocity traces against ground truth

# Returns
- `halluc_scenes_list`: List containing the scenes starting with the ground truth scene at `start_step`

# Examples
```julia
scene_list = get_hallucination_scenes(scene,models=models,duration=6)
scenelist2video(scene_list,filename="media/driving_vid.mp4")
```
"""
function get_hallucination_scenes(scene_halluc;models,start_step=1,duration=5,id_list=[],
        traj=traj_interaction,verbosity = false,timestep=INTERACTION_TIMESTEP,roadway=roadway_interaction)
        # Setting up
    halluc_scenes_list = []
    #scene_halluc = get_scene(start_step,traj) # Frame to start hallucination from
    push!(halluc_scenes_list,deepcopy(scene_halluc))
    
    nsteps = duration/timestep
    for (i,t) in enumerate(start_step:start_step+nsteps-1)
        
        if !isempty(id_list) keep_vehicle_subset!(scene_halluc,id_list) end
        
        actions = Array{Any}(undef,length(scene_halluc))

            # Propagation of scene forward
        get_actions!(actions,scene_halluc,roadway,models)

        tick!(scene_halluc,roadway,actions,timestep)
        
        push!(halluc_scenes_list,deepcopy(scene_halluc))
    end 
    return halluc_scenes_list
end

# function: Select a vehicle from starting scene and run it for a duration to get a video
"""
    function run_a_vehicle
- Select vehicle with id `veh_id` from frame number `start_frame` of track information in `traj`
- Assign default 2D driver model to the vehicle and run it for `duration` seconds on `roadway`
- Writes a video to location specified by `filename`

# Examples
```julia
run_a_vehicle(veh_id=29,duration=10.,roadway=roadway_test,filename="media/veh_test.mp4")
```
"""
function run_a_vehicle(;veh_id,start_frame = 1,duration=5.,filename="media/vehid_$(veh_id).mp4",
    traj = traj_interaction, roadway = roadway_interaction)
    scene_real = get_scene(start_frame,traj)
    veh = scene_real[findfirst(veh_id,scene_real)]
    
    scene = Scene(500)
    push!(scene,veh)
    models = make_def_models(scene)
    scene_list = get_hallucination_scenes(scene,models=models,duration=duration,roadway=roadway)
    scenelist2video(scene_list,filename=filename,roadway=roadway)
    return nothing
end
