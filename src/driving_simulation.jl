# function: keep subset of vehicles in the scene
"""
    function keep_vehicle_subset!(scene::Scene, ids::Vector{Int})
- Just keep a set of vehicles in scene as specified by `ids` (a list of vehicle ids)
- Obtained from `ngsim_env/julia/src/ngsim_utils.jl`

# Example
```julia
scene = get_scene(1, traj_interaction)
veh_id_list = [7,10,11,18,22,25,29]
keep_vehicle_subset!(scene,veh_id_list)
```
"""
function keep_vehicle_subset!(scene::Scene, ids)
    keep_ids = Set(ids)
    scene_ids = Set([veh.id for veh in scene])
    remove_ids = setdiff(scene_ids, keep_ids)
    for id in remove_ids
        delete!(scene, id)
    end
    return scene
end

# function: place 3 vehicles on a straight road: Enables quick testing of ideas
"""
    function place_vehs_on_straight_road()
- Generate a stright roadway and place 3 vehicles on it
- Roadway is generated using `gen_straight_roadway` so returns a road with 1 segment and all lanes in that segment

# Examples
```julia
scene,road_straight = place_vehs_on_straight_road()
```
"""
function place_vehs_on_straight_road()
    road_straight = gen_straight_roadway(4,400.)

    scene = Scene()

    veh1 = Vehicle(VehicleState(VecSE2(10.,0.,0.),road_straight,20.),VehicleDef(),1)
    push!(scene,veh1)

    veh2 = Vehicle(VehicleState(VecSE2(30.,0.,0.),road_straight,10.),VehicleDef(),2)
    push!(scene,veh2)

    veh3 = Vehicle(VehicleState(VecSE2(30.,5.,0.),road_straight,20.),VehicleDef(),3)
    push!(scene,veh3)

    return scene,road_straight
end

"""
    function place_vehs_on_separated_segments_road()
- Create two straight segments, turn them into lanes, and then a roadway with two separate segments
- Such a scenario was used to test whether lane changes can happen across segments. Answer is no

# Examples
```julia
scene,road_test = place_vehs_on_separated_segments_road()
```
"""
function place_vehs_on_separated_segments_road()
    test_road = Roadway()
    start1 = VecE2(0.,0.)
    end1 = VecE2(400.,0.)
    track1 = gen_straight_curve(start1,end1,2)
    lane1 = Lane(LaneTag(1,1),track1)
    push!(test_road.segments,RoadSegment(1,[lane1]))

    start2 = VecE2(0.,5.)
    end2 = VecE2(400.,5.)
    track2 = gen_straight_curve(start2,end2,2)
    lane2 = Lane(LaneTag(2,1),track2)
    push!(test_road.segments,RoadSegment(2,[lane2]))

    scene = Scene()

    veh1 = Vehicle(VehicleState(VecSE2(10.,0.,0.),test_road,20.),VehicleDef(),1)
    push!(scene,veh1)

    veh2 = Vehicle(VehicleState(VecSE2(30.,0.,0.),test_road,10.),VehicleDef(),2)
    push!(scene,veh2)

    veh3 = Vehicle(VehicleState(VecSE2(30.,5.,0.),test_road,20.),VehicleDef(),3)
    push!(scene,veh3)

    return scene,test_road
end

# functions: default param driver model, and timlanechanger driver model to all vehicles in scene
"""
    function make_def_models(scene)
- Takes an input `scene` and associates a default param `Tim2DDriver` to all the vehicles in the scene

# Examples
```julia
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

"""
	function make_TimLaneChanger_models(scene)

- Takes input `scene` and associates default param Tim2DDriver model to all vehs, with lane changer being `TimLaneChanger`
# Examples
```julia
scene = get_scene(traj_interaction)
models = make_TimLaneChanger_models(scene)
```
"""
function make_TimLaneChanger_models(scene)
    models = Dict{Int64,DriverModel}()
    for veh in scene
        models[veh.id] = Tim2DDriver(INTERACTION_TIMESTEP,mlane=TimLaneChanger(INTERACTION_TIMESTEP))
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
        traj,verbosity = false,timestep=INTERACTION_TIMESTEP,roadway)
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

# function: Select vehicles from a real scene and run them for a duration to get a video
"""
    function run_vehicles
- Select vehicles with ids in `id_list` from frame number `start_frame` of track information in `traj`
- Assign default 2D driver model to the vehicle and run it for `duration` seconds on `roadway`
- Writes a video to location specified by `filename`

# Arguments
- id_list: Vehicle ids in this list will be kept in the simulation. Others will be deleted
- traj: Vehicle traj data stored in the `Trajdata` type provided by `AutomotiveDrivingModels.jl`
- start_frame: Frame number of vehicle track data in `traj`
- roadway: a roadway object eg: `roadway_interaction`

# Examples
```julia
run_vehicles(id_list=[29,19,28,6,8,25,2,10,7,18,12],roadway=roadway_interaction,
    filename=joinpath(@__DIR__,"julia_notebooks/media/run_test_ext_long.mp4"))
```
"""
function run_vehicles(;id_list,start_frame=1,duration=10.,filename,traj,roadway)

    scene_real = get_scene(start_frame,traj)
    keep_vehicle_subset!(scene_real,id_list)

    models = make_def_models(scene_real)

    scene_list = get_hallucination_scenes(scene_real,models=models,
        id_list=id_list,duration=duration,traj=traj,roadway=roadway)
    scenelist2video(scene_list,filename=filename,roadway=roadway)
    return nothing
end

"""
    function test_barrier_vehicle
- Place a vehicle that does not move to test how far IDM+MOBIL driven vehicles look ahead

# Examples
```julia
test_barrier_vehicle(id_list=[20,29,19,28,6,8,25,2,10,7,18,12,100],roadway=road_ext,
    traj=traj_ext,filename=joinpath(@__DIR__,"../julia_notebooks/media/barrier_test.mp4"))
```
"""
function test_barrier_vehicle(;id_list,start_frame=1,duration=10.,
    roadway,traj,filename)
    scene = get_scene(start_frame,traj)
    keep_vehicle_subset!(scene,id_list)
    veh_20 = scene[findfirst(20,scene)]
    veh20_state = veh_20.state
    deleteat!(scene,findfirst(20,scene)) # Remove veh 20 from scene
    barrier_state = VehicleState(veh20_state.posG,veh20_state.posF,0.)
    barrier_id = 100
    barrier_veh = Vehicle(barrier_state,VehicleDef(),barrier_id)
    push!(scene,barrier_veh) # Insert barrier vehicle instead of the removed vehicle 20

    models = make_def_models(scene)
    models[100] = IntelligentDriverModel(v_des=0.)
    # deleteat!(id_list,findall(x->x==20,id_list))
    scene_list = get_hallucination_scenes(scene,models=models,id_list=id_list,
        duration=duration,traj=traj,roadway=roadway)
    scenelist2video(scene_list,filename=filename,roadway=roadway)
    return nothing
end