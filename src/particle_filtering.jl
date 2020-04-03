# Define global constants
const V_DES = 1; const SIGMA_IDM = 2; const T_HEADWAY = 3; const S_MIN=4; 
const POLITENESS = 5;const ADV_TH = 6;const SENSOR_SIGMA = 7;
const COOPERATION = 8;    


"""
@with_kw struct FilteringEnvironment

- Contains the roadway and trajectory information 
- To enable not having to pass roadway and traj as input args to all functions

# Example
```julia
f = FilteringEnvironment()
```
"""
@with_kw struct FilteringEnvironment
    roadway::Roadway{Float64} = make_roadway_interaction_with_extensions()
    traj = read_veh_tracks(roadway=roadway)
    timestep::Float64 = 0.1
end

# function: get frenet s
"""
    function get_frenet_s(scene;car_id=-1)

# Examples
```julia
true_next_pos = get_frenet_s(true_next_scene,car_id=1)
```
"""
function get_frenet_s(scene;car_id=-1)
    if car_id==-1 print("get_frenet_s says: Give valid car id") end
    veh = scene[findfirst(car_id,scene)]
    return veh.state.posF.s
end

# function: get lane id
"""
    function get_lane_id(scene,car_id)
# Examples
```julia
get_lane_id(scene,1)
```
"""
function get_lane_id(scene,car_id)
    veh = scene[findfirst(car_id,scene)]
    return veh.state.posF.roadind.tag.lane
end

# function: get lane change probability
"""
    function get_lane_change_prob(start_scene,particle;car_id=-1,num_samplings=10)
- Probability of lane changing start from `start_scene`
- hallucinating using `particle` for `car_id` using `num_samplings` hallucinations

# Examples
```julia
lp = get_lane_change_prob(scene,particle,car_id = 1,roadway=road_ext)
```
"""
function get_lane_change_prob(f::FilteringEnvironment,start_scene,particle;car_id=-1,
    num_samplings=10)
    if car_id==-1 @show "get_lane_change_prob says: Please give valid car_id" end
    start_lane = get_lane_id(start_scene,car_id)
    changed_count = 0; unchanged_count = 0
    for i in 1:num_samplings
        hpos,hlane = hallucinate_a_step(f,start_scene,particle,car_id=car_id)
        if hlane == start_lane
            unchanged_count += 1
	else
	    changed_count += 1
	end
    end
    return (changed_count+1)/(num_samplings+2)
end

# function: Generate uniform sampling to start the initial particle matrix
"""
    function initial_pmat(;limits,num_particles,seed)
- Generate initial particle matrix with `num_particles` particles with every col being a diff particle
- Range of values that parameters can take is specified in `limits`. Should be num_params rows x 2 cols

# Examples
```julia
limits = [10. 40.;0.1 10.;0.5 5.;1. 10.;0. 1.;-1. 1.;-20. 20.;0. 1.]
initial_pmat(limits=limits,num_particles=10,seed=4)
```
"""
function initial_pmat(;limits,num_particles,seed)
    Random.seed!(seed)
    num_params = size(limits,1)
    p_mat = fill(0.,num_params,num_particles)

    for i in 1:num_params
        p_mat[i,:] = rand(Uniform(limits[i,1],limits[i,2]),1,num_particles)
    end
    return p_mat
end

# function: hallucinate_a_step
"""
    function hallucinate_a_step(scene_input,particle;car_id=-1)
- Hallucinate one step starting from `scene_input` using parameters given by `particle`

# Examples
```julia
limits = [10. 40.;0.1 10.;0.5 5.;1. 10.;0. 1.;-1. 1.;0. 20.]
init_pmat = initial_pmat(limits=limits,num_particles=10,seed=4)
hallucinate_a_step(SCENE,init_pmat[:,9],car_id=1)
```
"""
function hallucinate_a_step(f::FilteringEnvironment,scene_input,particle;car_id=-1)
    if car_id==-1 @show "hallucinate_a_step says: Please give a valid car_id" end

    scene = deepcopy(scene_input)
    models = Dict{Int64,DriverModel}()

    for veh in scene
        if veh.id == car_id
            models[veh.id] = CooperativeIDM(c=particle[COOPERATION])
        else
            models[veh.id] = IntelligentDriverModel(v_des=50.)
        end
    end

    nticks = 1
    scene_list = simulate(scene,f.roadway,models,nticks,INTERACTION_TIMESTEP)

    new_scene = scene_list[2] # simulate stores start scene in 1st elem
    # Thus, we need elem 2 as the hallucinated scene
    halluc_state = new_scene.entities[findfirst(car_id,scene)].state
    halluc_pos = halluc_state.posF.s
    halluc_lane = get_lane_id(scene,car_id)

    return halluc_pos,halluc_lane
end

# function: weight and resample
"""
    function weight_and_resample(start_scene,true_nextpos,true_nextlane,p_mat;car_id=-1)
- Hallucination from `start_scene` 
- Compare against ground truth at `true_nextpos`, `true_nextlane`
- Assign weights to particles
- Perform resampling and return a new matrix of particles and associated weight vector

# Examples
```julia
# Test for one step on initial particle matrix
limits = [10. 40.;0.1 10.;0.5 5.;1. 10.;0. 1.;-1. 1.;0. 20.;0. 1.]
init_pmat = initial_pmat(limits=limits,num_particles=10,seed=4)
id = 6
scene = traj_ext[1]
true_next_scene = deepcopy(traj_ext[2])
true_nextpos = get_frenet_s(true_next_scene;car_id=id)
true_nextlane = get_lane_id(true_next_scene,id)
weight_and_resample(scene,true_nextpos,true_nextlane,init_pmat,car_id=id,roadway=road_ext)
```
"""
function weight_and_resample(f::FilteringEnvironment,start_scene,true_nextpos,true_nextlane,p_mat;
    car_id=-1,verbosity=false)
    if car_id==-1 @show "compute_particle_likelihood says: Please give valid car_id" end
    num_p = size(p_mat,2)
    lkhd_vec = Array{Float64}(undef,num_p)
    for i in 1:num_p
        if verbosity print("w_and_resample says: particle number = $i \n") end
        particle = p_mat[:,i]
        
        std_dev_acc = p_mat[SIGMA_IDM]
        if std_dev_acc <= 0 std_dev_acc = 0.1 end
        std_dev_pos = f.timestep*f.timestep*std_dev_acc
        hpos,hlane = hallucinate_a_step(f,start_scene,particle,car_id=car_id)
        
        start_lane = get_lane_id(start_scene,car_id)
        lane_has_changed = false

        if start_lane != true_nextlane
            lane_has_changed = true
        end

        p_lanechange = get_lane_change_prob(f,start_scene,particle,car_id=car_id)

        prob_lane = 0.5 # Initialize to random
        if lane_has_changed
            prob_lane = p_lanechange
        else
            prob_lane = 1-p_lanechange
        end
        
        prob_pos = pdf(Normal(hpos,std_dev_pos),true_nextpos)
        if verbosity
            print("weight and resample says: true_nextpos = $(true_nextpos) and hpos=$(hpos) and hlane=$(hlane)\n")
            print("weight and resample says: prob_pos = $(prob_pos) and prob_lane=$(prob_lane)\n")
        end
        lkhd_vec[i] = prob_lane*prob_pos
    end
    p_weight_vec = StatsBase.weights(lkhd_vec./sum(lkhd_vec)) # Convert to weights form to use julia sampling
    
    idx = sample(1:num_p,p_weight_vec,num_p)
    if verbosity print("weight and resample says: ids are $(idx)\n") end
    new_p_mat = p_mat[:,idx] #Careful that idx is (size,1) and not (size,2)
    
    return new_p_mat, p_weight_vec
end

# function: multistep_update
"""
- Run filtering over a trajectory by starting from a true scene
- Repeatedly calls `weight_and_resample` on a demonstration trajectory

# Caveats
- Hard coded usage of `true_scene_list` which is basically ground truth generated by `get_hallucination_scenes`
- Hard coded limits on initial particle distribution generation

# Returns
- `final_p_mat`: Matrix with particles in separate columns
- `iterwise_p_mat`: A list with the associated particle matrix at each iteration

# Examples
```julia
final_p_mat,iterwise_p_mat = multistep_update(car_id=1,start_frame=2,last_frame=99)
```
"""
function multistep_update(f::FilteringEnvironment;car_id,start_frame,last_frame,num_p=500,seed=1,verbosity=false)
    if verbosity print("car id = $(car_id)\n") end
        
        # Careful selectin of particles around initial velocity of data
        # Commented out for now as we are not yet at the stage of using TRAJ
    #startscene = get_scene(start_frame,TRAJ)
    #startpos,startvel = get_veh_info(startscene,car_id=car_id)
    #init_p_mat = sample_init_particles(num_p,v=startvel)

    limits = [10. 40.;0.1 10.;0.5 5.;1. 10.;0. 1.;-1. 1.;0. 20.]
    p_mat = initial_pmat(limits=limits,num_particles=num_p,seed=seed)
    iterwise_p_set = [] # Stores particle set at every iteration
    push!(iterwise_p_set,p_mat)
    
    for framenum in start_frame:last_frame
            # Get the truth from TRAJ
        #scene = get_scene(framenum+1,TRAJ)
        #trupos,truvel = get_veh_info(scene,car_id=car_id,traj=traj)
        
            # Get the scene to start hallucinating from
        #scene = get_scene(framenum,TRAJ)
        scene = deepcopy(true_scene_list[framenum])
        true_next_scene = deepcopy(true_scene_list[framenum+1])
        true_nextpos = get_frenet_s(true_next_scene;car_id=car_id)
        true_nextlane = get_lane_id(true_next_scene,car_id)
        # print("multistep update says: true_nextpos = $(true_nextpos), true_nextlane=$(true_nextlane)\n")
        p_mat_new, weight_vec = weight_and_resample(f,
        scene,true_nextpos,true_nextlane,p_mat,car_id=car_id)

            # Addition of dithering noise
        #params = [:v_des,:σ]
        #p_mat = addnoise(p_mat_new, weight_vec)
        
        p_mat = p_mat_new # No noise addition for now
        
            # Storage into list that contains particle matrix at every step
        push!(iterwise_p_set,p_mat)
    end

        # Find the mean particle after filtering is over
    mean_particle = mean(p_mat,dims=2)
    
    return mean_particle,iterwise_p_set
end