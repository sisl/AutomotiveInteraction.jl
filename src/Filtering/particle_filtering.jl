# Define global constants
const V_DES = 1; const SIGMA_IDM = 2; const T_HEADWAY = 3; const S_MIN=4; 
const POLITENESS = 5;const ADV_TH = 6;const SENSOR_SIGMA = 7;
const COOPERATION = 8;    

# struct: FilteringEnvironment
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

# function: particle to c-IDM model
"""
function cidm_from_particle(particle)

- Use particle to assign paramters of CooperativeIDM

# Example
```julia
# See hallucinate_a_step
```
"""
function cidm_from_particle(particle)
    return CooperativeIDM(
                c=particle[COOPERATION],
                idm = IntelligentDriverModel(
                            v_des = particle[V_DES],
                            σ=particle[SIGMA_IDM],T=particle[T_HEADWAY],
                            s_min=particle[S_MIN]
                )
            )
end

# function: hallucinate_a_step
"""
    function hallucinate_a_step(scene_input,particle;car_id=-1)
- Hallucinate one step starting from `scene_input` using parameters given by `particle`

# Examples
```julia
scene = f.traj[1]
particle = [29.,NaN,1.5,5.,0.35,0.1,NaN,1.0]
hallucinate_a_step(f,scene,particle,car_id=6)
```
"""
function hallucinate_a_step(f::FilteringEnvironment,scene_input,particle;car_id=-1)
    if car_id==-1 @show "hallucinate_a_step says: Please give a valid car_id" end

    scene = deepcopy(scene_input)
    models = Dict{Int64,DriverModel}()

    for veh in scene
        if veh.id == car_id
            models[veh.id] = cidm_from_particle(particle)
        else
            models[veh.id] = IntelligentDriverModel(v_des=15.)
        end
    end

    nticks = 1
    scene_list = simulate(scene,f.roadway,models,nticks,f.timestep)

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
scene = f.traj[1]
true_next_scene = deepcopy(f.traj[2])
true_nextpos = get_frenet_s(true_next_scene;car_id=id)
true_nextlane = get_lane_id(true_next_scene,id)
weight_and_resample(f,scene,true_nextpos,true_nextlane,init_pmat,car_id=id)
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
function multistep_update(f::FilteringEnvironment;car_id,start_frame,last_frame,num_p=500,seed=1)

- Run filtering over a trajectory by starting from a true scene
- Repeatedly calls `weight_and_resample` on a demonstration trajectory

# Caveats
- Hard coded limits on initial particle distribution generation

# Returns
- `final_p_mat`: Matrix with particles in separate columns. Mean of final particle set
- `iterwise_p_mat`: A list with the associated particle matrix at each iteration

# Examples
```julia
final_p_mat,iterwise_p_mat = multistep_update(f,car_id=6,start_frame=1,last_frame=99)
```
"""
function multistep_update(f::FilteringEnvironment;car_id,start_frame,last_frame,num_p=500,seed=1)
    start_scene = f.traj[start_frame]
    v = start_scene[findfirst(car_id,start_scene)].state.v
    limits = [v-2 v+2;0.1 5.;0.5 5.;1. 10.;0. 1.;-1. 1.;0. 1.;0. 1.]
    p_mat = initial_pmat(limits=limits,num_particles=num_p,seed=seed)
    iterwise_p_set = [] # Stores particle set at every iteration
    push!(iterwise_p_set,p_mat)
    
    for framenum in start_frame:last_frame
        #print("frame number = $(framenum)\n")
        scene = deepcopy(f.traj[framenum])
        true_next_scene = deepcopy(f.traj[framenum+1])
        true_nextpos = get_frenet_s(true_next_scene;car_id=car_id)
        true_nextlane = get_lane_id(true_next_scene,car_id)
        # print("multistep update says: true_nextpos = $(true_nextpos), true_nextlane=$(true_nextlane)\n")
        p_mat_new, weight_vec = weight_and_resample(f,scene,true_nextpos,
        true_nextlane,p_mat,car_id=car_id)

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

"""
    function obtain_driver_models(f::FilteringEnvironment,veh_id_list,num_particles,start_frame,last_frame)
Driver models for each vehicle in veh_id_list

# Arguments
- `veh_id_list` List with vehicle ids
- `start_frame` Frame to start filtering from
- `last_frame` Frame to end hallucination at

# Returns
- `models` Dict with veh id as key and IDM driver model as value
- `final_particles` Dict with veh id as key and avg particle over final particle set as value
- `mean_dist_mat`: Every elem is the mean dist of particle set at that iter (row) for that car (column)

# Example
```julia
veh_id_list = [6]
f = FilteringEnvironment()
new_models,final_particles,mean_dist = obtain_driver_models(f,veh_id_list,500,1,5)
```
"""
function obtain_driver_models(f::FilteringEnvironment,veh_id_list,num_particles,
    start_frame,last_frame)
    
    models = Dict{Int64,DriverModel}() # key is vehicle id, value is driver model
    final_particles = Dict{Int64,Array{Float64}}() # key is vehicle id, value is final particles
    
        # Loop over all the cars and get their corresponding IDM parameters by particle filter
    num_cars = length(veh_id_list)
    num_iters = last_frame-start_frame+2 # NEED TO CONFIRM THIS
    
        # num_iters x num_cars. Every elem is the mean dist of particle set at that iter for that car
    mean_dist_mat = fill(0.,num_iters,num_cars)
    
    for (ii,veh_id) in enumerate(veh_id_list)
        print("obtain_driver_models. vehicle id = $(veh_id) \n")
        
        mean_particle, iterwise_p_set = multistep_update(f,num_p=num_particles,
                car_id = veh_id,start_frame = start_frame,last_frame = last_frame)
        #print("mean_particle", mean_particle, "\n")
        
        final_particles[veh_id] = mean_particle
            # note: T=0.2 and s_min=1.
        models[veh_id] = cidm_from_particle(mean_particle)
        
        #print("After filtering mean_particle = $(mean_particle)\n")        
        # num_iters = length(iterwise_p_set) # SHOULD MATCH OUTSIDE LOOP VARIABLE
        mean_dist_over_iters = fill(0.,num_iters,1)
        for (jj,p_mat) in enumerate(iterwise_p_set)
            current_mean_particle = mean(p_mat,dims=2)
            mean_dist_over_iters[jj,1] = norm(current_mean_particle-mean_particle)
        end
        
        mean_dist_mat[:,ii] = mean_dist_over_iters    
    end
    
        # Average over the cars and plot the filtering progress over frames
    # avg_over_cars = mean(mean_dist_mat,dims=2)
    # plot(avg_over_cars)
    
    return models, final_particles, mean_dist_mat
end