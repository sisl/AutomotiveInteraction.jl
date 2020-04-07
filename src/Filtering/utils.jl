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

""" 
function get_veh_info(scene;car_id = -1)
- Get position and velocity of specific vehicle from scene
""" 
function get_veh_info(scene;car_id = -1)
    @assert car_id>0
    pos=scene[findfirst(car_id,scene)].state.posF.s
    vel = scene[findfirst(car_id,scene)].state.v
    return pos,vel
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


""" 
    function avg_dist_particles(p_mat,p_fin)

- Compute the avg distance over particle set from final particle

# Arguments
- `p_mat`: Matrix with particles in every column
- `p_fin`: 2x1 array with the final particle
"""

function avg_dist_particles(p_mat,p_fin)
    return sum(sqrt.(sum((p_mat .- p_fin).^2,dims=1)))*1/size(p_mat,2)
end

""" 
    function plot_particles(p_set_mat::Array{Float64,2},time::Float64)
- Plot the distribution of particles
# Caution:
    Only work for 2 parameters. Not developed for 3 parameter case yet

# Example:
```julia
set_for_plotting = to_particleMatrix(p_set_new)
plots = []
push!(plots,plot_particles(set_for_plotting,framenum*0.04))
```
"""
function plot_particles(p_set_mat::Array{Float64,2},time::Float64)
    # Check that number of params does not exceed 3

    @assert size(p_set_mat,1) <= 3
    plt = Plots.plot()
    # 2 parameter case	
    if size(p_set_mat,1) == 2
        plt = scatter!(p_set_mat[1,:],p_set_mat[2,:],
            leg=false,title="time=$(time)",xlim=(0,20),ylim=(0,4),
                xlabel="v_des(m/s)",ylabel="sigma")

    # 3 parameter case
    else
        plt = scatter!(p_set_mat[1,:],p_set_mat[2,:],p_set_mat[3,:],leg=false)
        scatter!([true_params[1]],[true_params[2]],[true_params[3]])
    end
    return plt
end