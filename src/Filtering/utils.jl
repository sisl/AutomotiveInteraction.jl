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

# function: pgfplots 2 gif using Reel
"""
    function pgfplots2gif(plots;filename="output.gif")
- Make a video using an array of plots. Uses the Reel library.

# Caveat
- Only works with .gif output type. If use .mp4, errors saying process exited ffmpeg
- Relies on the setting Reel.extension
Reel.extension(m::MIME"image/svg+xml") = "svg"
"""
function pgfplots2gif(plots;filename="output.gif")
    @assert typeof(filename) == String
    
    frames = Frames(MIME("image/svg+xml"), fps=1)
    
    for (i,plt) in enumerate(plots)
        push!(frames, plt)
        #PGFPlots.save("test/media/$i.pdf",plt) # Store the pic for debugging
    end
    write(filename, frames)
    print("pgfplots2gif: Making animation called $(filename)")
    return nothing
end # End of the reel gif writing function

# function: make pairwise particle variation videos, what pairs to make is the question
"""
- We have a list with corresponding particle matrix at every iteration of filtering
- We shall scatter two columns of said matrix
- We shall make a video with said scatter at every timestep

# Examples
```julia
seed = 2; num_p = 500
Random.seed!(seed)
final_p_mat,iterwise_p_mat = multistep_update(car_id=1,start_frame=2,last_frame=99,num_p=num_p);
plot_pairwise_particles(iterwise_p_mat,filename="test/media/seed2.gif")
```
"""
function plot_pairwise_particles(iterwise_p_mat;filename)
    num_params = size(iterwise_p_mat[1],1) # Number of rows tells us the number of parameters
    print("num_params = $num_params\n")
    pairs = collect(combinations(collect(1:num_params),2))
    num_pairs = length(pairs)
    print("num_pairs = $num_pairs\n")
    groupplots = PGFPlots.GroupPlot[]
    
        # Parameter names. These better match the top of particle_filtering.jl
    param_names = Dict(1=>"Desired velocity",2=>"Acceleration output noise",
        3=>"Min time headway",4=>"Min separation",5=>"Politeness",6=>"Advantage threshold",
        7=>"Headway sensor noise",8=>"Cooperation");

    for i in 1:length(iterwise_p_mat)
        print("plot_pairwise_particles: iternum = $i\n")
        g = PGFPlots.GroupPlot(7,4)
        p_mat = iterwise_p_mat[i]
        
        for j in 1:num_pairs
            kk = pairs[j]
            param_1 = kk[1];param_2 = kk[2]
            name_1 = param_names[param_1];name_2=param_names[param_2]
            p = PGFPlots.Axis([PGFPlots.Plots.Scatter(p_mat[param_1,:],p_mat[param_2,:])],
                    xlabel=name_1,ylabel=name_2)
            push!(g,p)
        end
        push!(groupplots,g)
    end

    pgfplots2gif(groupplots,filename=filename)
    return nothing
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