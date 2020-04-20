const INTERACTION_TIMESTEP = 0.1 # unit is second

"""
INTERACTIONTrajdata
The trajectory data stored in the original INTERACTION dataset format.
The dataset is a csv file with columns:
    track_id      - Int64 - Representing the id of the agent
    frame_id      - Int64 - Represents the frames in which the agent appears in the video
    timestamp_ms  - Int64 - represents the time the agent appears in the video. The unit is millisecond
    agent_type    - String - Can be person, car, truck and so on
    x             - Float64 - x position, in meter
    y             - Float64 - y position in meter
    vx            - Float64 - x velocity in m/s
    vy            - Float64 - y velocity in m/s
    psi_rad       - Float64 - yaw angle in radian
    length        - Float64 - Length of the vehicle, in meter
    width         - Float64 - Width of the vehicle, in meter

# Example
```julia
tdraw = INTERACTIONTrajdata("vehicle_tracks_000.csv");
```
"""
mutable struct INTERACTIONTrajdata
    df         :: DataFrame
    car2start  :: Dict{Int, Int}         # maps carindex to starting index in the df
    frame2cars :: Dict{Int, Vector{Int}} # maps frame to list of carids in the scene

    function INTERACTIONTrajdata(input_path::String)

        @assert(isfile(input_path))

        df = readtable(input_path, separator=',', header = true)

        car2start = Dict{Int, Int}()
        frame2cars = Dict{Int, Vector{Int}}()

        for (dfind, carid) in enumerate(df[:track_id])
            if !haskey(car2start, carid)
                car2start[carid] = dfind
            end

            frame = convert(Int, df[dfind, :frame_id])
            if !haskey(frame2cars, frame)
                frame2cars[frame] = [carid]
            else
                frame2cars[frame] = push!(frame2cars[frame], carid)
            end
        end

        new(df, car2start, frame2cars)
    end
end

# function: overload nframes from Records
"""
    Records.nframes

# Example
```julia
tdraw = INTERACTIONTrajdata("vehicle_tracks_000.csv")
nframes(tdraw)
```
"""
nframes(trajdata::INTERACTIONTrajdata) = maximum(keys(trajdata.frame2cars))

# function: carsinframe for memory allocation
"""
    carsinframe(trajdata::INTERACTIONTrajdata, frame::Int)
- From the dictionary `frame2cars` which has frame number as key and all vector of car ids in that frame
as value, this function extracts the value corresponding to key given by `frame`

# Example
```julia
# Return array with car ids in frame 1
carsinframe(tdraw,1)
```
"""
carsinframe(trajdata::INTERACTIONTrajdata, frame::Int) = get(trajdata.frame2cars, frame, Int[])

# function: car_df_index - find index of car in trajdata
"""
	function car_df_index

- Given frame and carid, find index of car in trajdata
- Returns 0 if it does not exist (CAVEAT THIS IS COMMENTED OUT)

"""
function car_df_index(trajdata::INTERACTIONTrajdata, carid::Int, frame::Int)
    df = trajdata.df

    lo = trajdata.car2start[carid] # The index in the dataframe where carid first appears
    framestart = df[lo, :frame_id] # The frame number where carid first appears
    
    retval = 0

    if framestart == frame
        retval = lo
    elseif frame ≥ framestart
        retval = frame - framestart + lo
        #n_frames = df[lo, :n_frames_in_dataset]
        #if retval > lo + n_frames
        #    retval = 0
        #end
    end

    retval
end

"""
    function read_veh_tracks(;roadway)
- Reads the vehicle tracks information from `../dataset/vehicle_tracks_000.csv`
- Modernize to stop using `ListRecord` and `Trajdata`
- Just use Vector of Scenes

# Arguments
- roadway: Used in the vehicle state to create `Frenet` coords for the vehicle on the provided roadway

# Example
```julia
road_ext = make_interaction_roadway_with_extensions()
trajdata = read_veh_tracks(roadway=road_ext)

# Then access a certain framenumber from the trajdata by
scene = trajdata[1]
render([road_ext,scene,IDOverlay(scene=scene)])
```
"""
function read_veh_tracks(;roadway)
    # Read the csv data into tdraw which recasts csv info into a dataframe and also provides two
    # dicts called car2start and frame2cars
    tdraw = INTERACTIONTrajdata(joinpath(@__DIR__,"dataset/vehicle_tracks_000.csv"))
    df = tdraw.df
    vehdefs = Dict{Int, VehicleDef}()

    Trajdata = Vector{Scene}(undef,nframes(tdraw))

    # Initialize the vehicles physically i.e type, length and width
    for (id, dfind) in tdraw.car2start
        # CAUTION: Hardcoding vehicle type to be just car shown by the first argument being 2
        vehdefs[id] = VehicleDef(2, df[dfind, :length], df[dfind, :width])
    end

    # Fill in the vehicle state information in terms of x,y and speed
    for frame in 1 : nframes(tdraw)
        scene = Scene(Entity{VehicleState,VehicleDef,Int64}, 
            length(carsinframe(tdraw,frame)))
        scene_ind = 0
        for id in carsinframe(tdraw, frame)
            dfind = car_df_index(tdraw, id, frame)

            posG = VecSE2(df[dfind, :x], df[dfind, :y], df[dfind, :psi_rad])
            vx = df[dfind,:vx]
            vy = df[dfind,:vy]
            speed = sqrt(vx*vx + vy*vy)
            push!(scene,Entity(VehicleState(posG, roadway, speed),vehdefs[id],id))
        end

        Trajdata[frame] = scene

    end
    
    return Trajdata
end

# function: start and end time extraction of vehicles
"""
function extract_timespan(trajscenesvector;offset=1,minlength=20,verbose=false)

- Extract the start time and end time of each vehicle
- Used in sampling a set of vehicles that drive together for a given timespan

# Inputs
- trajscenesvector: Is the output from read_veh_tracks i.e. vector of scenes
- offset from the start

# Returns
- Dict with car id as key and start and end times of the car id

# Examples
```julia
trajscenes = read_veh_tracks(roadway=roadway)
timestamped_trajscenes = extract_timespan(trajscenes)
```
"""
function extract_timespan(trajscenesvector;offset=1,minlength=20,verbose=false)
    index = Dict()
    scene_length = maximum(length(trajscenesvector[i]) for i in 1 : length(trajscenesvector))
    scene = Scene(Entity{VehicleState,VehicleDef,Int64}, scene_length)
    prev, cur = Set(), Set()
    n_frames = length(trajscenesvector)

    # iterate each frame collecting info about the vehicles
    for frame in offset : n_frames - offset
        if verbose > 0
            print("\rframe $(frame) / $(n_frames - offset)")
        end
        cur = Set()
        scene=trajscenesvector[frame]

        # add all the vehicles to the current set
        for veh in scene
            push!(cur, veh.id)
            # insert previously unseen vehicles into the index
            if !in(veh.id, prev)
                index[veh.id] = Dict("ts"=>frame)
            end
        end

        # find vehicles in the previous but not the current frame
        missing = setdiff(prev, cur)
        for id in missing
            # set the final frame for all these vehicles
            index[id]["te"] = frame - 1
        end

        # step forward
        prev = cur
    end

    # at this point, any ids in cur are in the last frame, so add them in 
    for id in cur
        index[id]["te"] = n_frames - offset
    end

    # postprocess to remove undesirable trajectories
    for (vehid, infos) in index
        # check for start and end frames 
        if !in("ts", keys(infos)) || !in("te", keys(infos))
            if verbose > 0
                println("delete vehid $(vehid) for missing keys")
            end
            delete!(index, vehid)

        # check for start and end frames greater than minlength
        elseif infos["te"] - infos["ts"] < minlength
            if verbose > 0
                println("delete vehid $(vehid) for below minlength")
            end
            delete!(index, vehid)
        end
    end

    return index
end


# Functions: Sampling vehicles that drive together
"""
    function random_sample_from_set_without_replacement(s, n)
Description
    This function samples n values from the set s without replacement, and 
    does not work with anything except a set s. Could use statsbase, but want 
    to avoid the dependency.
- Used by `sample_multiple_trajdata_vehicle`
Args:
    - s: a set
    - n: number of values to sample
Returns:
    - a subset of the values in s, as a list containing n elements
"""
function random_sample_from_set_without_replacement(s, n)
    @assert length(s) >= n
    sampled = Set()
    for i in 1:n
        cur = rand(s)
        push!(sampled, cur)
        delete!(s, cur)
    end
    return collect(sampled)
end

# Sample vehicles that stay in the scene together
"""
function sample_simultaneous_vehs

- Finds groups of vehicles that stay together on the road for a duration
- Adapted from `ngsim/julia/src/ngsim_utils.jl`

# Inputs
- timestamped_trajscenes: What we get from extract_timespan i.e. dict with vehid and ts, te

# Returns
- `egoids`: List of vehicles that drive together with the specified ego vehicle for `offset` num of steps
- `ts`: Start frame number
- `te`: End frame number

# Examples
```julia
a,b,c= sample_simultaneous_vehs(10,50,timestamped_trajscenes,egoid=6)
```
"""
function sample_simultaneous_vehs(
        n_veh::Int, 
        offset::Int,
        timestamped_trajscenes;
        max_resamples::Int = 100,
        egoid::Union{Nothing, Int} = nothing,
        verbose::Bool = true,
        rseed::Union{Nothing, Int} = nothing
    )
    
    if rseed != nothing
        Random.seed!(rseed)
    end
    traj_idx = 1
    
    trajinfos = timestamped_trajscenes
    
    # if passed in egoid and traj_idx, use those, otherwise, sample
    if egoid == nothing 
        # sample the first vehicle and start and end timesteps
        egoid = rand(collect(keys(trajinfos[traj_idx])))
    end
    
    ts = trajinfos[egoid]["ts"]
    te = trajinfos[egoid]["te"]
    # this sampling assumes ts:te-offset is a valid range
    # this is enforced by the initial computation of the index / trajinfo
    ts = rand(ts:te - offset)
    # after setting the start timestep randomly from the valid range, next 
    # update the end timestep to be offset timesteps following it 
    # this assume that we just want to simulate for offset timesteps
    te = ts + offset

    # find all other vehicles that have at least 'offset' many steps in common 
    # with the first sampled egoid starting from ts. If the number of such 
    # vehicles is fewer than n_veh, then resample
    # start with the set containing the first egoid so we don't double count it
    egoids = Set{Int}(egoid)
    for othid in keys(trajinfos)
        oth_ts = trajinfos[othid]["ts"]
        oth_te = trajinfos[othid]["te"]
        # other vehicle must start at or before ts and must end at or after te
        if oth_ts <= ts && te <= oth_te
            push!(egoids, othid)
        end
    end

    # check that there are enough valid ids from which to sample
    if length(egoids) < n_veh
        # if not, resample
        if verbose
            println("WARNING: insuffcient sampling ids in sample_multiple_trajdata_vehicle")
        end
        if max_resamples == 0
            error("ERROR: reached maximum resamples in sample_multiple_trajdata_vehicle")
        else
            return sample_simultaneous_vehs(
                n_veh, 
                offset, 
                max_resamples=max_resamples - 1,
                verbose=verbose)
        end
    end

    # reaching this point means there are sufficient ids, sample the ones to use
    egoids = random_sample_from_set_without_replacement(egoids, n_veh)

    return egoids, ts, te
end
