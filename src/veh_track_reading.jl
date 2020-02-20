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
Records.nframes(trajdata::INTERACTIONTrajdata) = maximum(keys(trajdata.frame2cars))

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

# function: read vehicle tracks from provided csv file into object called `traj_interaction`
"""
    function read_veh_tracks()
- Reads the vehicle tracks information from `../dataset/vehicle_tracks_000.csv`
- Returns `Trajdata` object containing list of scenes from the vehicle track information

# Example
```julia
traj_interaction = read_veh_tracks()
```
"""
function read_veh_tracks()
    # Read the csv data into tdraw which recasts csv info into a dataframe and also provides two
    # dicts called car2start and frame2cars
    tdraw = INTERACTIONTrajdata("../dataset/vehicle_tracks_000.csv")
    df = tdraw.df
    vehdefs = Dict{Int, VehicleDef}()
    states = Array{RecordState{VehicleState, Int}}(undef, nrow(df))
    frames = Array{RecordFrame}(undef, nframes(tdraw))

    # Initialize the vehicles physically i.e type, length and width
    for (id, dfind) in tdraw.car2start
        # CAUTION: Hardcoding vehicle type to be just car shown by the first argument being 2
        vehdefs[id] = VehicleDef(2, df[dfind, :length], df[dfind, :width])
    end

    # Fill in the vehicle state information in terms of x,y and speed
    state_ind = 0

    for frame in 1 : nframes(tdraw)

        frame_lo = state_ind+1

        for id in carsinframe(tdraw, frame)

            dfind = car_df_index(tdraw, id, frame)

            posG = VecSE2(df[dfind, :x], df[dfind, :y], df[dfind, :psi_rad])
            vx = df[dfind,:vx]
            vy = df[dfind,:vy]
            speed = sqrt(vx*vx + vy*vy)

            states[state_ind += 1] = RecordState(VehicleState(posG, roadway_interaction, speed), id)
        end

        frame_hi = state_ind
        frames[frame] = RecordFrame(frame_lo, frame_hi)
    end
    
    # Trajdata is defined in AutomotiveDrivingModels.jl/states/trajdatas.jl. It is a ListRecord
    # from Records.jl. Its utility is storing a list of scenes
    traj_interaction = Trajdata(INTERACTION_TIMESTEP, frames, states, vehdefs)
    return traj_interaction
end
