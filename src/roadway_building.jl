"""
	function append_to_curve!
"""
function append_to_curve!(target::Curve, newstuff::Curve)
    s_end = target[end].s
    for c in newstuff
        push!(target, CurvePt(c.pos, c.s+s_end, c.k, c.kd))
    end
    return target
end

"""
    function get_new_angle
- Does the actual angle calculation based on the x y coordinates
"""
function get_new_angle(tangent_vector::Array{Float64})
    # it might be a problem when we switch quadrants
    # use signs of tangent vector to get the quadrant of the heading 
    x = tangent_vector[1]
    y = tangent_vector[2]
    if x == 0. && y == 0.
        heading = 0.0
    elseif x == 0.
        heading = π/2 * sign(y) 
    elseif y == 0.
        heading = convert(Float64, π) # this could be either pi or -pi, but just go with pi
    elseif sign(x) == 1 && sign(y) == 1 # first quadrant
        heading = atan(y, x)
    elseif sign(x) == -1 && sign(y) == 1 # second quadrant
        heading = atan(y, x)
    elseif sign(x) == 1 && sign(y) == -1 # fourth quadrant
        heading = atan(y, x)
    elseif sign(x) == -1 && sign(y) == -1 # third quadrant
        heading = atan(y, x)
    end
    # bound_heading doesn't end up getting called cause Julia takes care of it apparently
    bound_heading(heading)

    return heading
end

"""
    function bound_heading
- Make the angle range from 0 to pi instead of going beyond
"""
function bound_heading(heading::Float64)
    if heading > π # send this to be just near -pi
        heading = -π + (heading - π)    # if heading is 3.15, then the new angle will be (-pi + (3.15-pi)) = -3.13
    elseif heading < -π # send this to be just near pi 
        heading = π + (heading + π)     # if heading is -3.15, then the new angle will be (pi + (-3.15+pi)) = 3.13
    end
    return heading
end

"""
    function append_headings
- Used create the angles and append them into the coordinates

# Examples
```julia
x_coods = [1089.07510, 1093.82626, 1101.19325, 1112.59899, 1123.96733, 1133.24150, 1146.47964]
y_coods = [936.31213, 936.92692,938.52419, 940.93865, 943.27882, 945.21039, 947.88488]
coods = hcat(x_coods,y_coods)
append_headings(coods)
```
"""
function append_headings(coordinates::Matrix{Float64})
    headings = ones(size(coordinates)[1])
    for i = 1:size(coordinates)[1]-1
        # need to make sure that math is right, and that bounds are kept
        tangent_vector = [coordinates[i+1,1]-coordinates[i,1], coordinates[i+1,2]-coordinates[i,2]]
        # @show tangent_vector
        current_heading = get_new_angle(tangent_vector)
        # @show current_heading
        headings[i] = current_heading
    end
    headings[end] = headings[end-1] # assume this is fine
    coordinates = hcat(coordinates, headings)
    return coordinates
end


# function: centerlines_txt2tracks_new
"""
    function centerlines_txt2tracks(filename)
- Reads a .txt file which contains x coords in col 1 and y coords in col2
- Returns a track, i.e. a `curve` from AutomotiveDrivingModels

# Examples
```julia
# See make_roadway_interaction()
```
"""
function centerlines_txt2tracks(filename)
    coods = readdlm(filename,',')
    coods_app = append_headings(coods) # Append with the angle
    
    mid_coods = coods_app'
    
    first_cood = VecSE2(mid_coods[1,1], mid_coods[2,1], mid_coods[3,1])
    second_cood = VecSE2(mid_coods[1,2], mid_coods[2,2], mid_coods[3,2])
    radius = 0.01
    nsamples = 20

    track = gen_bezier_curve(first_cood, second_cood, radius, radius, nsamples)
    
    nsamples = 20
    for i = 3:size(coods,1)
        turn1 = VecSE2(mid_coods[1, i-1], mid_coods[2, i-1], mid_coods[3, i-1])
        turn2 = VecSE2(mid_coods[1, i], mid_coods[2, i], mid_coods[3, i])
        curve = gen_bezier_curve(turn1, turn2, radius, radius, nsamples)
        append_to_curve!(track, curve)
    end

    return track
end


# function: read in the centerline text files and make `roadway_interaction`
"""
    function make_roadway_interaction()
# Overview
- Make the `DR_CHN_Merging` roadway by reading in the centerlines in `AutomotiveInteraction.jl/dataset`
- These centerlines have been modified from the original ones in the `centerlines_DR_CHN_Merging_ZS` folder
- The long lane being merged into by on ramp was split into before and after merge point in 2 separate txt files
- Eg: `output_centerline_1.txt` was split into `centerlines_b1.txt` (before merge point) and `centerlines_b2.txt` (after).
- Similarly, `output_centerline_5.txt` was split into `cetnerlines_g.txt` (before) and `centerlines_h.txt` (after)
- Finally, the files in `centerlines_DR_CHN_Merging_ZS` folder `output_centerline_<>.txt` had x coords in row 1 and y coords in row 2.
- The current `centerlines_<>.txt` have x in col 1 and y in col 2

# Details
- Reads in the centerline information from `AutomotiveInteraction.jl/dataset`
- Segment 1: Has 2 lanes: On ramp `a` is lane 1 and `b1` is lane 2
- Segment 2: Has 1 lane: `b1` is lane 1. Both `a` and `b1` connect into `b2`
- Segment 3: Has 2 lanes: `c` is lane 1 and `d` is lane 2
- Segment 4: Has 3 lanes: `g` is lane 1, `f1` is lane 2, `e1` is lane 3.
- Segment 5: Has 2 lanes: `f2` is lane 1, `e2` is lane 2. `e1` connects to `e2` and both `g` and `f1` to `e2`
    - Note that convention used here is that right most lane in direction of travel is given lane id 1

# Examples
```julia
roadway_interaction = make_roadway_interaction()
```
"""
function make_roadway_interaction()
    road = Roadway()
        # Make segment 1: the on ramp a and first part of lane: b1
    track_a = centerlines_txt2tracks("../dataset/centerlines_a.txt"); # Top most on ramp
    lane_a = Lane(LaneTag(1,1),track_a)

    track_b1 = centerlines_txt2tracks("../dataset/centerlines_b1.txt")
    lane_b1 = Lane(LaneTag(1,2),track_b1)
        
        # Make segment 2: second part of lane: b2. And connect both lanes of segment 1 into segment 2
    track_b2 = centerlines_txt2tracks("../dataset/centerlines_b2.txt")
    lane_b2 = Lane(LaneTag(2,1),track_b2)
    connect!(lane_a,lane_b2)
    connect!(lane_b1,lane_b2)

    push!(road.segments,RoadSegment(1,[lane_a,lane_b1]))
    push!(road.segments,RoadSegment(2,[lane_b2]))

        # Make segment 3: c and d
    track_c = centerlines_txt2tracks("../dataset/centerlines_c.txt")
    lane_c = Lane(LaneTag(3,1),track_c)

    track_d = centerlines_txt2tracks("../dataset/centerlines_d.txt")
    lane_d = Lane(LaneTag(3,2),track_d)

    push!(road.segments,RoadSegment(3,[lane_c,lane_d]))

            # Other side of the divider
        # Make segment 4: g,f1,e1 
    track_g = centerlines_txt2tracks("../dataset/centerlines_g.txt")
    lane_g = Lane(LaneTag(4,1),track_g)

    track_f1 = centerlines_txt2tracks("../dataset/centerlines_f1.txt")
    lane_f1 = Lane(LaneTag(4,2),track_f1)

    track_e1 = centerlines_txt2tracks("../dataset/centerlines_e1.txt")
    lane_e1 = Lane(LaneTag(4,3),track_e1)

        # Make segment 5: f2,e2. And connect 3 lanes of segment 4 into segment 5 two lanes
    track_f2 = centerlines_txt2tracks("../dataset/centerlines_f2.txt")
    lane_f2 = Lane(LaneTag(5,1),track_f2)

    track_e2 = centerlines_txt2tracks("../dataset/centerlines_e2.txt")
    lane_e2 = Lane(LaneTag(5,2),track_e2)
    connect!(lane_e1,lane_e2)
    connect!(lane_f1,lane_f2)
    connect!(lane_g,lane_f2)

    push!(road.segments,RoadSegment(4,[lane_g,lane_f1,lane_e1]))
    push!(road.segments,RoadSegment(5,[lane_f2,lane_e2]))

    return road
end

"""
    function make_roadway_ngsim()

- Read in the ngsim 101 roadway from the provided roadway file `../dataset/ngsim_101.txt`

# Examples
```julia
roadway_ngsim = make_roadway_ngsim()
```
"""
function make_roadway_ngsim()
    roadway_ngsim = open(io->read(io, MIME"text/plain"(), Roadway),"../dataset/ngsim_101.txt", "r")
    return roadway_ngsim
end
