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
- Eg: `output_centerline_1.txt` has been split into `centerlines_b1.txt` (before merge point) and `centerlines_b2.txt` (after).
- Similarly, `output_centerline_5.txt` has been split into `cetnerlines_g.txt` (before) and `centerlines_h.txt` (after)
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
    track_a = centerlines_txt2tracks(joinpath(@__DIR__,"../dataset/centerlines_a.txt")); # Top most on ramp
    lane_a = Lane(LaneTag(1,1),track_a,boundary_left=LaneBoundary(:broken,:white))

    track_b1 = centerlines_txt2tracks(joinpath(@__DIR__,"../dataset/centerlines_b1.txt")); # Top most on ramp
    lane_b1 = Lane(LaneTag(1,2),track_b1)

        # Make segment 2: second part of lane: b2. And connect both lanes of segment 1 into segment 2
    track_b2 = centerlines_txt2tracks(joinpath(@__DIR__,"../dataset/centerlines_b2.txt"))
    lane_b2 = Lane(LaneTag(2,1),track_b2)
    connect!(lane_a,lane_b2)
    connect!(lane_b1,lane_b2)

    push!(road.segments,RoadSegment(1,[lane_a,lane_b1]))
    push!(road.segments,RoadSegment(2,[lane_b2]))

        # Make segment 3: c and d
    track_c = centerlines_txt2tracks(joinpath(@__DIR__,"../dataset/centerlines_c.txt"))
    lane_c = Lane(LaneTag(3,1),track_c,boundary_left=LaneBoundary(:broken,:white))

    track_d = centerlines_txt2tracks(joinpath(@__DIR__,"../dataset/centerlines_d.txt"))
    lane_d = Lane(LaneTag(3,2),track_d)

    push!(road.segments,RoadSegment(3,[lane_c,lane_d]))

            # Other side of the divider
        # Make segment 4: g,f1,e1
    track_g = centerlines_txt2tracks(joinpath(@__DIR__,"../dataset/centerlines_g.txt"))
    lane_g = Lane(LaneTag(4,1),track_g,boundary_left=LaneBoundary(:broken,:white))

    track_f1 = centerlines_txt2tracks(joinpath(@__DIR__,"../dataset/centerlines_f1.txt"))
    lane_f1 = Lane(LaneTag(4,2),track_f1,boundary_left=LaneBoundary(:broken,:white))

    track_e1 = centerlines_txt2tracks(joinpath(@__DIR__,"../dataset/centerlines_e1.txt"))
    lane_e1 = Lane(LaneTag(4,3),track_e1)

        # Make segment 5: f2,e2. And connect 3 lanes of segment 4 into segment 5 two lanes
    track_f2 = centerlines_txt2tracks(joinpath(@__DIR__,"../dataset/centerlines_f2.txt"))
    lane_f2 = Lane(LaneTag(5,1),track_f2,boundary_left=LaneBoundary(:broken,:white))

    track_e2 = centerlines_txt2tracks(joinpath(@__DIR__,"../dataset/centerlines_e2.txt"))
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
    roadway_ngsim = open(io->read(io, MIME"text/plain"(), Roadway),joinpath(@__DIR__,"dataset/ngsim_101.txt"), "r")
    return roadway_ngsim
end

# function: To create road extensions
"""
- Takes a CurvePt `c` and a distance `l`
- Returns a curve of length `l` starting at `c` and preserving its angle

# Examples
```julia
c_new = straight_extension(c)
track_extension = gen_straight_segment(c,c_new)
append_to_curve(track_orig,track_extension)
```
"""
function straight_extension(c::CurvePt,l::Float64)
    x1 = c.pos.x
    y1 = c.pos.y
    t = c.pos.θ
    x2 = x1+l*cos(t)
    y2 = y1+l*sin(t)
    return CurvePt(VecSE2(x2,y2,t),c.s+l)
    #return gen_straight_curve(VecE2(x1,y1),VecE2(x2,y2),2)
end

# function: Take a track and straight extension it
"""
    function extend_track_straight!(track,l::Float64)
- Take a `track` and length `l` and extend straight after end

# Examples
```julia
extended_track_e2 = extend_track_straight(track_e2,50.)
```
"""
function extend_track_straight!(track,l::Float64)
    c_ext = straight_extension(track[end],l)
    push!(track,c_ext)
    return track
end

# function: roadway_interaction with straight extensions to avoid veh reversal
"""
- Make roadway interaction but with added extensions to avoid vehicle getting blocked at road end
"""
function make_roadway_interaction_with_extensions()
    road = Roadway()
        # Make segment 1: the on ramp a and first part of lane: b1
    track_a = centerlines_txt2tracks(joinpath(@__DIR__,"../dataset/centerlines_a.txt")); # Top most on ramp
    lane_a = Lane(LaneTag(1,1),track_a,boundary_left=LaneBoundary(:broken,:white))

    track_b1 = centerlines_txt2tracks(joinpath(@__DIR__,"../dataset/centerlines_b1.txt")); # Top most on ramp
    lane_b1 = Lane(LaneTag(1,2),track_b1)

        # Make segment 2: second part of lane: b2. And connect both lanes of segment 1 into segment 2
    track_b2_unext = centerlines_txt2tracks(joinpath(@__DIR__,"../dataset/centerlines_b2.txt"))
    track_b2 = extend_track_straight!(track_b2_unext,100.)
    lane_b2 = Lane(LaneTag(2,1),track_b2)
    connect!(lane_a,lane_b2)
    connect!(lane_b1,lane_b2)

    push!(road.segments,RoadSegment(1,[lane_a,lane_b1]))
    push!(road.segments,RoadSegment(2,[lane_b2]))

        # Make segment 3: c and d
    track_c_unext = centerlines_txt2tracks(joinpath(@__DIR__,"../dataset/centerlines_c.txt"))
    track_c = extend_track_straight!(track_c_unext,100.)
    lane_c = Lane(LaneTag(3,1),track_c,boundary_left=LaneBoundary(:broken,:white))

    track_d_unext = centerlines_txt2tracks(joinpath(@__DIR__,"../dataset/centerlines_d.txt"))
    track_d = extend_track_straight!(track_d_unext,100.)
    lane_d = Lane(LaneTag(3,2),track_d)

    push!(road.segments,RoadSegment(3,[lane_c,lane_d]))

            # Other side of the divider
        # Make segment 4: g,f1,e1
    track_g = centerlines_txt2tracks(joinpath(@__DIR__,"../dataset/centerlines_g.txt"))
    lane_g = Lane(LaneTag(4,1),track_g,boundary_left=LaneBoundary(:broken,:white))

    track_f1 = centerlines_txt2tracks(joinpath(@__DIR__,"../dataset/centerlines_f1.txt"))
    lane_f1 = Lane(LaneTag(4,2),track_f1,boundary_left=LaneBoundary(:broken,:white))

    track_e1 = centerlines_txt2tracks(joinpath(@__DIR__,"../dataset/centerlines_e1.txt"))
    lane_e1 = Lane(LaneTag(4,3),track_e1)

        # Make segment 5: f2,e2. And connect 3 lanes of segment 4 into segment 5 two lanes
    track_f2_unext = centerlines_txt2tracks(joinpath(@__DIR__,"../dataset/centerlines_f2.txt"))
    track_f2 = extend_track_straight!(track_f2_unext,100.)
    lane_f2 = Lane(LaneTag(5,1),track_f2,boundary_left=LaneBoundary(:broken,:white))

    track_e2_unext = centerlines_txt2tracks(joinpath(@__DIR__,"../dataset/centerlines_e2.txt"))
    track_e2 = extend_track_straight!(track_e2_unext,100.)
    lane_e2 = Lane(LaneTag(5,2),track_e2)
    connect!(lane_e1,lane_e2)
    connect!(lane_f1,lane_f2)
    connect!(lane_g,lane_f2)

    push!(road.segments,RoadSegment(4,[lane_g,lane_f1,lane_e1]))
    push!(road.segments,RoadSegment(5,[lane_f2,lane_e2]))

    return road
end

# function: Build roadways to test jumpy behavior in presence of discontinuity
"""
    function make_discont_roadway(;segment_length::Float64=20., separation::Float64=2.)
- Intended to test whether jumpy behavior is shown by vehicle

# Used by
`test_jumpy_vehicle`
"""
function make_discont_roadway_straight(;segment_length::Float64=20., separation::Float64=2.)
    track_a = gen_straight_curve(VecE2(0.,0.),VecE2(segment_length,0.),2)
    track_b = gen_straight_curve(VecE2(segment_length+separation,0.),VecE2(segment_length+separation+segment_length,0.),2)
    lane_a = Lane(LaneTag(1,1),track_a)
    lane_b = Lane(LaneTag(2,1),track_b)
    connect!(lane_a,lane_b)
    road = Roadway()
    push!(road.segments,RoadSegment(1,[lane_a]))
    push!(road.segments,RoadSegment(2,[lane_b]))
    return road
end

"""
    function make_discont_roadway_jagged(;segment_length::Float64=20., separation::Float64=2.)

- road with two horizontal segments but seperated in y to create a zig zag

# Examples
```julia
road_zigzag = make_discont_roadway_jagged()
```
"""
function make_discont_roadway_jagged(;segment_length::Float64=20., separation::Float64=2.)
    track_a = gen_straight_curve(VecE2(0.,0.),VecE2(segment_length,0.),2)
    track_b = gen_straight_curve(VecE2(segment_length+separation,10.),VecE2(segment_length+separation+segment_length,10.),2)
    lane_a = Lane(LaneTag(1,1),track_a)
    lane_b = Lane(LaneTag(2,1),track_b)
    connect!(lane_a,lane_b)
    road = Roadway()
    push!(road.segments,RoadSegment(1,[lane_a]))
    push!(road.segments,RoadSegment(2,[lane_b]))
    return road
end

# Function: highlight lanes by overlaying colors
"""
    function show_lane_overlays(road_ext::Roadway,traj_ext)
- Draw lane colored overlays on the road_ext

# Examples
```julia
show_lane_overlays(road_ext,traj_ext)
```
"""
function show_lane_overlays(road_ext::Roadway,traj_ext)
    lo_a = LaneOverlay(road_ext[LaneTag(1,1)],RGBA(0.,0.,1.,0.5))
    lo_b1 = LaneOverlay(road_ext[LaneTag(1,2)],RGBA(1.,0.,0.,0.5))
    lo_b2 = LaneOverlay(road_ext[LaneTag(2,1)],RGBA(0.,1.,0.,0.5))
    
    lo_c = LaneOverlay(road_ext[LaneTag(3,1)],RGBA(1.,1.,0.,0.5))
    lo_d = LaneOverlay(road_ext[LaneTag(3,2)],RGBA(0.,1.,1.,0.5))

    scene = traj_ext[1]
    return render(scene,road_ext,[lo_a,lo_b1,lo_b2,lo_c,lo_d])
end

# Special rendering to avoid lane boundary go till end
"""
struct MergingRoadway
- Wrapper around a roadway for special rendering avoiding lane boundary wierd in merge

# Example
```julia
road_ext = make_roadway_interaction_with_extensions()
roadway = MergingRoadway(road_ext)
render([roadway])
```
"""
struct MergingRoadway
    roadway::Roadway
end

function AutomotiveVisualization.add_renderable!(rendermodel::RenderModel,
    mr::MergingRoadway)
    #print("This is the (still code copied) rendering for merging lane boundary")
    lane_marking_width=0.15
    lane_dash_len=1.0
    lane_dash_spacing=2.0
    lane_dash_offset=0.00
    color_asphalt = colorant"0x708090"
    roadway = mr.roadway

    # render the asphalt between the leftmost and rightmost lane markers
    for seg in roadway.segments
        if !isempty(seg.lanes)
            laneR = seg.lanes[1]
            laneL = seg.lanes[end]

            pts = Array{Float64}(undef, 2, length(laneL.curve) + has_next(laneL) +
                                        length(laneR.curve) + has_next(laneR) +
                                        2*length(seg.lanes))
            pts_index = 0
            for pt in laneL.curve
                edgept = pt.pos + polar(laneL.width/2, pt.pos.θ + π/2)
                pts_index += 1
                pts[1, pts_index] = edgept.x
                pts[2, pts_index] = edgept.y
            end
            if has_next(laneL)
                pt = next_lane_point(laneL, roadway)
                edgept = pt.pos + polar(laneL.width/2, pt.pos.θ + π/2)
                pts_index += 1
                pts[1, pts_index] = edgept.x
                pts[2, pts_index] = edgept.y
            end
            for i in reverse(1:length(seg.lanes))
                lane = seg.lanes[i]
                if has_next(lane)
                    pt = next_lane_point(lane, roadway).pos
                else
                    pt = lane.curve[end].pos
                end
                pts_index += 1
                pts[1, pts_index] = pt.x
                pts[2, pts_index] = pt.y
            end

            if has_next(laneR)
                pt = next_lane_point(laneR, roadway)
                edgept = pt.pos + polar(laneR.width/2, pt.pos.θ - π/2)
                pts_index += 1
                pts[1, pts_index] = edgept.x
                pts[2, pts_index] = edgept.y
            end
            for j in length(laneR.curve) : -1 : 1
                pt = laneR.curve[j]
                edgept = pt.pos + polar(laneR.width/2, pt.pos.θ - π/2)
                pts_index += 1
                pts[1, pts_index] = edgept.x
                pts[2, pts_index] = edgept.y
            end
            for i in 1:length(seg.lanes)
                lane = seg.lanes[i]
                pt = lane.curve[1].pos
                pts_index += 1
                pts[1, pts_index] = pt.x
                pts[2, pts_index] = pt.y
            end

            add_instruction!(rendermodel, render_fill_region, (pts, color_asphalt))
        end
        # for lane in seg.lanes
        #     render!(rendermodel, lane, roadway)
        # end
    end

    # render the lane edges
    for seg in roadway.segments
        for lane in seg.lanes
            if lane.tag == LaneTag(1,1)
                # This is the top merge lane i.e. lane a. Make dashed line cut short
                curve_tweak = lane.curve[1:end-40]
                N = length(curve_tweak)
                halfwidth = lane.width/2

                pts_left = Array{Float64}(undef, 2, N)

                for (i,pt) in enumerate(curve_tweak)
                    p_left = pt.pos + polar(halfwidth, pt.pos.θ + π/2)
                    pts_left[1,i] = p_left.x
                    pts_left[2,i] = p_left.y
                end

                add_renderable!(rendermodel, lane.boundary_left, pts_left, lane_marking_width, lane_dash_len, lane_dash_spacing, lane_dash_offset)

            elseif lane.tag == LaneTag(4,1)
                # This is the bottom merge lane i.e. lane g maybe. Make dashed line cut short
                curve_tweak = lane.curve[1:end-80]
                N = length(curve_tweak)
                halfwidth = lane.width/2

                pts_left = Array{Float64}(undef, 2, N)

                for (i,pt) in enumerate(curve_tweak)
                    p_left = pt.pos + polar(halfwidth, pt.pos.θ + π/2)
                    pts_left[1,i] = p_left.x
                    pts_left[2,i] = p_left.y
                end

                add_renderable!(rendermodel, lane.boundary_left, pts_left, lane_marking_width, lane_dash_len, lane_dash_spacing, lane_dash_offset)

            else
                
                N = length(lane.curve)
                halfwidth = lane.width/2

                # always render the left lane marking
                pts_left = Array{Float64}(undef, 2, N)
                
                for (i,pt) in enumerate(lane.curve)
                    
                    p_left = pt.pos + polar(halfwidth, pt.pos.θ + π/2)

                    pts_left[1,i] = p_left.x
                    pts_left[2,i] = p_left.y
                end
                if has_next(lane)
                    lane2 = next_lane(lane, roadway)
                    pt = lane2.curve[1]
                    p_left = pt.pos + polar(lane2.width/2, pt.pos.θ + π/2)
                    pts_left = hcat(pts_left, [p_left.x, p_left.y])
                end

                add_renderable!(rendermodel, lane.boundary_left, pts_left, lane_marking_width, lane_dash_len, lane_dash_spacing, lane_dash_offset)

                # only render the right lane marking if this is the first lane
                if lane.tag.lane == 1
                    pts_right = Array{Float64}(undef, 2, N)

                    for (i,pt) in enumerate(lane.curve)
                    
                        p_right = pt.pos - polar(halfwidth, pt.pos.θ + π/2)

                        pts_right[1,i] = p_right.x
                        pts_right[2,i] = p_right.y
                    end

                    if has_next(lane)
                        lane2 = next_lane(lane, roadway)
                        pt = lane2.curve[1]
                        p_right = pt.pos - polar(lane2.width/2, pt.pos.θ + π/2)
                        pts_right = hcat(pts_right, [p_right.x, p_right.y])
                    end

                    add_renderable!(rendermodel, lane.boundary_right, pts_right, lane_marking_width, lane_dash_len, lane_dash_spacing, lane_dash_offset)
                end
            end # Ends the if else to check whether merging lane
        end
    end

    return rendermodel
end
