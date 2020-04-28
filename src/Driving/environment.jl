"""
environment.jl

This file defines the structs that capture information about
the experiment such as `FilteringEnvironment`,
`MergingEnvironment` and `MergingEnvironmentLower`
"""

# The upper merge
"""
    MergingEnvironment

Road network with merge point. This is for the upper merge i.e. a and b1

# Internals 

- `roadway::Roadway{Float64}` contains all the road segment and lane information
- `merge_point::VecSE2{Float64}` coordinate of the merge point in cartesian frame (0.0, 0.0, 0.0) by default
- `merge_proj::RoadProjection{Int64, Float64}` projection of the merge point on the roadway 
- `merge_index::RoadIndex`

# Example: Caution
- If you use all default values while constructing, does not work, so I provide merge_point explicitly
```julia
env_interaction = MergingEnvironment(merge_point = VecSE2(1064.5227,959.1559,-2.8938))
```
"""
@with_kw struct MergingEnvironment
    roadway::Roadway{Float64} = make_roadway_interaction_with_extensions()
    merge_point::VecSE2{Float64} = VecSE2(1064.5227, 959.1559, -2.8938)
    merge_proj::RoadProjection{Int64, Float64} = proj(merge_point, roadway)
    merge_index::RoadIndex{Int64, Float64} = RoadIndex(merge_proj.curveproj.ind, merge_proj.tag)
end

"""
    main_lane(env::MergingEnvironment)
returns the main lane of the merging scenario. Hardcoded to `b1`
"""
main_lane(env::MergingEnvironment) = env.roadway[LaneTag(1, 2)]
main_lane_tag(env::MergingEnvironment) = LaneTag(1,2)

"""
    merge_lane(env::MergingEnvironment)
returns the merging lane of the merging scenario. Harcoded to `a`
"""
merge_lane(env::MergingEnvironment) = env.roadway[LaneTag(1, 1)]
merging_lane_tag(env::MergingEnvironment) = LaneTag(1,1)


# Lower merge environment
"""
struct MergingEnvironmentLower
Environment for the lower merge i.e. road segment 4, lane tag 1 and lane tag 2

# Example
```julia
c = CooperativeIDM(env=MergingEnviornmentLower())
```
"""
@with_kw struct MergingEnvironmentLower
    roadway::Roadway{Float64} = make_roadway_interaction_with_extensions()
    merge_point::VecSE2{Float64} = VecSE2(1110.827, 944.598, 0.214)
    merge_proj::RoadProjection{Int64, Float64} = proj(merge_point, roadway)
    merge_index::RoadIndex{Int64, Float64} = RoadIndex(merge_proj.curveproj.ind, merge_proj.tag)
end

"""
    main_lane(env::MergingEnvironmentLower)
returns the main lane of the merging scenario. Hardcoded to `f1`
"""
main_lane(env::MergingEnvironmentLower) = env.roadway[LaneTag(4, 2)]

"""
main_lane_tag(env::MergingEnvironmentLower) = LaneTag(4,2)

Returns the lane tag of the main lane i.e. f1
"""
main_lane_tag(env::MergingEnvironmentLower) = LaneTag(4,2)

"""
    merge_lane(env::MergingEnvironmentLower)
returns the merging lane of the merging scenario. Harcoded to `g`
"""
merge_lane(env::MergingEnvironmentLower) = env.roadway[LaneTag(4, 1)]

"""
merge_lane_tag(env::MergingEnvironmentLower) = LaneTag(4,1)

Returns the lane tag of the merging lane. Used by the `find_merge_vehicle` method
in cooperative IDM
"""
merging_lane_tag(env::MergingEnvironmentLower) = LaneTag(4,1)

# struct: FilteringEnvironment
"""
@with_kw struct FilteringEnvironment

- Contains the roadway and trajectory information 
- To enable not having to pass roadway and traj as input args to all functions

# Fields
- roadway
- traj
- timestep
- `mergeenv`: Upper merging section (`MergingEnvironment`) vs lower (`MergingEnvironmentLower`)

# Example
```julia
f = FilteringEnvironment()
```
"""
@with_kw struct FilteringEnvironment
    roadway::Roadway{Float64} = make_roadway_interaction_with_extensions()
    traj = read_veh_tracks(roadway=roadway)
    timestep::Float64 = 0.1
    mergeenv = MergingEnvironment()
end
