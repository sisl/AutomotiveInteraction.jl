using AutomotiveDrivingModels
using Parameters

"""
    MergingEnvironment

Road network with merge point

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

## convenience 
"""
    main_lane(env::MergingEnvironment)
returns the main lane of the merging scenario. Hardcoded to `b1`
"""
main_lane(env::MergingEnvironment) = env.roadway[LaneTag(1, 2)]

"""
    merge_lane(env::MergingEnvironment)
returns the merging lane of the merging scenario. Harcoded to `a`
"""
merge_lane(env::MergingEnvironment) = env.roadway[LaneTag(1, 1)]