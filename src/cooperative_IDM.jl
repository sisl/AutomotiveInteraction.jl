## CooperativeIDM from AutonomousMerging developed by Maxime originally

"""
    CooperativeIDM <: DriverModel{LaneFollowingAccel}
The cooperative IDM (c-IDM) is a rule based driver model for merging scenarios. 
It controls the longitudinal actions of vehicles on the main lane. 
A cooperation level `c` controls how the vehicles reacts to the merging vehicle. 
When `c=0` the vehicle ignores the merging vehicle. When `c=1` the vehicle considers the merging 
vehicle as its front car when TTC_ego > TTC_mergin_vehicle. When it is not considering the merging vehicle, 
the car follows the IntelligentDriverModel.

# Fields 
    - `env::MergingEnvironment = MergingEnvironment(main_lane_angle = 0.0, merge_lane_angle = pi/6)` the merging environment
    - `idm::IntelligentDriverModel = IntelligentDriverModel(v_des = env.main_lane_vmax, d_cmf = 2.0, d_max=2.0, T = 1.5, s_min = 2.0, a_max = 2.0)` the default IDM
    - `c::Float64 = 0.0` the cooperation level
    - `fov::Float64 = 20.0` [m] A field of view, the merging vehicle is not considered if it is further than `fov`
"""
@with_kw mutable struct CooperativeIDM <: DriverModel{LaneFollowingAccel}
    env::MergingEnvironment = MergingEnvironment(merge_point = VecSE2(1064.5227,959.1559,-2.8938))
    idm::IntelligentDriverModel = IntelligentDriverModel(v_des = 15., 
                                                         d_cmf = 2.0, 
                                                         d_max=2.0,
                                                         T = 1.5,
                                                         s_min = 2.0,
                                                         a_max = 2.0)
    c::Float64 = 0.0 # cooperation level
    fov::Float64 = 20.0 # when to consider merge car
    # internals
    a::Float64 = 0.0
    a_merge::Float64 = 0.0
    a_idm::Float64 = 0.0
    other_acc::Float64 = 0.0
    s_des::Float64 = idm.s_min
    dist_at_merge::Float64 = 0.0
    ego_ttm::Float64 = 0.0
    veh_ttm::Float64 = 0.0
    front_car::Bool = false
    consider_merge::Bool = false
end

Base.rand(model::CooperativeIDM) = LaneFollowingAccel(model.a)
function AutomotiveDrivingModels.reset_hidden_state!(model::CooperativeIDM)
    reset_hidden_state!(model.idm)
    model.a = 0.0
    model.a_merge = 0.0
    model.a_idm = 0.0
    model.other_acc = 0.0
    model.s_des = model.idm.s_min
    model.dist_at_merge = 0.0
end

function AutomotiveDrivingModels.set_desired_speed!(model::CooperativeIDM, vdes::Float64)
    set_desired_speed!(model.idm, vdes)
end

function AutomotiveDrivingModels.observe!(model::CooperativeIDM, scene::Scene, roadway::Roadway, egoid::Int64)
    ego_ind = findfirst(egoid, scene)
    print("ego_id = $(egoid) says: OBSERVE COOPERATIVE IDM \n")
    ego = scene[ego_ind]
    fore = get_neighbor_fore_along_lane(scene, ego_ind, roadway, VehicleTargetPointFront(), VehicleTargetPointRear(), VehicleTargetPointFront())
    if fore.ind == nothing # uses the first vehicle on the lain as neighbor

        vehmin, vehind = findfirst_lane(scene, main_lane(model.env))
        headway = get_end(main_lane(model.env)) - ego.state.posF.s + vehmin.state.posF.s
        track_longitudinal!(model.idm, ego.state.v, vehmin.state.v, headway)
        print("No neighbor, ID: %d | neighbor %d | headway %2.1f \n", egoid, scene[vehind].id, headway)
    else
        # @printf("ID: %d | neighbor %d \n", egoid, scene[fore.ind].id)
        observe!(model.idm, scene, roadway, egoid)
    end
    a_idm = model.idm.a
    model.a_idm = a_idm 
    veh = find_merge_vehicle(model.env, scene,ego)
    if veh == nothing || veh.state.posF.s < model.fov
        println("No merge vehicle")
        model.a = model.a_idm
    else
        model.other_acc = 0.0
        model.a = 0.0
        model.a
        ego_ttm = time_to_merge(model.env, ego, model.a)
        veh_ttm = time_to_merge(model.env, veh, model.other_acc)
        model.ego_ttm = ego_ttm
        model.veh_ttm = veh_ttm
        if ( ego_ttm < 0.0 || ego_ttm < veh_ttm || veh_ttm == Inf)
            print("Ego TTM < Merge TTM, ignoring\n")
            ego_ttm < veh_ttm
            model.a = model.a_idm
            model.consider_merge = false
            model.front_car = false
        else
            model.consider_merge = true 
            if veh_ttm < model.c*ego_ttm 
                model.front_car = true
                headway = distance_projection(model.env, veh) - distance_projection(model.env, ego)
                headway -= veh.def.length 
                v_oth = veh.state.v
                v_ego = ego.state.v
                # @show "tracking front car"
                track_longitudinal!(model.idm, v_ego, v_oth, headway)
                model.a_merge = model.idm.a
                model.a = min(model.a_merge, model.a_idm)
            else 
                model.a = model.a_idm 
                model.front_car = false
            end
        end
    end
    return model
end

"""
    findfirst_lane(scene::Scene, lane::Lane)
find the first vehicle on the lane (in terms of longitudinal position)
"""
function findfirst_lane(scene::Scene, lane::Lane)
    s_min = Inf
    vehmin = nothing
    vehind = nothing 
    for (i, veh) in enumerate(scene)
        if veh.state.posF.roadind.tag == lane.tag && veh.state.posF.s < s_min
            s_min = veh.state.posF.s
            vehmin = veh
            vehind = i
        end
    end
    return vehmin, vehind
end

"""
    function get_end(lane::Lane)
- Gets the distance along lane of the last curve point on the lane
"""
function get_end(lane::Lane)
    return lane.curve[end].s
end

"""
    find_merge_vehicle(env::MergingEnvironment, scene::Scene)
returns the id of the merging vehicle if there is a vehicle on the merge lane.
"""
function find_merge_vehicle(env::MergingEnvironment, scene::Scene,ego_veh)
    ego_lane = get_lane(env.roadway,ego_veh)
    ego_lane_tag = ego_lane.tag # Let's use tag as I think it'll be faster to compare equality than entire lane
    merge_lane_tag = LaneTag(0,0)
        # Depending on the ego vehicle lane, decide whether merge lane is a or b1
    if ego_lane_tag == LaneTag(1,1) # Check if ego veh is on lane a
        merge_lane_tag = LaneTag(1,2)
    elseif ego_lane_tag == LaneTag(1,2)
        merge_lane_tag = LaneTag(1,1)
    end

    min_dist_to_merge = 10
    dist_to_merge_threshold = 10
    merge_veh = Vehicle(VehicleState(),VehicleDef(),111)
    for veh in scene
        lane = get_lane(env.roadway, veh)
        if lane.tag == merge_lane_tag
            d = -dist_to_merge(env,veh)
            if d < min_dist_to_merge
                min_dist_to_merge = d
                merge_veh = veh
            end
        end
    end
        # If there is a vehicle less than 10m away from merge point, worry about it otherwise chill
    if min_dist_to_merge < dist_to_merge_threshold
        return merge_veh
    end

    return nothing
end

"""
    time_to_merge(env::MergingEnvironment, veh::Vehicle, a::Float64 = 0.0)
return the time to reach the merge point using constant acceleration prediction. 
If the acceleration, `a` is not specified, it performs a constant velocity prediction.
"""
function time_to_merge(env::MergingEnvironment, veh::Vehicle, a::Float64 = 0.0)
    d = -dist_to_merge(env, veh)
    v = veh.state.v
    t = Inf
    if isapprox(a, 0) 
        t =  d/veh.state.v 
    else
        delta = v^2 + 2.0*a*d
        if delta < 0.0
            t = Inf
        else
            t = (-v + sqrt(delta)) / a 
        end
        if t < 0.0
            t = Inf
        end
    end
    return t
end

"""
    dist_to_merge(env::MergingEnvironment, veh::Vehicle)
returns the distance to the merge point.
"""
function dist_to_merge(env::MergingEnvironment, veh::Vehicle)
    lane = get_lane(env.roadway, veh)
    #if lane == main_lane(env)
    #    frenet_merge = get_frenet_relative_position(veh.state.posG, env.merge_index, env.roadway)
    #    dist = frenet_merge.Δs
    #else
        # No need for if else because lanes inevitably end at the merge point
        dist = veh.state.posF.s - get_end(lane)
    #end
    return dist
end
