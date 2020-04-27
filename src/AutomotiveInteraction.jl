# This is the module file. It allows communication between different files within the src folder
# Also, determines which functions are provided by AutomotiveInteraction whenever someone says
# using AutomotiveInteraction
module AutomotiveInteraction

using AutomotiveSimulator
using AutomotiveVisualization
using Reel

using DataFrames
using DelimitedFiles
using Parameters
using Random # For AbstractRNG in Base.rand for c-idm

using Distributions # For pdf(Normal) in weight_and_resample
using StatsBase # For weight_and_resample
using LinearAlgebra # Form norm calculation for iterwise particle dist

using Combinatorics # make param pairs for filtering progress pairwise plots
using PGFPlots

export
    MergingEnvironment,
    MergingEnvironmentLower,
    FilteringEnvironment
include("Driving/environment.jl")

export 
    keep_vehicle_subset!,
    make_def_models,
    make_IDM_models,
    make_cidm_models,
    make_TimLaneChanger_models,
    get_hallucination_scenes,
    run_vehicles,
    compare2truth,
    run_vehicles_curvept_overlay,
    test_barrier_vehicle,
    test_jumpy_vehicle
include("Driving/driving_simulation.jl")

export
    make_roadway_interaction,
    make_roadway_ngsim,
    make_roadway_interaction_with_extensions,
    make_discont_roadway_straight,
    make_discont_roadway_jagged,
    MergingRoadway,
    add_renderable!
include("Data_Processing/roadway_building.jl")

export 
    INTERACTIONTrajdata,
    carsinframe,
    car_df_index,
    read_veh_tracks,
    extract_timespan,
    sample_simultaneous_vehs
include("Data_Processing/veh_track_reading.jl")

export
    video_trajdata_replay,
    scenelist2video,
    video_overlay_scenelists,
    curvepts_overlay,
    LaneOverlay,
    scenelist2video_curvepts
include("Driving/visualization.jl")

export
    CooperativeIDM,
    find_merge_vehicle
include("Driving/cooperative_IDM.jl")

export
    MergeOverlay
include("Driving/overlays.jl")

export
    hallucinate_a_step,
    weight_and_resample,
    multistep_update,
    obtain_driver_models
include("Filtering/particle_filtering.jl")

export
    initial_pmat,
    plot_pairwise_particles,
    gen_imitation_traj,
    imitation_with_replay,
    compute_rmse,
    rmse_dict2mean,
    collision_check,
    test_collision
include("Filtering/utils.jl")

end # End module
