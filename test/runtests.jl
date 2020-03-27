# Run the stuff in this file by going to package mode (press ] in REPL) and saying `test`

using Revise
using Test
using AutomotiveInteraction
using AutomotiveSimulator
using AutomotiveVisualization

    # Make the roadway both without and with extension, and read vehicle tracks
roadway_no_ext = make_roadway_interaction();
road_ext = make_roadway_interaction_with_extensions();
traj_ext = read_veh_tracks(roadway=road_ext);

    # Run a list of vehicles with default param IDM+MOBIL driver models
# run_vehicles(id_list=[29,19,28,6,8,25,2,10,7,18,12],roadway=road_ext,traj=traj_ext,
#     filename=joinpath(@__DIR__,"../julia_notebooks/media/run_test_ext_long.mp4"))

    # In prep for AutoViz v0.8
#render([roadway],camera=StaticCamera(position = VecE2(1000.,1000.)))

# Do a barrier vehicle experiment
# test_barrier_vehicle(id_list=[20,29,19,28,6,8,25,2,10,7,18,12,100],roadway=road_ext,
#     traj=traj_ext,filename=joinpath(@__DIR__,"../julia_notebooks/media/barrier_test.mp4"))

# Do a merge experiment
run_vehicles(id_list=[8,6],roadway=road_ext,traj=traj_ext,
    filename=joinpath(@__DIR__,"../julia_notebooks/media/run_merge_test_idmdebug.mp4"))

    # Create the merging environment
env_interaction = MergingEnvironment(merge_point = VecSE2(1064.5227,959.1559,-2.8938))