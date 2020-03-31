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

    # Test the replay video making
video_trajdata_replay(range=1:100,trajdata=traj_ext,roadway=road_ext,filename="test.mp4")

    # Run a list of vehicles with default param IDM+MOBIL driver models
# run_vehicles(id_list=[29,19,28,6,8,25,2,10,7,18,12],roadway=road_ext,traj=traj_ext,
#     filename=joinpath(@__DIR__,"../julia_notebooks/media/run_test_ext_long.mp4"))

# Do a barrier vehicle experiment
# test_barrier_vehicle(id_list=[20,29,19,28,6,8,25,2,10,7,18,12,100],roadway=road_ext,
#     traj=traj_ext,filename=joinpath(@__DIR__,"../julia_notebooks/media/barrier_test.mp4"))

    # Generate model driven traj and compare to ground truth visually
id_list = [6,8,19,28,29]
scene_list_1 = run_vehicles(id_list=id_list,roadway=road_ext,traj=traj_ext,filename="model_driven.mp4")
scene_list_2 = traj_ext[1:length(scene_list_1)]
video_overlay_scenes(scene_list_1,scene_list_2,id_list=id_list,roadway=road_ext,filename="model_vs_truth.mp4")


    # Create the merging environment
env_interaction = MergingEnvironment(merge_point = VecSE2(1064.5227,959.1559,-2.8938))