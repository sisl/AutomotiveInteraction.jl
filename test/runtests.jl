# Run the stuff in this file by going to package mode (press ] in REPL) and saying `test`
using Revise
using Test
using AutomotiveInteraction
using AutomotiveSimulator
using AutomotiveVisualization
# using Reel

    # Make the roadway both without and with extension, and read vehicle tracks

@testset "driving" begin
roadway_no_ext = make_roadway_interaction();
road_ext = make_roadway_interaction_with_extensions();
traj_ext = read_veh_tracks(roadway=road_ext);

    # Test the replay video making
video_trajdata_replay(range=1:100,trajdata=traj_ext,roadway=road_ext,
filename="media/replay.mp4")

    # Generate model driven traj and compare to ground truth visually
id_list = [6,19,28,29,34,37,40,42,43,49,50]
compare2truth(id_list=id_list,start_frame=101,traj=traj_ext,roadway=road_ext,
filename = "media/compare.mp4")

    # Run a list of vehicles with default param IDM+MOBIL driver models
# run_vehicles(id_list=[29,19,28,6,8,25,2,10,7,18,12],roadway=road_ext,traj=traj_ext,
#     filename=joinpath(@__DIR__,"../julia_notebooks/media/run_test_ext_long.mp4"))

# Do a barrier vehicle experiment
# test_barrier_vehicle(id_list=[20,29,19,28,6,8,25,2,10,7,18,12,100],roadway=road_ext,
#     traj=traj_ext,filename=joinpath(@__DIR__,"../julia_notebooks/media/barrier_test.mp4"))
end

@testset "Filtering" begin
f = FilteringEnvironment()

    # hallucinate_a_step
# scene = f.traj[1]
# particle = [29.,NaN,1.5,5.,0.35,0.1,NaN,1.0]
# hallucinate_a_step(f,scene,particle,car_id=6)

#     # weight and resample
# limits = [10. 40.;0.1 10.;0.5 5.;1. 10.;0. 1.;-1. 1.;0. 20.;0. 1.]
# init_pmat = initial_pmat(limits=limits,num_particles=10,seed=4)
# id = 6
# scene = f.traj[1]
# true_next_scene = deepcopy(f.traj[2])
# true_nextpos = get_frenet_s(true_next_scene;car_id=id)
# true_nextlane = get_lane_id(true_next_scene,id)
# weight_and_resample(f,scene,true_nextpos,true_nextlane,init_pmat,car_id=id)

    # multistep_update
final_p_mat,iterwise_p_mat = multistep_update(f,car_id=6,start_frame=1,last_frame=50,num_p=100);
# Reel.extension(m::MIME"image/svg+xml") = "svg"
# plot_pairwise_particles(iterwise_p_mat,filename="media/particles.gif")

    # imitation trajectory with leaders using replay
f = FilteringEnvironment()
egoids = [28,29]
new_models,final_particles,mean_dist = obtain_driver_models(f,egoids,30,1,30)
start_frame = 1
scene_list_1 = imitation_with_replay(f,new_models,egoids=egoids,replay_ids=[6,8,13],start_frame=start_frame)
scene_list_2 = f.traj[start_frame:start_frame+length(scene_list_1)-1]
video_overlay_scenelists(scene_list_1,scene_list_2,roadway=f.roadway,filename="replay_imit.mp4",id_list=[6,8,13,28,29])
rmse_pos_dict,rmse_vel_dict = compute_rmse(scene_list_1,scene_list_2,id_list=egoids);
end # testset Filtering