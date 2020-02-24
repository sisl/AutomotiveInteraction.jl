using Revise
using Test
using AutomotiveInteraction
using AutomotiveDrivingModels
using AutoViz

roadway_no_ext = make_roadway_interaction()
road_ext = make_roadway_interaction_with_extensions()
traj_ext = read_veh_tracks(roadway=road_ext)
run_vehicles(id_list=[29,19,28,6,8,25,2,10,7,18,12],roadway=road_ext,traj=traj_ext,
    filename=joinpath(@__DIR__,"../julia_notebooks/media/run_test_ext_long.mp4"))
#render([roadway],camera=StaticCamera(position = VecE2(1000.,1000.)))
