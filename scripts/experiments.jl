# This file is used for performing experiments
# Run by changing path in REPL to scripts folder
# Then `include("experiments.jl")`


using AutomotiveSimulator
using AutomotiveInteraction

f = FilteringEnvironment()

 # Obtain this from sample_simultanous_vehicles

# Run vehicles using IDM and get a list of scenes

# Perform filtering to obtain driver models
 
# Run vehicles using obtained driver models

# Get rmse metrics to compare, and get collision metrics

#********Make idm models and see collision metrics
start_frame = 1
id_list = [4,6,8,13,19,28,29]
duration = 10.
scene_real = f.traj[start_frame]
if !isempty(id_list) keep_vehicle_subset!(scene_real,id_list) end

models = make_IDM_models(scene_real)

nticks = Int(ceil(duration/f.timestep))
scene_list = simulate(scene_real,f.roadway,models,nticks,f.timestep)

c_array = test_collision(scene_list,id_list)

truth_list = f.traj[start_frame:start_frame+nticks]
video_overlay_scenelists(scene_list,truth_list,id_list=id_list,roadway=f.roadway,
filename="media/idm_exp.mp4")