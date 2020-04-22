# This file is used for performing experiments
# Run by changing path in REPL to scripts folder
# Then `include("experiments.jl")`

using AutomotiveSimulator
using AutomotiveInteraction
using PGFPlots

include("helpers.jl")

f = FilteringEnvironment()
#veh_id_list = [4,6,8,13,19,28,29];ts=1
veh_id_list = [34,37,40,42,43,49,50];ts=88

print("Get the metrics for particle filering\n")
pos_pf,vel_pf,collisions_pf = extract_metrics(f,ts=ts,id_list=veh_id_list,
filename="media/pf_ts_88.mp4")

print("Get the metrics for c-idm\n")
pos_cidm,vel_cidm,collisions_cidm = extract_metrics(f,ts=ts,id_list=veh_id_list,
modelmaker=make_cidm_models,filename="media/cidm_ts_88.mp4")

print("Get the metrics for idm\n")
pos_idm,vel_idm,collisions_idm = extract_metrics(f,ts=ts,id_list=veh_id_list,
modelmaker=make_IDM_models,filename="media/idm_ts_88.mp4")

make_plots=true
if(make_plots)
    # Make rmse_pos plot
    p_pos_pf = pgfplot_vector(pos_pf,leg="pf");
    p_pos_cidm = pgfplot_vector(pos_cidm,leg="cidm");
    p_pos_idm = pgfplot_vector(pos_idm,leg="idm");
    ax_rmse_pos = PGFPlots.Axis([p_pos_pf,p_pos_cidm,p_pos_idm],
            xlabel="timestep",ylabel="rmse pos",title="rmse pos: c-IDM vs IDM");
    PGFPlots.save("media/rmse_pos.pdf",ax_rmse_pos);

    # Make collisions plot
    coll_pf = pgfplot_vector(collisions_pf,leg="pf");
    coll_idm = pgfplot_vector(collisions_cidm,leg="cidm");
    coll_cidm = pgfplot_vector(collisions_idm,leg="idm");
    ax_coll = PGFPlots.Axis([coll_pf,coll_cidm,coll_idm],
            xlabel="timestep",ylabel="iscolliding",
            title="Collision assessment: c-IDM vs IDM");
    PGFPlots.save("media/coll.pdf",ax_coll)
end