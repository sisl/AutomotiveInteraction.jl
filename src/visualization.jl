"""
    function video_trajdata_replay

Makes a video of the trajdata taking frame range as input

# Example
```julia
video_trajdata_replay(range=1:100,roadway=roadway,trajdata=traj_interaction,
    filename=joinpath(@__DIR__,"../julia_notebooks/media/replay_vid.mp4")
```
"""
function video_trajdata_replay(id_list = [];range=nothing,trajdata,roadway,filename)
    AutomotiveVisualization.set_render_mode(:fancy)
    frames = Frames(MIME("image/png"), fps=10)
    mr = MergingRoadway(roadway) # Wrapper for specialized render for merging lanes
    mp = VecE2(1064.5227, 959.1559)
    for i in range
        temp_scene = trajdata[i]
        if !isempty(id_list) keep_vehicle_subset!(temp_scene,id_list) end
        
        scene_visual = render([mr,temp_scene,
                        IDOverlay(scene=temp_scene,font_size=12)],
                        camera=StaticCamera(position=mp,zoom=5.)
                        )
            
        push!(frames,scene_visual)
    end
    write(filename,frames)
    print("Making interaction replay video called $filename")
    
    return nothing
end

# function: make a video from a list of scenes
"""
    function scenelist2video(scene_list;filename = "media/scenelist_to_video.mp4")
- Make video from a list of scenes (generally generated using `get_hallucination_scenes`

# Examples
```julia
# See run_vehicles function in driving_simulation.jl
```
"""
function scenelist2video(scene_list;id_list=[],roadway,filename)
    AutomotiveVisualization.set_render_mode(:fancy)
    frames = Frames(MIME("image/png"),fps = 10)
    mr = MergingRoadway(roadway) # Wrapper for specialized render for merging lanes
    mp = VecE2(1064.5227, 959.1559)

    # Loop over list of scenes and convert to video
    for i in 1:length(scene_list)
        if !isempty(id_list) keep_vehicle_subset!(scene_list[i],id_list) end
        scene_visual = render([mr,scene_list[i],
                        IDOverlay(scene=scene_list[i]),
                        TextOverlay(text=["frame=$(i)"],font_size=12)],
                        camera=StaticCamera(position=mp,zoom=5.)
        )
        push!(frames,scene_visual)
    end
    print("Making video filename: $(filename)\n")
    write(filename,frames)
    return nothing
end

# function: make a video from a list of scenes with curvepts overlayed
"""
    function scenelist2video(scene_list;filename = "media/scenelist_to_video.mp4")
- Make video from a list of scenes (generally generated using `get_hallucination_scenes`

# Examples
```julia
scene = Scene(500)
get!(scene,traj_interaction,1);
scene_list = get_hallucination_scenes(scene,models=models);
scenelist2video(scene_list,filename="media/scenelist_to_video.mp4")
```
"""
function scenelist2video_curvepts(scene_list;id_list=[],roadway,filename)
    frames = Frames(MIME("image/png"),fps = 10)
    
    # Loop over list of scenes and convert to video
    for i in 1:length(scene_list)
        if !isempty(id_list) keep_vehicle_subset!(scene_list[i],id_list) end
        scene_visual = render(scene_list[i],roadway,
        [IDOverlay(),TextOverlay(text=["frame=$(i)"],font_size=12),curvepts_overlay(roadway,colorant"yellow",0.05)],
        cam=FitToContentCamera(0.),
        #cam = SceneFollowCamera(10.)
        )
        push!(frames,scene_visual)
    end
    print("Making video filename: $(filename)\n")
    write(filename,frames)
    return nothing
end


function scenelist2video_mergeoverlay(scene_list;id_list=[],roadway,filename)
    frames = Frames(MIME("image/png"),fps = 10)
    env = MergingEnvironment()
    # Loop over list of scenes and convert to video
    for i in 1:length(scene_list)
        if !isempty(id_list) keep_vehicle_subset!(scene_list[i],id_list) end
        scene_visual = render(scene_list[i],roadway,
        [IDOverlay(),TextOverlay(text=["frame=$(i)"],font_size=12),MergeOverlay(env)],
        cam=FitToContentCamera(0.),
        #cam = SceneFollowCamera(10.)
        )
        push!(frames,scene_visual)
    end
    print("Making video filename: $(filename)\n")
    write(filename,frames)
    return nothing
end