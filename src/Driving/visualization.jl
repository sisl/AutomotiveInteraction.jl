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
    frames = Frames(MIME("image/png"), fps=10)
    mr = MergingRoadway(roadway) # Wrapper for specialized render for merging lanes
    mp = VecE2(1064.5227, 959.1559)
    for i in range
        temp_scene = trajdata[i]
        if !isempty(id_list) keep_vehicle_subset!(temp_scene,id_list) end
        renderables = [
            mr,
            (FancyCar(car=temp_scene[j]) for j in 1:length(temp_scene))...,
            IDOverlay(scene=temp_scene),
            TextOverlay(text=["frame=$(i)"],font_size=12)
        ]
        scene_visual = render(renderables,camera=StaticCamera(position=mp,zoom=5.))
            
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

"""
function video_overlay_scenelists(scene_list_1,scene_list_2,roadway,filename)

# Example
```julia
scene_list_1 = run_vehicles(id_list=[6,8,19,28,29],roadway=road_ext,traj=traj_ext,
filename="model_driven.mp4")
scene_list_2 = traj_ext[1:length(scene_list_1)]
video_overlay_scenes(scene_list_1,scene_list_2,id_list=[6,8,19,28,29],
roadway=road_ext,filename="model_vs_truth.mp4")
```
"""
function video_overlay_scenelists(scene_list_1,scene_list_2;
    id_list=[],roadway,filename)
    @assert length(scene_list_1) == length(scene_list_2)
    frames = Frames(MIME("image/png"),fps = 10)
    mr = MergingRoadway(roadway) # Wrapper for specialized render for merging lanes
    mp = VecE2(1064.5227, 959.1559)

    # Loop over list of scenes and convert to video
    for i in 1:length(scene_list_1)
        if !isempty(id_list) keep_vehicle_subset!(scene_list_1[i],id_list) end
        if !isempty(id_list) keep_vehicle_subset!(scene_list_2[i],id_list) end
        scene1 = scene_list_1[i]
        scene2 = scene_list_2[i]

        # By default, all the cars are assigned random colors
        # Explicitly color code the cars to have only 2 colors
        renderables = [
            mr,
            (FancyCar(car=scene1[j],color=colorant"blue") for j in 1:length(scene1))...,
            IDOverlay(scene=scene1),
            (FancyCar(car=scene2[j],color=colorant"red") for j in 1:length(scene2))...,
            IDOverlay(scene=scene2),
            TextOverlay(text=["frame=$(i)"],font_size=12)
        ]
        scene_visual = render(renderables,camera=StaticCamera(position=mp,zoom=5.))
        push!(frames,scene_visual)
    end
    print("video_overlay_scenelists: List 1 in blue, list 2 in red. Filename: $(filename)\n")
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
