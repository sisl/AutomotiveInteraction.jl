# function: get scene from trajdata
"""
    function get_scene(framenum::Int64,traj)

Get a specific scene from trajdata

# Example:
```julia
scene = get_scene(1,int_trajdata)
```
"""
function get_scene(framenum::Int64,traj=traj_interaction)
    scene = Scene(500)
    get!(scene,traj,framenum)
    return scene
end


# function: replay traj_data video
"""
    function video_trajdata_replay

Makes a video of the trajdata taking frame range as input

# Example
```julia
video_trajdata_replay(range=1:100,roadway=roadway,traj=traj_interaction,filename="video.mp4")
```
"""
function video_trajdata_replay(id_list = [];range=nothing,traj=traj_interaction,
	roadway=roadway_interaction, filename="media/interaction_vids/$(range).mp4")

    frames = Frames(MIME("image/png"), fps=10)
    scene = Scene(500)
    for i in range
        temp_scene = get_scene(i,traj)
        if !isempty(id_list) keep_vehicle_subset!(temp_scene,id_list) end
        
        scene_visual = render(temp_scene, 
            roadway,
            #[IDOverlay(colorant"white",12),TextOverlay(text=["frame=$(i)"],font_size=12)],
            #cam=SceneFollowCamera(10.),
            cam=FitToContentCamera(0.),
            #canvas_width = 2250,
        )
        push!(frames,scene_visual)
    end
    write(filename,frames)
    print("Making interaction replay video called $filename")
    
    return nothing
end

# Useful overlays
"""
    my_overlay
Overlaying hallucinated trajectory on the ground truth
# Fields
- `color::Colorant`
- `scene::Scene`
"""
struct my_overlay <: SceneOverlay
    scene::Scene
    color # Needs to be of form colorant"Colorname"
end

function AutoViz.render!(rendermodel::RenderModel,overlay::my_overlay, 
        scene::Scene, roadway::Roadway)
    AutoViz.render!(rendermodel,overlay.scene,car_color = overlay.color)
    return rendermodel
end

"""
    curve_pts_overlay
Displays circles at the curve points that constitute the lanes of a road: 
- color: the color of the blinker
- size: the size of the blinker 
""" 
struct curvepts_overlay <: SceneOverlay
    roadway
    color::Colorant # yellow 
    size::Float64
end

function AutoViz.render!(rendermodel::RenderModel, overlay::curvepts_overlay, 
        scene::Frame{Entity{S,D,I}}, roadway::R) where {S,D,I,R}
    
    
    num_segments = length(roadway.segments)
    
    # Loop over segments
    for seg_i in 1:num_segments
        seg = roadway.segments[seg_i]
        num_lanes = length(seg.lanes)
        
        # Within a segment, loop over lanes
        for lane_i in 1:num_lanes
            lane = seg.lanes[lane_i]
            
            # Within a lane, loop over the curve points
            num_curvepts = length(lane.curve)
            for cpt_i in 1:num_curvepts
                cpt = lane.curve[cpt_i]
                
                add_instruction!(rendermodel,render_circle,
                    (cpt.pos.x,cpt.pos.y,overlay.size,overlay.color))
            end
        end
    end
    
end

# function: make a video from a list of scenes
"""
    function scenelist2video(scene_list;filename = "media/mobil/scene_to_video.mp4")
- Make video from a list of scenes

# Examples
```julia
scene_list = get_hallucination_scenes(scene2,models=models,traj=int_trajdata,roadway=roadway2)
scenelist2video(scene_list,roadway=roadway2,filename="media/interaction_vids/merge.mp4")
```
"""
function scenelist2video(scene_list;id_list=[],
        filename = "media/mobil/scene_to_video.mp4",roadway=roadway_ngsim)
    frames = Frames(MIME("image/png"),fps = 10)
    
    # Loop over list of scenes and convert to video
    for i in 1:length(scene_list)
        if !isempty(id_list) keep_vehicle_subset!(scene_list[i],id_list) end
        scene_visual = render(scene_list[i],roadway,
        [IDOverlay(colorant"white",12)],
        cam=FitToContentCamera(0.),
        #cam = SceneFollowCamera(10.)
        )
        push!(frames,scene_visual)
    end
    print("Making video filename: $(filename)\n")
    write(filename,frames)
    return nothing
end
