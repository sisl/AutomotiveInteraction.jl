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
- color: the color of the point
- size: the size of the point

# Examples
```julia
render(scene,road,[curvepts_overlay(roadway_ext,colorant"yellow",0.05)])
```
""" 
struct curvepts_overlay <: SceneOverlay
    roadway
    color::Colorant # eg: colorant"yellow"
    size::Float64
end

function AutoViz.render!(rendermodel::RenderModel, overlay::curvepts_overlay, scene::Frame{Entity{S,D,I}}, roadway::R) where {S,D,I,R}
    
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

"""
    LaneOverlay

- Intended to debug lanes and how vehicle 4 is teleporting from lane a to lane b2

# Examples
```julia
lane_overlay = LaneOverlay(roadway[LaneTag(1,1)], RGBA(0.0,0.0,1.0,0.5))
render(scene, roadway, [lane_overlay], cam=FitToContentCamera(0.))
```
"""
struct LaneOverlay <: SceneOverlay
    lane::Lane
    color::Colorant
end

function AutoViz.render!(rendermodel::RenderModel, overlay::LaneOverlay, scene::Scene, roadway::Roadway)
    render!(rendermodel, overlay.lane, roadway, color_asphalt=overlay.color) # this display a lane with the specified color
    return rendermodel
end