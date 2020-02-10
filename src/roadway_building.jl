"""
	function append_to_curve!
"""
function append_to_curve!(target::Curve, newstuff::Curve)
    s_end = target[end].s
    for c in newstuff
        push!(target, CurvePt(c.pos, c.s+s_end, c.k, c.kd))
    end
    return target
end



"""
    function get_new_angle
- Does the actual angle calculation based on the x y coordinates
"""
function get_new_angle(tangent_vector::Array{Float64})
    # it might be a problem when we switch quadrants
    # use signs of tangent vector to get the quadrant of the heading 
    x = tangent_vector[1]
    y = tangent_vector[2]
    if x == 0. && y == 0.
        heading = 0.0
    elseif x == 0.
        heading = π/2 * sign(y) 
    elseif y == 0.
        heading = convert(Float64, π) # this could be either pi or -pi, but just go with pi
    elseif sign(x) == 1 && sign(y) == 1 # first quadrant
        heading = atan(y, x)
    elseif sign(x) == -1 && sign(y) == 1 # second quadrant
        heading = atan(y, x)
    elseif sign(x) == 1 && sign(y) == -1 # fourth quadrant
        heading = atan(y, x)
    elseif sign(x) == -1 && sign(y) == -1 # third quadrant
        heading = atan(y, x)
    end
    # bound_heading doesn't end up getting called cause Julia takes care of it apparently
    bound_heading(heading)

    return heading
end

"""
    function bound_heading
- Make the angle range from 0 to pi instead of going beyond
"""
function bound_heading(heading::Float64)
    if heading > π # send this to be just near -pi
        heading = -π + (heading - π)    # if heading is 3.15, then the new angle will be (-pi + (3.15-pi)) = -3.13
    elseif heading < -π # send this to be just near pi 
        heading = π + (heading + π)     # if heading is -3.15, then the new angle will be (pi + (-3.15+pi)) = 3.13
    end
    return heading
end

"""
    function append_headings
- Used create the angles and append them into the coordinates

# Examples
```julia
x_coods = [1089.07510, 1093.82626, 1101.19325, 1112.59899, 1123.96733, 1133.24150, 1146.47964]
y_coods = [936.31213, 936.92692,938.52419, 940.93865, 943.27882, 945.21039, 947.88488]
coods = hcat(x_coods,y_coods)
append_headings(coods)
```
"""
function append_headings(coordinates::Matrix{Float64})
    headings = ones(size(coordinates)[1])
    for i = 1:size(coordinates)[1]-1
        # need to make sure that math is right, and that bounds are kept
        tangent_vector = [coordinates[i+1,1]-coordinates[i,1], coordinates[i+1,2]-coordinates[i,2]]
        # @show tangent_vector
        current_heading = get_new_angle(tangent_vector)
        # @show current_heading
        headings[i] = current_heading
    end
    headings[end] = headings[end-1] # assume this is fine
    coordinates = hcat(coordinates, headings)
    return coordinates
end



"""
    function centerlines_txt2tracks(filename)

- Input: text file with centerline coordinates with x in row 1 and y in row2
- Output: Track that cen be used to create a lane

- Requires: `using DelimitedFiles`
# Example
```julia
track0 = centerlines_txt2tracks("centerlines_DR_CHN_Merging_ZS/output_centerline_0.txt")
```
"""
function centerlines_txt2tracks(filename)
    centerlines = readdlm(filename,',')
    x0 = centerlines[1,:]
    y0 = centerlines[2,:]
    coods = hcat(x0,y0)
    coods_app = append_headings(coods) # Append with the angle
    
    mid_coods = coods_app'
    
    first_cood = VecSE2(mid_coods[1,1], mid_coods[2,1], mid_coods[3,1])
    second_cood = VecSE2(mid_coods[1,2], mid_coods[2,2], mid_coods[3,2])
    radius = 0.01
    nsamples = 20

    track = gen_bezier_curve(first_cood, second_cood, radius, radius, nsamples)
    
    nsamples = 20
    for i = 3:size(coods,1)
        turn1 = VecSE2(mid_coods[1, i-1], mid_coods[2, i-1], mid_coods[3, i-1])
        turn2 = VecSE2(mid_coods[1, i], mid_coods[2, i], mid_coods[3, i])
        curve = gen_bezier_curve(turn1, turn2, radius, radius, nsamples)
        append_to_curve!(track, curve)
    end

    return track
end
