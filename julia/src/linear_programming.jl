# linear_programming.jl
# RVO2 Julia Implementation - Linear Programming Solvers
#
# Based on the RVO2 C++ library
# SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
# SPDX-License-Identifier: Apache-2.0

include("types.jl")

"""
Solves a one-dimensional linear program on a specified line subject to linear 
constraints defined by lines and a circular constraint.

Returns true if successful, false otherwise.
"""
function linear_program1(lines::Vector{Line}, line_no::Int, radius::Float64, 
                        opt_velocity::Vector2, direction_opt::Bool)::Tuple{Bool, Vector2}
    
    line = lines[line_no]
    dot_product = dot(line.point, line.direction)
    discriminant = dot_product * dot_product + radius * radius - abs_sq(line.point)
    
    if discriminant < 0.0
        # Max speed circle fully invalidates line line_no
        return false, Vector2()
    end
    
    sqrt_discriminant = sqrt(discriminant)
    t_left = -dot_product - sqrt_discriminant
    t_right = -dot_product + sqrt_discriminant
    
    for i in 1:(line_no-1)
        denominator = det(line.direction, lines[i].direction)
        numerator = det(lines[i].direction, line.point - lines[i].point)
        
        if Base.abs(denominator) <= RVO_EPSILON
            # Lines line_no and i are (almost) parallel
            if numerator < 0.0
                return false, Vector2()
            end
            continue
        end
        
        t = numerator / denominator
        
        if denominator >= 0.0
            # Line i bounds line line_no on the right
            t_right = min(t_right, t)
        else
            # Line i bounds line line_no on the left
            t_left = max(t_left, t)
        end
        
        if t_left > t_right
            return false, Vector2()
        end
    end
    
    result = Vector2()
    if direction_opt
        # Optimize direction
        if dot(opt_velocity, line.direction) > 0.0
            # Take right extreme
            result = line.point + t_right * line.direction
        else
            # Take left extreme
            result = line.point + t_left * line.direction
        end
    else
        # Optimize closest point
        t = dot(opt_velocity - line.point, line.direction)
        t = clamp(t, t_left, t_right)
        result = line.point + t * line.direction
    end
    
    return true, result
end

"""
Solves a two-dimensional linear program subject to linear constraints defined 
by lines and a circular constraint.

Returns the index of the failed constraint, or length(lines) if successful.
"""
function linear_program2(lines::Vector{Line}, radius::Float64, opt_velocity::Vector2, 
                        direction_opt::Bool)::Tuple{Int, Vector2}
    
    result = Vector2()
    
    if direction_opt
        # Optimize direction. Note that the optimization velocity is of unit length in this case.
        result = opt_velocity * radius
    elseif abs_sq(opt_velocity) > radius * radius
        # Optimize closest point and outside circle
        result = normalize(opt_velocity) * radius
    else
        # Optimize closest point and inside circle
        result = opt_velocity
    end
    
    for i in 1:length(lines)
        if det(lines[i].direction, lines[i].point - result) > 0.0
            # Result does not satisfy constraint i. Compute new optimal result.
            temp_result = result
            
            success, new_result = linear_program1(lines, i, radius, opt_velocity, direction_opt)
            if !success
                result = temp_result
                return i, result
            end
            result = new_result
        end
    end
    
    return length(lines) + 1, result
end

"""
Solves a two-dimensional linear program subject to linear constraints defined by 
lines and a circular constraint when linearProgram2 fails.
"""
function linear_program3(lines::Vector{Line}, num_obst_lines::Int, begin_line::Int, 
                        radius::Float64, result::Vector2)::Vector2
    
    distance = 0.0
    new_result = result
    
    for i in begin_line:length(lines)
        if det(lines[i].direction, lines[i].point - new_result) > distance
            # Result does not satisfy constraint of line i
            proj_lines = Vector{Line}()
            
            # Add obstacle lines
            for j in 1:num_obst_lines
                push!(proj_lines, lines[j])
            end
            
            # Add agent lines before the current one
            for j in (num_obst_lines + 1):(i - 1)
                line = Line()
                
                determinant = det(lines[i].direction, lines[j].direction)
                
                if Base.abs(determinant) <= RVO_EPSILON
                    # Line i and line j are parallel
                    if dot(lines[i].direction, lines[j].direction) > 0.0
                        # Line i and line j point in the same direction
                        continue
                    end
                    
                    # Line i and line j point in opposite direction
                    line.point = 0.5 * (lines[i].point + lines[j].point)
                else
                    line.point = lines[i].point + 
                               (det(lines[j].direction, lines[i].point - lines[j].point) / 
                                determinant) * lines[i].direction
                end
                
                line.direction = normalize(lines[j].direction - lines[i].direction)
                push!(proj_lines, line)
            end
            
            temp_result = new_result
            
            fail_index, lp2_result = linear_program2(proj_lines, radius, 
                                                   Vector2(-lines[i].direction.y, lines[i].direction.x), 
                                                   true)
            
            if fail_index < length(proj_lines) + 1
                # This should in principle not happen. Keep current result.
                new_result = temp_result
            else
                new_result = lp2_result
            end
            
            distance = det(lines[i].direction, lines[i].point - new_result)
        end
    end
    
    return new_result
end