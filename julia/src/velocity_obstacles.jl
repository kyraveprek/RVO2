# velocity_obstacles.jl
# RVO2 Julia Implementation - Velocity Obstacle and ORCA Line Construction
#
# Based on the RVO2 C++ library
# SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
# SPDX-License-Identifier: Apache-2.0

include("types.jl")

"""
Compute ORCA line for agent-agent interaction.
Returns the constructed ORCA line.
"""
function compute_agent_orca_line(agent::Agent, other::Agent, time_step::Float64)::Line
    relative_position = other.position - agent.position
    relative_velocity = agent.velocity - other.velocity
    dist_sq = abs_sq(relative_position)
    combined_radius = agent.radius + other.radius
    combined_radius_sq = combined_radius * combined_radius
    
    line = Line()
    u = Vector2()
    
    inv_time_horizon = 1.0 / agent.time_horizon
    
    if dist_sq > combined_radius_sq
        # No collision
        w = relative_velocity - inv_time_horizon * relative_position
        # Vector from cutoff center to relative velocity
        w_length_sq = abs_sq(w)
        
        dot_product = dot(w, relative_position)
        
        if dot_product < 0.0 && dot_product * dot_product > combined_radius_sq * w_length_sq
            # Project on cut-off circle
            w_length = sqrt(w_length_sq)
            unit_w = w / w_length
            
            line.direction = Vector2(unit_w.y, -unit_w.x)
            u = (combined_radius * inv_time_horizon - w_length) * unit_w
        else
            # Project on legs
            leg = sqrt(dist_sq - combined_radius_sq)
            
            if det(relative_position, w) > 0.0
                # Project on left leg
                line.direction = Vector2(relative_position.x * leg - relative_position.y * combined_radius,
                                       relative_position.x * combined_radius + relative_position.y * leg) / dist_sq
            else
                # Project on right leg
                line.direction = -Vector2(relative_position.x * leg + relative_position.y * combined_radius,
                                        -relative_position.x * combined_radius + relative_position.y * leg) / dist_sq
            end
            
            u = dot(relative_velocity, line.direction) * line.direction - relative_velocity
        end
    else
        # Collision. Project on cut-off circle of time timeStep
        inv_time_step = 1.0 / time_step
        
        # Vector from cutoff center to relative velocity
        w = relative_velocity - inv_time_step * relative_position
        
        w_length = Base.abs(w)
        unit_w = w / w_length
        
        line.direction = Vector2(unit_w.y, -unit_w.x)
        u = (combined_radius * inv_time_step - w_length) * unit_w
    end
    
    # ORCA constraint: each agent takes half responsibility
    line.point = agent.velocity + 0.5 * u
    
    return line
end

"""
Check if a velocity satisfies all ORCA constraints.
"""
function satisfies_constraints(velocity::Vector2, lines::Vector{Line})::Bool
    for line in lines
        if det(line.direction, line.point - velocity) > RVO_EPSILON
            return false
        end
    end
    return true
end

"""
Find the closest feasible velocity to the preferred velocity.
"""
function project_to_feasible_region(pref_velocity::Vector2, lines::Vector{Line}, 
                                   max_speed::Float64)::Vector2
    # Simple projection - find closest point that satisfies all constraints
    result = pref_velocity
    
    # First, ensure we're within the speed limit
    if Base.abs(result) > max_speed
        result = normalize(result) * max_speed
    end
    
    # Project onto each constraint
    max_iterations = 10
    for iteration in 1:max_iterations
        violated = false
        
        for line in lines
            violation = det(line.direction, line.point - result)
            if violation > RVO_EPSILON
                # Project onto this constraint
                result = result + violation * Vector2(-line.direction.y, line.direction.x)
                violated = true
                
                # Ensure we stay within speed limit
                if Base.abs(result) > max_speed
                    result = normalize(result) * max_speed
                end
            end
        end
        
        if !violated
            break
        end
    end
    
    return result
end