# constraints.jl
# RVO2 Julia Implementation - ORCA Constraint Construction
#
# Based on the RVO2 C++ library
# SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
# SPDX-License-Identifier: Apache-2.0

include("types.jl")

"""
Create ORCA constraints for all neighbor interactions of an agent.
This is the core of the ORCA algorithm - it creates half-plane constraints
that represent the feasible velocity region for collision avoidance.
"""
function create_orca_constraints!(agent::Agent, time_step::Float64)
    empty!(agent.orca_lines)
    
    inv_time_horizon = 1.0 / agent.time_horizon
    
    # Create agent ORCA lines
    for (_, other) in agent.agent_neighbors
        relative_position = other.position - agent.position
        relative_velocity = agent.velocity - other.velocity
        dist_sq = abs_sq(relative_position)
        combined_radius = agent.radius + other.radius
        combined_radius_sq = combined_radius * combined_radius
        
        line = Line()
        u = Vector2()
        
        if dist_sq > combined_radius_sq
            # No collision - compute velocity obstacle
            w = relative_velocity - inv_time_horizon * relative_position
            w_length_sq = abs_sq(w)
            
            dot_product_w_rel = dot(w, relative_position)
            
            if dot_product_w_rel < 0.0 && 
               dot_product_w_rel * dot_product_w_rel > combined_radius_sq * w_length_sq
                # Project on cut-off circle
                w_length = sqrt(w_length_sq)
                unit_w = w / w_length
                
                line.direction = Vector2(unit_w.y, -unit_w.x)
                u = (combined_radius * inv_time_horizon - w_length) * unit_w
            else
                # Project on legs of velocity obstacle
                leg = sqrt(dist_sq - combined_radius_sq)
                
                if det(relative_position, w) > 0.0
                    # Project on left leg
                    line.direction = Vector2(
                        relative_position.x * leg - relative_position.y * combined_radius,
                        relative_position.x * combined_radius + relative_position.y * leg
                    ) / dist_sq
                else
                    # Project on right leg
                    line.direction = -Vector2(
                        relative_position.x * leg + relative_position.y * combined_radius,
                        -relative_position.x * combined_radius + relative_position.y * leg
                    ) / dist_sq
                end
                
                u = dot(relative_velocity, line.direction) * line.direction - relative_velocity
            end
        else
            # Collision - emergency avoidance using cut-off circle
            inv_time_step = 1.0 / time_step
            
            w = relative_velocity - inv_time_step * relative_position
            w_length = Base.abs(w)
            unit_w = w / w_length
            
            line.direction = Vector2(unit_w.y, -unit_w.x)
            u = (combined_radius * inv_time_step - w_length) * unit_w
        end
        
        # ORCA principle: each agent takes half responsibility
        line.point = agent.velocity + 0.5 * u
        push!(agent.orca_lines, line)
    end
end

"""
Check if a velocity satisfies all ORCA constraints.
"""
function is_velocity_feasible(velocity::Vector2, constraints::Vector{Line})::Bool
    for constraint in constraints
        # Check if velocity is in the feasible half-plane
        if det(constraint.direction, constraint.point - velocity) > RVO_EPSILON
            return false
        end
    end
    return true
end

"""
Get the violation amount for a velocity against a constraint.
Positive values indicate constraint violation.
"""
function get_constraint_violation(velocity::Vector2, constraint::Line)::Float64
    return det(constraint.direction, constraint.point - velocity)
end