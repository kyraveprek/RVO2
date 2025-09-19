# rvo_algorithm.jl
# RVO2 Julia Implementation - Main ORCA Algorithm
#
# Based on the RVO2 C++ library
# SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
# SPDX-License-Identifier: Apache-2.0

include("types.jl")
include("linear_programming.jl")
include("velocity_obstacles.jl")
include("constraints.jl")

"""
Compute neighbors for an agent within its sensing range.
"""
function compute_neighbors!(agent::Agent, all_agents::Vector{Agent})
    empty!(agent.agent_neighbors)
    
    range_sq = agent.neighbor_dist * agent.neighbor_dist
    
    for other in all_agents
        if other.id != agent.id
            dist_sq = abs_sq(agent.position - other.position)
            
            if dist_sq < range_sq
                if length(agent.agent_neighbors) < agent.max_neighbors
                    push!(agent.agent_neighbors, (dist_sq, other))
                else
                    # Find the furthest neighbor and replace if this one is closer
                    max_dist_idx = argmax([neighbor[1] for neighbor in agent.agent_neighbors])
                    if dist_sq < agent.agent_neighbors[max_dist_idx][1]
                        agent.agent_neighbors[max_dist_idx] = (dist_sq, other)
                    end
                end
            end
        end
    end
    
    # Sort neighbors by distance
    sort!(agent.agent_neighbors, by = x -> x[1])
end

"""
Compute the new velocity for an agent using the ORCA algorithm.
"""
function compute_new_velocity!(agent::Agent, time_step::Float64)
    # Create ORCA constraints from neighbor interactions
    create_orca_constraints!(agent, time_step)
    
    num_obst_lines = 0  # No obstacles implemented yet
    
    # Solve the linear program to find the optimal velocity
    line_fail, result = linear_program2(agent.orca_lines, agent.max_speed, 
                                       agent.pref_velocity, false)
    
    if line_fail <= length(agent.orca_lines)
        # linear_program2 failed, use linear_program3
        result = linear_program3(agent.orca_lines, num_obst_lines, line_fail, 
                               agent.max_speed, result)
    end
    
    agent.new_velocity = result
end

"""
Update agent position based on its velocity.
"""
function update_agent!(agent::Agent, time_step::Float64)
    agent.velocity = agent.new_velocity
    agent.position = agent.position + agent.velocity * time_step
end

"""
Run one simulation step for all agents.
"""
function simulation_step!(agents::Vector{Agent}, time_step::Float64)
    # Compute neighbors for all agents
    for agent in agents
        compute_neighbors!(agent, agents)
    end
    
    # Compute new velocities for all agents
    for agent in agents
        compute_new_velocity!(agent, time_step)
    end
    
    # Update positions
    for agent in agents
        update_agent!(agent, time_step)
    end
end

"""
Create an agent with specified parameters.
"""
function create_agent(id::Int, position::Vector2, velocity::Vector2, 
                     pref_velocity::Vector2, radius::Float64, max_speed::Float64,
                     neighbor_dist::Float64, max_neighbors::Int, 
                     time_horizon::Float64)::Agent
    
    agent = Agent(id)
    agent.position = position
    agent.velocity = velocity
    agent.pref_velocity = pref_velocity
    agent.radius = radius
    agent.max_speed = max_speed
    agent.neighbor_dist = neighbor_dist
    agent.max_neighbors = max_neighbors
    agent.time_horizon = time_horizon
    agent.time_horizon_obst = time_horizon  # Same as agent time horizon for now
    
    return agent
end

"""
Check if agents have reached their goals (within a threshold).
"""
function agents_at_goals(agents::Vector{Agent}, goals::Vector{Vector2}, 
                        threshold::Float64 = 1.0)::Bool
    for (i, agent) in enumerate(agents)
        if i <= length(goals)
            if Base.abs(agent.position - goals[i]) > threshold
                return false
            end
        end
    end
    return true
end

"""
Set preferred velocities toward goals.
"""
function set_preferred_velocities!(agents::Vector{Agent}, goals::Vector{Vector2})
    for (i, agent) in enumerate(agents)
        if i <= length(goals)
            direction = goals[i] - agent.position
            distance = Base.abs(direction)
            
            if distance > RVO_EPSILON
                # Set preferred velocity toward goal
                max_dist = agent.max_speed
                agent.pref_velocity = normalize(direction) * min(distance, max_dist)
            else
                # Already at goal
                agent.pref_velocity = Vector2(0.0, 0.0)
            end
        else
            agent.pref_velocity = Vector2(0.0, 0.0)
        end
    end
end