# test_example.jl
# RVO2 Julia Implementation - Simple Test Example
#
# Based on the RVO2 C++ library
# SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
# SPDX-License-Identifier: Apache-2.0

include("rvo_algorithm.jl")

"""
Simple test scenario with two agents moving toward each other.
"""
function test_two_agents()
    println("Testing RVO2 Julia Implementation")
    println("=================================")
    
    # Create two agents moving toward each other
    agent1 = create_agent(1, Vector2(-5.0, 0.0), Vector2(1.0, 0.0), Vector2(1.0, 0.0),
                         1.0, 2.0, 10.0, 5, 2.0)
    
    agent2 = create_agent(2, Vector2(5.0, 0.0), Vector2(-1.0, 0.0), Vector2(-1.0, 0.0),
                         1.0, 2.0, 10.0, 5, 2.0)
    
    agents = [agent1, agent2]
    goals = [Vector2(5.0, 0.0), Vector2(-5.0, 0.0)]
    
    time_step = 0.1
    max_steps = 100
    
    println("Initial positions:")
    for (i, agent) in enumerate(agents)
        println("Agent $i: position = ($(agent.position.x), $(agent.position.y)), velocity = ($(agent.velocity.x), $(agent.velocity.y))")
    end
    println()
    
    # Simulation loop
    for step in 1:max_steps
        # Set preferred velocities toward goals
        set_preferred_velocities!(agents, goals)
        
        # Run simulation step
        simulation_step!(agents, time_step)
        
        # Print progress every 10 steps
        if step % 10 == 0
            println("Step $step:")
            for (i, agent) in enumerate(agents)
                println("  Agent $i: pos = ($(round(agent.position.x, digits=2)), $(round(agent.position.y, digits=2))), " *
                       "vel = ($(round(agent.velocity.x, digits=2)), $(round(agent.velocity.y, digits=2)))")
            end
            
            # Check minimum distance
            min_dist = Base.abs(agents[1].position - agents[2].position)
            println("  Minimum distance: $(round(min_dist, digits=2))")
            println()
        end
        
        # Check if agents reached their goals
        if agents_at_goals(agents, goals, 0.5)
            println("Agents reached their goals at step $step!")
            break
        end
        
        # Safety check - prevent infinite loops
        if step == max_steps
            println("Maximum steps reached.")
        end
    end
    
    println("Final positions:")
    for (i, agent) in enumerate(agents)
        println("Agent $i: position = ($(round(agent.position.x, digits=2)), $(round(agent.position.y, digits=2)))")
    end
    
    # Check if collision avoidance worked
    final_dist = Base.abs(agents[1].position - agents[2].position)
    min_required_dist = agents[1].radius + agents[2].radius
    
    if final_dist >= min_required_dist - 0.1  # Small tolerance
        println("✓ Collision avoidance successful!")
    else
        println("✗ Collision occurred! Final distance: $(round(final_dist, digits=2)), required: $(round(min_required_dist, digits=2))")
    end
end

"""
Test scenario with four agents moving in a cross pattern.
"""
function test_four_agents_cross()
    println("\nTesting Four Agents Cross Scenario")
    println("==================================")
    
    # Create four agents in cross formation
    agents = [
        create_agent(1, Vector2(-5.0, 0.0), Vector2(0.0, 0.0), Vector2(0.0, 0.0),
                    0.5, 1.5, 8.0, 3, 2.5),
        create_agent(2, Vector2(5.0, 0.0), Vector2(0.0, 0.0), Vector2(0.0, 0.0),
                    0.5, 1.5, 8.0, 3, 2.5),
        create_agent(3, Vector2(0.0, -5.0), Vector2(0.0, 0.0), Vector2(0.0, 0.0),
                    0.5, 1.5, 8.0, 3, 2.5),
        create_agent(4, Vector2(0.0, 5.0), Vector2(0.0, 0.0), Vector2(0.0, 0.0),
                    0.5, 1.5, 8.0, 3, 2.5)
    ]
    
    goals = [Vector2(5.0, 0.0), Vector2(-5.0, 0.0), Vector2(0.0, 5.0), Vector2(0.0, -5.0)]
    
    time_step = 0.1
    max_steps = 150
    
    println("Initial positions:")
    for (i, agent) in enumerate(agents)
        println("Agent $i: position = ($(agent.position.x), $(agent.position.y))")
    end
    println()
    
    # Simulation loop
    for step in 1:max_steps
        set_preferred_velocities!(agents, goals)
        simulation_step!(agents, time_step)
        
        if step % 15 == 0
            println("Step $step:")
            for (i, agent) in enumerate(agents)
                println("  Agent $i: pos = ($(round(agent.position.x, digits=2)), $(round(agent.position.y, digits=2)))")
            end
            
            # Check minimum distance between any two agents
            min_dist = Inf
            for i in 1:length(agents)
                for j in (i+1):length(agents)
                    dist = Base.abs(agents[i].position - agents[j].position)
                    min_dist = min(min_dist, dist)
                end
            end
            println("  Minimum inter-agent distance: $(round(min_dist, digits=2))")
            println()
        end
        
        if agents_at_goals(agents, goals, 0.8)
            println("All agents reached their goals at step $step!")
            break
        end
    end
    
    # Final collision check
    collision_occurred = false
    min_dist = Inf
    for i in 1:length(agents)
        for j in (i+1):length(agents)
            dist = Base.abs(agents[i].position - agents[j].position)
            min_dist = min(min_dist, dist)
            if dist < agents[i].radius + agents[j].radius - 0.1
                collision_occurred = true
            end
        end
    end
    
    if !collision_occurred
        println("✓ All agents avoided collisions! Minimum distance: $(round(min_dist, digits=2))")
    else
        println("✗ Collision detected! Minimum distance: $(round(min_dist, digits=2))")
    end
end

# Run tests
if abspath(PROGRAM_FILE) == @__FILE__
    test_two_agents()
    test_four_agents_cross()
end