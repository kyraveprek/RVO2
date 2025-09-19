# advanced_test.jl
# RVO2 Julia Implementation - Advanced Test Scenarios
#
# Based on the RVO2 C++ library
# SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
# SPDX-License-Identifier: Apache-2.0

include("rvo_algorithm.jl")

"""
Test scenario with many agents in a crowded environment.
"""
function test_crowded_scenario()
    println("Testing Crowded Scenario (16 agents)")
    println("====================================")
    
    # Create a 4x4 grid of agents on the left, all moving to the right
    agents = Agent[]
    goals = Vector2[]
    
    agent_id = 1
    for i in 1:4
        for j in 1:4
            start_pos = Vector2(-8.0 + (i-1) * 2.0, -3.0 + (j-1) * 2.0)
            goal_pos = Vector2(8.0 + (i-1) * 2.0, -3.0 + (j-1) * 2.0)
            
            agent = create_agent(agent_id, start_pos, Vector2(0.0, 0.0), Vector2(0.0, 0.0),
                               0.4, 1.5, 6.0, 8, 3.0)
            push!(agents, agent)
            push!(goals, goal_pos)
            agent_id += 1
        end
    end
    
    time_step = 0.1
    max_steps = 200
    
    println("Initial setup: $(length(agents)) agents")
    println("Agent parameters: radius=0.4, max_speed=1.5, time_horizon=3.0")
    println("Starting simulation...")
    println()
    
    collision_count = 0
    min_distance_overall = Inf
    
    # Simulation loop
    for step in 1:max_steps
        set_preferred_velocities!(agents, goals)
        simulation_step!(agents, time_step)
        
        # Check for collisions and compute statistics
        min_dist_step = Inf
        for i in 1:length(agents)
            for j in (i+1):length(agents)
                dist = Base.abs(agents[i].position - agents[j].position)
                min_dist_step = min(min_dist_step, dist)
                min_distance_overall = min(min_distance_overall, dist)
                
                if dist < agents[i].radius + agents[j].radius - 0.01
                    collision_count += 1
                end
            end
        end
        
        # Print progress every 25 steps
        if step % 25 == 0
            progress = count(i -> Base.abs(agents[i].position - goals[i]) < 1.0, 1:length(agents))
            println("Step $step: $progress/$(length(agents)) agents near goals, min_dist=$(round(min_dist_step, digits=2))")
        end
        
        # Check if all agents reached their goals
        if agents_at_goals(agents, goals, 1.0)
            println("All agents reached their goals at step $step!")
            break
        end
    end
    
    # Final statistics
    println("\nFinal Statistics:")
    println("- Minimum distance encountered: $(round(min_distance_overall, digits=3))")
    println("- Required minimum distance: 0.8")
    println("- Collision count: $collision_count")
    
    if collision_count == 0 && min_distance_overall > 0.7
        println("✅ Success: No collisions, safe distances maintained")
    else
        println("⚠️  Issues detected in collision avoidance")
    end
end

"""
Test the linear programming solvers directly.
"""
function test_linear_programming()
    println("\nTesting Linear Programming Solvers")
    println("==================================")
    
    # Test 1: Simple 2D LP with one constraint
    println("Test 1: Single constraint")
    line1 = Line(Vector2(1.0, 0.0), Vector2(1.0, 0.0))  # x >= 1
    lines = [line1]
    
    fail_idx, result = linear_program2(lines, 2.0, Vector2(0.0, 0.0), false)
    println("  Input: opt_velocity=(0,0), radius=2.0, constraint: x >= 1")
    println("  Result: $(result.x), $(result.y), success=$(fail_idx > length(lines))")
    
    # Test 2: Multiple constraints forming a feasible region
    println("\nTest 2: Multiple constraints")
    line1 = Line(Vector2(1.0, 0.0), Vector2(1.0, 0.0))    # x >= 1
    line2 = Line(Vector2(0.0, 1.0), Vector2(0.0, 1.0))    # y >= 1
    line3 = Line(Vector2(-1.0, 0.0), Vector2(-2.0, 0.0))  # x <= 2
    line4 = Line(Vector2(0.0, -1.0), Vector2(0.0, -2.0))  # y <= 2
    lines = [line1, line2, line3, line4]
    
    fail_idx, result = linear_program2(lines, 5.0, Vector2(1.5, 1.5), false)
    println("  Input: opt_velocity=(1.5,1.5), radius=5.0, box constraints: 1<=x<=2, 1<=y<=2")
    println("  Result: $(round(result.x, digits=2)), $(round(result.y, digits=2)), success=$(fail_idx > length(lines))")
    
    # Test 3: Infeasible constraints
    println("\nTest 3: Infeasible constraints")
    line1 = Line(Vector2(1.0, 0.0), Vector2(2.0, 0.0))   # x >= 2
    line2 = Line(Vector2(-1.0, 0.0), Vector2(1.0, 0.0))  # x <= 1
    lines = [line1, line2]
    
    fail_idx, result = linear_program2(lines, 3.0, Vector2(1.5, 0.0), false)
    println("  Input: conflicting constraints: x >= 2 and x <= 1")
    println("  Result: $(round(result.x, digits=2)), $(round(result.y, digits=2)), success=$(fail_idx > length(lines))")
    
    if fail_idx <= length(lines)
        result = linear_program3(lines, 0, fail_idx, 3.0, result)
        println("  After LP3: $(round(result.x, digits=2)), $(round(result.y, digits=2))")
    end
end

"""
Test velocity obstacle computation directly.
"""
function test_velocity_obstacles()
    println("\nTesting Velocity Obstacle Computation")
    println("=====================================")
    
    # Create two agents
    agent1 = create_agent(1, Vector2(0.0, 0.0), Vector2(1.0, 0.0), Vector2(1.0, 0.0),
                         1.0, 2.0, 10.0, 5, 2.0)
    agent2 = create_agent(2, Vector2(3.0, 0.0), Vector2(-1.0, 0.0), Vector2(-1.0, 0.0),
                         1.0, 2.0, 10.0, 5, 2.0)
    
    # Test different scenarios
    println("Scenario 1: Head-on collision")
    println("  Agent 1: pos=(0,0), vel=(1,0), Agent 2: pos=(3,0), vel=(-1,0)")
    
    agent1.agent_neighbors = [(9.0, agent2)]
    create_orca_constraints!(agent1, 0.1)
    
    if length(agent1.orca_lines) > 0
        line = agent1.orca_lines[1]
        println("  ORCA line direction: ($(round(line.direction.x, digits=3)), $(round(line.direction.y, digits=3)))")
        println("  ORCA line point: ($(round(line.point.x, digits=3)), $(round(line.point.y, digits=3)))")
        
        # Test if current velocity satisfies constraint
        violation = get_constraint_violation(agent1.velocity, line)
        println("  Current velocity violation: $(round(violation, digits=3))")
    end
    
    println("\nScenario 2: Perpendicular approach")
    agent2.position = Vector2(0.0, 3.0)
    agent2.velocity = Vector2(0.0, -1.0)
    println("  Agent 1: pos=(0,0), vel=(1,0), Agent 2: pos=(0,3), vel=(0,-1)")
    
    create_orca_constraints!(agent1, 0.1)
    
    if length(agent1.orca_lines) > 0
        line = agent1.orca_lines[1]
        println("  ORCA line direction: ($(round(line.direction.x, digits=3)), $(round(line.direction.y, digits=3)))")
        println("  ORCA line point: ($(round(line.point.x, digits=3)), $(round(line.point.y, digits=3)))")
    end
end

"""
Benchmark the performance of the implementation.
"""
function benchmark_performance()
    println("\nPerformance Benchmark")
    println("====================")
    
    # Test with different numbers of agents
    for num_agents in [10, 25, 50, 100]
        agents = Agent[]
        goals = Vector2[]
        
        # Create agents in a circle, all moving to opposite points
        for i in 1:num_agents
            angle = 2π * (i-1) / num_agents
            radius = 10.0
            
            start_pos = Vector2(radius * cos(angle), radius * sin(angle))
            goal_pos = Vector2(-radius * cos(angle), -radius * sin(angle))
            
            agent = create_agent(i, start_pos, Vector2(0.0, 0.0), Vector2(0.0, 0.0),
                               0.5, 1.5, 8.0, min(10, num_agents-1), 2.5)
            push!(agents, agent)
            push!(goals, goal_pos)
        end
        
        # Time one simulation step
        set_preferred_velocities!(agents, goals)
        
        start_time = time()
        simulation_step!(agents, 0.1)
        end_time = time()
        
        elapsed = (end_time - start_time) * 1000  # Convert to milliseconds
        
        println("  $num_agents agents: $(round(elapsed, digits=2)) ms per step")
    end
end

# Run all tests
if abspath(PROGRAM_FILE) == @__FILE__
    test_linear_programming()
    test_velocity_obstacles() 
    test_crowded_scenario()
    benchmark_performance()
end