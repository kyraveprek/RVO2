# simple_validation.jl
# RVO2 Julia Implementation - Simple Validation Test

include("rvo_algorithm.jl")

function validate_basic_rvo()
    println("Validating Basic RVO2 Implementation")
    println("===================================")
    
    # Test 1: Two agents head-on collision
    println("\n1. Two-agent head-on collision test")
    agent1 = create_agent(1, Vector2(-3.0, 0.0), Vector2(1.0, 0.0), Vector2(1.0, 0.0),
                         0.5, 1.0, 10.0, 5, 2.0)
    agent2 = create_agent(2, Vector2(3.0, 0.0), Vector2(-1.0, 0.0), Vector2(-1.0, 0.0),
                         0.5, 1.0, 10.0, 5, 2.0)
    
    agents = [agent1, agent2]
    
    # Run a few steps
    for i in 1:20
        compute_neighbors!(agents[1], agents)
        compute_neighbors!(agents[2], agents)
        
        compute_new_velocity!(agents[1], 0.1)
        compute_new_velocity!(agents[2], 0.1)
        
        update_agent!(agents[1], 0.1)
        update_agent!(agents[2], 0.1)
        
        if i % 5 == 0
            distance = Base.abs(agents[1].position - agents[2].position)
            println("  Step $i: distance = $(round(distance, digits=2))")
        end
    end
    
    final_distance = Base.abs(agents[1].position - agents[2].position)
    min_required = agents[1].radius + agents[2].radius
    
    if final_distance > min_required
        println("  ✅ SUCCESS: Collision avoided (distance: $(round(final_distance, digits=2)) > $(min_required))")
    else
        println("  ❌ FAILED: Collision occurred (distance: $(round(final_distance, digits=2)) <= $(min_required))")
    end
    
    # Test 2: ORCA constraint validation
    println("\n2. ORCA constraint validation")
    
    # Reset agents
    agent1.position = Vector2(-2.0, 0.0)
    agent1.velocity = Vector2(1.0, 0.0)
    agent2.position = Vector2(2.0, 0.0)
    agent2.velocity = Vector2(-1.0, 0.0)
    
    # Compute neighbors and constraints
    compute_neighbors!(agent1, [agent1, agent2])
    create_orca_constraints!(agent1, 0.1)
    
    if length(agent1.orca_lines) > 0
        line = agent1.orca_lines[1]
        println("  ORCA line direction: ($(round(line.direction.x, digits=3)), $(round(line.direction.y, digits=3)))")
        println("  ORCA line point: ($(round(line.point.x, digits=3)), $(round(line.point.y, digits=3)))")
        
        # Check if the line is properly constructed
        if Base.abs(line.direction) > 0.1  # Should have non-zero direction
            println("  ✅ ORCA constraint properly constructed")
        else
            println("  ❌ ORCA constraint invalid")
        end
    else
        println("  ❌ No ORCA constraints generated")
    end
    
    # Test 3: Linear programming basic test
    println("\n3. Linear programming validation")
    
    # Simple constraint: x >= 1
    constraint = Line(Vector2(1.0, 0.0), Vector2(1.0, 0.0))
    constraints = [constraint]
    
    fail_idx, result = linear_program2(constraints, 2.0, Vector2(0.0, 0.0), false)
    
    if fail_idx > length(constraints)
        violation = get_constraint_violation(result, constraint)
        if violation <= 1e-6  # Should satisfy constraint
            println("  ✅ Linear programming solver working correctly")
            println("     Result: ($(round(result.x, digits=2)), $(round(result.y, digits=2)))")
        else
            println("  ❌ Linear programming result violates constraint")
        end
    else
        println("  ❌ Linear programming failed to find solution")
    end
    
    println("\nValidation Complete!")
end

validate_basic_rvo()