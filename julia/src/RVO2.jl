# RVO2.jl
# RVO2 Julia Implementation - Main Module
#
# This module provides a complete Julia implementation of the RVO2 (Optimal Reciprocal 
# Collision Avoidance) algorithm based on the C++ reference implementation.
#
# Based on the RVO2 C++ library
# SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
# SPDX-License-Identifier: Apache-2.0

"""
    RVO2

A Julia implementation of the RVO2 (Optimal Reciprocal Collision Avoidance) algorithm.

This module provides collision-free navigation for multiple agents using the ORCA
(Optimal Reciprocal Collision Avoidance) algorithm. Each agent computes a set of
velocity constraints based on its neighbors and solves a linear programming problem
to find the optimal collision-free velocity.

# Key Features
- Proper ORCA line construction with velocity obstacles
- Linear programming solvers for constraint optimization  
- Responsibility sharing between agents (reciprocal avoidance)
- Support for multi-agent scenarios with smooth, oscillation-free motion

# Basic Usage
```julia
using RVO2

# Create agents
agent1 = create_agent(1, Vector2(-5.0, 0.0), Vector2(0.0, 0.0), Vector2(1.0, 0.0),
                     1.0, 2.0, 10.0, 5, 2.0)
agent2 = create_agent(2, Vector2(5.0, 0.0), Vector2(0.0, 0.0), Vector2(-1.0, 0.0),
                     1.0, 2.0, 10.0, 5, 2.0)

agents = [agent1, agent2]
goals = [Vector2(5.0, 0.0), Vector2(-5.0, 0.0)]

# Simulation loop
for step in 1:100
    set_preferred_velocities!(agents, goals)
    simulation_step!(agents, 0.1)
    
    if agents_at_goals(agents, goals)
        break
    end
end
```
"""
module RVO2

# Export main types
export Vector2, Line, Agent

# Export core functions
export create_agent, simulation_step!, compute_new_velocity!, update_agent!
export set_preferred_velocities!, agents_at_goals
export compute_neighbors!, create_orca_constraints!

# Export linear programming functions
export linear_program1, linear_program2, linear_program3

# Export constraint functions
export is_velocity_feasible, get_constraint_violation

# Export utilities
export abs_sq, normalize, det, dot

# Include implementation files
include("types.jl") 
include("linear_programming.jl")
include("velocity_obstacles.jl")
include("constraints.jl")
include("rvo_algorithm.jl")

end # module RVO2