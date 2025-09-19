# RVO2 Julia Implementation

This directory contains a Julia implementation of the RVO2 (Optimal Reciprocal Collision Avoidance) algorithm based on the C++ reference implementation.

## Overview

The implementation follows the ORCA (Optimal Reciprocal Collision Avoidance) algorithm as described in the original RVO2 library. It provides collision-free navigation for multiple agents by:

1. **Velocity Obstacle Computation**: Calculating velocity obstacles for agent-agent interactions
2. **ORCA Line Construction**: Creating half-plane constraints representing feasible velocity regions
3. **Linear Programming**: Solving optimization problems to find optimal velocities
4. **Responsibility Sharing**: Each agent takes half responsibility for collision avoidance (0.5 factor)

## Key Features Fixed

### 1. ORCA Line Construction
- ✅ Proper velocity obstacle computation for agent-agent interactions
- ✅ Cut-off circle handling for collision scenarios  
- ✅ Left/right leg calculations for proper constraint geometry
- ✅ Proper responsibility sharing (0.5 factor) between agents

### 2. Linear Programming Solver
- ✅ `linear_program1`: 1D LP on a line with circular constraints
- ✅ `linear_program2`: 2D LP with multiple linear constraints
- ✅ `linear_program3`: Handling infeasible cases with constraint projection

### 3. Velocity Obstacle Mathematics
- ✅ Proper relative position and velocity calculations
- ✅ Combined radius and time horizon handling
- ✅ Correct velocity obstacle geometry computation

### 4. Collision vs. No-Collision Cases
- ✅ No collision: project on velocity obstacle boundary
- ✅ Collision imminent: emergency avoidance with cut-off circles
- ⚠️ Static obstacles: not yet implemented (future work)

## Files Structure

- `types.jl`: Core data structures (Vector2, Line, Agent)
- `linear_programming.jl`: LP solver functions (linearProgram1, linearProgram2, linearProgram3)
- `velocity_obstacles.jl`: Velocity obstacle geometry calculations  
- `constraints.jl`: ORCA line construction and constraint handling
- `rvo_algorithm.jl`: Main ORCA implementation and simulation functions
- `RVO2.jl`: Main module file with exports
- `test_example.jl`: Test scenarios demonstrating collision avoidance

## Usage

```julia
include("src/rvo_algorithm.jl")

# Create agents
agent1 = create_agent(1, Vector2(-5.0, 0.0), Vector2(0.0, 0.0), Vector2(1.0, 0.0),
                     1.0, 2.0, 10.0, 5, 2.0)
agent2 = create_agent(2, Vector2(5.0, 0.0), Vector2(0.0, 0.0), Vector2(-1.0, 0.0),
                     1.0, 2.0, 10.0, 5, 2.0)

agents = [agent1, agent2]
goals = [Vector2(5.0, 0.0), Vector2(-5.0, 0.0)]

# Simulation loop
time_step = 0.1
for step in 1:100
    set_preferred_velocities!(agents, goals)
    simulation_step!(agents, time_step)
    
    # Check if goals reached
    if agents_at_goals(agents, goals)
        break
    end
end
```

## Running Tests

```bash
cd julia/src
julia test_example.jl
```

This will run two test scenarios:
1. Two agents moving toward each other (head-on collision avoidance)
2. Four agents in cross formation (multi-agent intersection scenario)

## Algorithm Details

The implementation correctly follows the ORCA algorithm:

1. **Neighbor Detection**: Find nearby agents within sensing range
2. **ORCA Constraint Generation**: For each neighbor, create half-plane constraints
3. **Linear Programming**: Solve for optimal velocity within feasible region
4. **Velocity Update**: Apply the computed velocity while respecting speed limits

### Key Mathematical Components

- **Velocity Obstacles**: Geometric regions of velocities that lead to collision
- **ORCA Lines**: Half-plane constraints derived from velocity obstacles  
- **Responsibility Sharing**: Each agent accounts for 50% of collision avoidance
- **Linear Programming**: Efficient optimization for multi-constraint problems

## Validation

The implementation has been tested with:
- ✅ Two-agent head-on collision scenario
- ✅ Four-agent cross intersection scenario
- ✅ Collision avoidance verification
- ✅ Smooth motion without oscillations

## Comparison with C++ Reference

This Julia implementation closely follows the C++ reference code in `src/Agent.cc`, particularly:
- `computeNewVelocity()` function structure
- Linear programming solver implementations
- ORCA line construction mathematics
- Constraint handling logic

## Future Improvements

- [ ] Add static obstacle support
- [ ] Implement KD-tree for efficient neighbor searching
- [ ] Add more comprehensive test scenarios
- [ ] Performance optimization for large numbers of agents
- [ ] Visualization tools for debugging and demonstration