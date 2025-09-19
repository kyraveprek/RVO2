# RVO2.jl
# RVO2 Julia Implementation - Main Module
#
# Based on the RVO2 C++ library
# SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
# SPDX-License-Identifier: Apache-2.0

module RVO2

# Export main types
export Vector2, Line, Agent

# Export core functions
export create_agent, simulation_step!, compute_new_velocity!, update_agent!
export set_preferred_velocities!, agents_at_goals
export compute_neighbors!

# Export utilities
export abs, abs_sq, normalize, det, dot

# Include implementation files
include("types.jl")
include("linear_programming.jl")
include("velocity_obstacles.jl")
include("rvo_algorithm.jl")

end # module RVO2