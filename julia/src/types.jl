# types.jl
# RVO2 Julia Implementation - Core Data Types
#
# Based on the RVO2 C++ library
# SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
# SPDX-License-Identifier: Apache-2.0

"""
A two-dimensional vector.
"""
mutable struct Vector2
    x::Float64
    y::Float64
end

Vector2() = Vector2(0.0, 0.0)

# Vector operations
Base.:+(a::Vector2, b::Vector2) = Vector2(a.x + b.x, a.y + b.y)
Base.:-(a::Vector2, b::Vector2) = Vector2(a.x - b.x, a.y - b.y)
Base.:-(a::Vector2) = Vector2(-a.x, -a.y)
Base.:*(a::Vector2, scalar::Real) = Vector2(a.x * scalar, a.y * scalar)
Base.:*(scalar::Real, a::Vector2) = Vector2(a.x * scalar, a.y * scalar)
Base.:/(a::Vector2, scalar::Real) = Vector2(a.x / scalar, a.y / scalar)

# Dot product
dot(a::Vector2, b::Vector2) = a.x * b.x + a.y * b.y

# Vector utilities
abs_sq(v::Vector2) = v.x * v.x + v.y * v.y
Base.abs(v::Vector2) = sqrt(abs_sq(v))
normalize(v::Vector2) = v / Base.abs(v)

# Determinant (cross product in 2D)
det(a::Vector2, b::Vector2) = a.x * b.y - a.y * b.x

"""
A directed line representing a half-plane constraint.
The half-plane to the left of the line is the feasible region.
"""
mutable struct Line
    direction::Vector2  # Direction of the line
    point::Vector2      # A point on the line
end

Line() = Line(Vector2(), Vector2())

"""
An agent in the RVO simulation.
"""
mutable struct Agent
    id::Int
    position::Vector2
    velocity::Vector2
    new_velocity::Vector2
    pref_velocity::Vector2
    radius::Float64
    max_speed::Float64
    neighbor_dist::Float64
    max_neighbors::Int
    time_horizon::Float64
    time_horizon_obst::Float64
    
    # Internal data
    agent_neighbors::Vector{Tuple{Float64, Agent}}  # distance, agent
    obstacle_neighbors::Vector{Any}  # For obstacles (not implemented yet)
    orca_lines::Vector{Line}
end

function Agent(id::Int = 0)
    Agent(id, Vector2(), Vector2(), Vector2(), Vector2(), 
          0.0, 0.0, 0.0, 0, 0.0, 0.0, 
          Vector{Tuple{Float64, Agent}}(), Vector{Any}(), Vector{Line}())
end

const RVO_EPSILON = 1e-6