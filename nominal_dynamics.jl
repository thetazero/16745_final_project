using SatellitePlayground
using LinearAlgebra
using SatelliteDynamics
SP = SatellitePlayground

function hat(ω)
    # TODO: implement the hat function 
    ω̂ = [0 -ω[3] ω[2]
        ω[3] 0 -ω[1]
        -ω[2] ω[1] 0]

    return ω̂
end

""" quaternionToMatrix (q)
    Arguments:
     - q: Scalar-first unit quaternion                               | [4,]

    Returns:
     - Q: Rotation matrix representing the same rotation
"""
function quaternionToMatrix(q)
    s, v = q[1], q[2:4]
    return I(3) + 2 * hat(v) * (s * I(3) + hat(v))
end

struct attitude_params
    position::Vector{Float64}
    J::Matrix{Float64}
end

function rk4(p::attitude_params, x, u, t, dt, derivative)
    k₁ = dt * derivative(p, x, u, t)
    k₂ = dt * derivative(p, x + k₁ / 2, u, t + dt * 0.5)
    k₃ = dt * derivative(p, x + k₂ / 2, u, t + dt * 0.5)
    k₄ = dt * derivative(p, x + k₃, u, t + dt)

    x⁺ = x + (1 / 6) * (k₁ + 2 * k₂ + 2 * k₃ + k₄)

    x⁺[4:7] /= norm(x⁺[4:7])

    return x⁺
end


"""
Returns the dynamics at a certain position, time
"""
function nominal_attitude_dynamics(p::attitude_params, x, control, time::Epoch)
    ω, q = x[1:3], x[4:7]
    ᵇQⁿ = quaternionToMatrix(q)'
    ᵇmagnetic = ᵇQⁿ * SatellitePlayground.IGRF13(p.position, time)
    u = cross(control, ᵇmagnetic)
    return [
        p.J \ (u - cross(ω, p.J * ω))
        SP.qdot(q, ω)
    ]
end

# """
# Reaction wheel dynamics
# """
# function nominal_attitude_dynamics(p::attitude_params, x, u, time::Epoch)
#     ω, q = x[1:3], x[4:7]
#     return [
#         p.J \ (u - cross(ω, p.J * ω))
#         SP.qdot(q, ω)
#     ]
# end
