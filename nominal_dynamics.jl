using SatellitePlayground
using LinearAlgebra
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

"""
Returns the dynamics at a certain position, time
"""
function nominal_attitude_dynamics(x, position, J, time, control)
    ω, q = x[1:3], x[4:7]
    ᵇQⁿ = quaternionToMatrix(q)'
    ᵇmagnetic = ᵇQⁿ * SatellitePlayground.IGRF13(position, time)
    u = cross(control, ᵇmagnetic)
    return [
        J \ (u - cross(ω, J * ω))
        SP.qdot(q, ω)
    ]
end
