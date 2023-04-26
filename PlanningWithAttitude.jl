module PlanningWithAttitude

using LinearAlgebra

function hat(ω)
    ω̂ = [0 -ω[3] ω[2]
        ω[3] 0 -ω[1]
        -ω[2] ω[1] 0]

    return ω̂
end

function rptoq(ϕ)
    (1 / sqrt(1 + ϕ' * ϕ)) * [1; ϕ]
end
function qtorp(q)
    q[2:4] / q[1]
end

function L(q)
    _L = [q[1] -q[2:end]'
        q[2:end] q[1]*I+hat(q[2:end])]
    return _L
end

function R(q)
    _R = [q[1] -q[2:end]'
        q[2:end] q[1]*I-hat(q[2:end])]
    return _R
end

H = zeros(4, 3)
H = [0 0 0
    1 0 0
    0 1 0
    0 0 1]

function G(q)
    q_mat = [-q[2] q[1] q[4] -q[3]
        -q[3] -q[4] q[1] q[2]
        -q[4] q[3] -q[2] q[1]]
    _G = [I zeros(3, 4)
        zeros(3, 3) q_mat]
    return _G
end
end