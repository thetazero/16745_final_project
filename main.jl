using SatellitePlayground
using Plots
using LinearAlgebra

include("nominal_dynamics.jl")
include("rollout.jl")

function measure(state, params, t)
    return (
        state.attitude,
        state.angular_velocity
    )
end

function control(measurement)
    (attitude, angular_velocity) = measurement
    # return zero(Control)
    return Control([0.001, 0.001, 0.001])
end

function log_step(hist, state)
    push!(
        hist,
        state.attitude
    )
end

@time (data, time) = SatellitePlayground.simulate(control, measure=measure,
    log_step=log_step, dt=0.1)
plot(time, data, title="Attitude", xlabel="Time (s)", ylabel="Quaternion Units (idk)", labels=["q1" "q2" "q3" "q4" "q norm"])

