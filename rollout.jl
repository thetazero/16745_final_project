using SatellitePlayground

function rollout(state, steps, dt)
    function log_step(hist, state)
        push!(hist, state)
    end
    function log_end(hist)
        return hist
    end
    function terminate(state, params, time, i)
        return false
    end
    return SatellitePlayground.simulate(control, log_step=log_step, max_iterations=steps, dt=dt,
        terminal_condition=terminate, log_end=log_end, initial_condition=state)
end