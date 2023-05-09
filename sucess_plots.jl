using SatellitePlayground
SP = SatellitePlayground
include("benchmark.jl")

# Sucess rate vs. initial attitude error
success_rates = []
initial_attitude_errors = []
for q_err in 1:1:15
  @show q_err
  success_rate = benchmark(4, 10, q_err, q_err + 1, 0, false)
  push!(success_rates, success_rate)
  push!(initial_attitude_errors, q_err + 1)
end

plot(initial_attitude_errors, success_rates, xlabel="Initial attitude error (deg)", ylabel="Success rate", title="Success rate vs. initial attitude error", legend=false)