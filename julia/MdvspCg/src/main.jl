#!/usr/bin/env julia

using JuMP

function build_main_problem(inst::Instance)
    # Creates a model without any attached optimizer
    m = Model()

    # Adds some control elements to the model object
    m[:asg_relax] = true

    # Creates a dummy column to provide an (artificial) initial solution.
    @variable(m, dummy_initial_sol[i in 1:inst.num_trips] >= 0)
    @objective(m, Min, sum(1e7 * dummy_initial_sol[i] for i in 1:inst.num_trips))

    # Adds the two constraints of the set partitioning-like formulation
    @constraint(m, task_asg[i in 1:inst.num_trips], dummy_initial_sol[i] >= 1)
    @constraint(m, depot_cap[k in 1:inst.num_depots], 0 <= inst.depot_capacity[k])

    # Adds a container to hold the columns generated.
    m[:path] = Vector{VariableRef}(undef, 0)

    return m
end

function add_column!(main::JuMP.Model, depot_id::Int, cost::Int, path::Vector{Int})
    # Creates a new column
    x = @variable(main, lower_bound = 0.0)
    push!(main[:path], x)
    set_name(x, "p_$(depot_id)_$(num_variables(main))")

    # Defines it cost
    set_objective_coefficient(main, x, cost)

    # Adds the coefficient for task assignment constraints
    for i in path
        set_normalized_coefficient(main[:task_asg][i], x, 1.0)
    end

    # Adds the coefficient for the depot
    set_normalized_coefficient(main[:depot_cap][depot_id], x, 1.0)
end
