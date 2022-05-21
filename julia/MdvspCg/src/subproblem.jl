#!/usr/bin/env julia

import JuMP.optimize!

# We need this structure to prevent memory allocations while 
# the shortest path algorithm runs. Other option would be 
# named tuples, but I think struct makes more sense here.
mutable struct Subproblem
    inst::Instance
    depot_id::Int
    
    dist::Vector{Float64}
    pred::Vector{Int}
    
    inqueue::Vector{Bool}
    queue::Vector{Int}

    _sol_path::Vector{Int}
    _sol_cost::Int

    function Subproblem(inst::Instance, depot_id::Int)
        new(
            inst,
            depot_id,
            
            fill(Inf64, inst.num_trips + 2),
            fill(-1, inst.num_trips + 2),
            
            fill(false, inst.num_trips + 2),
            [],

            [],
            0.0
        )
    end
end

function optimize!(prob::Subproblem, depot_dual::Float64, task_duals::Vector{Float64})
    # Artificial nodes representing source and sink
    O = prob.inst.num_trips + 1
    D = prob.inst.num_trips + 2

    # Puts data structures to initial state.
    fill!(prob.dist, Inf64)
    fill!(prob.pred, -1)
    fill!(prob.inqueue, false)
    empty!(prob.queue)

    # This is only used for debugging purpose, to see if the algorithm
    # entered a cicle of negative cost.
    cnt = zeros(Int, prob.inst.num_trips + 2)

    # Set the initial state of each task, departing from the
    # depot when the path is feasible.
    for i in 1:prob.inst.num_trips
        cost = pullout_cost(prob.inst, prob.depot_id, i)
        if cost != -1
            prob.dist[i] = cost - depot_dual
            prob.pred[i] = O
            cnt[i] += 1
            prob.inqueue[i] = true
            push!(prob.queue, i)
        end
    end

    # Repeats the relaxation step until nothing changes in the path.
    while !isempty(prob.queue)
        v = pop!(prob.queue)
        prob.inqueue[v] = false
        idual = task_duals[v]

        # Relax all deadheading arcs
        for i in prob.inst.dh_forward[v]
            succ = i.succ
            cost = i.cost - idual

            if prob.dist[v] + cost < prob.dist[succ]
                prob.dist[succ] = prob.dist[v] + cost
                prob.pred[succ] = v

                if !prob.inqueue[succ]
                    push!(prob.queue, succ)
                    prob.inqueue[succ] = true
                    cnt[succ] += 1

                    if cnt[succ] >= prob.inst.num_trips + 2
                        @error "Negative cycle detected in subproblem of depot #$(prob.depot_id)."
                    end
                end
            end
        end

        # Relax the arc that conects the current task to the sink node
        cost = pullin_cost(prob.inst, prob.depot_id, v)
        if cost != -1
            cost -= idual

            if prob.dist[v] + cost < prob.dist[D]
                prob.dist[D] = prob.dist[v] + cost
                prob.pred[D] = v
            end
        end
    end

    # After solving the shortest path problem, extracts the path 
    # from the pred list, and compute the original column cost.

    # Cleans the old solution if any.
    empty!(prob._sol_path)
    prob._sol_cost = 0.0

    # Only extracts the path if the cost to sink node is not too close to zero.
    if prob.dist[D] <= -0.0000001
        last = prob.pred[D]
        prob._sol_cost += pullin_cost(prob.inst, prob.depot_id, last)
        append!(prob._sol_path, last)

        while prob.pred[last] != O
            prob._sol_cost += deadhead_cost(prob.inst, prob.pred[last], last)
            append!(prob._sol_path, prob.pred[last])
            
            last = prob.pred[last]
        end
        prob._sol_cost += pullout_cost(prob.inst, prob.depot_id, last)
    end

    return (;
        cost=prob._sol_cost,
        path=prob._sol_path
    )
end

