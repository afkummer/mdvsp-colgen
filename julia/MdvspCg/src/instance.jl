#!/usr/bin/env julia

mutable struct Instance
    num_depots::Int
    num_trips::Int
    depot_capacity::Vector{Int}
    adj_matrix::Matrix{Int}

    dh_forward::Vector{Vector{@NamedTuple{succ::Int, cost::Int}}}
    dh_backward::Vector{Vector{@NamedTuple{pred::Int, cost::Int}}}

    function Instance(num_depots, num_trips)
        L = num_depots + num_trips
        new(
            num_depots,
            num_trips,
            zeros(Int, num_depots),
            fill(-1, L, L),
            Vector{Vector{@NamedTuple{succ::Int, cost::Int}}}(undef, num_trips),
            Vector{Vector{@NamedTuple{pred::Int, cost::Int}}}(undef, num_trips)
        )
    end
end

# Helper functions
pullout_cost(inst::Instance, depot_id::Int, task_id::Int) = inst.adj_matrix[depot_id, inst.num_depots+task_id]

pullin_cost(inst::Instance, depot_id::Int, task_id::Int) = inst.adj_matrix[inst.num_depots+task_id, depot_id]

deadhead_cost(inst::Instance, pred_id::Int, succ_id::Int) = inst.adj_matrix[inst.num_depots+pred_id, inst.num_depots+succ_id]

function read_huisman_instance(fname::String)
    open(fname, "r") do fid
        header = split(readline(fid))

        num_depots = parse(Int, header[1])
        num_trips = parse(Int, header[2])

        inst = Instance(num_depots, num_trips)
        inst.depot_capacity = parse.(Int, header[3:end])

        row_idx = 1
        while !eof(fid)
            inst.adj_matrix[row_idx, :] = parse.(Int, split(readline(fid)))
            row_idx += 1
        end

        _create_deadhead_cache(inst)

        return inst
    end
end

function _create_deadhead_cache(inst::Instance)
    for i in 1:inst.num_trips
        fwd = Vector{@NamedTuple{succ::Int, cost::Int}}(undef, 0)
        bwd = Vector{@NamedTuple{pred::Int, cost::Int}}(undef, 0)

        for j in 1:inst.num_trips
            cost = deadhead_cost(inst, i, j)
            if cost != -1
                push!(fwd, (; :succ => j, :cost => cost))
            end
        
            cost = deadhead_cost(inst, j, i)
            if cost != -1
                push!(bwd, (; :pred => j, :cost => cost))
            end
        end

        inst.dh_forward[i] = fwd
        inst.dh_backward[i] = bwd
    end
end