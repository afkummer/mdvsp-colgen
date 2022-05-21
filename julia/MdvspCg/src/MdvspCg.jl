module MdvspCg

using GLPK

include("instance.jl")

export Instance,
    pullout_cost,
    pullin_cost,
    deadhead_cost,
    read_huisman_instance

include("main.jl")

export build_main_problem, 
    add_column!

include("subproblem.jl")

export optimize!

function gencol()
    inst = read_huisman_instance("../mdvsp-colgen/instances/m4n500s0.inp")
    #inst = read_huisman_instance("../mdvsp-colgen/instances/m8n1500s4.inp")

    main = build_main_problem(inst)
    sp = map(1:inst.num_depots) do k
        Subproblem(inst, k)
    end

    # write_to_file(main, "main.lp")

    # Configures the optimizer
    set_optimizer(main, GLPK.Optimizer)

    for iter in 1:5000
        optimize!(main)
        #println("Iter: $iter  Main obj: $(objective_value(main))")

        depot_duals = dual.(main[:depot_cap])
        task_duals = dual.(main[:task_asg])
        newcols = Vector{Any}(undef, inst.num_depots)

        Threads.@threads for k in 1:inst.num_depots
            newcols[k] = optimize!(sp[k], depot_duals[k], task_duals)
        end

        converged = true
        for k in 1:inst.num_depots
            if !isempty(newcols[k].path)
                converged = false
                add_column!(main, k, newcols[k].cost, newcols[k].path)
            end
        end

        if converged
            println("** COLGEN CONVERGED! **")
            break
        end

        # write_to_file(main, "main.$iter.lp")
    end

    write_to_file(main, "main.final.lp")

    return
end

end # module
