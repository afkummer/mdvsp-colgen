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

    timings = open("timings.csv", "w")
    write(timings, "event,iter,time,bytes,gctime\n")
    function tm_append(msg, iter, tm)
        write(timings, "$(msg),$(iter),$(tm.time),$(tm.bytes),$(tm.gctime)\n")
    end

    # Configures the optimizer
    set_optimizer(main, GLPK.Optimizer)

    for iter in 1:10000
        tm = @timed optimize!(main)
        tm_append("solve_main", iter, tm)
    
        println("Iter: $iter  Main obj: $(objective_value(main))")

        depot_duals = dual.(main[:depot_cap])
        task_duals = dual.(main[:task_asg])
        newcols = Vector{Any}(undef, inst.num_depots)

        tm = @timed begin
            Threads.@threads for k in 1:inst.num_depots
                newcols[k] = optimize!(sp[k], depot_duals[k], task_duals)
            end
        end
        tm_append("solve_pricing", iter, tm)

        converged = true
        tm = @timed begin
            for k in 1:inst.num_depots
                if !isempty(newcols[k].path)
                    converged = false
                    add_column!(main, k, newcols[k].cost, newcols[k].path)
                end
            end
        end
        tm_append("add_columns", iter, tm)

        if converged
            if !main[:asg_relax]
                println("** COLGEN CONVERGED! **")
                break
            else
                println("*** CHANGING RELAXATION STATUS ***")
                tm = @timed set_main_asg_equals!(main)
                tm_append("set_equals", iter, tm)
            end
        end

        # write_to_file(main, "main.$iter.lp")
    end
    
    write_to_file(main, "main.final.lp")
    close(timings)

    return main
end

end # module
