using CPUTime

function y(x_start, dx, x_end)
    y = []
    for xi in x_start:dx:x_end
        push!(y, xi^2 + xi + 1)
        println(xi)
    end
    return y
end

@CPUtime ylist = y(0, 1, 10)
println(ylist)