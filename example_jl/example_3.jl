using Plots


function makeplot()
    anim = @animate for i in 1:200
        scatter(
            t->sinpi(t+i/100),
            t->cospi(t+i/100),
            [0],
            size=(400,400),
            xlims = (-1.5, 1.5),
            ylims = (-1.5, 1.5),
            aspect_ratio =1,
        )
    end
    gif(anim, "example_3_by_jl.gif", fps = 6)
end


@time makeplot()
