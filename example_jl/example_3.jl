using Plots

anim = @animate for i in 1:200
    #xs = range(0:0.1:2*pi)
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
gif(anim, "anim_fps15.gif", fps = 6)

