using Plots

# anim = @animate for theta in 0:pi/10:2*pi
#     plot()

anim = @animate for i in 1:10
    plot(t->sinpi(t+i/5), range(0, 2, length=100))
end