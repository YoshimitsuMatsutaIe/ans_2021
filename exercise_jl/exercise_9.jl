using CPUTime
using Plots, Colors
using PyCall



mutable struct Node
    x
    y
    cost
    parent
end



function dijkstra(gridmap::Matrix{Int64}, start::Tuple{Int64}, goal::Tuple{Int64})
    """ダイクストラで最短経路探索"""
    
    x_max = size(gridmap)[1]
    y_max = size(gridmap)[2]

    function options(x, y)
        """移動可能なノード"""
        options = []
        if x+1 <= x_max
            if y+1 <= y_max & gridmap[y+1, x+1] == 0
                push!(options, (1, 1, sqrt(2)))
            end
            if y-1 >= y_max & gridmap[y-1, x+1] == 0
                push!(options, (1, -1, sqrt(2)))
            end
            if gridmap[y, x+1] == 0
                push!(options, (1, 0, 1))
            end
        end
        if x-1 >= 0
            if y+1 <= y_max & gridmap[y+1, x-1] == 0
                push!(options, (-1, 1, sqrt(2)))
            end
            if y-1 >= y_max & gridmap[y-1, x-1] == 0
                push!(options, (-1, -1, sqrt(2)))
            end
            if gridmap[y, x-1] == 0
                push!(options, (-1, 0, 1))
            end
        else
            if y+1 <= y_max & gridmap[y+1, x] == 0
                push!(options, (0, 1, 1))
            end
            if y-1 >= 0 & gridmap[y-1, x] == 0
                push!(options, (0, -1, 1))
            end
        end
    end

    start_node = Node(start[1], start[2], 0, nothing)
    goal_node = Node(goal[1], goal[2], typemax(Float64), nothing)

    open_set = Dict()
    closed_set = Dict()

    









end












function main_1()
    map_example = [
        0 0 0 0 0 0 1 0 0 0
        0 1 0 0 0 0 0 0 1 0
        0 1 1 1 1 0 0 0 1 0
        0 0 0 0 0 0 0 0 1 0
        0 0 0 1 1 0 0 0 1 0
        0 0 0 1 1 0 0 0 1 0
        0 0 0 1 1 0 0 0 0 0
        0 0 0 0 0 0 1 0 0 0
        0 0 0 0 0 0 1 0 0 0
        0 0 0 0 0 0 1 0 0 0
    ]  # mapの例．0は通過可能な点，1は通貨不可能な点を示す

    return map_example

end


