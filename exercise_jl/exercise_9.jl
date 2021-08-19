using CPUTime
using Plots, Colors
using PyCall



mutable struct Node
    x
    y
    cost
    parent
end



function dijkstra(gridmap::Matrix{Int32})
    """ダイクストラで最短経路探索"""
    
    x_max = size(gridmap)[1]
    y_max = size(gridmap)[2]

    function option(x, y)
        """移動可能なノードを返す"""
        option = []
        if x+1 <= x_max
            if y+1 <= y_max & gridmap[y+1, x+1]==0
                push!(option, (1, 1, sqrt(2)))
            end
            if 








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


m = main_1()