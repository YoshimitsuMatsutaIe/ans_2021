using CPUTime
using Plots, Colors
using PyCall: pyimport
#using Conda
#cv2 = pyimport("cv2")
cv2 = pyimport_conda("cv2", "opencv")

mutable struct Node
    x::Int64
    y::Int64
    cost::Float64
    parent::Tuple{Any, Any}
end


function options(x, y, x_max, y_max, gridmap)
    """移動可能なノード
    ・なぜか関数dijkstra内で定義したらエラー
    ・[]が良くない？
    """

    options = []
    if x+1 <= x_max
        if y+1 <= y_max
            if gridmap[y+1, x+1] == 0
                push!(options, (1, 1, sqrt(2)))
            end
        end
        if y >= 2
            if gridmap[y-1, x+1] == 0
                push!(options, (1, -1, sqrt(2)))
            end
        end
        if gridmap[y, x+1] == 0
            push!(options, (1, 0, 1))
        end
    end
    if x >= 2
        if y+1 <= y_max
            if gridmap[y+1, x-1] == 0
                push!(options, (-1, 1, sqrt(2)))
            end
        end
        if y >= 2
            if gridmap[y-1, x-1] == 0
                push!(options, (-1, -1, sqrt(2)))
            end
        end
        if gridmap[y, x-1] == 0
            push!(options, (-1, 0, 1))
        end
    else
        if y+1 <= y_max
            if gridmap[y+1, x] == 0
                push!(options, (0, 1, 1))
            end
        end
        if y >= 2
            if gridmap[y-1, x] == 0
            push!(options, (0, -1, 1))
            end
        end
    end

    return options
end

function dijkstra(gridmap::Matrix{Int64}, start, goal)
    """ダイクストラで最短経路探索"""
    
    x_max = size(gridmap)[1]
    y_max = size(gridmap)[2]

    function find_MinCost_id(d)
        """一番costが小さいnodeのidを探す"""
        for (i, id) in enumerate(keys(d))
            if i==1
                global minid = id
            elseif d[id].cost < d[minid].cost
                minid = id
            end
        end
        minid
    end

    start_node = Node(start[1], start[2], 0, (nothing, nothing))
    goal_node = Node(goal[1], goal[2], typemax(Float64), (nothing, nothing))

    open_set = Dict()
    closed_set = Dict()

    start_id = (start_node.x, start_node.y)
    open_set[start_id] = start_node

    while true
        temp_id = find_MinCost_id(open_set)
        temp_node = open_set[temp_id]

        if temp_node.x == goal_node.x & temp_node.y == goal_node.y
            println("探索終了")
            goal_node.parent = temp_node.parent
            goal_node.cost = temp_node.cost
            break
        end

        delete!(open_set, temp_id)
        closed_set[temp_id] = temp_node

        os = options(temp_node.x, temp_node.y, x_max, y_max, gridmap)
        for o in os
            Δx, Δy, Δcost = o
            new_node = Node(temp_node.x + Δx, temp_node.y + Δy, temp_node.cost + Δcost, temp_id)
            new_id = (new_node.x, new_node.y)

            if haskey(closed_set, new_id)
                """決定済みのとき"""
                continue
            elseif !haskey(open_set, new_id)
                """未探索のとき"""
                open_set[new_id] = new_node
            else
                """未決定のとき"""
                if open_set[new_id].cost >= new_node.cost
                    open_set[new_id] = new_node
                end
            end
        end
    end

    function compute_optiomal_path(start_node, goal_node, closed_set)
        """決定済み集合から最短経路を組む"""

        rx, ry = [goal_node.x], [goal_node.y]
        parent = goal_node.parent
        cost = goal_node.cost

        while parent != (start_node.x, start_node.y)
            node = closed_set[parent]
            push!(rx, node.x)
            push!(ry, node.y)
            cost += node.cost
            parent = node.parent
        end

        push!(rx, start_node.x)
        push!(ry, start_node.y)

        rx, ry, cost
    end


    rx, ry, total_cost = compute_optiomal_path(start_node, goal_node, closed_set)

    rx, ry, total_cost
end


function draw(rx, ry, start, goal, gridmap)
    plot(
        rx, ry,
        aspect_ratio=1, label="optiomal path", legend=:bottomright,
        xlims=(0, 11), ylims=(0, 11)
    )
    scatter!(start, markershape=:diamond, label="start")
    scatter!(goal, markershape=:star6, label="goal")
    
    # 障害物位置のデータ作成
    obs_id = findall(gridmap .> 0)  # 障害物のindex検索
    obs_x = Vector{Int64}(undef, length(obs_id))
    obs_y = Vector{Int64}(undef, length(obs_id))
    for i in 1:length(obs_id)
        obs_x[i] = obs_id[i][2]
        obs_y[i] = obs_id[i][1]
        
    end
    scatter!(obs_x, obs_y, marlershape=:star, label="obstacle")
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

    start = (1, 1)
    goal = (10, 10)

    rx, ry, total_cost = dijkstra(map_example, start, goal)  # ダイクストラ法実行
    draw(rx, ry, start, goal, map_example)


end


@time main_1()