using CPUTime


# 簡単な実装
function primality_test_1(n)
    print(n)
    println(" is ...")
    if n < 2
        println("no")
    else
        for i in 2:n
            #println(i)
            if i == n
                println("prime number!")
            elseif n % i == 0
                println("no")
                break
            end
        end
    end
end




@time primality_test_1(41263)