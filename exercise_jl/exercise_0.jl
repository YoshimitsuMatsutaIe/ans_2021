using CPUTime


"""
素数判定
"""
function is_prime_simple(n::T) where T
    if n < 2
        return false
    elseif n == 2
        return true
    else
        for i in 2:n-1
            #println(i, "...")
            if n % i == 0
                return false
            end
            #println("わりきれない")
        end
    end
    return true
end

function do_2(n₀, n₁)
    ns = n₀:n₁
    prime_list = is_prime_simple.(ns)
    num = prime_list' * prime_list
    println(n₀, " から ", n₁ ," に素数は", num, "個ある")
end



#is_prime_simple(999961)
@time is_prime_simple(999961)  # 100万以下で最大の素数


#do_2(1, 10)
#@time do_2(1, 999961)