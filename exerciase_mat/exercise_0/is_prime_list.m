function is_prime_list(n_list)
    result = [];
    for i = n_list
        result(end+1) = is_prime(i) ;
    end
    if find(result == 0)
        disp("There are prime numbers in the list")
    else 
        disp("There are no prime numbers in the list")
    end
end