function exercise_0()
%EXERCISE_0 この関数の概要をここに記述
%   詳細説明をここに記述

%% 解法1 %%
% 与えられた任意の整数が素数かどうかの判定 %
prompt = "Please input a number: ";
n = input(prompt);
is_prime(n);

% 与えられた任意の整数リストに素数が入っているかの判定 %
prompt_1 = "Please input numbers(Enter in comma separated format.(ex.[1,2,3])): ";
n_list = input(prompt_1);
is_prime_list(n_list);

%% 解法2 %%
% 与えられた任意の整数が素数かどうかの判定 %
prompt = "Please input a number: ";
n = input(prompt);
judge_1 = isprime(n); % logical配列を返すことに注意．0はFalse,1はTrue.
if judge_1 == 1
    disp("True")
else
    disp("False")
end

% 与えられた任意の整数リストに素数が入っているかの判定 %
prompt_1 = "Please input numbers(Enter in comma separated format.(ex.[1,2,3])): ";
n_list = input(prompt_1);
judge_2 = isprime(n_list);
if find(judge_2 == 1)
    disp("There are prime numbers in the list")
else
    disp("There are no prime numbers in the list")
end

end

function a = is_prime(n)
global a    
a = 0;
switch n
    case n == 1
        a = -1;
        disp("False")
    otherwise
        for i = 2:1:n-1
            if mod(n,i) == 0
                a = a + 1;
            end
        end
        if a == 0
            disp("True")
        else
            disp("False")
        end
end
end

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