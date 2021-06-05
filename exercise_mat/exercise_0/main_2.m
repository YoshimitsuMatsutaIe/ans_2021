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