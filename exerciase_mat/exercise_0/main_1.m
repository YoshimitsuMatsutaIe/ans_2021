%% 解法1 %%
% 与えられた任意の整数が素数かどうかの判定 %
prompt = "Please input a number: ";
n = input(prompt);
is_prime(n);

% 与えられた任意の整数リストに素数が入っているかの判定 %
prompt_1 = "Please input numbers(Enter in comma separated format.(ex.[1,2,3])): ";
n_list = input(prompt_1);
is_prime_list(n_list);