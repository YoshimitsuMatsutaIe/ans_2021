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