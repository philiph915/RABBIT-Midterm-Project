function [y] = repeat_vector(x,M)

n = length(x);
y = [];
counter = 1;
for k=1:M
    y(counter:counter+n-1) = x;
    counter = counter + n;
end % end of for
y = y';

end

