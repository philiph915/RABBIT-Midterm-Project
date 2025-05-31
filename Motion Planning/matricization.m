function [A] = matricization(a,M,N)
%#codegen
[m,n] = size(a);
if m<n
    b = a';
    a = b;
end

p = length(a);
if p~= M *N
    disp(' Error ');
    A = 0;
else
    A = zeros(M,N);
    counter = 1;
for k=1:N
    A(:,k) = a(counter:counter+M-1);
    counter = counter+M;
end % end of for
end % end of if

end

