function [x,y,z] = planet(r,n)
%
% planet_sphere(r,[n])
% 
% r = radius
% n = number of points ( optional )
%
% author: Anders Edfors Vannevik
%
 
if (nargin == 1 )
    n = 20; % Default
end
 
if (nargin == 0 || nargin > 2)
    fprintf(1,'error -> wrong number of arguments\n');
    return;
end
 
[x1,y1,z1] = sphere(n);
 
x = r.*x1;
y = r.*y1;
z = r.*z1;
 
end
