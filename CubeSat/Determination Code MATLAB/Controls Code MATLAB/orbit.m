function [x,y,z] = orbit(a,e,omega,inc,w,n)
%
% orbit(a,e,i)
%
% a: semimajor axis
% e: excentricity
% i: inclination
% w: argument of perigee
% n: number of points [optional]
 
if (nargin == 5 )
    n = 100; % Default
end
 
if (nargin == 0 || nargin > 6)
    fprintf(1,'error -> wrong number of arguments\n');
    return;
end
 
% v = true anomaly
v = 0:2*pi/n:2*pi;
 
% 2-body problem orbit determination
r = a*(1-e^2)./(1+e*cos(v));
 
x = r.*cos(v);
y = r.*sin(v);
z = zeros(1,length(x));
 
[x,y,z] = pointrot_oiw(x,y,z,omega,inc,w);
end
