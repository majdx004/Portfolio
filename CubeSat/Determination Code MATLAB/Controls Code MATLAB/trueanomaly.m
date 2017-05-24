function [x,y,z] = trueanomaly(a,e,omega,inc,w,v)
%
% a: semimajor axis
% e: excentricity
% i: inclination
% w: argument of perigee
% v: true anomaly
%
% 2-body problem orbit determination
 
r = a*(1-e^2)./(1+e*cos(v));
 
x = r.*cos(v);
y = r.*sin(v);
z = zeros(1,length(x));
 
[x,y,z] = pointrot_oiw(x,y,z,omega,inc,w);
end