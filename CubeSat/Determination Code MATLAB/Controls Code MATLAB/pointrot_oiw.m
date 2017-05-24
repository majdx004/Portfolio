function [x2,y2,z2] = pointrot_oiw(x1,y1,z1,omega,inc,w)
%
% pointrot_oiw(omega,inc,w)
%
% omega: right ascension angle
% inc : inclination angle
% w : argument of perigee angle
%
% point rotation in the plane ( frame fixed )
%
%  Rz(omega)*Rx(inc)*Rz(w)
%
 
Mrot = [cos(omega) * cos(w) - sin(omega) * cos(inc) * sin(w) -cos(omega) * sin(w) - sin(omega) * cos(inc) * cos(w) sin(omega) * sin(inc); 
        sin(omega) * cos(w) + cos(omega) * cos(inc) * sin(w) -sin(omega) * sin(w) + cos(omega) * cos(inc) * cos(w) -cos(omega) * sin(inc); 
        sin(inc) * sin(w) sin(inc) * cos(w) cos(inc);];
 
 
RotResult = Mrot*[x1;y1;z1];
 
    
x2 = RotResult(1,:);
y2 = RotResult(2,:);
z2 = RotResult(3,:);
 
end
