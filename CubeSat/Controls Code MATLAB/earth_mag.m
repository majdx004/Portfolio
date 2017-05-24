%% Sun Synchronous Orbit: Find XYZ positions and Plot
 
T = linspace(0,98.9*60,101); %min orbital period
 
% Orbital Parameters for Sun Synchronous Orbit (pulled from web)
a = 6378+705; %km semi major axis
inc = 98.2; %degrees inclination
e = 0; %eccentricity
omega = 270; %RAAN degrees
w = 90; %argument of perigee degrees
v = 0; % true anomaly degrees
 
% Gets the position and velocity vectors 
[x,y,z] = orbit(a, e, omega*pi/180, inc*pi/180, w*pi/180);
[xv,yv,zv] = trueanomaly(a,e,omega*pi/180,inc*pi/180,w*pi/180,v*pi/180);
r = 6371;
[xp,yp,zp] = planet(r,20); % earth radius ( km )
 
if( inc > 90 )
    orbit_color = 'red'; % retrograde
elseif (inc == 90 || inc == 180 )
    orbit_color = 'black'; % polar
else 
    orbit_color = 'blue'; % direct
end
 
figure(1);
subplot(2,2,1)
plot3(x,y,z,orbit_color,'Linewidth',2); % direct
axis equal
hold on
 
plot3(xv,yv,zv,'blacko','Linewidth',2); % plot true anomaly
surf(xp,yp,zp,'EdgeAlpha',0.4);   % plot the planet
colormap([0  0.5  0.8]);
scale = 2;
axis([-scale*a scale*a -scale*a scale*a -scale*a scale*a])
hold on
 
 
% Get Axis properties 
axis_data = get(gca);
xmin = axis_data.XLim(1);
xmax = axis_data.XLim(2);
ymin = axis_data.YLim(1);
ymax = axis_data.YLim(2);
zmin = axis_data.ZLim(1);
zmax = axis_data.ZLim(2);
 
% I, J ,K vectors
plot3([xmin,xmax],[0 0],[0 0],'black','Linewidth',1); plot3(xmax,0,0,'black>','Linewidth',1.5);
hold on
plot3([0 0],[ymin,ymax],[0 0],'black','Linewidth',1); plot3(0,ymax,0,'blue>','Linewidth',1.5);
hold on
plot3([0 0],[0 0],[zmin,zmax],'black','Linewidth',1); plot3(0,0,zmax,'red^','Linewidth',1.5);
hold on 
 
% right ascending node line plot
xomega_max = xmax*cos(omega*pi/180);
xomega_min = xmin*cos(omega*pi/180);
yomega_max = ymax*sin(omega*pi/180);
yomega_min = ymin*sin(omega*pi/180);
 
xlabel('I');
ylabel('J');
zlabel('K');
 
plot3([xomega_min xomega_max], [yomega_min yomega_max], [0 0], 'g','Linewidth',1.5);
hold on
 
% add equatorial plan
xe = [xmin xmax;xmin xmax]; ye = [ymax ymax;ymin ymin]; ze = [0 0; 0 0];
eq_alpha = 0.3; % transparancy 
mesh(xe,ye,ze,'FaceAlpha',eq_alpha,'FaceColor',[0.753,0.753,0.753]);
 
grid on
hold off
 
%% Calculate Earth's Mag Field: Convert XYZ to Lat/Lon coordinates
[lat,lon,h]=xyz2ell(x,y,z,a,0);
 
alt = 705*ones(1,101);
lat = 180*lat/pi;
lon = 180*lon/pi;
 
for k =1:length(lat)
    [xyz,h,dec,dip,f] = wrldmagm(alt(k),lat(k),lon(k),2016);
    f_save(k) = f;
    xmag(k) = xyz(1);
    ymag(k) = xyz(2);
    zmag(k) = xyz(3);
    
    
end
 
% F is total magnetic field intensity
f_save = f_save.*(1e-9); % convert nT to T
xmag = xmag.*(1e-9); % convert nT to T
ymag = ymag.*(1e-9); % convert nT to T
zmag = zmag.*(1e-9); % convert nT to T
 
% Plot of magnetic field (y) vs lattitude location (x)
subplot(2,2,2)
plot(lat,xmag,'r.');
title('Mag Earth X')
xlabel('Lattitude (Degrees)');
ylabel('Magnetic Field (T)');
grid on;
 
subplot(2,2,3)
plot(lat,ymag,'r.');
title('Mag Earth Y')
xlabel('Lattitude (Degrees)');
ylabel('Magnetic Field (T)');
grid on;
 
subplot(2,2,4)
plot(lat,zmag,'r.');
title('Mag Earth Z')
xlabel('Lattitude (Degrees)');
ylabel('Magnetic Field (T)');
grid on;
 
 
earth_mag_x = sum(xmag)/length(xmag);
earth_mag_y = sum(ymag)/length(ymag);
earth_mag_z = sum(zmag)/length(zmag);
 
Tx = [T;xmag];
Ty = [T;ymag];
Tz = [T;zmag];
Tx = Tx'; % magnetic fields with corresponding time in orbit
Ty = Ty'; % magnetic fields with corresponding time in orbit
Tz = Tz'; % magnetic fields with corresponding time in orbit
