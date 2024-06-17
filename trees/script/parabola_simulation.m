%% Simulation of Parabola in 3D invironment

% Define two point of interest where it pass the catenary

reel_z = 0.38;
p_ugv = [4.330 -0.446 0.008]; % UGV sin contar reel
p_uav = [0.862 -0.640 6.192]; % UAV

p1 = [p_ugv(1,1) p_ugv(1,2) p_ugv(1,3)+reel_z]; 
p2 = p_uav;

% Definition of catenary parameters

L = 1.181;
param_par = [0.907 -1.480 0.386];
param_cat = [0.905 2.352 14.880]

%Vcat=[8.5,1.8,0.799998;8.47917,1.7125,0.814312;8.45833,1.625,0.832662;8.4375,1.5375,0.855083;8.41667,1.45,0.88162;8.39583,1.3625,0.912324;8.375,1.275,0.947254;8.35417,1.1875,0.986479;8.33333,1.1,1.03008;8.3125,1.0125,1.07813;8.29167,0.925,1.13073;8.27083,0.8375,1.18799]
p = param_par(1,1);
q = param_par(1,2);
r = param_par(1,3);
num_point_per_unit_length = 20;

x1 = 0;
x2 = sqrt((p1(1,1)-p2(1,1))^2+(p1(1,2)-p2(1,2))^2);
a1= p * (x1^3)/3 + q * (x1^2)/2 + r*x1 ;
a2= p * (x2^3)/3 + q * (x2^2)/2 + r*x2 ;
a = a2-a1

X0 = param_cat(1,1);
Y0 = param_cat(1,2);
C =  param_cat(1,3);
A1 = (C^2 * sinh((x1 - X0)/C)+ (Y0 - C)*x1);
A2 = (C^2 * sinh((x2 - X0)/C)+ (Y0 - C)*x2);
A= A2-A1
% Direction Vector from UGV to UAV

delta_x = (p2(1,1) - p1(1,1));
delta_y = (p2(1,2) - p1(1,2));
delta_z = (p2(1,3) - p1(1,3));
dxy_ = sqrt(delta_x * delta_x + delta_y * delta_y );
dist_= sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z );
u_x = delta_x /dxy_;
u_y = delta_y /dxy_;
d_ = dxy_;
z = abs(p2(1,3)-p1(1,3));

if (dxy_ < 0.0001)
	u_x = 0.0;
    u_y = 0.0;
	d_ = fabs(p2_.z-p1_.z);
end
	
if (d_ < 1.0)
        if (d_ < abs(p2(1,3)-p1(1,3)))
            num_point_catenary = ceil(abs(p2(1,3)-p1(1,3)) * 10.0);
        else
            num_point_catenary = ceil(d_ * 10.0);
        end
else
	num_point_catenary = ceil(num_point_per_unit_length * d_);
end

v=[];
% Catenary Equation
step = 0;
for c = 0:num_point_catenary
    c_x = p1(1,1) + u_x * step;
    c_y = p1(1,2) + u_y * step;
    
    Y = p*step*step+q*step+r; % Catenary equation
    c_z = Y;

    step = step + d_/ num_point_catenary;  
    v=[v;c_x,c_y,c_z];
end

v
param_par
plot3(v(:,1), v(:,2), v(:,3))
hold on;
plot3(p1(1,1),p1(1,2),p1(1,3),'or')
hold on
plot3(p2(1,1),p2(1,2),p2(1,3),'*r')
grid;