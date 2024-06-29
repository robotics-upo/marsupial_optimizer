clear;
clc;

par1=[4.13810 -0.00476 0.00000 0.65000 -0.35000 6.20000 0.889 -1.456 0.380]; %Posicion 36 catenary (coll car 36 - 38)
par2=[4.13810 -0.00476 0.00000 0.65000 -0.35000 6.20000 0.811206 -1.20008 0.442935];%Posicion 36 parabola

num_point_per_unit_length = 20;
delta_x = (par1(1,4) - par1(1,1));
delta_y = (par1(1,5) - par1(1,2));
delta_z = (par1(1,6) - par1(1,3));
dxy_ = sqrt(delta_x * delta_x + delta_y * delta_y );
dist_= sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z );
u_x = delta_x /dxy_;
u_y = delta_y /dxy_;
d_ = dxy_;
d_z = abs(par1(1,3)-par1(1,6));

if (dxy_ < 0.0001)
	u_x = 0.0;
    u_y = 0.0;
	d_ = d_z;
end
	
if (d_ < 1.0)
        if (d_ < abs(par1(1,3)-par1(1,3)))
            num_point_catenary = ceil(abs(par1(1,3)-par1(1,3)) * 10.0);
        else
            num_point_catenary = ceil(d_ * 10.0);
        end
else
	num_point_catenary = ceil(num_point_per_unit_length * d_);
end

v_pts_par1=[]; v_pts_par2=[];
% Catenary Equation
step = 0;
for c = 0:num_point_catenary
    x = par1(1,1) + u_x * step;
    y = par1(1,2) + u_y * step;
    
    z_p1 = par1(1,7)*step*step+par1(1,8)*step+par1(1,9); % Catenary equation
    z_p2 = par2(1,7)*step*step+par2(1,8)*step+par2(1,9); % Catenary equation

    step = step + d_/ num_point_catenary;  
    v_pts_par1=[v_pts_par1;x,y,z_p1];
    v_pts_par2=[v_pts_par2;x,y,z_p2];
end

grid
plot3(par1(1,1),par1(1,2),par1(1,3)+0.38,'oc')
hold on
plot3(par1(1,4),par1(1,5),par1(1,6),'*c')
hold on
plot3(v_pts_par1(:,1), v_pts_par1(:,2), v_pts_par1(:,3),'b')
hold on;
plot3(v_pts_par2(:,1), v_pts_par2(:,2), v_pts_par2(:,3),'r')
grid;