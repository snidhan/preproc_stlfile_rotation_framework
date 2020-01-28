%% Name : Sheel Nidhan
%  Date - 12 Januaty, 2020
%  The code reads in the stl files and rotate it by some degrees as per needed
%  Translation can also be included with minimal effort


clear;
clc;
%% Reading the stl file

filename = 'spd_ar6_a0_4.stl';
filename_out = 'spd_ar6_a0_4_angle1_10.stl';

[v,t_norm2] = stlread(filename,2);
[p,t_norm1,tnorm] = stlread(filename,1);

v_rotated = zeros(size(v));
%% Rotate each point by a given angle

% v(:,3) - axial direction,z ; v(:,1) - vertical, x ; v(:,2) - horizontal, y

alpha = -10*(pi/180);   % given in radians


for i = 1:size(v,1)
   coord_x = v(i,1); coord_y = v(i,2); coord_z = v(i,3);
   
   vec = [coord_y coord_x coord_z];
   
   dist_x_axis = norm(cross((vec-[0 1 0]), vec));
   
   theta  = myatan(coord_y, coord_z);
   
   coord_z_modified = dist_x_axis*cos(theta+alpha);
   coord_y_modified = dist_x_axis*sin(theta+alpha);
   
   v_rotated(i,:) = [coord_y_modified  coord_x coord_z_modified];
end

[p,t]=fv2pt(v_rotated,length(v_rotated)/3);%gets points and triangles

idx = [2 1 3];
ps = p(:,idx);

%% Write out stl file

stlwrite(filename_out,t,ps,'mode','ascii')

%%
function [p,t]=fv2pt(v,fnum)

%gets points and triangle indexes given vertex and facet number
c=size(v,1);

%triangles with vertex id data
t=zeros(3,fnum);
t(:)=1:c;

%now we have to keep unique points fro vertex
[p,i,j]=unique(v,'rows'); %now v=p(j) p(i)=v;
t(:)=j(t(:));
t=t';

end

