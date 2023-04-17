function [value] = distance(h,ob)
%UNTITLED3 Summary of this function goes here

%% Initial Empy Value
value = [];
xc = h(1);
yc = h(2);
zc = h(3);

xo = ob(1);
yo = ob(2);
zo = ob(3);
%% ONSTANT VALUES OF THE FUNSTION THESE VALUES NEED TO BE POSITIVE >0
ax = 0.1;
ay = 0.1;
az = 0.1;
 %% THIS VALUE NEEDS TO BE POSITIVE DEFINED
n = 2;

aux_x = ((xc-xo)^n)/ax;
aux_y = ((yc-yo)^n)/ay;
aux_z = ((zc-zo)^n)/az;

value = (-aux_x-aux_y-aux_z);

end

