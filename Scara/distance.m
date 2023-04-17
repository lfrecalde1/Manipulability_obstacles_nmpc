function [value] = distance(h,ob)
%UNTITLED3 Summary of this function goes here

%% Initial Empy Value
value = [];
xc = h(1);
yc = h(2);

xo = ob(1);
yo = ob(2);

%% ONSTANT VALUES OF THE FUNSTION THESE VALUES NEED TO BE POSITIVE >0
ax = 0.1;
ay = 0.1;
 %% THIS VALUE NEEDS TO BE POSITIVE DEFINED
n = 2;

aux_x = ((xc-xo)^n)/ax;
aux_y = ((yc-yo)^n)/ay;

value = (-aux_x-aux_y);

end

