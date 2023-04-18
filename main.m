

% ---INPUT---- 

a = [
   2;
   2;
   2
];


m = [
   0;
   0;
   0
];

theta = [
    0;
    0;
    0
];


joint_rates = [
    2;
    2;
    2
];

joint_acce = [
    0;
    0;
    0
];

external_force = [
    0;
    0;
    0
];

external_torque = [
    0;
    0;
    0
];


% -------------

theta = deg2rad(theta);

d = [
    a(1)*cos(theta(1))/2,  a(2)*cos(theta(2))/2 , a(3)*cos(theta(3))/2;
    a(1)*sin(theta(1))/2,  a(2)*sin(theta(2))/2 , a(3)*sin(theta(3))/2;
    0, 0, 0
];

r = d;



[w,v, w_dot, v_dot, g] = forward_comp(joint_rates, joint_acce, d, r, theta);

disp(w)
disp(v)

                                                                                                                                                                                                                                                                                                                                                                         