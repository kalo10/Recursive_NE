function [w, v, w_dot, v_dot, g] = forward_comp(joint_rates, joint_acce, d, r, theta)
    w = zeros(3);
    w_dot = zeros(3);
    v = zeros(3);
    v_dot = zeros(3);

    Q = @(theta) [
        cos(theta) -sin(theta) 0;
        sin(theta) cos(theta) 0;
        0 0 1
    ];

    e = [0 0 1]';
    w(1:3, 1) = e*joint_rates(1);
    v(1:3, 1) = cross(w(:, 1), d(:, 1));
    w_dot(:, 1) = e*joint_acce(1) + cross(w(:,1), e)*joint_rates(1);
    v_dot(:, 1) = cross(w_dot(:, 1), d(:, 1)) + cross(w(:, 1), cross(w(:, 1), d(:, 1)));

    for i = 2:3
        w(:, i) = Q(theta(i-1))'*w(:, i-1) + e*joint_rates(i);
        v(:, i) = Q(theta(i-1))'*(v(:, i-1) + cross(w(:, i-1), r(:, i-1))) + cross(w(:,i),d(:, i));
        w_dot(:, i) = Q(theta(i-1))'*w_dot(:, i-1) + e*joint_acce(i) + cross(w(:,i), e)*joint_rates(i);
        v_dot(:, i) = Q(theta(i-1))'*(v_dot(:, i-1) + cross(w_dot(:, i-1), r(:, i-1)) + cross(w(:, i-1), cross(w(:, i-1), r(:, i-1)))) + cross(w_dot(:, i), d(:, i)) + cross(w(:, i), cross(w(:, i), d(:, i)));
    end
     
    g = -9.8*[0; 0; 1];
    
end
