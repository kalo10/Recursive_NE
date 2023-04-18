function [joint_reaction, joint_torque] = backward_comp(w, w_dot, v_dot, r, d, m, g, external_force, external_torque)
    
    I = @(n) [
            0 0 0;
            0 0 0;
            0 0 (m(n)^2)/12
        ];
    f = zeros(3);
    n = zeros(3);

    joint_reaction = zeros(3);
    joint_torque = zeros(3);
    for i = 1:3
        f(:, i) = m(i)*v_dot(:, i);
        n(:, i) = I(i)*w_dot(:, i) + cross(w(:, i), I(i)*w(:, i));
    end
    

    joint_reaction(:, 3) = external_force + f(:, 3) - m(3)*g;
    for i = 2:-1:1
        joint_reaction(:, i) = -joint_reaction(:, i+1) + f(:, i) - m(i)*g;  
    end

    joint_torque(:, 3) = external_torque + n(:, 3) + cross(d(:, 3), joint_reaction(:, 3)) + cross(r(:, 3), -joint_reaction(:, i+1));
    for i = 2:-1:1
        joint_torque(:, i) = -joint_torque(:, i+1) + n(:, i) + cross(d(:, i), joint_reaction(:, i)) + cross(r(:, i), -joint_reaction(:, i+1));  
    end
    
end