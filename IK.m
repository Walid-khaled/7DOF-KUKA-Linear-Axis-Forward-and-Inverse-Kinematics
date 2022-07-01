function [q, q_dot] = IK(q_0, link_lengths, p_global, flag)
[T, T1, T2, T3, T4, ~, ~, cur_pos] = FK(q_0, link_lengths);
count = 1;
color_list = {'blue', 'red', 'black'};
figure
hold on
tic
while norm(p_global(1:3) - cur_pos(1:3)) > 1e-02
    %[q, q_dot] = PseudoInverse(q_0, link_lengths, p_global, flag);
    %[q, q_dot] = Damped_LS(q_0, link_lengths, p_global);
    %[q, q_dot] = Null_Space(q_0, link_lengths, p_global, flag);
    [q, q_dot] = TaskAugmentation(q_0, link_lengths, p_global);
    [T, T1, T2, T3, T4, T5, ~, cur_pos] =  FK(q, link_lengths);
    
    clf;
    view(3)
    Visualize_Robot(q, q_dot, T, T1, T2, T3, T4, T5, cur_pos, p_global, color_list, 0)
    pause(0.1);
    
    q_0 = q;
    count = count + 1; 
end
Visualize_Robot(q, q_dot, T, T1, T2, T3, T4, T5, cur_pos, p_global, color_list, 1)
fprintf('Pos = \n')
disp(cur_pos)
toc
end
