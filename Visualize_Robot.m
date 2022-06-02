function [] = Visualize_Robot(q, q_dot, T, T1, T2, T3, T4, cur_pos, p_global, color_list, style)
limit = 1.3;
%% Plotting
hold on
grid on 
line([-limit limit],[0 0],[0 0],'Color','k','LineStyle','--')
line([0 0],[-limit limit],[0 0],'Color','k','LineStyle','--')
line([0 0],[0 0],[-limit limit],'Color','k','LineStyle','--')


config_joint_1 = T1;
config_joint_2 = T2;
config_joint_3 = T3;
config_joint_4 = T4;
config_joint_5 = T;

Tbase = eye(4);

if style == 1
    line([Tbase(1,4) config_joint_1(1,4)],[Tbase(2,4) config_joint_1(2,4)],[Tbase(3,4) config_joint_1(3,4)],...
        'linewidth', 1,'Color',color_list{1}, 'LineStyle', '--');
    line([config_joint_1(1,4) config_joint_2(1,4)],[config_joint_1(2,4) config_joint_2(2,4)],...
         [config_joint_1(3,4) config_joint_2(3,4)],'linewidth', 1,'Color',color_list{2}, 'LineStyle', '--')
    line([config_joint_2(1,4) config_joint_3(1,4)],[config_joint_2(2,4) config_joint_3(2,4)],...
         [config_joint_2(3,4) config_joint_3(3,4)],'linewidth', 1,'Color',color_list{3}, 'LineStyle', '--')
    line([config_joint_3(1,4) config_joint_4(1,4)],[config_joint_3(2,4) config_joint_4(2,4)],...
         [config_joint_3(3,4) config_joint_4(3,4)],'linewidth', 1,'Color',color_list{1}, 'LineStyle', '--')
    line([config_joint_4(1,4) config_joint_5(1,4)],[config_joint_4(2,4) config_joint_5(2,4)],...
         [config_joint_4(3,4) config_joint_5(3,4)],'linewidth', 1,'Color',color_list{2}, 'LineStyle', '--')

else 
    line([Tbase(1,4) config_joint_1(1,4)],[Tbase(2,4) config_joint_1(2,4)],[Tbase(3,4) config_joint_1(3,4)],...
        'linewidth', 3,'Color',color_list{1});
    line([config_joint_1(1,4) config_joint_2(1,4)],[config_joint_1(2,4) config_joint_2(2,4)],...
         [config_joint_1(3,4) config_joint_2(3,4)],'linewidth', 3,'Color',color_list{2})
    line([config_joint_2(1,4) config_joint_3(1,4)],[config_joint_2(2,4) config_joint_3(2,4)],...
         [config_joint_2(3,4) config_joint_3(3,4)],'linewidth', 3,'Color',color_list{3})
    line([config_joint_3(1,4) config_joint_4(1,4)],[config_joint_3(2,4) config_joint_4(2,4)],...
         [config_joint_3(3,4) config_joint_4(3,4)],'linewidth', 3,'Color',color_list{1})
    line([config_joint_4(1,4) config_joint_5(1,4)],[config_joint_4(2,4) config_joint_5(2,4)],...
         [config_joint_4(3,4) config_joint_5(3,4)],'linewidth', 3,'Color',color_list{2})


end
plot3(Tbase(1,4),Tbase(2,4),Tbase(3,4),'go','linewidth', 4,'MarkerSize', 4)
plot3(config_joint_1(1,4),config_joint_1(2,4),config_joint_1(3,4),'go','linewidth', 4,'MarkerSize',4)
plot3(config_joint_2(1,4),config_joint_2(2,4),config_joint_2(3,4),'go','linewidth', 4,'MarkerSize',4)
plot3(config_joint_3(1,4),config_joint_3(2,4),config_joint_3(3,4),'go','linewidth', 4,'MarkerSize',4)
plot3(config_joint_4(1,4),config_joint_4(2,4),config_joint_4(3,4),'go','linewidth', 4,'MarkerSize',4)
plot3(config_joint_5(1,4),config_joint_5(2,4),config_joint_5(3,4),'go','linewidth', 4,'MarkerSize',4)

plot3(Tbase(1,4),Tbase(2,4),Tbase(3,4),'o')
plot3(config_joint_1(1,4),config_joint_1(2,4),config_joint_1(3,4),'o')
plot3(config_joint_2(1,4),config_joint_2(2,4),config_joint_2(3,4),'o')
plot3(config_joint_3(1,4),config_joint_3(2,4),config_joint_3(3,4),'o')
plot3(config_joint_4(1,4),config_joint_4(2,4),config_joint_4(3,4),'o')
plot3(config_joint_5(1,4),config_joint_5(2,4),config_joint_5(3,4),'o')

% text(1.5, 2.8, 'Joint state', 'color', 'red', 'FontSize', 14)
% str = {string(q(1)), string(q(2)), string(q(3)), string(q(4)), string(q(5)), string(q(6)), string(q(7))};
% text(1.6,2.15,str)
% 
% text(2.05, 2.8, 'Joint state velocities', 'color', 'blue', 'FontSize', 14)
% str = {string(q_dot(1)), string(q_dot(2)), string(q_dot(3)), string(q_dot(4)), string(q_dot(5)), string(q_dot(6)), string(q_dot(7))};
% text(2.20,2.15,str)
% 
% text(1.5, 1.20, {'End-effector','position'}, 'color', 'red', 'FontSize', 14)
% str = {string(cur_pos(1)), string(cur_pos(2)), string(cur_pos(3)), string(cur_pos(4)), string(cur_pos(5)), string(cur_pos(6))};
% text(1.6, 0.5, str)
% 
% text(2.05, 1.20, {'End-effector','desired position'}, 'color', 'blue', 'FontSize', 14)
% str = {string(p_global(1)), string(p_global(2)), string(p_global(3)), string(p_global(4)), string(p_global(5)), string(p_global(6))};
% text(2.30, 0.5, str)

text(1.5, 0, 2.8, 'Joint state', 'color', 'red', 'FontSize', 14)
str = {string(q(1)), string(q(2)), string(q(3)), string(q(4)), string(q(5)), string(q(6)), string(q(7))};
text(1.6, 0, 1.5, str)

text(2.5, 0, 2.4,'Joint state velocities', 'color', 'blue', 'FontSize', 14)
str = {string(q_dot(1)), string(q_dot(2)), string(q_dot(3)), string(q_dot(4)), string(q_dot(5)), string(q_dot(6)), string(q_dot(7))};
text(2.8, 0, 1.05, str)

text(1.5, 0, 0, {'End-effector','position'}, 'color', 'red', 'FontSize', 14)
str = {string(cur_pos(1)), string(cur_pos(2)), string(cur_pos(3)), string(cur_pos(4)), string(cur_pos(5)), string(cur_pos(6))};
text(1.65, 0, -1.35, str)

text(2.6, 0, -0.35, {'End-effector','desired position'}, 'color', 'blue', 'FontSize', 14)
str = {string(p_global(1)), string(p_global(2)), string(p_global(3)), string(p_global(4)), string(p_global(5)), string(p_global(6))};
text(2.9, 0, -1.75, str)

xlim([-3,3])
ylim([-3,3])
zlim([-3,3])
xlabel('x') 
ylabel('y') 
zlabel('z') 
title('KUKA on Linear Axis', 'FontSize', 14)
end



