%% Redundancy Resolution
clear all;
close all;
clc;

%% Initial state
q_0 = [0, 0, 0, 0, 0, 0, 0];
%q_0 = [0.5, 10, 10, 10, 10, 0, 0];
link_lengths = [675, 350, 1150, 1200, -41, 240]*1e-3;

%% Desired pos
%p_global = [300*1e-3 256*1e-3 500*1e-3 pi/4 pi/5 0]';
%p_global = [1 1.5 0.9 -2 2 1.7]';
p_global = [-1.3 1.17 0.58 0 0 0]';
fprintf('Desired Pos = \n')
disp(p_global)

%% Getting the pseudo inverse
flag = 0; % for the weighted pseudoInverse flag = 1
q = IK(q_0, link_lengths, p_global, flag);
[T, T1, T2, T3, T4, T5, T6, Pos] = FK(q, link_lengths);
