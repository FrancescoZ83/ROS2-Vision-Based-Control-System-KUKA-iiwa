close all
clear all
clc

%% Paths

lin_pol_jnt_path = 'D:\Shared Content\bags\lin_pol_jnt';
lin_trap_jnt_path = 'D:\Shared Content\bags\lin_trap_jnt';
cir_pol_jnt_path = 'D:\Shared Content\bags\cir_pol_jnt';
cir_trap_jnt_path = 'D:\Shared Content\bags\cir_trap_jnt';

lin_pol_op_path = 'D:\Shared Content\bags\lin_pol_op';
lin_trap_op_path = 'D:\Shared Content\bags\lin_trap_op';
cir_pol_op_path = 'D:\Shared Content\bags\cir_pol_op';
cir_trap_op_path = 'D:\Shared Content\bags\cir_trap_op';

%% Bag Objects

lin_pol_jnt_bag = ros2bag(lin_pol_jnt_path);
lin_trap_jnt_bag = ros2bag(lin_trap_jnt_path);
cir_pol_jnt_bag = ros2bag(cir_pol_jnt_path);
cir_trap_jnt_bag = ros2bag(cir_trap_jnt_path);

lin_pol_op_bag = ros2bag(lin_pol_op_path);
lin_trap_op_bag = ros2bag(lin_trap_op_path);
cir_pol_op_bag = ros2bag(cir_pol_op_path);
cir_trap_op_bag = ros2bag(cir_trap_op_path);

%% Read Messages from Topics

effort_topic = '/effort_controller/commands';

lin_pol_jnt_sel = select(lin_pol_jnt_bag, "Topic", effort_topic);
lin_trap_jnt_sel = select(lin_trap_jnt_bag, "Topic", effort_topic);
cir_pol_jnt_sel = select(cir_pol_jnt_bag, "Topic", effort_topic);
cir_trap_jnt_sel =select(cir_trap_jnt_bag, "Topic", effort_topic);

lin_pol_op_sel = select(lin_pol_op_bag, "Topic", effort_topic);
lin_trap_op_sel = select(lin_trap_op_bag, "Topic", effort_topic);
cir_pol_op_sel = select(cir_pol_op_bag, "Topic", effort_topic);
cir_trap_op_sel = select(cir_trap_op_bag, "Topic", effort_topic);


lin_pol_jnt_msgs = readMessages(lin_pol_jnt_sel);
lin_trap_jnt_msgs = readMessages(lin_trap_jnt_sel);
cir_pol_jnt_msgs = readMessages(cir_pol_jnt_sel);
cir_trap_jnt_msgs = readMessages(cir_trap_jnt_sel);

lin_pol_op_msgs = readMessages(lin_pol_op_sel);
lin_trap_op_msgs = readMessages(lin_trap_op_sel);
cir_pol_op_msgs = readMessages(cir_pol_op_sel);
cir_trap_op_msgs = readMessages(cir_trap_op_sel);

%% Process and Analyze data


lin_pol_jnt_dt = cellfun(@(msg) msg.data, lin_pol_jnt_msgs, 'UniformOutput', false);
lin_pol_jnt_dt_array = cell2mat(lin_pol_jnt_dt);
lin_pol_jnt_dt = cellfun(@(msg) msg.data', lin_pol_jnt_msgs, 'UniformOutput', false);
lin_pol_jnt_dt_array = cell2mat(lin_pol_jnt_dt);
numMessages = size(lin_pol_jnt_dt_array, 1);
lin_pol_jnt_ts = linspace(0, numMessages - 1, numMessages);
figure;
hold on;
for i = 1:size(lin_pol_jnt_dt_array, 2)
    plot(lin_pol_jnt_ts, lin_pol_jnt_dt_array(:, i), 'DisplayName', ['Joint ', num2str(i)], 'LineWidth', 1.5);
end
hold off;title('Linear Polinomial Joint Space');
xlabel('Trajectory Time Points');
ylabel('Effort Value');
legend show;
grid on;
xlim([0 530]);



lin_trap_jnt_dt = cellfun(@(msg) msg.data, lin_trap_jnt_msgs, 'UniformOutput', false);
lin_trap_jnt_dt_array = cell2mat(lin_trap_jnt_dt);
lin_trap_jnt_dt = cellfun(@(msg) msg.data', lin_trap_jnt_msgs, 'UniformOutput', false);
lin_trap_jnt_dt_array = cell2mat(lin_trap_jnt_dt);
numMessages = size(lin_trap_jnt_dt_array, 1);
lin_trap_jnt_ts = linspace(0, numMessages - 1, numMessages);
figure;
hold on;
for i = 1:size(lin_trap_jnt_dt_array, 2)
    plot(lin_trap_jnt_ts, lin_trap_jnt_dt_array(:, i), 'DisplayName', ['Joint ', num2str(i)], 'LineWidth', 1.5);
end
hold off;
title('Linear Trapezoidal Joint Space');
xlabel('Trajectory Time Points');
ylabel('Effort Value');
legend show;
grid on;
xlim([0 530]);



cir_pol_jnt_dt = cellfun(@(msg) msg.data, cir_pol_jnt_msgs, 'UniformOutput', false);
cir_pol_jnt_dt_array = cell2mat(cir_pol_jnt_dt);
cir_pol_jnt_dt = cellfun(@(msg) msg.data', cir_pol_jnt_msgs, 'UniformOutput', false);
cir_pol_jnt_dt_array = cell2mat(cir_pol_jnt_dt);
numMessages = size(cir_pol_jnt_dt_array, 1);
cir_pol_jnt_ts = linspace(0, numMessages - 1, numMessages);
figure;
hold on;
for i = 1:size(cir_pol_jnt_dt_array, 2)
    plot(cir_pol_jnt_ts, cir_pol_jnt_dt_array(:, i), 'DisplayName', ['Joint ', num2str(i)], 'LineWidth', 1.5);
end
hold off;
title('Circular Polynomial Joint Space');
xlabel('Trajectory Time Points');
ylabel('Effort Value');
legend show;
grid on;
xlim([0 530]);



cir_trap_jnt_dt = cellfun(@(msg) msg.data, cir_trap_jnt_msgs, 'UniformOutput', false);
cir_trap_jnt_dt_array = cell2mat(cir_trap_jnt_dt);
cir_trap_jnt_dt = cellfun(@(msg) msg.data', cir_trap_jnt_msgs, 'UniformOutput', false);
cir_trap_jnt_dt_array = cell2mat(cir_trap_jnt_dt);
numMessages = size(cir_trap_jnt_dt_array, 1);
cir_trap_jnt_ts = linspace(0, numMessages - 1, numMessages);
figure;
hold on;
for i = 1:size(cir_trap_jnt_dt_array, 2)
    plot(cir_trap_jnt_ts, cir_trap_jnt_dt_array(:, i), 'DisplayName', ['Joint ', num2str(i)], 'LineWidth', 1.5);
end
hold off;
title('Circular Trapezoidal Joint Space');
xlabel('Trajectory Time Points');
ylabel('Effort Value');
legend show;
grid on;
xlim([0 530]);



lin_pol_op_dt = cellfun(@(msg) msg.data, lin_pol_op_msgs, 'UniformOutput', false);
lin_pol_op_dt_array = cell2mat(lin_pol_op_dt);
lin_pol_op_dt = cellfun(@(msg) msg.data', lin_pol_op_msgs, 'UniformOutput', false);
lin_pol_op_dt_array = cell2mat(lin_pol_op_dt);
numMessages = size(lin_pol_op_dt_array, 1);
lin_pol_op_ts = linspace(0, numMessages - 1, numMessages);
figure;
hold on;
for i = 1:size(lin_pol_op_dt_array, 2)
    plot(lin_pol_op_ts, lin_pol_op_dt_array(:, i), 'DisplayName', ['Joint ', num2str(i)], 'LineWidth', 1.5);
end
hold off;title('Linear Polinomial Operational Space');
xlabel('Trajectory Time Points');
ylabel('Effort Value');
legend show;
grid on;
xlim([0 530]);



lin_trap_op_dt = cellfun(@(msg) msg.data, lin_trap_op_msgs, 'UniformOutput', false);
lin_trap_op_dt_array = cell2mat(lin_trap_op_dt);
lin_trap_op_dt = cellfun(@(msg) msg.data', lin_trap_op_msgs, 'UniformOutput', false);
lin_trap_op_dt_array = cell2mat(lin_trap_op_dt);
numMessages = size(lin_trap_op_dt_array, 1);
lin_trap_op_ts = linspace(0, numMessages - 1, numMessages);
figure;
hold on;
for i = 1:size(lin_trap_op_dt_array, 2)
    plot(lin_trap_op_ts, lin_trap_op_dt_array(:, i), 'DisplayName', ['Joint ', num2str(i)], 'LineWidth', 1.5);
end
hold off;
title('Linear Trapezoidal Operational Space');
xlabel('Trajectory Time Points');
ylabel('Effort Value');
legend show;
grid on;
xlim([0 530]);



cir_pol_op_dt = cellfun(@(msg) msg.data, cir_pol_op_msgs, 'UniformOutput', false);
cir_pol_op_dt_array = cell2mat(cir_pol_op_dt);
cir_pol_op_dt = cellfun(@(msg) msg.data', cir_pol_op_msgs, 'UniformOutput', false);
cir_pol_op_dt_array = cell2mat(cir_pol_op_dt);
numMessages = size(cir_pol_op_dt_array, 1);
cir_pol_op_ts = linspace(0, numMessages - 1, numMessages);
figure;
hold on;
for i = 1:size(cir_pol_op_dt_array, 2)
    plot(cir_pol_op_ts, cir_pol_op_dt_array(:, i), 'DisplayName', ['Joint ', num2str(i)], 'LineWidth', 1.5);
end
hold off;
title('Circular Polynomial Operational Space');
xlabel('Trajectory Time Points');
ylabel('Effort Value');
legend show;
grid on;
xlim([0 530]);



cir_trap_op_dt = cellfun(@(msg) msg.data, cir_trap_op_msgs, 'UniformOutput', false);
cir_trap_op_dt_array = cell2mat(cir_trap_op_dt);
cir_trap_op_dt = cellfun(@(msg) msg.data', cir_trap_op_msgs, 'UniformOutput', false);
cir_trap_op_dt_array = cell2mat(cir_trap_op_dt);
numMessages = size(cir_trap_op_dt_array, 1);
cir_trap_op_ts = linspace(0, numMessages - 1, numMessages);
figure;
hold on;
for i = 1:size(cir_trap_op_dt_array, 2)
    plot(cir_trap_op_ts, cir_trap_op_dt_array(:, i), 'DisplayName', ['Joint ', num2str(i)], 'LineWidth', 1.5);
end
hold off;
title('Circular Trapezoidal Operational Space');
xlabel('Trajectory Time Points');
ylabel('Effort Value');
legend show;
grid on;
xlim([0 530]);
