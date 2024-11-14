clear all
clc
close all
system('./run_linux')

filename = './data/data.csv';

T = readtable(filename);
VariableNames = T.Properties.VariableNames;

Arr = table2array(T);
[m,n] = size(Arr);

figure(1);
subplot(2,1,1);
plot(Arr(:,1), Arr(:,2), 'Color', 'b', 'LineWidth', 3);
hold on;
plot(Arr(:,1), Arr(:,3), 'Color', 'r', 'LineWidth', 3);
xlim([1 5]);
ylim([-0.1 0.1]);
xlabel('time (sec)');
ylabel('r-direction force (N)');
legend('reference', 'real')

subplot(2,1,2)
plot(Arr(:,1), Arr(:,4), 'Color', 'b', 'LineWidth', 3);
hold on;
plot(Arr(:,1), Arr(:,5), 'Color', 'r', 'LineWidth', 3);
xlim([1 5]);
ylim([-0.1 0.1]);
xlabel('time (sec)');
ylabel('theta-direction force (N)');
legend('reference', 'real')