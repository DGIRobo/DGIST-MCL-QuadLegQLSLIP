
Arr_PD = table2array(T_PD);
% 시작 시간과 끝 시간 설정
start_time = 3.5; % 시작 시간
end_time = 5;   % 끝 시간

% t 벡터를 기준으로 시작 시간과 끝 시간에 해당하는 인덱스 찾기
start_idx = find(t >= start_time, 1);
end_idx = find(t >= end_time, 1);

% 찾은 인덱스를 확인하기 위해 disp 사용
disp(start_idx);
disp(end_idx);

% 해당 시간 범위의 데이터 플롯
figure(1)
subplot(2, 1, 1);
plot(t(start_idx:end_idx), grf_x_PD(start_idx:end_idx),'r', 'DisplayName', 'Sensor','LineWidth', 2);
hold on;
ylabel('GRF Sensor (N)','FontSize',13);
ylim([-300 0]);
grid on

subplot(2, 1, 2);
plot(t(start_idx:end_idx),touch_PD(start_idx:end_idx),'LineWidth', 2);
xlabel('Time (seconds)','FontSize', 13); % x축 레이블
ylabel('Touch sensor','FontSize',13);
grid on; % 격자 표시



sgtitle('Ground Reaction Force (N)','FontSize',20);

%%
plot(t(start_idx:end_idx), r_ref_PD(start_idx:end_idx),'k--', 'DisplayName', 'r_{ref}','LineWidth', 2);
hold on;
plot(t(start_idx:end_idx), r_act_PD(start_idx:end_idx),'r-.', 'DisplayName', 'r_{act}','LineWidth', 2);
xlabel('Time','FontSize',12);
ylabel('r Direction (m)','FontSize',12);
ylim([0.1 0.6]);
title('r Direaction','FontSize',12); % 그래프 제목
grid on; % 격자 표시
legend;

