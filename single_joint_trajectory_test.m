%traj test

start_pos = -1000
end_pos = 1000
steps = 500
start_velocity = 0
end_velocity = 0

[q,qd,qdd] = jtraj(start_pos, end_pos, steps, start_velocity, end_velocity)

% Generate points to plot
t = 0:0.1:10;
pos = polyval(q,t);
vel = polyval(qd,t);
acc = polyval(qdd,t);

% Position
subplot(3,1,1)
plot(t,pos);
title('Turtle Motion','fontsize',18);
xlabel('Time (min)');
ylabel('Position (ft)');

% Velocity
subplot(3,1,2)
%plot(t_pnts,v_pnts,'*');
hold on
plot(t,vel);
legend('Measured points','2^n^d order polynomial',3)
xlabel('Time (min)');
ylabel('Velocity (ft/min)');

% Acceleration
subplot(3,1,3)
plot(t,acc);
xlabel('Time (min)');
ylabel('Acceleration(ft/min^2)');

pause
