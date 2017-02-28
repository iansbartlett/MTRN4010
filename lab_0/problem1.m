%Problem 1: Simulating a pendulum in state space
%Ian Bartlett z3419581

A = 100; %rad/s^2
B = 2; %1/s
C = 1; %rad/(volt*s^2)

dt = 0.001; %1ms timestep
t = 0:dt:7;

% Dynamics model:
xdot = @(x, u) [x(2), -A*sind(x(1))-B*x(2)+C*u];

% First case - no control
x = zeros(length(t),2);
x(1,:) = [60 0];

for i = 2:length(t)
   x(i,:) = x(i-1,:) + dt*xdot(x(i-1,:),0);
end

% Second case - control input u
u = zeros(length(t), 1);
u(t >= 1 & t <= 2) = 10;
u(t >= 3 & t <= 4) = 20;

x2 = zeros(length(t),2);
x2(1,:) = [60 0];

for i = 2:length(t)
   x2(i,:) = x2(i-1,:) + dt*xdot(x2(i-1,:),u(i-1));
end

% Plot everything

figure()
subplot(2,2,1)
plot(t,x(:,1))
title('Unforced Position')
xlabel('Time (s)')
ylabel('Angle (deg)')

subplot(2,2,2)
plot(t,x(:,2))
title('Unforced Angular Velocity')
xlabel('Time (s)')
ylabel('Angular Velocity (deg/s)')

subplot(2,2,3)
plot(t,x2(:,1))
title('Forced Position')
xlabel('Time (s)')
ylabel('Angle (deg)')

subplot(2,2,4)
plot(t,x2(:,2))
title('Forced Angular Velocity')
xlabel('Time (s)')
ylabel('Angular Velocity (deg/s)')

