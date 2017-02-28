% Problem 2 - Simulation of a discrete-time state space model
% Ian Bartlett

% State Transition Matrix
A = [[0.9 0.1 0.1]; [0.1 0.9 0.3]; [0 -0.3 0.9]];
B = [0 0 1]';

k = 1:500;

% Control input
u = zeros(length(k), 1);
u(k >= 10 & k <= 20) = 1;
u(k >= 21 & k <= 30) = 2;

% Simulate Case 1
% A for loop works, but seems inefficient. Can this whole operation be vectorized?

x = zeros(length(k),3);
x(1,:) = [0 0 0];

for i = k(2:end)
   x(i,:) = (A*x(i-1, :)' + B*u(i-1))';
end	

% Simulate Case 2

x2 = zeros(length(k),3);
x2(1,:) = [10 20 15];

for i = k(2:end)
   x2(i,:) = (A*x2(i-1, :)')';
end

figure(1)
subplot(2,2,1);
plot(k,x(:,1));
title('State Variable 1')
xlabel('Index')
ylabel('State Variable')

subplot(2,2,2);
plot(k,x(:,2));
title('State Variable 2')
xlabel('Index')
ylabel('State Variable')

subplot(2,2,3);
plot(k,x(:,3));
title('State Variable 3')
xlabel('Index')
ylabel('State Variable')

subplot(2,2,4);
plot3(x(:,1),x(:,2),x(:,3))
title('State Space Trajectory')
xlabel('State Variable 1')
ylabel('State Variable 2')
zlabel('State Variable 3')


figure(2)
subplot(2,2,1);
plot(k,x2(:,1));
title('State Variable 1')
xlabel('Index')
ylabel('State Variable')

subplot(2,2,2);
plot(k,x2(:,2));
title('State Variable 2')
xlabel('Index')
ylabel('State Variable')

subplot(2,2,3);
plot(k,x2(:,3));
title('State Variable 3')
xlabel('Index')
ylabel('State Variable')

subplot(2,2,4);
plot3(x2(:,1),x2(:,2),x2(:,3))
title('State Space Trajectory')
xlabel('State Variable 1')
ylabel('State Variable 2')
zlabel('State Variable 3')


