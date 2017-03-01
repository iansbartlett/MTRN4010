% Problem 4 - Control Demo
% Ian Bartlett z3419581

close all

dt = 0.01; %Timestep
L = 2; %m
v = 3; %m/s

% Dynamical system
% Call the steering angle alpha the control input, u

dx = @(x, u) [v*cos(x(3)),v*sin(x(3)),tan(u)*v/L];

num_steps = 580;

% Case 1

x = zeros(num_steps,3);

x(1,:) = [0, 0, 0];

for i = 2:num_steps
    x(i,:) =  x(i-1,:) + dt*dx(x(i-1,:),0.2);
end

% Case 2

x2 = zeros(num_steps,3);

x2(1,:) = [0, 0, 0];

u = zeros(num_steps, 1);
%u(1:50) = pi/6;
%u(100:150) = -pi/6;

u(1:100) = 1;
u(101:200) = 1;
u(291:390) = -1;
u(391:490) = -1;

%u = atan(-1*(L/v)*cos(t/t(end))./(sin(t/t(end)).^2+1));
%u = atan2(-L*atan2(((1-3*cos(2*2*pi*t/t(end))).*csc(2*pi*t/t(end))),(5+cos(2*2*pi*t/t(end)))),v);

for i = 2:num_steps
    x2(i,:) =  x2(i-1,:) + dt*dx(x2(i-1,:),u(i-1));
end

% Plot everything

figure();
subplot(2,2,1);
plot(x(:,1),x(:,2));
title('X-Y Position')
xlabel('x coordinate')
ylabel('y coordinate')
 
subplot(2,2,2);
plot(1:num_steps,x(:,3))
title('Angle (rads)')
xlabel('Timestep')
ylabel('Angle')

subplot(2,2,3);
plot3(x(:,1),x(:,2),x(:,3))
title('Position State Space')
xlabel('x coordinate')
ylabel('y coordinate')
zlabel('Angular coordinate')

%subplot(2,2,4);
%plot3(x(:,4),x(:,5),x(:,6))
%title('Velocity State Space')
%xlabel('x velocity')
%ylabel('y velocity')
%zlabel('Angular velocity')

figure();
subplot(2,2,1);
plot(x2(:,1),x2(:,2));
title('X-Y Position')
xlabel('x coordinate')
ylabel('y coordinate')
 
subplot(2,2,2);
plot(1:num_steps,x2(:,3))
title('Angle (rads)')
xlabel('Timestep')
ylabel('Angle')

subplot(2,2,3);
plot3(x2(:,1),x2(:,2),x2(:,3))
title('Position State Space')
xlabel('x coordinate')
ylabel('y coordinate')
zlabel('Angular coordinate')

%subplot(2,2,4);
%plot3(x2(:,4),x2(:,5),x2(:,6))
%title('Velocity State Space')
%xlabel('x velocity')
%ylabel('y velocity')
%zlabel('Angular velocity')

%figure()
%plot(u)
%title('Control input')
%xlabel('Timestep')
%ylabel('Control Input')
