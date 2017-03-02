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
% Here we will simply apply a constant control input, and watch the 
% vehicle curve away

x = zeros(num_steps,3);

x(1,:) = [0, 0, 0];

for i = 2:num_steps
    x(i,:) =  x(i-1,:) + dt*dx(x(i-1,:),0.2);
end

% Case 2
% Here we apply a fixed open-loop control input, to steer
% the vehicle in a figure-eight
x2 = zeros(num_steps,3);
x2(1,:) = [0, 0, 0];

u = zeros(num_steps, 1);

u(1:100) = 1;
u(101:200) = 1;
u(291:390) = -1;
u(391:490) = -1;

for i = 2:num_steps
    x2(i,:) =  x2(i-1,:) + dt*dx(x2(i-1,:),u(i-1));
end

% Case 3
% Here we apply a simple proportional feedback controller 
% to steer the vehicle to a given heading.

theta_setpoint = pi/2;

x3 = zeros(num_steps,3);
x3(1,:) = [0, 0, 0];

%Control gain - manually tuned. 
K = 0.7;

for i = 2:num_steps
    u_ctrl = -K*(x3(i-1,3) - theta_setpoint);

    %Clamp output to avoid tan singularities
    if (u_ctrl > pi/3)
	    U_ctrl = pi/3;
    elseif (u_ctrl < -pi/3)
	    u_ctrl = -pi/3;
    end

    x3(i,:) =  x3(i-1,:) + dt*dx(x3(i-1,:),u_ctrl);

end

% Case 4
% Here we apply a small perturbation to one of our parameters, and
% observe the difference between the two.

x4 = zeros(num_steps,3);
x4_perturbed = zeros(num_steps,3);
norm_error = zeros(num_steps, 1);

x4(1,:) = [0, 0, 0];
x4_perturbed(1,:) = [0, 0, 0];
norm_error(1) = 0;

% Perturb L parameter by 0.1
L_perturbed = 2.1;

dx_perturbed = @(x, u) [v*cos(x(3)),v*sin(x(3)),tan(u)*v/L_perturbed];

for i = 2:num_steps
    x4(i,:) =  x(i-1,:) + dt*dx(x4(i-1,:),0.2);
    x4_perturbed(i,:) =  x4_perturbed(i-1,:) + dt*dx_perturbed(x4_perturbed(i-1,:),0.2);
    norm_error(i) = norm(x4(i,:)-x4_perturbed(i,:));
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

figure();
subplot(2,2,1);
plot(x3(:,1),x3(:,2));
title('X-Y Position')
xlabel('x coordinate')
ylabel('y coordinate')
 
subplot(2,2,2);
plot(1:num_steps,x3(:,3))
title('Angle (rads)')
xlabel('Timestep')
ylabel('Angle')

subplot(2,2,3);
plot3(x3(:,1),x3(:,2),x3(:,3))
title('Position State Space')
xlabel('x coordinate')
ylabel('y coordinate')
zlabel('Angular coordinate')

figure();
subplot(2,2,1);
plot(x4(:,1),x4(:,2),'b- ', x4_perturbed(:,1),x4_perturbed(:,2),'g- ');
title('X-Y Position')
xlabel('x coordinate')
ylabel('y coordinate')
legend(['Baseline'], ['Disturbed'])
 
subplot(2,2,2);
plot(1:num_steps,x4(:,3),'b- ', 1:num_steps, x4_perturbed(:,3), 'g- ')
title('Angle (rads)')
xlabel('Timestep')
ylabel('Angle')
legend(['Baseline'], ['Disturbed'])

subplot(2,2,3);
plot3(x4(:,1),x4(:,2),x4(:,3),'b- ',...
      x4_perturbed(:,1),x4_perturbed(:,2),x4_perturbed(:,3),'g- ')
title('Position State Space')
xlabel('x coordinate')
ylabel('y coordinate')
zlabel('Angular coordinate')
legend(['Baseline'], ['Disturbed'])

subplot(2,2,4)
plot(1:num_steps, norm_error);
title('State-Space Position Error Norm')
xlabel('Timestep')
ylabel('Error Norm')
