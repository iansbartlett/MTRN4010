% Problem 3 - Visualization of Gaussians
% Ian Bartlett z3419581

%Desired variances:
vars = [2 16 0.2 0.01 10000];

x = -10:0.01:11;

figure()
hold on;
for i = 1:length(vars)
    plot(x, normpdf(x,1,sqrt(vars(i))));
end

legend(['Var = 2'], ['Var = 16'], ['Var = 0.2'], ['Var = 0.01'], ['Var = 10000'])

hold off
