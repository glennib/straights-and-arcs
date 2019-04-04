
% p = [Some waypoints];
% scenario = Proprietary scenario, but please go through the "reduce" file
% to see where you need to replace code, I don't think it's much work to
% make it compatible with any other obstacle detection.

%%
p_new = wpreduce.reduce(p, scenario, 5);

%%
R_accept = 1;
R_min = 40;
path = wpconnect.Path(p_new, R_accept, R_min);

%%

figure(1)
clf
plot(scenario)
hold on
h_old = plot(p(2,:), p(1,:), 'r-o');
h_new = plot(p_new(2,:), p_new(1,:), 'b-*');
h_path = plot(path, '-k');
axis equal
xlabel('East [m]')
ylabel('North [m]')
legend([h_old, h_new, h_path(1)], {'Original', 'Reduced', 'Dubins'}, 'location', 'northeast')

