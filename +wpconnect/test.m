
p = [
    0, 0;
    100, 100;
    200, 0;
    100, -100;
    0, -100;
    80, -200;
    0, -250;
    -100, -200;
    ]';

% big_arc = wpconnect.Arc([0;0], 1000, [0,2*pi], 1, 0);
% l = 0:25:big_arc.length;
% p = big_arc.position(l);
% 
R_accept = 10;
R_min = 50;

%%
path = wpconnect.Path(p, R_accept, R_min);
% return
%%

l_plot = linspace(0, path.length, 500);
p_plot = path.position(l_plot);
chi_plot = path.tangent(l_plot);
r_plot = path.turn_rate(l_plot, 5);

%%

figure(1)
clf
scatter(p(2,:), p(1,:), 'ok')
hold on
plot(p(2,:), p(1,:), ':k')
plot(path)
% scatter(p_plot(2,:), p_plot(1,:))
grid on
axis equal

figure(2)
clf
plot(l_plot, rad2deg(chi_plot))
grid on

figure(3)
clf
plot(l_plot, rad2deg(r_plot))
grid on
