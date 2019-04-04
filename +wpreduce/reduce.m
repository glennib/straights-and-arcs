function [p_new] = reduce(p, scenario, L_min, L_padding)
if nargin < 4 || isempty(L_padding)
    L_padding = 0;
end

DEBUG = false;
PAUSE = false;

PAUSE = DEBUG && PAUSE;

if DEBUG
    figure(30)
    clf
    plot(scenario, L_padding)
    axis equal
    scatter(p(2,:), p(1,:), 'b.');
end

N_p = size(p,2);
i_current = N_p;
p_new = p(:,i_current);
while true
    p_current = p(:,i_current);
    if DEBUG
        if exist('h_current', 'var')
            delete(h_current)
        end
        h_current  = scatter(p_current(2), p_current(1), 'y.');
    end
    for j = 1:i_current % should be i_current-1, but need error handling
        p_test = p(:,j);
        straight = wpconnect.Straight(p_test, p_current, 0);
%         l = 0:L_min:straight.length;
        N_colpts = ceil(straight.length / L_min);
        l = linspace(0, straight.length, N_colpts);
        p_intermediates = straight.position(l);
        if DEBUG
            disp([i_current, j, size(p_intermediates)]) % debug
        end
        col = scenario.is_collision(p_intermediates(1,:), p_intermediates(2,:), L_padding);
        if DEBUG
            if exist('h_free', 'var')
                delete(h_free)
            end
            if exist('h_col', 'var')
                delete(h_col)
            end
            h_free = scatter(p_intermediates(2,~col), p_intermediates(1,~col), 'g.');
            h_col = scatter(p_intermediates(2,col), p_intermediates(1,col), 'r.');
        end
        if PAUSE
            pause
        end
        if ~any(col)
            p_new = [
                p_test, p_new;
                ]; %#ok<AGROW>
            i_current = j;
            if DEBUG
                plot(straight, '-k')
            end
            break
        end
    end
    if i_current == 1
        break
    end
    if PAUSE
        pause
    end
end

