classdef Path
    properties
        segments
        p
        R_accept
        R_turn
        length
    end
    
    methods
        function this = Path(p, R_accept, R_turn_min)
            if nargin < 3 || isempty(R_turn_min)
                R_turn_min = 0;
            end
            
            this.p = p;
            N_straights = size(p, 2) - 1;
            N_arcs = size(p, 2) - 2;
            
            if numel(R_accept) == 1
                R_accept = repmat(R_accept, 1, N_arcs);
            end
            
            N_R_accept = numel(R_accept);
            if N_R_accept  > 1 && N_R_accept ~= N_arcs
                warning('Number of R_accept elements not equal to number of arcs. Setting missing R_accept entries to R_turn_min(1).')
                R_accept((N_R_accept+1):N_arcs) = R_turn_min(1);
            end
            
            dp = diff(p,[],2);
            dist = sqrt(dp(1,:).^2 + dp(2,:).^2);
            dp3 = [dp; zeros(1,size(dp,2))];
            cp = cross(dp3(:,1:end-1), dp3(:,2:end));
            turning = sign(cp(3,:)); % +1 for clockwise, -1 for counterclockwise
            
            beta = atan2(dp(2,:), dp(1,:));

            diff_angle = turning.*(beta(2:end) - beta(1:end-1));
            alpha = abs(abs(diff_angle/2)-deg2rad(90));
            
            i_parallels = abs(alpha-deg2rad(90)) < eps;
            R_turn(~i_parallels) = R_accept(~i_parallels) .* tan(alpha(~i_parallels));
            R_turn(i_parallels) = 0.0;
            i_resize_turn = R_turn < R_turn_min;
            R_accept(i_resize_turn & ~i_parallels) = R_turn_min ./ tan(alpha(i_resize_turn & ~i_parallels));
            R_accept(i_parallels) = 0.0;
            
            %Control that R_accept is not too large
            R_accept = min(R_accept, min(dist(1:N_arcs), dist(2:N_arcs+1)));
            R_accept_org = R_accept;
            change = true;
            %Loop until R_accept is within bounds for all corners
            while change
                change = false;
                for i = 1:N_arcs-1
                    if R_accept(i) + R_accept(i+1) > dist(i+1)
                        R_accept(i) = dist(i+1)-R_accept(i+1);
                        change = true;
                    end
                end
            end
            
            %Do forward pass to check if R_accept can be relaxed at some
            %corners
            R_accept(1) = min([R_accept_org(1), dist(i+1)-R_accept(i+1), dist(1)]);
            for i = 2:N_arcs-1
                R_accept(i) = min([R_accept_org(i), dist(i+1)-R_accept(i+1), dist(i)-R_accept(i-1)]);
            end
            R_accept(N_arcs) = min([R_accept_org(N_arcs), dist(N_arcs+1), dist(N_arcs)-R_accept(N_arcs-1)]);
            
            R_turn = R_accept .* tan(alpha);         
            this.R_turn = R_turn;
            this.R_accept = R_accept;

            gamma = beta(1:end-1) + turning * deg2rad(90);

            p_turn_start = NaN(2,size(p,2)-2);
            p_turn_end = NaN(2, size(p,2)-2);
            p_r = NaN(2,N_arcs);
            theta = NaN(2, N_arcs);
            for i = 1:N_arcs
                p_turn_start(:,i) = p(:,i+1) - R_accept(i) * [cos(beta(i)); sin(beta(i))];
                p_turn_end(:,i) = p(:,i+1) + R_accept(i) * [cos(beta(i+1)); sin(beta(i+1))];
                p_r(:,i) = p_turn_start(:,i) + R_turn(i) * [cos(gamma(i)); sin(gamma(i))];
                theta(:,i) = [beta(i)-turning(i)*deg2rad(90), beta(i)+turning(i)*(-deg2rad(90)+deg2rad(180)-2*alpha(i))]';
            end
            N_segments = N_arcs + N_straights;
            this.segments = wpconnect.Segment.empty();
            
            if N_arcs == 0
                this.segments(1) = wpconnect.Straight(p(:,1), p(:,2), 0);
            else
                this.segments(1) = wpconnect.Straight(p(:,1), p_turn_start(:,1), 0);
                for i = 1:N_arcs-1
                    this.segments(i*2) = wpconnect.Arc(p_r(:,i), R_turn(i), theta(:,i)', turning(i), this.segments(i*2-1).length_end);
                    this.segments(i*2+1) = wpconnect.Straight(p_turn_end(:,i), p_turn_start(:,i+1), this.segments(i*2).length_end);
                end
                this.segments(N_segments-1) = wpconnect.Arc(p_r(:,N_arcs), R_turn(N_arcs), theta(:,N_arcs)', turning(N_arcs), this.segments(N_segments-2).length_end);
                this.segments(N_segments) = wpconnect.Straight(p_turn_end(:,N_arcs), p(:,end), this.segments(N_segments-1).length_end);
            end
            this.length = this.segments(N_segments).length_end;
            
            %Delete singular arcs
            i = 1;
            while i <= length(this.segments) 
               if isa(this.segments(i), 'wpconnect.Arc') && this.segments(i).R <= eps
                   this.segments(i) = [];
               else    
                    i = i+1;
               end
            end
            
        end
        
        function p = position(this, length)
            p = NaN(2,numel(length));
            for i = 1:numel(this.segments)
                i_within = length >= this.segments(i).length_to & length <= this.segments(i).length_end;
                if any(i_within)
                    p(:,i_within) = this.segments(i).position(length(i_within));
                end
            end
        end
        
        function chi = tangent(this, length)
            if ~issorted(length)
                warning('The length vector is not sorted. May have issues with tangent continuity.')
            end
            chi = NaN(1, numel(length));
            prev_chi = NaN;
            for i = 1:numel(this.segments)
                i_within = length >= this.segments(i).length_to & length <= this.segments(i).length_end;
                if any(i_within)
                    chi(i_within) = this.segments(i).tangent(length(i_within));
                    
                    % The following snippet makes sure the tangential
                    % angles are continuous, but may cause inconsistent
                    % behavior depending on what the first (lowest) value
                    % of `length` is.
                    if ~isnan(prev_chi)
                        i_first = find(i_within);
                        while abs(prev_chi - chi(i_first)) > deg2rad(180)
                            difference = prev_chi - chi(i_first);
                            chi(i_within) = chi(i_within) + sign(difference) * deg2rad(360);
                        end
                    end
                    prev_chi = chi(find(i_within, 1, 'last'));
                end
            end
        end
        
        function r = turn_rate(this, length, U)
            r = NaN(1,numel(length));
            for i = 1:numel(this.segments)
                i_within = length >= this.segments(i).length_to & length <= this.segments(i).length_end;
                if any(i_within)
                    if numel(U) > 1
                        r(i_within) = this.segments(i).turn_rate(length(i_within), U(i_within));
                    else
                        r(i_within) = this.segments(i).turn_rate(length(i_within), U);
                    end
                end
            end
        end
        
        function arcs = get_arcs(this)
            arcs = this.segments(2:2:end-1);
        end
        
        function straights = get_straights(this)
            straights = this.segments(1:2:end);
        end
        
        function out = plot(this, linespec, varargin)
            if nargin < 2 || isempty(linespec)
                linespec = '';
            end
            h = gobjects(1, numel(this.segments));
            hold on
            color_index = get(gca, 'ColorOrderIndex');
            for i = 1:numel(this.segments)
                set(gca, 'ColorOrderIndex', color_index)
                h(i) = plot(this.segments(i), linespec);
            end
            if nargout >= 1
                out = h;
            end
        end
    end
end

