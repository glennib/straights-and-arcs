classdef Arc < wpconnect.Segment
    %ARC Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        pr
        R
        theta
        turn
        curve_angle
    end
    
    methods
        function this = Arc(pr, R, theta, turn, length_to)
            length = abs(diff(theta))*R;
            p0 = pr + [cos(theta(1)); sin(theta(1))] * R;
            pf = pr + [cos(theta(2)); sin(theta(2))] * R;
            this = this@wpconnect.Segment(length, length_to, p0, pf);
            this.pr = pr;
            this.R = R;
            this.theta = theta;
            this.turn = turn;
            this.curve_angle = pi-abs(diff(theta));
        end
        
        function p = position(this, length)
            if isempty(length)
                p = [];
                return
            end
            length_local = length - this.length_to;
            theta_local = this.turn*(length_local / this.R) + this.theta(1);
            p = this.pr + [cos(theta_local); sin(theta_local)] * this.R;
            p(:, length < this.length_to | length > this.length_end) = NaN;
        end
        
        function chi = tangent(this, length)
            if isempty(length)
                p = [];
                return
            end
            length_local = length - this.length_to;
            theta_local = this.turn*(length_local / this.R) + this.theta(1);
            chi = theta_local + deg2rad(90) * this.turn;
            chi(length < this.length_to | length > this.length_end) = NaN;
        end
        
        function r = turn_rate(this, length, U)
            if isempty(length)
                r = [];
                return
            end
            r = U/this.R*this.turn;
            if numel(U) == 1
                r = repmat(r, size(length));
            end
            r(length < this.length_to | length > this.length_end) = NaN;
        end
        
        function out = plot(this, linespec, varargin)
            if nargin < 2 || isempty(linespec)
                linespec = [];
            end
            th = -this.theta + deg2rad(90);
            h = wpconnect.Arc.circleseg(this.pr(2), this.pr(1), this.R, th, [], linespec, varargin{:});
            if nargout >= 1
                out = h;
            end
        end
    end
    
    methods(Static)
        function out = circleseg(x, y, R, th, N, linespec, varargin)
            if nargin < 5 || isempty(N)
                N = 100;
            end
            if nargin < 6 || isempty(linespec)
                linespec = '';
            end
            if nargin < 7 || isempty(varargin)
                varargin = {};
            end


            TH = linspace(th(1), th(2), N);

            X = x + R * cos(TH);
            Y = y + R * sin(TH);

            h(1) = plot(X, Y, linespec, varargin{:});
            % h(2) = scatter(X(1), Y(1), '+');
            % h(3) = scatter(X(end), Y(end), 's');

            if nargout >= 1
                out = h;
            end
        end
    end
end

