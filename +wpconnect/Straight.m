classdef Straight < wpconnect.Segment
    properties
    end
    
    methods
        function this = Straight(p0, pf, length_to)
            length = vecnorm(pf-p0);
            this = this@wpconnect.Segment(length, length_to, p0, pf);
        end
        
        function p = position(this, length)
            if isempty(length)
                p = [];
                return
            end
            length_local = length - this.length_to;
            dp = this.pf - this.p0;
            p = this.p0 + length_local .* dp / this.length;
            p(:, length < this.length_to | length > this.length_end) = NaN;
        end
        
        function chi = tangent(this, length)
            if isempty(length)
                chi = [];
                return
            end
            dp = this.pf - this.p0;
            chi = atan2(dp(2), dp(1));
            chi = repmat(chi, size(length));
            chi(length < this.length_to | length > this.length_end) = NaN;
        end
        
        function r = turn_rate(this, length, ~)
            if isempty(length)
                r = [];
                return
            end
            r = zeros(size(length));
            r(length < this.length_to | length > this.length_end) = NaN;
        end
        
        function out = plot(this, linespec, varargin)
            if nargin < 2 || isempty(linespec)
                linespec = '';
            end
            h = plot([this.p0(2), this.pf(2)], [this.p0(1), this.pf(1)], linespec, varargin{:});
            if nargout >= 1
                out = h;
            end
        end
    end
end

