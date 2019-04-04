classdef Segment < matlab.mixin.Heterogeneous
    %SEGMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        length
        length_to
        length_end
        p0
        pf
    end
    
    methods
        function this = Segment(length, length_to, p0, pf)
            this.length = length;
            this.length_to = length_to;
            this.length_end = length_to + length;
            this.p0 = p0;
            this.pf = pf;
        end
    end
    
    methods(Abstract)
        p = position(this, length)
        chi = tangent(this, length)
        r = turn_rate(this, length, U)
    end
end

