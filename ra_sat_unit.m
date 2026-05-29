function y = ra_sat_unit(x)
%RA_SAT_UNIT Unit saturation used by the sliding-mode boundary layer.
y = min(max(x, -1.0), 1.0);
end
