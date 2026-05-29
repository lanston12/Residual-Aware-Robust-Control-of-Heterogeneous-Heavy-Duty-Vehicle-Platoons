function [alphaBrake, Fmin, Fmax] = ra_brake_fade_profile(t, P)
%RA_BRAKE_FADE_PROFILE Time-varying braking authority boundary.

fade = ones(P.N, 1);
if t >= P.fadeStart && t <= P.fadeEnd
    fadeShape = sin(pi * (t - P.fadeStart) / (P.fadeEnd - P.fadeStart))^2;
    rearSeverity = linspace(0.30, 0.65, P.N).';
    if P.N > 5
        rearSeverity = linspace(0.30, 0.45, P.N).';
    end
    fade = 1 - rearSeverity * fadeShape;
end

alphaBrake = max(0.02, P.vehicle.alphaBrakeBase .* fade);
Fmin = -alphaBrake .* P.vehicle.m .* P.g;
Fmax = P.vehicle.Fmax;
end
