function leaderState = ra_leader_profile(t, P)
%RA_LEADER_PROFILE Leader motion used by the co-simulation template.
% Output leaderState = [x, v, a, e].

v0 = P.v0;
a = 0.0;

if t >= 12 && t < 22
    a = 0.35;
elseif t >= 34 && t < 42
    a = -1.25;
elseif t >= 62 && t < 70
    a = 0.65;
elseif t >= 90 && t < 98
    a = -0.65;
end

% Closed-form position/velocity approximation used only to seed the
% co-simulation interface. TruckSim can replace this with a driven leader.
v = max(0.1, v0 + integralAccelApprox(t));
x = v0 * t + 0.5 * integralVelocityCorrection(t);
leaderState = [x, v, a, 0.0];
end

function dv = integralAccelApprox(t)
segments = [
    12, 22,  0.35
    34, 42, -1.25
    62, 70,  0.65
    90, 98, -0.65];
dv = 0.0;
for k = 1:size(segments, 1)
    t0 = segments(k, 1);
    t1 = segments(k, 2);
    a = segments(k, 3);
    dv = dv + a * max(0.0, min(t, t1) - t0);
end
end

function dx2 = integralVelocityCorrection(t)
segments = [
    12, 22,  0.35
    34, 42, -1.25
    62, 70,  0.65
    90, 98, -0.65];
dx2 = 0.0;
for k = 1:size(segments, 1)
    t0 = segments(k, 1);
    t1 = segments(k, 2);
    a = segments(k, 3);
    dtActive = max(0.0, min(t, t1) - t0);
    dx2 = dx2 + a * dtActive^2;
    if t > t1
        dx2 = dx2 + 2 * a * (t1 - t0) * (t - t1);
    end
end
end
