function egoState = ra_fallback_trucksim_plant_step(tsCmd, forceRequest, P)
%RA_FALLBACK_TRUCKSIM_PLANT_STEP Lightweight placeholder plant.
%   This function only preserves the signal contract before TruckSim is
%   inserted. It is not a substitute for TruckSim validation.

persistent x v a f alpha

if isempty(x)
    x = P.initial.p;
    v = P.initial.v;
    a = P.initial.a;
    f = P.initial.F;
    alpha = P.initial.alphaBrake;
end

if ~isempty(forceRequest)
    f = forceRequest(:);
end

drag = ra_nominal_road_load(v, P);
a = (f + drag) ./ P.vehicle.m;
v = max(0, v + P.Ts * a);
x = x + P.Ts * v;

if ~isempty(tsCmd) && size(tsCmd, 2) >= 3
    alpha = max(0.02, P.vehicle.alphaBrakeBase .* (1 - 0.02 * tsCmd(:, 3)));
end

egoState = [x, v, a, f, alpha, ones(P.N, 1)];
end
