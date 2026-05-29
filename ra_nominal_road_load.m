function dNom = ra_nominal_road_load(v, P)
%RA_NOMINAL_ROAD_LOAD Nominal force-level road load for all followers.

v = v(:);
dNom = -P.vehicle.m .* P.g .* P.vehicle.Cr ...
    -0.5 * P.rho .* P.vehicle.CdA .* v.^2;
end
