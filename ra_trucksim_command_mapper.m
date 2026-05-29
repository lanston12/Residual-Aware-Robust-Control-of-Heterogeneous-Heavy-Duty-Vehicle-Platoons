function [tsCmd, forceRequest] = ra_trucksim_command_mapper(uFilt, alphaBrake, P)
%RA_TRUCKSIM_COMMAND_MAPPER Map force commands to TruckSim-style inputs.
%   tsCmd columns:
%   [steer_rad, throttle_0_1, brake_0_1, gear_cmd, force_request_N]

uFilt = uFilt(:);
alphaBrake = alphaBrake(:);
N = P.N;

forceRequest = uFilt;
tsCmd = zeros(N, 5);

for i = 1:N
    fMax = max(P.vehicle.Fmax(i), eps);
    fMin = -alphaBrake(i) * P.vehicle.m(i) * P.g;
    brakeDen = max(abs(fMin), eps);

    tsCmd(i, 1) = 0;
    tsCmd(i, 2) = min(1, max(0, uFilt(i) / fMax));
    tsCmd(i, 3) = min(1, max(0, -uFilt(i) / brakeDen));
    tsCmd(i, 4) = 1;
    tsCmd(i, 5) = uFilt(i);
end
end
