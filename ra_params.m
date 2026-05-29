function P = ra_params(profile)
%RA_PARAMS Parameters for the residual-aware TruckSim co-simulation model.
%   P = RA_PARAMS() returns the five-truck brake-fade recovery setting used
%   as the default Simulink/TruckSim co-simulation template.
%
%   P = RA_PARAMS("composite20") returns the 20-truck composite stress-test
%   setting. The Simulink builder uses the compact five-truck setting by
%   default because it is easier to inspect and wire to TruckSim.

if nargin < 1 || strlength(string(profile)) == 0
    profile = "compact5";
end
profile = lower(string(profile));

P = struct();
P.profile = profile;
P.g = 9.81;
P.rho = 1.225;

switch profile
    case "composite20"
        P.N = 20;
        P.Ts = 0.08;
        P.Tend = 125.0;
        P.v0 = 23.5;
        P.r0 = 8.0;
        P.h = 1.85;
        P.tauC = 0.32;
        P.maxPredictionHorizon = 1.20;
        P.packetLossRate = 0.20;
        P.blackoutStart = 52.0;
        P.blackoutEnd = 52.65;
        P.fadeStart = 38.0;
        P.fadeEnd = 74.0;
        P.crosswindSpeed = 20.5;

        idx = (1:P.N).';
        phase = (idx - 1) / max(P.N - 1, 1);
        basePattern = [15000; 49000; 25000; 42000];
        mass = basePattern(mod(idx - 1, numel(basePattern)) + 1);
        mass = mass + 1700 * sin(2 * pi * phase + 0.4);
        mass = min(max(mass, 15000), 49000);
        alphaBrakeBase = 0.36 - 0.09 * phase;

        P.control.lambdaBar = 0.58;
        P.control.kpBar = 1.34;
        P.control.kiBar = 0.052;
        P.control.ksBar = 0.052;
        P.control.phiBar = 0.14;
        P.control.beta = 0.22;
        P.control.kvLag = 0.10;
        P.control.ffAccel = 0.32;
        P.control.observerBandwidth = 6.0;
        P.control.commandFilterHz = 1.1;

    otherwise
        P.N = 5;
        P.Ts = 0.04;
        P.Tend = 130.0;
        P.v0 = 24.0;
        P.r0 = 8.0;
        P.h = 1.80;
        P.tauC = 0.32;
        P.maxPredictionHorizon = 1.20;
        P.packetLossRate = 0.0;
        P.blackoutStart = inf;
        P.blackoutEnd = inf;
        P.fadeStart = 40.0;
        P.fadeEnd = 72.0;
        P.crosswindSpeed = 20.0;

        mass = [15000; 49000; 25000; 49000; 15000];
        alphaBrakeBase = linspace(0.32, 0.23, P.N).';

        P.control.lambdaBar = 0.58;
        P.control.kpBar = 1.36;
        P.control.kiBar = 0.060;
        P.control.ksBar = 0.055;
        P.control.phiBar = 0.13;
        P.control.beta = 0.23;
        P.control.kvLag = 0.11;
        P.control.ffAccel = 0.30;
        P.control.observerBandwidth = 6.0;
        P.control.commandFilterHz = 1.2;
end

idx = (1:P.N).';
phase = (idx - 1) / max(P.N - 1, 1);
massSpan = max(max(mass) - min(mass), 1);

P.vehicle.m = mass(:);
P.vehicle.tauAct = 0.30 + 0.20 * (mass(:) - min(mass)) / massSpan;
P.vehicle.tauMin = min(P.vehicle.tauAct);
P.vehicle.kappa = P.vehicle.tauAct / P.vehicle.tauMin;
P.vehicle.CdA = 6.2 + 2.8 * (mass(:) - min(mass)) / massSpan;
P.vehicle.Cr = 0.0055 + 0.0030 * phase;
P.vehicle.aMax = 1.10 * ones(P.N, 1);
P.vehicle.Fmax = P.vehicle.m .* P.vehicle.aMax;
P.vehicle.alphaBrakeBase = alphaBrakeBase(:);
P.vehicle.wAmp = 460 * sin(idx * 0.72 + 0.3);
P.vehicle.wFreq = 0.045 + 0.012 * idx;

P.predictor.deltaWarn = 0.35;
P.predictor.deltaSafe = 0.95;
P.predictor.maxHorizon = P.maxPredictionHorizon;

P.scheduling.kappaMin = 1.0;
P.scheduling.cDelta = 1.15;
P.scheduling.cTau = 0.65;

P.observer.bandwidth = P.control.observerBandwidth;
P.filter.commandHz = P.control.commandFilterHz;

P.antiwindup.rhoZ = 0.85;
P.antiwindup.epsZ = 1e-3;

P.initial.p = -((1:P.N).' .* (P.r0 + P.h * P.v0));
P.initial.v = P.v0 * ones(P.N, 1);
P.initial.a = zeros(P.N, 1);
P.initial.F = zeros(P.N, 1);
P.initial.alphaBrake = P.vehicle.alphaBrakeBase;

P.interface.stateColumns = ["x_m", "vx_mps", "ax_mps2", ...
    "Fx_N", "alpha_brake", "valid"];
P.interface.egoStateColumns = P.interface.stateColumns;
P.interface.trucksimCommandColumns = ["steer_rad", "throttle_0_1", ...
    "brake_0_1", "gear_cmd", "force_request_N"];
end
