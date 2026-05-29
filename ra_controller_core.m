function [uRaw, uSat, uFilt, ctrlStateNext, diagBus] = ra_controller_core(t, egoState, predState, packetAge, alphaBrake, ctrlState, P)
%RA_CONTROLLER_CORE Residual-aware cooperative longitudinal controller.
%   Implements the paper-level modules used by the compact manuscript:
%   delay-aware predecessor prediction, UB-LESO disturbance estimation,
%   DNGS gain scheduling, residual-aware sliding control, saturation, AS-LPF,
%   and moving-set anti-windup.
%
%   diagBus columns:
%   [spacing_error, sliding_surface, d_hat, packet_age, confidence,
%    alpha_brake, antiwindup_gate, saturation_margin, z_int,
%    u_raw, u_sat, u_filt]

if nargin < 6 || isempty(ctrlState)
    ctrlState = ra_init_controller_state(P);
end

N = P.N;
Ts = P.Ts;
uRaw = zeros(N, 1);
uSat = zeros(N, 1);
uFilt = zeros(N, 1);
diagBus = zeros(N, 12);

ctrlStateNext = ctrlState;
roadLoad = ra_nominal_road_load(egoState(:, 2), P);
alphaBrake = alphaBrake(:);
packetAge = packetAge(:);

for i = 1:N
    m = P.vehicle.m(i);
    tau = P.vehicle.tauAct(i);
    p = egoState(i, 1);
    v = egoState(i, 2);
    a = egoState(i, 3);
    uPrev = ctrlState.uFilt(i);

    kappa = max(P.scheduling.kappaMin, 1 + P.scheduling.cDelta * packetAge(i) + P.scheduling.cTau * tau);
    lambda = P.control.lambdaBar / sqrt(kappa);
    kp = P.control.kpBar / kappa;
    ki = P.control.kiBar / (kappa^2);
    ks = P.control.ksBar / kappa;
    phi = P.control.phiBar * sqrt(kappa);

    horizon = predState(i, 5);
    pEgo = p + v * horizon + 0.5 * a * horizon^2;
    vEgo = v + a * horizon;

    e = predState(i, 1) - pEgo - P.r0 - P.h * vEgo;
    eDot = predState(i, 2) - vEgo - P.h * a;
    s = eDot + lambda * e;

    eta = confidence_from_age(packetAge(i), P);
    bHat = 1 / m;
    omega = P.observer.bandwidth / sqrt(kappa);
    y = eDot;
    obsErr = y - ctrlState.lesoX1(i);
    uRef = predState(i, 3) - a;
    x1Dot = ctrlState.lesoX2(i) + uRef + bHat * uPrev + 2 * omega * obsErr;
    x2Dot = omega^2 * obsErr;
    ctrlStateNext.lesoX1(i) = ctrlState.lesoX1(i) + Ts * x1Dot;
    ctrlStateNext.lesoX2(i) = ctrlState.lesoX2(i) + Ts * x2Dot;
    dHat = ctrlStateNext.lesoX2(i);

    previewTerm = P.control.beta * predState(i, 4);
    lagTerm = P.control.kvLag * (predState(i, 2) - vEgo) / sqrt(kappa);
    aNom = (predState(i, 2) - vEgo - previewTerm) / max(P.h, 0.2) ...
        + P.control.ffAccel * predState(i, 3) + lagTerm;
    aCmd = aNom - kp * s - ki * ctrlState.z(i) - ks * ra_sat_unit(s / phi) - dHat;

    uRaw(i) = m * aCmd - roadLoad(i);
    fMin = -alphaBrake(i) * m * P.g;
    fMax = P.vehicle.Fmax(i);
    uSat(i) = min(fMax, max(fMin, uRaw(i)));

    upperLocked = (uRaw(i) >= fMax) && (s > 0);
    lowerLocked = (uRaw(i) <= fMin) && (s < 0);
    chi = double(~(upperLocked || lowerLocked));
    zRaw = ctrlState.z(i) + Ts * chi * s;
    zBrake = P.antiwindup.rhoZ * abs(fMin) / max(m * ki, P.antiwindup.epsZ);
    zTraction = P.antiwindup.rhoZ * fMax / max(m * ki, P.antiwindup.epsZ);
    ctrlStateNext.z(i) = min(zTraction, max(-zBrake, zRaw));

    alphaU = 1 - exp(-2 * pi * P.filter.commandHz * Ts);
    uFilt(i) = uPrev + alphaU * (uSat(i) - uPrev);
    uFilt(i) = min(fMax, max(fMin, uFilt(i)));
    ctrlStateNext.uFilt(i) = uFilt(i);

    satMargin = min(fMax - uRaw(i), uRaw(i) - fMin);
    diagBus(i, :) = [e, s, dHat, packetAge(i), eta, alphaBrake(i), chi, ...
        satMargin, ctrlStateNext.z(i), uRaw(i), uSat(i), uFilt(i)];
end

% Keep time visible to MATLAB Coder without changing the interface.
if t < -realmax
    diagBus(:) = 0;
end
end

function eta = confidence_from_age(age, P)
den = max(P.predictor.deltaSafe - P.predictor.deltaWarn, eps);
eta = (P.predictor.deltaSafe - age) / den;
eta = min(1, max(0, eta));
end
