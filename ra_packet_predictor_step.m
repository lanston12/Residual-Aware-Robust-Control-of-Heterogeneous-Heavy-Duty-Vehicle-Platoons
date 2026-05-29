function [predState, packetAge, confidence, lastPacketNext] = ra_packet_predictor_step(t, egoState, leaderState, lastPacket, packetLossMask, P)
%RA_PACKET_PREDICTOR_STEP Packet-age-aware predecessor state predictor.
%   egoState:       N-by-6 [x, vx, ax, Fx, alpha_brake, valid]
%   leaderState:    1-by-4 [x, vx, ax, preview_error]
%   lastPacket:     N-by-5 [x, vx, ax, preview_error, time_stamp]
%   packetLossMask: N-by-1, true means the packet at this step is lost.
%   predState:      N-by-5 [x_hat, vx_hat, ax_hat, preview_error_hat, horizon]

N = P.N;

if nargin < 4 || isempty(lastPacket) || ~isequal(size(lastPacket), [N, 5])
    lastPacket = zeros(N, 5);
    for i = 1:N
        if i == 1
            src = leaderState(:).';
        else
            src = [egoState(i - 1, 1:3), 0];
        end
        lastPacket(i, :) = [src(1), src(2), src(3), src(4), t];
    end
end

if nargin < 5 || isempty(packetLossMask)
    packetLossMask = false(N, 1);
end

packetLossMask = logical(packetLossMask(:));
lastPacketNext = lastPacket;
predState = zeros(N, 5);
packetAge = zeros(N, 1);
confidence = zeros(N, 1);

for i = 1:N
    if ~packetLossMask(i)
        if i == 1
            src = leaderState(:).';
            lastPacketNext(i, :) = [src(1), src(2), src(3), src(4), t];
        else
            src = egoState(i - 1, :);
            lastPacketNext(i, :) = [src(1), src(2), src(3), 0, t];
        end
    end

    pkt = lastPacketNext(i, :);
    age = max(0, t - pkt(5));
    eta = confidence_from_age(age, P);
    horizon = min(P.predictor.maxHorizon, age + P.vehicle.tauAct(i));

    predState(i, 1) = pkt(1) + pkt(2) * horizon + 0.5 * eta * pkt(3) * horizon^2;
    predState(i, 2) = pkt(2) + eta * pkt(3) * horizon;
    predState(i, 3) = eta * pkt(3);
    predState(i, 4) = eta * pkt(4);
    predState(i, 5) = horizon;

    packetAge(i) = age;
    confidence(i) = eta;
end
end

function eta = confidence_from_age(age, P)
den = max(P.predictor.deltaSafe - P.predictor.deltaWarn, eps);
eta = (P.predictor.deltaSafe - age) / den;
eta = min(1, max(0, eta));
end
