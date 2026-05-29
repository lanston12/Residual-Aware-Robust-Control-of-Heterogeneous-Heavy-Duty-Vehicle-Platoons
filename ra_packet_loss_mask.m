function lossMask = ra_packet_loss_mask(t, P)
%RA_PACKET_LOSS_MASK Deterministic packet-loss and blackout scheduler.
% The mask is true when the follower does not receive a fresh predecessor
% packet during the current control tick.

lossMask = false(P.N, 1);

if t >= P.blackoutStart && t <= P.blackoutEnd
    lossMask(:) = true;
    return
end

if P.packetLossRate <= 0
    return
end

idx = (1:P.N).';
hashLike = 0.5 + 0.5 * sin(12.9898 * idx + 78.233 * floor(t / P.Ts));
lossMask = hashLike < P.packetLossRate;
end
