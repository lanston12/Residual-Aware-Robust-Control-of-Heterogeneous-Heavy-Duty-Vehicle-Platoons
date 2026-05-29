function ctrlState = ra_init_controller_state(P)
%RA_INIT_CONTROLLER_STATE Initial internal states for the control modules.

ctrlState.z = zeros(P.N, 1);
ctrlState.lesoX1 = zeros(P.N, 1);
ctrlState.lesoX2 = zeros(P.N, 1);
ctrlState.uFilt = zeros(P.N, 1);
end
