function spec = ra_trucksim_interface_spec(P)
%RA_TRUCKSIM_INTERFACE_SPEC Interface contract for the TruckSim S-function.

if nargin < 1 || isempty(P)
    P = ra_params("compact5");
end

spec.modelName = "ResidualAware_TruckSim_Cosim";
spec.sFunctionPlaceholder = "TruckSim S-Function Interface (placeholder)";
spec.expectedTruckSimBlock = "vs_sf or the project-specific TruckSim S-function";

spec.inputs(1).name = "tsCmd";
spec.inputs(1).size = [P.N, 5];
spec.inputs(1).columns = P.interface.trucksimCommandColumns;
spec.inputs(1).description = "Command matrix sent to TruckSim for each truck.";

spec.inputs(2).name = "forceRequest";
spec.inputs(2).size = [P.N, 1];
spec.inputs(2).columns = "force_request_N";
spec.inputs(2).description = "Longitudinal force request retained for direct-force TruckSim configurations.";

spec.outputs(1).name = "egoState";
spec.outputs(1).size = [P.N, 6];
spec.outputs(1).columns = P.interface.egoStateColumns;
spec.outputs(1).description = "Truck states returned by TruckSim and consumed by the controller.";

spec.notes = [
    "Replace the placeholder subsystem with one TruckSim S-function per vehicle or a multiplexed S-function wrapper.", ...
    "Map TruckSim channels XCG_TM, VX_TM, AX_TM or the project equivalents into egoState(:,1:3).", ...
    "Map wheel/axle longitudinal force into egoState(:,4), brake authority into egoState(:,5), and validity into egoState(:,6).", ...
    "Keep the matrix dimensions unchanged so the control and prediction modules remain compatible."
    ];
end
