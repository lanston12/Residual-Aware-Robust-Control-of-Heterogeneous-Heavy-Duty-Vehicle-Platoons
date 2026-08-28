# Residual-Aware Robust Control of Heterogeneous Heavy-Duty Vehicle Platoons

Code and numerical results accompanying the manuscript:

"Residual-Aware Robust Control of Heterogeneous Heavy-Duty
Vehicle Platoons Under Delay and Actuation Constraints"

submitted to IEEE Transactions on Vehicular Technology.

## Repository scope

This repository contains:

1. the numerical scenario scripts and frozen result data used
   to generate the reported figures and tables;
2. plotting scripts for the revised manuscript;
3. a legacy_trucksim_template of the residual-aware
   controller;
4. a legacy MATLAB/Simulink–TruckSim integration template.

## Important implementation note

The residual analysis in the revised manuscript is formulated
through a bounded disturbance-estimation interface and does not
require a unique observer realization.

The publication scenarios use the estimator realization documented
for each scenario in `publication/PROVENANCE.md`.
The dedicated packet-loss Monte Carlo test uses direct simulated
disturbance compensation to isolate communication-induced prediction
residuals and is not used to validate the disturbance-estimation layer.

## Paper–code correspondence

| Paper item | Script/data | Main mechanism |
| ... |

## Requirements

- MATLAB ...
- Simulink ...
- Optimization Toolbox ...
- TruckSim ... only for ...

## Data availability

Basically, the data presented in the paper are produced by the source code and the cosimulation model. 

