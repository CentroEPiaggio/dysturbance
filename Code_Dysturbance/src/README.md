# Gait performance analysis

Code developed by CSIC, and optimized for Eurobench purposes by Tecnalia.

## Installation guidelines

Follow the upper [Readme](../README.md) indications.

## Description

The original entry point `Main.m` is deprecated, and has been removed.

The current entry point is [computePI.m](computePI.m).

```octave
computePI("[path_to]/jointAngle.csv", "[path_to]/anthropometry.yaml")
```

The two parameters are:

- `jointAngle.csv`: a `csv` file containing the joint angles recorded, assuming the first column ia a timestamp in second.
- `anthropometry.yaml`: yaml file containing anthropometric data related to the subject.
  We are expecting values for `Shank`, `Thigh`, `Arm`, `Trunk`, `Foot`.

The current code is to be launched **per trial**.
There is no inter-trial computation for the moment.

## Initial code structure

The current documentation may not be updated.

Matlab algorithm to obtain some spatiotemporal data (step length, step and stride time) for human gait.

The main algorithm uses four functions to obtain the desired spatiotemporals:

- `find_leg_extension.m`:
  Detects each leg extension as the minimum after each peak in the angle of the knee.
  It returns a matrix, where the first row contains the angle at leg extension (a negative angle in this case), and the second row will contain the indices where the leg extension occurs.
- `segment_gait.m`:
  Segments the gait cycle using the leg extension.
- `calculate_events.m`:
  Saves the beginning of each stride (segment) as the heel strike.
  Since we used the leg extension to mark the beginning of each stride, it coincides with the heel strike.
- `calculate_spatiotemporal.m`:
  - Calculates the stride and the step time using the Heel Strike (obtained in calculate_events.m; above explained).
  - Uses these pre-processed parameters to obtain the step length.
  - As a bonus: it also takes the angles measured by the inertial sensors to calculate the joint positions.
