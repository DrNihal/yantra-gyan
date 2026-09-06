# Results

This directory contains the numerical and graphical outputs of Yantra Gyan #01.

## Files

- `case_A_nominal_*` — nominal-model experiment.
- `case_B_uncertain_*` — 20% mass/inertia uncertainty.
- `case_C_disturbed_*` — uncertainty plus external disturbance.
- `YG01_combined_3D_tracking.png` — combined 3-D comparison.
- `YG01_tracking_error_comparison.png` — tracking-error comparison.
- `summary.json` / `summary.csv` — numerical summary.
- `*_data.npz` — raw saved simulation data for reproducibility.

## Primary metrics

Position RMSE and maximum position error are defined in `analysis.py`.

The interpretation of the results is documented in:

`../results_and_discussion.md`
