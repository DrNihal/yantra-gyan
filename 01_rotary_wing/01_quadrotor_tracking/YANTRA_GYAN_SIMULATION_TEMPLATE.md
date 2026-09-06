# Yantra Gyan — Simulation Package Template

Every future simulation should follow this structure:

```text
Yantra_Gyan_XX_Name/
│
├── README.md
├── dynamics_*.md
├── controller_design.md / algorithm.md
├── results_and_discussion.md
├── requirements.txt
│
├── clearly_commented_source_files.py
│
└── results/
    ├── figures/
    ├── raw_data/
    └── summary/
```

## Documentation rule

The documentation explains the mathematics and experiment.

The code explains the implementation.

The results document explains what the numbers mean.

## Code rule

Every non-obvious executable statement should have a concise comment explaining:
1. what it does,
2. why it is needed,
3. and, where useful, which equation or physical concept it represents.

Comments should explain the code, not merely translate syntax into English.
