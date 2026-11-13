# Q1 smooth dance skill

This is the previously trained Q1 single-skill package integrated for the local walking/dance controller.

- Export commit: `2bfc9d26216e40622033bda8df2f5cb129e5bc85`
- Runtime: 50 Hz, `obs[*,720] + goal[*,546] -> action[*,22] + joint_target[*,22]`
- Policy SHA-256: `a27ef093b55a34fbd680d8e624c02736158d4d34d504e617b84193b95374aeab`
- Motion SHA-256: `9bc30aa9fe7909c83b249f33db4028d2a5c7b645a5ee20c12924646c2c845e16`

The package has the same joint order, default pose, observation layout, action scale, and PD gains as `dual_v3b`.
It is switched as a separate `SkillModel`; the controller blends joint targets for 0.4 seconds because the two policies do not share a latent decoder.

This package is not cleared for unprotected hardware operation. Validate in simulation, then use the vendor-approved low-level mode, a harness, an independent emergency stop, and staged low-risk tests.
