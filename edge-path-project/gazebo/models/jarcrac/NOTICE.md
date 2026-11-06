# JARCRAC model attribution and scope

This model is a **simplified Gazebo Harmonic approximation** of the JARCRAC model used in the NUVE-LAB / Mevea Ruskotunturi project. The original `Runko_part.STL` mesh and the derived geometry, inertial data, and generator parameters come from that source model. Source materials identify Copyright (C) 2018–2021 Mevea Ltd and Copyright (C) 2026 NUVE-LAB and describe the repository as GPL-3.0-or-later. A copy of the supplied GPL license is in `LICENSE.GPL-3.0`.

The JARCRAC visual mesh is derived from NUVE-LAB / Mevea source material. Redistribution rights for third-party geometry should be verified independently of the project's software license.

The visual is `Runko_part.STL`; collision geometry intentionally remains a simplified bounding box. The twelve-wheel arrangement does not include Mevea's rope-particle tracks, hydraulic powertrain or calibrated braking torque. Physics fidelity is **not** claimed. The 32-layer 180° front LiDAR uses Gazebo synthetic noise rather than a verified commercial sensor model. Flat-ground AEB results do not establish operation on uneven forest terrain.
