# 2026-09-14 pickup safety and dynamic-chain regression

This snapshot contains the deployed, evidence-validated changes for:

- tiered empty-base pickup speed: 1.40 m/s transit, 0.60 m/s inside the final 1.60 m dock envelope;
- handoff gating on measured base speed before fine docking;
- deterministic dynamic-obstacle predictor startup without requiring PyTorch;
- UDPv4 Fast-DDS service transport and real `/map` + `/cube_obstacle_map` startup probes;
- A-zone x=0.18 placement line and 0.35 s physical placement settling.

Key evidence:

- `mixed_3blue_A_2red_C_20260914.log`: original blue-cube-3 collision signature;
- `blue3_A_tiered_speed_dynamic10_20260914.log`: directed blue3 regression pass;
- `blue5_A_inset018_settle035_20260914.log`: directed A placement pass;
- `three_blue_A_final_20260914.log`: blue5/blue4/blue3 continuous pass, 181.187 s, zero recoveries.

No world coordinates, wall/stone geometry, obstacle routes, robot mass, or global safety radius were changed.
