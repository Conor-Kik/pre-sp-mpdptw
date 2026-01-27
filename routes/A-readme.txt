This folder contains solutions to the MPDPTW benchmark instances provided by I. Aziez, J.-F. Côté, and L.-C. Coelho.

The routes were generated using the PRE-SP framework and are exact (proven optimal) with respect to the objective of minimizing total travel time.

Each instance has an uncapacitated (uncap) route and may also have a capicated (cap) route if it differs.

Note on reported arrival times: The time propagation variable S is used only to enforce time-feasibility via lower-bound constraints (waiting is allowed and is not penalised in the objective). Service times at each node are included in the time propagation (i.e., travel + service time is accounted for when computing subsequent times), even if this is not explicitly highlighted in the printed routes. As a result, the printed service start times may not correspond to a fully “tight” propagated schedule, and can appear inconsistent, especially for instances with weaker time windows (e.g., W instances).