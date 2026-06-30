# Coverage Planner Research Ledger

Purpose: keep durable research notes for the fresh V2 coverage planner. This is not the implementation plan by itself; it is the evidence log that the plan should cite when choosing algorithms, metrics, and edge-case handling.

This document deliberately stores source summaries and links instead of copying full papers into the repo. Many papers are copyrighted or hosted under publisher terms. Open-access PDFs may be kept locally for reading, but this ledger is the durable source-tracking artifact.

## Local Seed Paper

| ID | Source | Local copy | Why it matters |
|---|---|---|---|
| `shah_2025_lawn_cpp` | Nikunj Shah, Utsav Dey, Kenji Nishimiya. "End-to-End Framework for Robot Lawnmower Coverage Path Planning using Cellular Decomposition." arXiv:2506.06028, ICRA 2025 Field Robotics workshop. <https://arxiv.org/abs/2506.06028> | `/Users/martinhaghani/Downloads/2506.06028v1.pdf` | Closest source to our problem. It treats lawnmower CPP as an end-to-end pipeline from user boundary to GPS waypoints, optimizes decomposition angle, merges decomposition sections, previews paths, and evaluates coverage, mowing distance, non-mowing distance, turns, and distance per coverage. |

Key takeaways from `shah_2025_lawn_cpp`:

- The authors explicitly target irregular lawn shapes, operational efficiency, and lawn aesthetics, not just mathematical coverage.
- Their `AdaptiveDecompositionCPP` iterates through decomposition angles from 0 to 180 degrees, decomposes the area at each angle, merges sections, and selects a decomposition with fewer regions.
- Their comparison reports lower non-mowing distance from merging decomposed regions. This supports our instinct that raw cell count is not a good final objective.
- Their metric set is useful for this repo: coverage percentage, non-mowing distance, mowing distance, number of decompositions, number of turns, and distance per coverage.
- Their own conclusion notes that section merging and sub-section mowing order remain optimization opportunities. For our mower, this is exactly where motion primitives and endpoint routing must enter.

## Core Coverage Planning Sources

| ID | Source | What to use |
|---|---|---|
| `choset_1997_bcd` | Howie Choset and Philippe Pignon. "Coverage Path Planning: The Boustrophedon Decomposition." Field and Service Robotics, 1997. <https://publications.ri.cmu.edu/coverage-path-planning-the-boustrophedon-decomposition> | Exact cellular decomposition: decompose free space into cells, cover each cell with back-and-forth motions, then solve an exhaustive route through the cell adjacency graph. Use as the theoretical baseline, not as the final lawn strategy. |
| `choset_2000_known_spaces` | Howie Choset. "Coverage of Known Spaces: The Boustrophedon Cellular Decomposition." Autonomous Robots, 2000. <https://publications.ri.cmu.edu/coverage-of-known-spaces-the-boustrophedon-cellupdar-decomposition> | BCD is a generalization of trapezoidal decomposition and can produce more efficient coverage paths. Useful for known, polygonal-ish lawn maps and obstacle splits. |
| `choset_2001_survey` | Howie Choset. "Coverage for robotics - A survey of recent results." Annals of Mathematics and Artificial Intelligence, 2001. <https://publications.ri.cmu.edu/coverage-for-robotics-a-survey-of-recent-results> | Separates heuristic, approximate, partial-approximate, and exact cellular approaches. Important conclusion: complete algorithms usually use exact cellular decomposition explicitly or implicitly. |
| `galceran_2013_survey` | Enric Galceran and Marc Carreras. "A survey on coverage path planning for robotics." Robotics and Autonomous Systems, 2013. DOI `10.1016/j.robot.2013.09.004`. <https://colab.ws/articles/10.1016/j.robot.2013.09.004> | Broad taxonomy and field applications. Useful as a checkpoint when deciding whether an idea is a known CPP family or an ad hoc local fix. |
| `acar_2002_morse` | Ercan Acar, Howie Choset, Alfred Rizzi, Prasad Atkar, Douglas Hull. "Morse Decompositions for Coverage Tasks." IJRR, 2002. DOI `10.1177/027836402320556359`. <https://journals.sagepub.com/doi/10.1177/027836402320556359> | Critical points of a Morse function define cell boundaries; changing the Morse function changes coverage pattern. This supports multi-angle decomposition and task-specific decomposition axes. |
| `acar_2002_sensor_based` | E. Acar and H. Choset. "Sensor-based Coverage: Incremental Construction of Cellular Decompositions." WAFR, 2002. <https://publications.ri.cmu.edu/sensor-based-coverage-incremental-construction-of-cellular-decompositions> | Combines Morse decomposition for open areas with generalized Voronoi behavior in narrow/cluttered spaces. This directly supports our proposed split: stripe broad body regions, but treat narrow corridors/notches through skeleton/centerline logic. |

## Lawnmower-Specific And Edge Coverage Sources

| ID | Source | What to use |
|---|---|---|
| `tian_2022_edge_mowing` | Zhaofeng Tian and coauthor. "Edge Coverage Path Planning for Robot Mowing." arXiv:2209.05405. <https://arxiv.org/abs/2209.05405> | Edge coverage is not just normal interior CPP. Naive obstacle dilation/circumcircle methods leave concave-edge regions unmowed. This supports modeling the boundary band/headland as an explicit task family. |
| `sportelli_2021_systematic` | Mino Sportelli et al. "Robotic Mowing of Tall Fescue at 90 mm Cutting Height: Random Trajectories vs. Systematic Trajectories." Agronomy 2021. DOI `10.3390/agronomy11122567`. <https://www.mdpi.com/2073-4395/11/12/2567> | Systematic trajectories can reach similar or better mowed area with far less travel than random mowing. Use as motivation for planned, stripe-like coverage over random exploration. The reported working efficiency was about 80% for systematic trajectories and about 35% for random trajectories on their tall-fescue trials. |

## Libraries And Modular Framework Sources

| ID | Source | What to use |
|---|---|---|
| `fields2cover_2022` | Gonzalo Mier, Joao Valente, Sytze de Bruin. "Fields2Cover: An open-source coverage path planning library for unmanned agricultural vehicles." arXiv:2210.07838. <https://arxiv.org/abs/2210.07838> | The modular architecture is useful: headland generator, swath generator, route planner, path planner. Our V2 should preserve this separation but add mower-specific task classification and motion primitives. |
| `fields2cover_route_docs` | Fields2Cover route planning tutorial. <https://fields2cover.github.io/source/tutorials/route_planning.html> | Route planning searches swath order, supports metaheuristics, boustrophedon order, snake order, spiral order, and custom order. Snake order is especially relevant because it skips one swath to reduce sharp turns. |
| `fields2benchmark_2025` | Gonzalo Mier, Ana Maria Casado Fauli, Joao Valente, Sytze de Bruin. "Fields2Benchmark: An open-source benchmark for coverage path planning methods in agriculture." Smart Agricultural Technology 12:101156, 2025. DOI `10.1016/j.atech.2025.101156`. <https://research.wur.nl/en/publications/fields2benchmark-an-open-source-benchmark-for-coverage-path-plann> | Supports evaluating CPP modularly across field decomposition, swath generation, headland generation, route planning, and path planning. Also notes non-convex fields and obstacles as benchmark needs. |
| `shapely_docs` | Shapely 2.1 manual. <https://shapely.readthedocs.io/en/stable/manual.html> | Use for exact polygon operations, buffers, intersections, minimum rotated rectangle, polygon validity repair, polygonize, Voronoi helpers. |
| `skimage_medial_axis_docs` | scikit-image morphology docs. <https://scikit-image.org/docs/stable/api/skimage.morphology.html> | `medial_axis(..., return_distance=True)` computes the medial axis as ridges of the distance transform. Useful for a lab-only skeleton task report; runtime can later replace this with a polygonal or grid C++ implementation. |
| `ortools_routing_docs` | Google OR-Tools routing docs. <https://developers.google.com/optimization/routing> | Useful for route ordering once we have a graph of coverage tasks/candidates and motion edge costs. It handles TSP/VRP-style problems and dropped visits/resource constraints. |
| `networkx_tsp_docs` | NetworkX TSP approximation docs. <https://networkx.org/documentation/stable/reference/algorithms/generated/networkx.algorithms.approximation.traveling_salesman.traveling_salesman_problem.html> | Useful for small lab prototypes and sanity checks, especially converting incomplete graphs to complete graphs through all-pairs shortest paths. |

## Optimization And Turn-Cost Sources

| ID | Source | What to use |
|---|---|---|
| `krupke_2024_turn_costs` | Dominik Krupke. "Near-Optimal Coverage Path Planning with Turn Costs." ALENEX 2024 / arXiv:2310.20340. <https://arxiv.org/abs/2310.20340> | Explicitly models arbitrary polygonal environments, turn costs, partial/prize-collecting coverage, and heterogeneous passage costs. This supports treating low-value notches as optional/prize-collecting only after we quantify missed area. |
| `ramesh_2024_submodular_sector_cover` | Megnath Ramesh et al. "Approximate Environment Decompositions for Robot Coverage Planning using Submodular Set Cover." arXiv:2409.03120. <https://arxiv.org/abs/2409.03120> | Decompose into possibly overlapping sectors that each support a lawnmower path at an angle. Useful as an alternative candidate generator to avoid over-trusting BCD cuts. |
| `kapoutsis_2017_darp` | Athanasios Kapoutsis, Savvas Chatzichristofis, Elias Kosmatopoulos. "DARP: Divide Areas Algorithm for Optimal Multi-Robot Coverage Path Planning." <https://kapoutsis.info/wp-content/uploads/2017/02/j3.pdf> | Mostly multi-robot, but its grid partitioning and connectivity constraints are useful future references for multi-mower or multi-task balancing. Not a primary single-mower algorithm for this project. |

## Sources I Intentionally Do Not Treat As Primary For V2

- Pure neural/reinforcement methods: useful for unknown/dynamic spaces, but hard to debug and unnecessary for known lawn polygons. The current user need is explainable geometry and route quality.
- Pure grid/STC coverage: robust and complete, but tends to produce many abrupt turns and less lawn-like visual output. Useful as a fallback or diagnostic baseline, not the primary lawn aesthetic path.
- K-means track clustering: the Shah 2025 paper calls out the need for a user-chosen `K` and local-optimum behavior, which makes it a poor foundation for an automatic mower planner.

## Research Conclusions For This Repo

1. **Do not make "zones" the first-class objective.** The literature decomposes because it makes coverage and routing tractable. Our planner should output coverage tasks and a route, not a visually plausible zone map.
2. **Use exact decomposition for completeness, but skeletons for semantics.** BCD/Morse cuts explain where sweep topology changes. Medial-axis/skeleton branches explain where the lawn behaves like a corridor, dead-end corridor, or notch.
3. **Model edge coverage separately.** Lawnmower edge quality is important enough to deserve its own coverage task family.
4. **Generate alternatives and score them.** The best decomposition angle, merging policy, stripe order, and task order should be selected by route/coverage score, not fixed by human drawing.
5. **Routing must include motion costs.** A decomposition with fewer cells can still be worse if it creates impossible turns or long non-cutting travel. Motion primitive feasibility must be part of candidate scoring.
6. **"Perfect for any lawn" must mean verified or explicitly rejected.** Within a valid static 2D map and configured mower model, the planner can prove safety and coverage against sampled/swept geometry. For truly impossible regions, it should show an explicit unreachable/low-value/not-supported result rather than hiding the problem.
