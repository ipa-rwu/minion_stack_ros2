include "minion_laser_only.lua"


TRAJECTORY_BUILDER.pure_localization = true

POSE_GRAPH.optimize_every_n_nodes = 2
POSE_GRAPH.constraint_builder.sampling_ratio = 0.05
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.global_constraint_search_after_n_seconds = 20.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 6
MAP_BUILDER.num_background_threads = 8

POSE_GRAPH.global_sampling_ratio = 0.003

return options