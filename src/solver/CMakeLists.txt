OPTIMIZER_ADD_HEADERS(
  bcm_sdp_solver.h
  constrained_l1_solver.h
  l1_solver.h
  rank_restricted_sdp_solver.h
  rbr_sdp_solver.h
  riemannian_staircase.h
  sdp_solver.h
  solver_options.h
  summary.h)

OPTIMIZER_ADD_SOURCES(
  constrained_l1_solver.cc
  rank_restricted_sdp_solver.cc
  rbr_sdp_solver.cc
  riemannian_staircase.cc)

OPTIMIZER_ADD_GTEST(l1_solver_test l1_solver_test.cc)