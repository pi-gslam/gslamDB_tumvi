include(PICMake)

set(CMAKE_CXX_STANDARD 11)
pi_add_target(gslamDB_tumvi SHARED src REQUIRED GSLAM)

pi_report_target()
pi_install(TARGETS gslamDB_tumvi)
