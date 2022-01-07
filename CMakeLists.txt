cmake_minimum_required(VERSION 3.0)

project(mdvsp-colgen CXX)

add_definitions(
   -Wall 
   -Wextra 
   -std=c++17
   -ggdb3
   -fopenmp
   -Wno-deprecated-copy
   -Wno-deprecated-declarations
)

include_directories(src)

set(GLPK_PREFIX /home/alberto/work/glpk-5.0/prefix)
message("-- Using as GLPK prefix: ${GLPK_PREFIX}")
include_directories(${GLPK_PREFIX}/include)
link_directories(${GLPK_PREFIX}/lib)

set(CPLEX_PREFIX /home/alberto/.local/ibm/ILOG/CPLEX_Studio201/)
message("-- Using as CPLEX prefix: ${CPLEX_PREFIX}")
include_directories(
   ${CPLEX_PREFIX}/cplex/include 
   ${CPLEX_PREFIX}/concert/include
)
link_directories(
   ${CPLEX_PREFIX}/cplex/lib/x86-64_linux/static_pic 
   ${CPLEX_PREFIX}/concert/lib/x86-64_linux/static_pic
)

add_executable(main
   # Common implementation
   src/main.cpp
   src/Instance.cpp
   
   # Compact formulation with Coin-OR CBC
   src/ModelCbc.cpp

   # Implementations of the master problem
   src/colgen/CgMasterBase.cpp
   src/colgen/CgMasterGlpk.cpp
   src/colgen/CgMasterCplex.cpp

   # Implementation of pricing algorithms.
   src/colgen/CgPricingBase.cpp
   src/colgen/PricingBellman.cpp
   src/colgen/PricingSpfa.cpp
   src/colgen/PricingGlpk.cpp
   src/colgen/PricingCbc.cpp   
   src/colgen/PricingCplex.cpp
)

target_link_libraries(main
   # GLPK dependencies
   -lglpk

   # Coin-OR CBC dependencies
   -lCbcSolver 
   -lCbc 
   -lpthread 
   -lrt 
   -lCgl 
   -lOsiClp 
   -lClpSolver 
   -lClp 
   -lOsi 
   -lCoinUtils 
   -lbz2 
   -lz 
   -llapack 
   -lblas 
   -lm

   # OpenMP support library
   -lgomp

   # IBM CPLEX dependencies   
   -pthread
   -lilocplex
   -lconcert
   -lcplex   
   -ldl

   # Boost program-options
   -lboost_program_options
)
