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
   -march=native
   -mtune=native
   -flto
)

set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_DEBUG "-Og -ggdb3")

if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "Debug")
   message("-- Using debug flags by default.")
endif()
message("-- Build type: ${CMAKE_BUILD_TYPE}")

if(NOT GLPK_PREFIX)
   message("-- Falling GLPK_PREFIX back to defaults in /usr")
   set(GLPK_PREFIX "/usr")
else()
   message("-- Using as GLPK prefix: ${GLPK_PREFIX}")
   include_directories(${GLPK_PREFIX}/include)
   link_directories(${GLPK_PREFIX}/lib)
endif()

if(NOT CPLEX_PREFIX)
   message("-- Warning: CPLEX installation not specified, so disabling dependencies.")
   add_definitions(-UHAVE_CPLEX)
else()
   message("-- Using as CPLEX prefix: ${CPLEX_PREFIX}")
   include_directories(
      ${CPLEX_PREFIX}/cplex/include
      ${CPLEX_PREFIX}/concert/include
   )
   link_directories(
      ${CPLEX_PREFIX}/cplex/lib/x86-64_linux/static_pic
      ${CPLEX_PREFIX}/concert/lib/x86-64_linux/static_pic
   )
   add_definitions(-DHAVE_CPLEX)
endif()

# Uncomment the line below to solve MIP-based pricing 
# problems as a linear program.
# add_definitions(-DMIP_PRICING_LP)

include_directories(src)

set(mdvsp_SOURCES
   # Common implementation
   src/main.cpp
   src/Instance.cpp
   
   # Compact formulation with Coin-OR CBC
   src/ModelCbc.cpp

   # Implementations of the master problem
   src/colgen/CgMasterBase.cpp
   src/colgen/CgMasterGlpk.cpp
   src/colgen/CgMasterClp.cpp

   # Implementation of pricing algorithms.
   src/colgen/CgPricingBase.cpp
   src/colgen/PricingBellman.cpp
   src/colgen/PricingSpfa.cpp
   src/colgen/PricingGlpk.cpp
   src/colgen/PricingCbc.cpp
)

set(mdvsp_LIBRARIES
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

   # Boost program-options
   -lboost_program_options
)

if(CPLEX_PREFIX)
   set(mdvsp_SOURCES
      ${mdvsp_SOURCES}
      src/colgen/CgMasterCplex.cpp
      src/colgen/PricingCplex.cpp
   )
   set(mdvsp_LIBRARIES
      ${mdvsp_LIBRARIES}
      -pthread
      -lilocplex
      -lconcert
      -lcplex
      -ldl
   )
endif()

add_executable(main ${mdvsp_SOURCES})

target_link_libraries(main
   ${mdvsp_LIBRARIES}

   # Enables link-time optimization
   -flto
)
