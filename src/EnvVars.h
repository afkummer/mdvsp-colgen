#pragma once

#include <iostream>
#include <string>
#include <limits>
#include <cstring>

/**
 * Defines the maximum number of arcs to explore during the pricing solver.
 * It only affects the root node of the column generation. 
 * Default value: max value of int32
 */
#define MAX_LABEL_EXPANSIONS "MAX_LABEL_EXPANSIONS"

/**
 * Defines the maximum number of arcs to explore during the pricing solver.
 * It only affects the truncated column generation step of the solver.
 * Default value: max value of int32
 */
#define MAX_LABEL_EXPANSIONS_TCG "MAX_LABEL_EXPANSIONS_TCG"

/**
 * Flags whether the adjacency list of deadhead arcs should be sorted by cost.
 * Default value: 0 (false)
 */
#define SORT_DEADHEAD_ARCS "SORT_DEADHEAD_ARCS"

/**
 * Limits the number of column generation iterations in each step of the
 * truncated column generation algorithm.
 * Default value: 20
 */
#define TCG_MAX_SUB_ITERATIONS "TCG_MAX_SUB_ITERATIONS"

/**
 * Defines the algorithm used to select the next variable to fix in TCG.
 * Values: simple (default), grasp
 */
#define TCG_VAR_SEL "TCG_VAR_SEL"
#define TCG_VAR_SEL_SIMPLE 0
#define TCG_VAR_SEL_GRASP 1

/**
 * Defines the strategy to compute CL for GRASP
 * Values: direct (default), eval
 */
#define TCG_GRASP_STRATEGY "TCG_GRASP_STRATEGY"
#define TCG_GRASP_STRATEGY_DIRECT 0
#define TCG_GRASP_STRATEGY_EVAL 1

/**
 * Defines the alpha value used to form the RCL.
 * Values: [0.0, 1.0], default of 0.2
 */
#define TCG_GRASP_ALPHA "TCG_GRASP_ALPHA" 

inline auto getEnvMaxLabelExpansions() noexcept -> int {
   if (getenv(MAX_LABEL_EXPANSIONS)) {
      int value = std::stoi(getenv(MAX_LABEL_EXPANSIONS));
      if (value > 0) {
         std::cout << "Read MAX_LABEL_EXPANSIONS = " << value << "\n";
         return value;
      } else {
         std::cout << "Bad value for MAX_LABEL_EXPANSIONS: " << getenv(MAX_LABEL_EXPANSIONS) << std::endl;
         exit(EXIT_FAILURE);
      }
   }
   return std::numeric_limits<int>::max();
}

inline auto getEnvMaxLabelExpansionsTcg() noexcept -> int {
   if (getenv(MAX_LABEL_EXPANSIONS_TCG)) {
      int value = std::stoi(getenv(MAX_LABEL_EXPANSIONS_TCG));
      if (value > 0) {
         std::cout << "Read MAX_LABEL_EXPANSIONS_TCG = " << value << "\n";
         return value;
      } else {
         std::cout << "Bad value for MAX_LABEL_EXPANSIONS_TCG: " << getenv(MAX_LABEL_EXPANSIONS_TCG) << std::endl;
         exit(EXIT_FAILURE);
      }
   }
   return std::numeric_limits<int>::max();
}

inline auto getEnvSortDeadheadArcs() noexcept -> bool {
   if (getenv(SORT_DEADHEAD_ARCS)) {
      bool value = std::stoi(getenv(SORT_DEADHEAD_ARCS)) == 0 ? false : true;
      std::cout << "Read SORT_DEADHEAD_ARCS = " << value << "\n";
      return value;      
   }
   return false;
}

inline auto getEnvMaxTcgSubIter() noexcept -> int {
   if (getenv(TCG_MAX_SUB_ITERATIONS)) {
      int value = std::stoi(getenv(TCG_MAX_SUB_ITERATIONS));
      if (value > 0) {
         std::cout << "Read TCG_MAX_SUB_ITERATIONS = " << value << "\n";
      } else {
         std::cout << "Bad value for TCG_MAX_SUB_ITERATIONS: " << getenv(TCG_MAX_SUB_ITERATIONS) << std::endl;
         exit(EXIT_FAILURE);
      }
      return value;
   }
   return 20;
}

inline auto getEnvTcgVarSelection() noexcept -> int {
   const auto rawValue = getenv(TCG_VAR_SEL);
   if (!rawValue)
      return TCG_VAR_SEL_SIMPLE;
   std::cout << "Read TCG_VAR_SEL = " << rawValue << "\n";
   if (strcmp("simple", rawValue) == 0)
      return TCG_VAR_SEL_SIMPLE;
   if (strcmp("grasp", rawValue) == 0)
      return TCG_VAR_SEL_GRASP;
   
   std::cout << "Bad value for TCG_VAR_SEL: " << rawValue << std::endl;
   exit(EXIT_FAILURE);

   return -1;
}

inline auto getEnvTcgGraspStrategy() noexcept -> int {
   const auto rawValue = getenv(TCG_GRASP_STRATEGY);
   if (!rawValue)
      return TCG_GRASP_STRATEGY_DIRECT;
   std::cout << "Read TCG_GRASP_STRATEGY = " << rawValue << "\n";
   if (strcmp("direct", rawValue) == 0)
      return TCG_GRASP_STRATEGY_DIRECT;
   if (strcmp("eval", rawValue) == 0)
      return TCG_GRASP_STRATEGY_EVAL;
   
   std::cout << "Bad value for TCG_GRASP_STRATEGY: " << rawValue << std::endl;
   exit(EXIT_FAILURE);

   return -1;
}

inline auto getEnvTcgGraspAlpha() noexcept -> double {
   const auto rawValue = getenv(TCG_GRASP_ALPHA);
   if (!rawValue)
      return 0.2;
   const auto value = std::stod(rawValue);
   std::cout << "Read TCG_GRASP_ALPHA = " << value << "\n";
   return value;
}