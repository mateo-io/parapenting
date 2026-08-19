cmake_minimum_required(VERSION 3.20)
project(ParapentingPhysicsTests LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Every suite here is written with assert(). Release defines NDEBUG, which
# compiles all of them away and leaves the tests passing by doing nothing -
# 605 checks in PhysicsTests alone were inert. Keep them live in every build
# type; these are test binaries, not shipping code.
add_compile_options(-UNDEBUG)

# Default to Release when the caller does not choose. The solver's calibrated
# bounds were measured on an optimised build, and some of them are genuinely
# rounding-sensitive: the deep symmetric frontal in CoupledTests peaks at
# L 0.710 / R 0.710 and 2.06 rad/s optimised, and at L 0.710 / R 0.947 and
# 5.40 rad/s unoptimised, which trips a bound that is not actually broken.
# A bare `cmake -S Tests -B build` used to produce that second build and a
# confusing failure, so the default is pinned here rather than left to the
# caller to remember.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

add_executable(parapenting_physics_tests
    PhysicsTests.cpp
    ../Source/Parapenting/Physics/ParagliderDynamics.cpp
    ../Source/Parapenting/Physics/PayloadRigidBody.cpp
    ../Source/Parapenting/Physics/HarnessGeometry.cpp
    ../Source/Parapenting/Physics/AtmosphereModel.cpp
    ../Source/Parapenting/Physics/DiurnalCycle.cpp
    ../Source/Parapenting/Physics/GroundEffectModel.cpp
    ../Source/Parapenting/Physics/LandingRolloutModel.cpp
    ../Source/Parapenting/Physics/TerrainModel.cpp
    ../Source/Parapenting/Physics/WingCatalogue.cpp
    ../Source/Parapenting/Physics/EquipmentSetup.cpp
    ../Source/Parapenting/Physics/RouteCatalogue.cpp
    ../Source/Parapenting/Physics/SpanwiseCanopyModel.cpp
    ../Source/Parapenting/Physics/TrainingScenario.cpp
    ../Source/Parapenting/Physics/ChallengeEvaluator.cpp
    ../Source/Parapenting/Physics/WeatherSnapshot.cpp
    ../Source/Parapenting/Physics/HeightfieldGrid.cpp
    ../Source/Parapenting/Physics/HapticFeedback.cpp
    ../Source/Parapenting/Physics/PilotProgression.cpp
    ../Source/Parapenting/Physics/AccessibilityProfile.cpp
    ../Source/Parapenting/Physics/GraphicsProfile.cpp
    ../Source/Parapenting/Physics/AerodynamicPolar.cpp
    ../Source/Parapenting/Physics/CameraFeedback.cpp
    ../Source/Parapenting/Physics/AudioFeedback.cpp
    ../Source/Parapenting/Physics/PilotPose.cpp
    ../Source/Parapenting/Physics/PilotSkeletonAim.cpp
    ../Source/Parapenting/Physics/GliderRigSnapshot.cpp
    ../Source/Parapenting/Physics/CanopyLoadPose.cpp
    ../Source/Parapenting/Physics/SuspensionSystem.cpp
    ../Source/Parapenting/Physics/ResearchManeuver.cpp
    ../Source/Parapenting/Physics/InputBindingProfile.cpp
    ../Source/Parapenting/Physics/WindsockModel.cpp
    ../Source/Parapenting/Physics/LandingCircuitModel.cpp
    ../Source/Parapenting/Physics/GroundLaunchModel.cpp
    ../Source/Parapenting/Physics/FlightDebrief.cpp
    ../Source/Parapenting/Physics/PreflightBriefing.cpp
    ../Source/Parapenting/Physics/FlightNavigation.cpp
)
target_include_directories(parapenting_physics_tests PRIVATE
    ../Source/Parapenting/Physics
)
target_compile_options(parapenting_physics_tests PRIVATE -Wall -Wextra -Wpedantic)

add_executable(parapenting_flight_probe
    FlightProbe.cpp
    ../Source/Parapenting/Physics/ParagliderDynamics.cpp
    ../Source/Parapenting/Physics/PayloadRigidBody.cpp
    ../Source/Parapenting/Physics/HarnessGeometry.cpp
    ../Source/Parapenting/Physics/AtmosphereModel.cpp
    ../Source/Parapenting/Physics/DiurnalCycle.cpp
    ../Source/Parapenting/Physics/GroundEffectModel.cpp
    ../Source/Parapenting/Physics/LandingRolloutModel.cpp
    ../Source/Parapenting/Physics/TerrainModel.cpp
    ../Source/Parapenting/Physics/WingCatalogue.cpp
    ../Source/Parapenting/Physics/EquipmentSetup.cpp
    ../Source/Parapenting/Physics/RouteCatalogue.cpp
    ../Source/Parapenting/Physics/SpanwiseCanopyModel.cpp
    ../Source/Parapenting/Physics/TrainingScenario.cpp
    ../Source/Parapenting/Physics/ChallengeEvaluator.cpp
    ../Source/Parapenting/Physics/WeatherSnapshot.cpp
    ../Source/Parapenting/Physics/HeightfieldGrid.cpp
    ../Source/Parapenting/Physics/AerodynamicPolar.cpp
    ../Source/Parapenting/Physics/SuspensionSystem.cpp
)
target_include_directories(parapenting_flight_probe PRIVATE
    ../Source/Parapenting/Physics
)
target_compile_options(parapenting_flight_probe PRIVATE -Wall -Wextra -Wpedantic)

# Performance plan L4. The atmosphere sample, which no profile ever
# contained and which the wind and lift work grows. Asserts nothing and is
# not part of check-build.sh, like the solver profile beside it.
add_executable(parapenting_air_profile
    AirProfile.cpp
    ../Source/Parapenting/Physics/ParagliderDynamics.cpp
    ../Source/Parapenting/Physics/PayloadRigidBody.cpp
    ../Source/Parapenting/Physics/HarnessGeometry.cpp
    ../Source/Parapenting/Physics/AtmosphereModel.cpp
    ../Source/Parapenting/Physics/DiurnalCycle.cpp
    ../Source/Parapenting/Physics/GroundEffectModel.cpp
    ../Source/Parapenting/Physics/LandingRolloutModel.cpp
    ../Source/Parapenting/Physics/TerrainModel.cpp
    ../Source/Parapenting/Physics/WingCatalogue.cpp
    ../Source/Parapenting/Physics/EquipmentSetup.cpp
    ../Source/Parapenting/Physics/RouteCatalogue.cpp
    ../Source/Parapenting/Physics/SpanwiseCanopyModel.cpp
    ../Source/Parapenting/Physics/TrainingScenario.cpp
    ../Source/Parapenting/Physics/ChallengeEvaluator.cpp
    ../Source/Parapenting/Physics/WeatherSnapshot.cpp
    ../Source/Parapenting/Physics/HeightfieldGrid.cpp
    ../Source/Parapenting/Physics/AerodynamicPolar.cpp
    ../Source/Parapenting/Physics/SuspensionSystem.cpp
)
target_include_directories(parapenting_air_profile PRIVATE
    ../Source/Parapenting/Physics
)
target_compile_options(parapenting_air_profile PRIVATE -Wall -Wextra -Wpedantic)

add_executable(parapenting_regression_matrix
    RegressionMatrix.cpp
    ../Source/Parapenting/Physics/ParagliderDynamics.cpp
    ../Source/Parapenting/Physics/PayloadRigidBody.cpp
    ../Source/Parapenting/Physics/HarnessGeometry.cpp
    ../Source/Parapenting/Physics/AtmosphereModel.cpp
    ../Source/Parapenting/Physics/DiurnalCycle.cpp
    ../Source/Parapenting/Physics/GroundEffectModel.cpp
    ../Source/Parapenting/Physics/LandingRolloutModel.cpp
    ../Source/Parapenting/Physics/TerrainModel.cpp
    ../Source/Parapenting/Physics/WingCatalogue.cpp
    ../Source/Parapenting/Physics/EquipmentSetup.cpp
    ../Source/Parapenting/Physics/SpanwiseCanopyModel.cpp
    ../Source/Parapenting/Physics/HeightfieldGrid.cpp
    ../Source/Parapenting/Physics/WeatherSnapshot.cpp
    ../Source/Parapenting/Physics/AerodynamicPolar.cpp
    ../Source/Parapenting/Physics/SuspensionSystem.cpp
)
target_include_directories(parapenting_regression_matrix PRIVATE
    ../Source/Parapenting/Physics
)
target_compile_options(parapenting_regression_matrix PRIVATE
    -Wall -Wextra -Wpedantic
)

add_executable(parapenting_geometry_tests
    GeometryTests.cpp
    ../Source/Parapenting/Physics/CanopyLoadPose.cpp
    ../Source/Parapenting/Physics/CanopyGeometry.cpp
    ../Source/Parapenting/Physics/SectionProfile.cpp
    ../Source/Parapenting/Physics/BillowRelaxation.cpp
    ../Source/Parapenting/Physics/PanelUnfolder.cpp
    ../Source/Parapenting/Physics/SuspensionSystem.cpp
)
target_include_directories(parapenting_geometry_tests PRIVATE
    ../Source/Parapenting/Physics
)
target_compile_options(parapenting_geometry_tests PRIVATE
    -Wall -Wextra -Wpedantic
)

add_executable(parapenting_suspension_tests
    SuspensionTests.cpp
    ../Source/Parapenting/Physics/CoupledParagliderSolver.cpp
    ../Source/Parapenting/Physics/VortexStepMethodSolver.cpp
    ../Source/Parapenting/Physics/SectionPolarTable.cpp
    ../Source/Parapenting/Physics/SectionPolarCache.cpp
    ../Source/Parapenting/Physics/SectionViscousSolver.cpp
    ../Source/Parapenting/Physics/ApparentMassTensor.cpp
    ../Source/Parapenting/Physics/CanopyPressureSolver.cpp
    ../Source/Parapenting/Physics/CanopyMembraneSolver.cpp
    ../Source/Parapenting/Physics/CanopyCollapseSolver.cpp
    ../Source/Parapenting/Physics/SuspensionSystem.cpp
    ../Source/Parapenting/Physics/GroundEffectModel.cpp
    ../Source/Parapenting/Physics/CanopyGeometry.cpp
    ../Source/Parapenting/Physics/SectionProfile.cpp
    ../Source/Parapenting/Physics/BillowRelaxation.cpp
    ../Source/Parapenting/Physics/SuspensionGraph.cpp
    ../Source/Parapenting/Physics/HarnessGeometry.cpp
    ../Source/Parapenting/Physics/TensionCableSolver.cpp
    ../Source/Parapenting/Physics/ParagliderDynamics.cpp
    ../Source/Parapenting/Physics/PayloadRigidBody.cpp
    ../Source/Parapenting/Physics/HarnessGeometry.cpp
    ../Source/Parapenting/Physics/AerodynamicPolar.cpp
    ../Source/Parapenting/Physics/SpanwiseCanopyModel.cpp
    ../Source/Parapenting/Physics/SuspensionSystem.cpp
    ../Source/Parapenting/Physics/GroundEffectModel.cpp
    ../Source/Parapenting/Physics/WingCatalogue.cpp
)
target_include_directories(parapenting_suspension_tests PRIVATE
    ../Source/Parapenting/Physics
)
target_compile_options(parapenting_suspension_tests PRIVATE
    -Wall -Wextra -Wpedantic
)

add_executable(parapenting_payload_tests
    PayloadTests.cpp
    ../Source/Parapenting/Physics/PayloadRigidBody.cpp
    ../Source/Parapenting/Physics/HarnessGeometry.cpp
    ../Source/Parapenting/Physics/EquipmentSetup.cpp
    ../Source/Parapenting/Physics/ParagliderDynamics.cpp
    ../Source/Parapenting/Physics/AerodynamicPolar.cpp
    ../Source/Parapenting/Physics/SpanwiseCanopyModel.cpp
    ../Source/Parapenting/Physics/SuspensionSystem.cpp
    ../Source/Parapenting/Physics/GroundEffectModel.cpp
    ../Source/Parapenting/Physics/WingCatalogue.cpp
)
target_include_directories(parapenting_payload_tests PRIVATE
    ../Source/Parapenting/Physics
)
target_compile_options(parapenting_payload_tests PRIVATE
    -Wall -Wextra -Wpedantic
)

add_executable(parapenting_aerodynamics_tests
    AerodynamicsTests.cpp
    ../Source/Parapenting/Physics/SectionPolarTable.cpp
    ../Source/Parapenting/Physics/SectionPolarCache.cpp
    ../Source/Parapenting/Physics/SectionProfile.cpp
    ../Source/Parapenting/Physics/SectionViscousSolver.cpp
    ../Source/Parapenting/Physics/VortexStepMethodSolver.cpp
    ../Source/Parapenting/Physics/ApparentMassTensor.cpp
    ../Source/Parapenting/Physics/CanopyGeometry.cpp
    ../Source/Parapenting/Physics/SectionProfile.cpp
    ../Source/Parapenting/Physics/BillowRelaxation.cpp
    ../Source/Parapenting/Physics/ParagliderDynamics.cpp
    ../Source/Parapenting/Physics/PayloadRigidBody.cpp
    ../Source/Parapenting/Physics/HarnessGeometry.cpp
    ../Source/Parapenting/Physics/AerodynamicPolar.cpp
    ../Source/Parapenting/Physics/SpanwiseCanopyModel.cpp
    ../Source/Parapenting/Physics/SuspensionSystem.cpp
    ../Source/Parapenting/Physics/GroundEffectModel.cpp
)
target_include_directories(parapenting_aerodynamics_tests PRIVATE
    ../Source/Parapenting/Physics
)
target_compile_options(parapenting_aerodynamics_tests PRIVATE
    -Wall -Wextra -Wpedantic
)

add_executable(parapenting_pressure_tests
    PressureTests.cpp
    ../Source/Parapenting/Physics/CanopyPressureSolver.cpp
    ../Source/Parapenting/Physics/SectionPolarTable.cpp
    ../Source/Parapenting/Physics/SectionPolarCache.cpp
    ../Source/Parapenting/Physics/SectionProfile.cpp
    ../Source/Parapenting/Physics/SectionViscousSolver.cpp
    ../Source/Parapenting/Physics/VortexStepMethodSolver.cpp
    ../Source/Parapenting/Physics/CanopyGeometry.cpp
    ../Source/Parapenting/Physics/SectionProfile.cpp
    ../Source/Parapenting/Physics/BillowRelaxation.cpp
    ../Source/Parapenting/Physics/ParagliderDynamics.cpp
    ../Source/Parapenting/Physics/PayloadRigidBody.cpp
    ../Source/Parapenting/Physics/HarnessGeometry.cpp
    ../Source/Parapenting/Physics/AerodynamicPolar.cpp
    ../Source/Parapenting/Physics/SpanwiseCanopyModel.cpp
    ../Source/Parapenting/Physics/SuspensionSystem.cpp
    ../Source/Parapenting/Physics/GroundEffectModel.cpp
)
target_include_directories(parapenting_pressure_tests PRIVATE
    ../Source/Parapenting/Physics
)
target_compile_options(parapenting_pressure_tests PRIVATE
    -Wall -Wextra -Wpedantic
)

add_executable(parapenting_membrane_tests
    MembraneTests.cpp
    ../Source/Parapenting/Physics/CanopyMembraneSolver.cpp
    ../Source/Parapenting/Physics/ParagliderDynamics.cpp
    ../Source/Parapenting/Physics/PayloadRigidBody.cpp
    ../Source/Parapenting/Physics/HarnessGeometry.cpp
    ../Source/Parapenting/Physics/AerodynamicPolar.cpp
    ../Source/Parapenting/Physics/SpanwiseCanopyModel.cpp
    ../Source/Parapenting/Physics/SuspensionSystem.cpp
    ../Source/Parapenting/Physics/GroundEffectModel.cpp
)
target_include_directories(parapenting_membrane_tests PRIVATE
    ../Source/Parapenting/Physics
)
target_compile_options(parapenting_membrane_tests PRIVATE
    -Wall -Wextra -Wpedantic
)

add_executable(parapenting_calibration_tests
    CalibrationTests.cpp
    ../Source/Parapenting/Physics/CalibrationManeuver.cpp
    ../Source/Parapenting/Physics/CoupledParagliderSolver.cpp
    ../Source/Parapenting/Physics/VortexStepMethodSolver.cpp
    ../Source/Parapenting/Physics/SectionPolarTable.cpp
    ../Source/Parapenting/Physics/SectionPolarCache.cpp
    ../Source/Parapenting/Physics/SectionProfile.cpp
    ../Source/Parapenting/Physics/SectionViscousSolver.cpp
    ../Source/Parapenting/Physics/ApparentMassTensor.cpp
    ../Source/Parapenting/Physics/CanopyPressureSolver.cpp
    ../Source/Parapenting/Physics/CanopyMembraneSolver.cpp
    ../Source/Parapenting/Physics/CanopyCollapseSolver.cpp
    ../Source/Parapenting/Physics/TensionCableSolver.cpp
    ../Source/Parapenting/Physics/SuspensionGraph.cpp
    ../Source/Parapenting/Physics/CanopyGeometry.cpp
    ../Source/Parapenting/Physics/SectionProfile.cpp
    ../Source/Parapenting/Physics/BillowRelaxation.cpp
    ../Source/Parapenting/Physics/PayloadRigidBody.cpp
    ../Source/Parapenting/Physics/HarnessGeometry.cpp
    ../Source/Parapenting/Physics/ParagliderDynamics.cpp
    ../Source/Parapenting/Physics/AerodynamicPolar.cpp
    ../Source/Parapenting/Physics/SpanwiseCanopyModel.cpp
    ../Source/Parapenting/Physics/SuspensionSystem.cpp
    ../Source/Parapenting/Physics/GroundEffectModel.cpp
)
target_include_directories(parapenting_calibration_tests PRIVATE
    ../Source/Parapenting/Physics
)
target_compile_options(parapenting_calibration_tests PRIVATE
    -Wall -Wextra -Wpedantic
)

add_executable(parapenting_calibration_settled
    CalibrationSettled.cpp
    ../Source/Parapenting/Physics/CalibrationManeuver.cpp
    ../Source/Parapenting/Physics/CoupledParagliderSolver.cpp
    ../Source/Parapenting/Physics/VortexStepMethodSolver.cpp
    ../Source/Parapenting/Physics/SectionPolarTable.cpp
    ../Source/Parapenting/Physics/SectionPolarCache.cpp
    ../Source/Parapenting/Physics/SectionProfile.cpp
    ../Source/Parapenting/Physics/SectionViscousSolver.cpp
    ../Source/Parapenting/Physics/ApparentMassTensor.cpp
    ../Source/Parapenting/Physics/CanopyPressureSolver.cpp
    ../Source/Parapenting/Physics/CanopyMembraneSolver.cpp
    ../Source/Parapenting/Physics/CanopyCollapseSolver.cpp
    ../Source/Parapenting/Physics/TensionCableSolver.cpp
    ../Source/Parapenting/Physics/SuspensionGraph.cpp
    ../Source/Parapenting/Physics/CanopyGeometry.cpp
    ../Source/Parapenting/Physics/SectionProfile.cpp
    ../Source/Parapenting/Physics/BillowRelaxation.cpp
    ../Source/Parapenting/Physics/PayloadRigidBody.cpp
    ../Source/Parapenting/Physics/HarnessGeometry.cpp
    ../Source/Parapenting/Physics/ParagliderDynamics.cpp
    ../Source/Parapenting/Physics/AerodynamicPolar.cpp
    ../Source/Parapenting/Physics/SpanwiseCanopyModel.cpp
    ../Source/Parapenting/Physics/SuspensionSystem.cpp
    ../Source/Parapenting/Physics/GroundEffectModel.cpp
)
target_include_directories(parapenting_calibration_settled PRIVATE
    ../Source/Parapenting/Physics
)
target_compile_options(parapenting_calibration_settled PRIVATE
    -Wall -Wextra -Wpedantic
)

add_executable(parapenting_coupled_tests
    CoupledTests.cpp
    ../Source/Parapenting/Physics/CoupledParagliderSolver.cpp
    ../Source/Parapenting/Physics/VortexStepMethodSolver.cpp
    ../Source/Parapenting/Physics/SectionPolarTable.cpp
    ../Source/Parapenting/Physics/SectionPolarCache.cpp
    ../Source/Parapenting/Physics/SectionProfile.cpp
    ../Source/Parapenting/Physics/SectionViscousSolver.cpp
    ../Source/Parapenting/Physics/ApparentMassTensor.cpp
    ../Source/Parapenting/Physics/CanopyPressureSolver.cpp
    ../Source/Parapenting/Physics/CanopyMembraneSolver.cpp
    ../Source/Parapenting/Physics/CanopyCollapseSolver.cpp
    ../Source/Parapenting/Physics/TensionCableSolver.cpp
    ../Source/Parapenting/Physics/SuspensionGraph.cpp
    ../Source/Parapenting/Physics/CanopyGeometry.cpp
    ../Source/Parapenting/Physics/SectionProfile.cpp
    ../Source/Parapenting/Physics/BillowRelaxation.cpp
    ../Source/Parapenting/Physics/PayloadRigidBody.cpp
    ../Source/Parapenting/Physics/HarnessGeometry.cpp
    ../Source/Parapenting/Physics/ParagliderDynamics.cpp
    ../Source/Parapenting/Physics/AerodynamicPolar.cpp
    ../Source/Parapenting/Physics/SpanwiseCanopyModel.cpp
    ../Source/Parapenting/Physics/SuspensionSystem.cpp
    ../Source/Parapenting/Physics/GroundEffectModel.cpp
)
target_include_directories(parapenting_coupled_tests PRIVATE
    ../Source/Parapenting/Physics
)
target_compile_options(parapenting_coupled_tests PRIVATE
    -Wall -Wextra -Wpedantic
)

add_executable(parapenting_solver_profile
    SolverProfile.cpp
    ../Source/Parapenting/Physics/CoupledParagliderSolver.cpp
    ../Source/Parapenting/Physics/VortexStepMethodSolver.cpp
    ../Source/Parapenting/Physics/SectionPolarTable.cpp
    ../Source/Parapenting/Physics/SectionPolarCache.cpp
    ../Source/Parapenting/Physics/SectionProfile.cpp
    ../Source/Parapenting/Physics/SectionViscousSolver.cpp
    ../Source/Parapenting/Physics/ApparentMassTensor.cpp
    ../Source/Parapenting/Physics/CanopyPressureSolver.cpp
    ../Source/Parapenting/Physics/CanopyMembraneSolver.cpp
    ../Source/Parapenting/Physics/CanopyCollapseSolver.cpp
    ../Source/Parapenting/Physics/TensionCableSolver.cpp
    ../Source/Parapenting/Physics/SuspensionGraph.cpp
    ../Source/Parapenting/Physics/CanopyGeometry.cpp
    ../Source/Parapenting/Physics/BillowRelaxation.cpp
    ../Source/Parapenting/Physics/PayloadRigidBody.cpp
    ../Source/Parapenting/Physics/HarnessGeometry.cpp
    ../Source/Parapenting/Physics/ParagliderDynamics.cpp
    ../Source/Parapenting/Physics/AerodynamicPolar.cpp
    ../Source/Parapenting/Physics/SpanwiseCanopyModel.cpp
    ../Source/Parapenting/Physics/SuspensionSystem.cpp
    ../Source/Parapenting/Physics/GroundEffectModel.cpp
)
target_include_directories(parapenting_solver_profile PRIVATE
    ../Source/Parapenting/Physics
)
target_compile_options(parapenting_solver_profile PRIVATE
    -Wall -Wextra -Wpedantic
)

add_executable(parapenting_solver_lod
    SolverLod.cpp
    ../Source/Parapenting/Physics/CoupledParagliderSolver.cpp
    ../Source/Parapenting/Physics/VortexStepMethodSolver.cpp
    ../Source/Parapenting/Physics/SectionPolarTable.cpp
    ../Source/Parapenting/Physics/SectionPolarCache.cpp
    ../Source/Parapenting/Physics/SectionProfile.cpp
    ../Source/Parapenting/Physics/SectionViscousSolver.cpp
    ../Source/Parapenting/Physics/ApparentMassTensor.cpp
    ../Source/Parapenting/Physics/CanopyPressureSolver.cpp
    ../Source/Parapenting/Physics/CanopyMembraneSolver.cpp
    ../Source/Parapenting/Physics/CanopyCollapseSolver.cpp
    ../Source/Parapenting/Physics/TensionCableSolver.cpp
    ../Source/Parapenting/Physics/SuspensionGraph.cpp
    ../Source/Parapenting/Physics/CanopyGeometry.cpp
    ../Source/Parapenting/Physics/BillowRelaxation.cpp
    ../Source/Parapenting/Physics/PayloadRigidBody.cpp
    ../Source/Parapenting/Physics/HarnessGeometry.cpp
    ../Source/Parapenting/Physics/ParagliderDynamics.cpp
    ../Source/Parapenting/Physics/AerodynamicPolar.cpp
    ../Source/Parapenting/Physics/SpanwiseCanopyModel.cpp
    ../Source/Parapenting/Physics/SuspensionSystem.cpp
    ../Source/Parapenting/Physics/GroundEffectModel.cpp
)
target_include_directories(parapenting_solver_lod PRIVATE
    ../Source/Parapenting/Physics
)
target_compile_options(parapenting_solver_lod PRIVATE
    -Wall -Wextra -Wpedantic
)

add_executable(parapenting_model_agreement
    ModelAgreement.cpp
    ../Source/Parapenting/Physics/CoupledParagliderSolver.cpp
    ../Source/Parapenting/Physics/CalibrationManeuver.cpp
    ../Source/Parapenting/Physics/VortexStepMethodSolver.cpp
    ../Source/Parapenting/Physics/SectionPolarTable.cpp
    ../Source/Parapenting/Physics/SectionPolarCache.cpp
    ../Source/Parapenting/Physics/SectionProfile.cpp
    ../Source/Parapenting/Physics/SectionViscousSolver.cpp
    ../Source/Parapenting/Physics/ApparentMassTensor.cpp
    ../Source/Parapenting/Physics/CanopyPressureSolver.cpp
    ../Source/Parapenting/Physics/CanopyMembraneSolver.cpp
    ../Source/Parapenting/Physics/CanopyCollapseSolver.cpp
    ../Source/Parapenting/Physics/TensionCableSolver.cpp
    ../Source/Parapenting/Physics/SuspensionGraph.cpp
    ../Source/Parapenting/Physics/CanopyGeometry.cpp
    ../Source/Parapenting/Physics/BillowRelaxation.cpp
    ../Source/Parapenting/Physics/PayloadRigidBody.cpp
    ../Source/Parapenting/Physics/HarnessGeometry.cpp
    ../Source/Parapenting/Physics/ParagliderDynamics.cpp
    ../Source/Parapenting/Physics/AerodynamicPolar.cpp
    ../Source/Parapenting/Physics/SpanwiseCanopyModel.cpp
    ../Source/Parapenting/Physics/SuspensionSystem.cpp
    ../Source/Parapenting/Physics/GroundEffectModel.cpp
    ../Source/Parapenting/Physics/EquipmentSetup.cpp
    ../Source/Parapenting/Physics/WingCatalogue.cpp
)
target_include_directories(parapenting_model_agreement PRIVATE
    ../Source/Parapenting/Physics
)
target_compile_options(parapenting_model_agreement PRIVATE
    -Wall -Wextra -Wpedantic
)

add_executable(parapenting_pitch_axis_trace
    PitchAxisTrace.cpp
    ../Source/Parapenting/Physics/CoupledParagliderSolver.cpp
    ../Source/Parapenting/Physics/VortexStepMethodSolver.cpp
    ../Source/Parapenting/Physics/SectionPolarTable.cpp
    ../Source/Parapenting/Physics/SectionPolarCache.cpp
    ../Source/Parapenting/Physics/SectionProfile.cpp
    ../Source/Parapenting/Physics/SectionViscousSolver.cpp
    ../Source/Parapenting/Physics/ApparentMassTensor.cpp
    ../Source/Parapenting/Physics/CanopyPressureSolver.cpp
    ../Source/Parapenting/Physics/CanopyMembraneSolver.cpp
    ../Source/Parapenting/Physics/CanopyCollapseSolver.cpp
    ../Source/Parapenting/Physics/TensionCableSolver.cpp
    ../Source/Parapenting/Physics/SuspensionGraph.cpp
    ../Source/Parapenting/Physics/CanopyGeometry.cpp
    ../Source/Parapenting/Physics/BillowRelaxation.cpp
    ../Source/Parapenting/Physics/PayloadRigidBody.cpp
    ../Source/Parapenting/Physics/HarnessGeometry.cpp
    ../Source/Parapenting/Physics/ParagliderDynamics.cpp
    ../Source/Parapenting/Physics/AerodynamicPolar.cpp
    ../Source/Parapenting/Physics/SpanwiseCanopyModel.cpp
    ../Source/Parapenting/Physics/SuspensionSystem.cpp
    ../Source/Parapenting/Physics/GroundEffectModel.cpp
)
target_include_directories(parapenting_pitch_axis_trace PRIVATE
    ../Source/Parapenting/Physics
)
target_compile_options(parapenting_pitch_axis_trace PRIVATE
    -Wall -Wextra -Wpedantic
)

# The same solver, asked for its modes rather than for a trace of them.
add_executable(parapenting_pitch_eigenmodes
    PitchEigenmodes.cpp
    ../Source/Parapenting/Physics/CoupledParagliderSolver.cpp
    ../Source/Parapenting/Physics/VortexStepMethodSolver.cpp
    ../Source/Parapenting/Physics/SectionPolarTable.cpp
    ../Source/Parapenting/Physics/SectionPolarCache.cpp
    ../Source/Parapenting/Physics/SectionProfile.cpp
    ../Source/Parapenting/Physics/SectionViscousSolver.cpp
    ../Source/Parapenting/Physics/ApparentMassTensor.cpp
    ../Source/Parapenting/Physics/CanopyPressureSolver.cpp
    ../Source/Parapenting/Physics/CanopyMembraneSolver.cpp
    ../Source/Parapenting/Physics/CanopyCollapseSolver.cpp
    ../Source/Parapenting/Physics/TensionCableSolver.cpp
    ../Source/Parapenting/Physics/SuspensionGraph.cpp
    ../Source/Parapenting/Physics/CanopyGeometry.cpp
    ../Source/Parapenting/Physics/BillowRelaxation.cpp
    ../Source/Parapenting/Physics/PayloadRigidBody.cpp
    ../Source/Parapenting/Physics/HarnessGeometry.cpp
    ../Source/Parapenting/Physics/ParagliderDynamics.cpp
    ../Source/Parapenting/Physics/AerodynamicPolar.cpp
    ../Source/Parapenting/Physics/SpanwiseCanopyModel.cpp
    ../Source/Parapenting/Physics/SuspensionSystem.cpp
    ../Source/Parapenting/Physics/GroundEffectModel.cpp
)
target_include_directories(parapenting_pitch_eigenmodes PRIVATE
    ../Source/Parapenting/Physics
)
target_compile_options(parapenting_pitch_eigenmodes PRIVATE
    -Wall -Wextra -Wpedantic
)

add_executable(parapenting_determinism_tests
    DeterminismTests.cpp
    ../Source/Parapenting/Physics/SuspensionGraph.cpp
    ../Source/Parapenting/Physics/HarnessGeometry.cpp
    ../Source/Parapenting/Physics/TensionCableSolver.cpp
    ../Source/Parapenting/Physics/ParagliderSolverClock.cpp
    ../Source/Parapenting/Physics/ResearchCoefficientRegistry.cpp
    ../Source/Parapenting/Physics/CanopyGeometry.cpp
    ../Source/Parapenting/Physics/SectionProfile.cpp
    ../Source/Parapenting/Physics/BillowRelaxation.cpp
    ../Source/Parapenting/Physics/SolverStateHash.cpp
    ../Source/Parapenting/Physics/ParagliderDynamics.cpp
    ../Source/Parapenting/Physics/PayloadRigidBody.cpp
    ../Source/Parapenting/Physics/HarnessGeometry.cpp
    ../Source/Parapenting/Physics/AerodynamicPolar.cpp
    ../Source/Parapenting/Physics/SpanwiseCanopyModel.cpp
    ../Source/Parapenting/Physics/SuspensionSystem.cpp
    ../Source/Parapenting/Physics/GroundEffectModel.cpp
    ../Source/Parapenting/Physics/WingCatalogue.cpp
)
target_include_directories(parapenting_determinism_tests PRIVATE
    ../Source/Parapenting/Physics
)
target_compile_options(parapenting_determinism_tests PRIVATE
    -Wall -Wextra -Wpedantic
)

add_executable(parapenting_terrain_survey_tests
    TerrainSurveyTests.cpp
    ../Source/Parapenting/Physics/TerrainModel.cpp
    ../Source/Parapenting/Physics/HeightfieldGrid.cpp
    ../Source/Parapenting/Physics/RouteCatalogue.cpp
    ../Source/Parapenting/Physics/ParagliderDynamics.cpp
    ../Source/Parapenting/Physics/PayloadRigidBody.cpp
    ../Source/Parapenting/Physics/HarnessGeometry.cpp
    ../Source/Parapenting/Physics/AerodynamicPolar.cpp
    ../Source/Parapenting/Physics/SpanwiseCanopyModel.cpp
    ../Source/Parapenting/Physics/SuspensionSystem.cpp
    ../Source/Parapenting/Physics/WingCatalogue.cpp
    ../Source/Parapenting/Physics/GroundEffectModel.cpp
)
target_include_directories(parapenting_terrain_survey_tests PRIVATE
    ../Source/Parapenting/Physics
)
target_compile_options(parapenting_terrain_survey_tests PRIVATE
    -Wall -Wextra -Wpedantic
)

add_executable(parapenting_still_air_probe
    StillAirProbe.cpp
    ../Source/Parapenting/Physics/ParagliderDynamics.cpp
    ../Source/Parapenting/Physics/PayloadRigidBody.cpp
    ../Source/Parapenting/Physics/HarnessGeometry.cpp
    ../Source/Parapenting/Physics/AerodynamicPolar.cpp
    ../Source/Parapenting/Physics/SpanwiseCanopyModel.cpp
    ../Source/Parapenting/Physics/SuspensionSystem.cpp
    ../Source/Parapenting/Physics/GroundEffectModel.cpp
    ../Source/Parapenting/Physics/WingCatalogue.cpp
)
target_include_directories(parapenting_still_air_probe PRIVATE
    ../Source/Parapenting/Physics
)
target_compile_options(parapenting_still_air_probe PRIVATE
    -Wall -Wextra -Wpedantic
)

add_executable(parapenting_collapse_tests
    CollapseTests.cpp
    ../Source/Parapenting/Physics/CanopyCollapseSolver.cpp
    ../Source/Parapenting/Physics/CanopyPressureSolver.cpp
    ../Source/Parapenting/Physics/CanopyGeometry.cpp
    ../Source/Parapenting/Physics/SectionProfile.cpp
    ../Source/Parapenting/Physics/BillowRelaxation.cpp
    ../Source/Parapenting/Physics/SuspensionGraph.cpp
    ../Source/Parapenting/Physics/HarnessGeometry.cpp
    ../Source/Parapenting/Physics/TensionCableSolver.cpp
    ../Source/Parapenting/Physics/ParagliderDynamics.cpp
    ../Source/Parapenting/Physics/AerodynamicPolar.cpp
    ../Source/Parapenting/Physics/SpanwiseCanopyModel.cpp
    ../Source/Parapenting/Physics/SuspensionSystem.cpp
    ../Source/Parapenting/Physics/GroundEffectModel.cpp
    ../Source/Parapenting/Physics/PayloadRigidBody.cpp
)
target_include_directories(parapenting_collapse_tests PRIVATE
    ../Source/Parapenting/Physics
)
target_compile_options(parapenting_collapse_tests PRIVATE
    -Wall -Wextra -Wpedantic
)
