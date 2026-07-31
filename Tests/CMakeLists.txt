cmake_minimum_required(VERSION 3.20)
project(ParapentingPhysicsTests LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(parapenting_physics_tests
    PhysicsTests.cpp
    ../Source/Parapenting/Physics/ParagliderDynamics.cpp
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

add_executable(parapenting_regression_matrix
    RegressionMatrix.cpp
    ../Source/Parapenting/Physics/ParagliderDynamics.cpp
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
    ../Source/Parapenting/Physics/CanopyGeometry.cpp
    ../Source/Parapenting/Physics/PanelUnfolder.cpp
)
target_include_directories(parapenting_geometry_tests PRIVATE
    ../Source/Parapenting/Physics
)
target_compile_options(parapenting_geometry_tests PRIVATE
    -Wall -Wextra -Wpedantic
)

add_executable(parapenting_determinism_tests
    DeterminismTests.cpp
    ../Source/Parapenting/Physics/ParagliderSolverClock.cpp
    ../Source/Parapenting/Physics/ResearchCoefficientRegistry.cpp
    ../Source/Parapenting/Physics/SolverStateHash.cpp
    ../Source/Parapenting/Physics/ParagliderDynamics.cpp
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
