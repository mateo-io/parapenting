#include "ParagliderDynamics.h"
#include "AtmosphereModel.h"
#include "TerrainModel.h"
#include "TerrainRenderLayout.h"
#include "RouteFrame.h"
#include "RouteCatalogue.h"
#include "WingCatalogue.h"
#include "SpanwiseCanopyModel.h"
#include "TrainingScenario.h"
#include "ChallengeEvaluator.h"
#include "WeatherSnapshot.h"
#include "HeightfieldGrid.h"
#include "HapticFeedback.h"
#include "PilotProgression.h"
#include "AccessibilityProfile.h"
#include "GraphicsProfile.h"
#include "AerodynamicPolar.h"
#include "CameraFeedback.h"
#include "AudioFeedback.h"
#include "EquipmentSetup.h"
#include "PilotPose.h"
#include "PilotSkeletonAim.h"
#include "GliderRigSnapshot.h"
#include "CanopyLoadPose.h"
#include "InputBindingProfile.h"
#include "WindsockModel.h"
#include "LandingCircuitModel.h"
#include "GroundLaunchModel.h"
#include "FlightDebrief.h"
#include "PreflightBriefing.h"
#include "FlightNavigation.h"
#include "GroundEffectModel.h"
#include "LandingRolloutModel.h"
#include "SuspensionSystem.h"
#include "ResearchManeuver.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <string>

using namespace Parapenting::Physics;

static void StepFor(ParagliderDynamics& dynamics, FlightState& state,
                    const ControlInput& controls, double seconds)
{
    constexpr double dt = 1.0 / 120.0;
    for (int i = 0; i < static_cast<int>(seconds / dt); ++i)
        dynamics.Step(state, controls, Atmosphere{}, dt);
}

static double BankAngle(const FlightState& state)
{
    return std::asin(std::clamp(
        state.attitude.Rotate({0.0, 1.0, 0.0}).z, -1.0, 1.0));
}

static double HeadingAngle(const FlightState& state)
{
    return std::atan2(
        2.0 * (state.attitude.w * state.attitude.z
            + state.attitude.x * state.attitude.y),
        1.0 - 2.0 * (state.attitude.y * state.attitude.y
            + state.attitude.z * state.attitude.z));
}

int main()
{
    {
        const auto& geometry = Epic2MlSuspensionGeometry();
        assert(std::abs(geometry.canopyToRiserM - 7.3) < 1e-9);
        assert(std::abs(geometry.totalLineLengthM - 254.0) < 1e-9);
        const auto trim = EvaluateSuspensionLoads(
            geometry, {66.0, 27.0, 1030.0});
        const auto accelerated = EvaluateSuspensionLoads(
            geometry, {90.0, 27.0, 1030.0, 1.0});
        const auto braked = EvaluateSuspensionLoads(
            geometry, {50.0, 27.0, 1030.0, 0.0, 1.0});
        assert(trim.lineDragN > 7.0);
        assert(std::abs(trim.aFraction + trim.bFraction
            + trim.cFraction + trim.brakeFraction - 1.0) < 1e-9);
        assert(accelerated.aFraction > trim.aFraction);
        assert(accelerated.cFraction < trim.cFraction);
        assert(braked.brakeFraction > 0.1);
        assert(braked.pitchMomentNm > trim.pitchMomentNm);

        SuspensionInput unloadedInput{
            18.0, 27.0, 120.0, 0.0, 0.0, 0.82, 0.0, 0.0};
        unloadedInput.canopyPressure = 0.35;
        const auto unloaded = EvaluateSuspensionLoads(
            geometry, unloadedInput);
        assert(unloaded.left.tensionN[0] < unloaded.right.tensionN[0]);
        assert(unloaded.left.slackFraction[0]
            > unloaded.right.slackFraction[0]);
    }
    {
        const WindsockPose calm = EvaluateWindsockPose({}, 0.0, 0.0);
        const WindsockPose north =
            EvaluateWindsockPose({5.0, 0.0, 0.0}, 0.3, 2.0);
        const WindsockPose east =
            EvaluateWindsockPose({0.0, -5.0, 0.0}, 0.3, 2.0);
        assert(calm.inflation == 0.0);
        assert(calm.directionWorld.z < -0.9);
        assert(north.inflation == 1.0);
        assert(north.directionWorld.x > 0.98);
        assert(std::abs(north.directionWorld.z) < 0.1);
        assert(east.directionWorld.y < -0.98);
        assert(north.lengthScale > calm.lengthScale);
        assert(north.radiusScale > calm.radiusScale);
        const WindsockPose repeated =
            EvaluateWindsockPose({5.0, 0.0, 0.0}, 0.3, 2.0);
        assert(north.directionWorld.x == repeated.directionWorld.x);
        assert(north.directionWorld.y == repeated.directionWorld.y);
    }
    {
        auto bindings = InputBindingProfile::Standard();
        assert(bindings.Key(FlightBindingAction::LeftBrake) == "Left");
        assert(bindings.Rebind(
            FlightBindingAction::LeftBrake, "NumPadFour")
            == RebindResult::Changed);
        assert(bindings.Key(FlightBindingAction::LeftBrake) == "NumPadFour");
        assert(bindings.Rebind(
            FlightBindingAction::RightBrake, "NumPadFour")
            == RebindResult::Swapped);
        assert(bindings.Key(FlightBindingAction::RightBrake) == "NumPadFour");
        assert(bindings.Key(FlightBindingAction::LeftBrake) == "Right");
        const auto beforeProtected = bindings;
        assert(bindings.Rebind(FlightBindingAction::LeftBrake, "F7")
            == RebindResult::RejectedProtected);
        assert(bindings.keyNames == beforeProtected.keyNames);
        assert(InputBindingProfile::Compact().Key(
            FlightBindingAction::BrakesRelease) == "Up");
        assert(InputBindingProfile::RightHand().Key(
            FlightBindingAction::WeightShiftLeft) == "N");
    }
    {
        const auto relaxed = EvaluateCanopyLoadPose(0.0, 1.0);
        const auto loaded = EvaluateCanopyLoadPose(1.0, 4.8);
        assert(relaxed.spanScale == 1.0);
        assert(relaxed.chordScale == 1.0);
        assert(relaxed.extraArchDropCm == 0.0);
        assert(loaded.spanScale < relaxed.spanScale);
        assert(loaded.chordScale > relaxed.chordScale);
        assert(loaded.camberScale < relaxed.camberScale);
        assert(loaded.extraArchDropCm > 20.0);
        assert(loaded.lineStretchCm > 6.0);
        assert(loaded.rippleAmplitudeCm > 4.0);
        const auto repeated = EvaluateCanopyLoadPose(1.0, 4.8);
        assert(loaded.spanScale == repeated.spanScale);
        assert(loaded.rippleAmplitudeCm == repeated.rippleAmplitudeCm);
    }
    {
        const auto neutral = EvaluatePilotPose({});
        PilotPoseInput braking;
        braking.leftBrake = 0.8;
        braking.leftBrakeForceN = 52.0;
        const auto leftBrake = EvaluatePilotPose(braking);
        // 55 rather than the 60 this used to demand. The hand now travels
        // along the brake line instead of straight down, so the same handle
        // travel spends part of itself going aft and outboard - which is the
        // point of the change, and a threshold near the full travel would be
        // asserting that the pull is vertical.
        assert(leftBrake.leftHandCm.z < neutral.leftHandCm.z - 55.0);
        assert(leftBrake.leftHandCm.x < neutral.leftHandCm.x);
        assert(leftBrake.rightHandCm.z == neutral.rightHandCm.z);

        PilotPoseInput shifted;
        shifted.harnessRollRad = 0.22;
        shifted.weightShift = 0.8;
        shifted.incidentSeverity = 0.6;
        const auto active = EvaluatePilotPose(shifted);
        assert(active.rigOffsetCm.y > neutral.rigOffsetCm.y + 20.0);
        assert(active.rigOffsetCm.z < neutral.rigOffsetCm.z);
        assert(active.rigRotationDegrees.z > 10.0);
        const auto length = [](const Vec3& a, const Vec3& b)
        {
            const Vec3 delta = a - b;
            return std::sqrt(delta.x * delta.x + delta.y * delta.y
                + delta.z * delta.z);
        };
        assert(std::abs(length(leftBrake.leftShoulderCm, leftBrake.leftElbowCm)
            - PilotUpperArmLengthCm) < 1e-8);
        assert(std::abs(length(leftBrake.leftElbowCm, leftBrake.leftHandCm)
            - PilotForearmLengthCm) < 1e-8);
        // The pull follows the brake line: down, aft and outboard together,
        // because that is where the line runs. A vertical drop is what made
        // the old pull read as a lever rather than a hand on a handle.
        {
            PilotPoseInput up;
            // Quarter brake: enough to measure the direction of the pull,
            // little enough that the arm can still reach without the reach
            // clamp bending the answer. At full travel the clamp steepens the
            // path, which is the arm running out of room rather than the line
            // changing direction, so it is the wrong place to measure this.
            PilotPoseInput pulled;
            pulled.leftBrake = 0.25;
            const auto handsUp = EvaluatePilotPose(up);
            const auto pulling = EvaluatePilotPose(pulled);
            const Vec3 travel = pulling.leftHandCm - handsUp.leftHandCm;
            assert(travel.z < 0.0);
            assert(travel.x < 0.0);
            assert(travel.y < 0.0);
            // Diagonal, not a drop: the horizontal component is a real
            // fraction of the vertical one rather than a rounding artefact.
            const double horizontal =
                std::sqrt(travel.x * travel.x + travel.y * travel.y);
            assert(horizontal > 0.35 * std::abs(travel.z));

            // Travel runs along the line from the pulley, so the hand stays on
            // that line while the arm can still reach.
            const Vec3 lineRun = handsUp.leftHandCm - up.leftBrakePulleyCm;
            const double along = Dot(Normalized(lineRun), Normalized(travel));
            assert(along > 0.999);

            // Moving the pulley moves the direction of the pull with it. This
            // is what a two-liner's different riser geometry changes, without
            // anything else having to know.
            PilotPoseInput moved = pulled;
            moved.leftBrakePulleyCm = {-13.0, -21.0, 120.0};
            const auto higher = EvaluatePilotPose(moved);
            assert(higher.leftHandCm.z > pulling.leftHandCm.z);

            // One hand's pull never reaches across to the other.
            assert(pulling.rightHandCm.x == handsUp.rightHandCm.x);
            assert(pulling.rightHandCm.y == handsUp.rightHandCm.y);
            assert(pulling.rightHandCm.z == handsUp.rightHandCm.z);

            // Monotone: more brake is always more travel, never a reversal
            // partway down as the reach clamp engages.
            double previous = 0.0;
            for (int step = 1; step <= 20; ++step)
            {
                PilotPoseInput partial;
                partial.leftBrake = step / 20.0;
                const auto pose = EvaluatePilotPose(partial);
                const Vec3 moved2 = pose.leftHandCm - handsUp.leftHandCm;
                const double distance = Length(moved2);
                assert(distance >= previous - 1e-9);
                previous = distance;
            }
        }

        PilotPoseInput fullBrake;
        fullBrake.leftBrake = 1.0;
        fullBrake.leftBrakeForceN = 65.0;
        const auto full = EvaluatePilotPose(fullBrake);
        assert(std::abs(length(full.leftShoulderCm, full.leftElbowCm)
            - PilotUpperArmLengthCm) < 1e-8);
        assert(std::abs(length(full.leftElbowCm, full.leftHandCm)
            - PilotForearmLengthCm) < 1e-8);

        // The leg chain has to hold its lengths across weight shift and surge
        // for the same reason the arm chain does: it is skinned now, so a
        // varying segment reads as a stretching limb rather than a moved one.
        for (const auto& legPose : {neutral, active, full})
        {
            const double leftThigh =
                length(legPose.leftHipCm, legPose.leftKneeCm);
            const double rightThigh =
                length(legPose.rightHipCm, legPose.rightKneeCm);
            const double leftShin =
                length(legPose.leftKneeCm, legPose.leftAnkleCm);
            const double rightShin =
                length(legPose.rightKneeCm, legPose.rightAnkleCm);
            assert(std::abs(leftThigh - rightThigh) < 1e-8);
            assert(std::abs(leftShin - rightShin) < 1e-8);
            assert(std::abs(leftThigh
                - length(neutral.leftHipCm, neutral.leftKneeCm)) < 1e-8);
            assert(std::abs(leftShin
                - length(neutral.leftKneeCm, neutral.leftAnkleCm)) < 1e-8);
        }
    }
    {
        const auto previous = BuildGliderRigSnapshot({1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        GliderRigSnapshotInput currentInput{1.0 + 1.0 / 120.0,
            0.1, 0.0, 0.4, 0.8, 0.0, 35.0, 0.0, 0.0, 0.0,
            24.0, 45.0};
        currentInput.telemetry.canopyPressure = 0.6;
        const auto current = BuildGliderRigSnapshot(currentInput, &previous);
        const auto halfway = InterpolateGliderRigSnapshot(previous, current, 0.5);
        assert(std::abs(halfway.brakeTravel[0] - 0.4) < 1e-12);
        assert(halfway.brakeTravel[1] == 0.0);
        assert(std::abs(halfway.telemetry.canopyPressure - 0.8) < 1e-12);
        assert(halfway.brakeTravelVelocityPerS[0] > 40.0);
        const auto bounded = InterpolateGliderRigSnapshot(previous, current, 4.0);
        assert(bounded.brakeTravel == current.brakeTravel);
        for (int side = 0; side < GliderRigSideCount; ++side)
        {
            const Vec3 riser = current.riserTopRigCm[side][0]
                - current.carabinerRigCm[side];
            assert(std::abs(std::sqrt(riser.x * riser.x + riser.y * riser.y
                + riser.z * riser.z) - 45.0) < 1e-12);
        }
    }
    {
        // Riser count is data. A two-liner has to work without anything
        // downstream counting risers for itself.
        GliderRigSnapshotInput twoLiner{};
        twoLiner.riserCount = 2;
        twoLiner.riserForeAftCm = {4.0, -4.0, 0.0, 0.0};
        const auto two = BuildGliderRigSnapshot(twoLiner);
        assert(two.riserCount == 2);

        // The rear riser is the last one present, so the brake pulley moves
        // from the C to the B without anything being told about two-liners.
        assert(RearRiserIndex(two.riserCount) == 1);
        assert(RearRiserIndex(4) == 3);
        for (int side = 0; side < GliderRigSideCount; ++side)
        {
            const Vec3 pulley = two.riserTopRigCm[side][
                RearRiserIndex(two.riserCount)];
            const Vec3 hand = side == 0 ? two.pilot.leftHandCm
                                        : two.pilot.rightHandCm;
            // The handle hangs below its own pulley on both sides.
            assert(hand.z < pulley.z);
        }

        // Riser tops honour the fore/aft the wing was given, and the riser
        // length is preserved as the distance from the carabiner.
        for (int side = 0; side < GliderRigSideCount; ++side)
            for (int riser = 0; riser < two.riserCount; ++riser)
            {
                const Vec3 run = two.riserTopRigCm[side][riser]
                    - two.carabinerRigCm[side];
                assert(std::abs(Length(run) - twoLiner.riserLengthCm) < 1e-9);
            }
        assert(two.riserTopRigCm[0][0].x > two.riserTopRigCm[0][1].x);

        // A wing swap must not blend two riser sets into a half-existing
        // third one, so the count is taken whole from the current boundary.
        const auto three = BuildGliderRigSnapshot({});
        assert(three.riserCount == GliderRigRiserCount);
        const auto blended = InterpolateGliderRigSnapshot(three, two, 0.5);
        assert(blended.riserCount == 2);

        // Nonsense counts are clamped rather than indexing off the array.
        GliderRigSnapshotInput silly{};
        silly.riserCount = 99;
        assert(BuildGliderRigSnapshot(silly).riserCount
            == GliderRigRiserCount);
        silly.riserCount = 0;
        assert(BuildGliderRigSnapshot(silly).riserCount == 1);
        // The line plan carries the same fact so a wing's riser set travels
        // with it; that half is checked in the suspension suite, which is
        // where the line plan is linked.
    }
    {
        // Bone aiming. The rig solves joint positions; a skinned mesh also
        // needs the rotations, or the skin shears at every joint.
        const auto unitLength = [](const Quaternion& q)
        {
            return std::abs(std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y
                + q.z * q.z) - 1.0) < 1e-12;
        };
        const auto sameDirection = [](const Vec3& a, const Vec3& b)
        {
            const Vec3 d = Normalized(a) - Normalized(b);
            return Length(d) < 1e-9;
        };

        // The defining property: the rotation carries rest onto target.
        const Vec3 rest{0.0, 0.0, 1.0};
        for (const Vec3& target : {Vec3{1.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0},
             Vec3{0.0, 0.0, 1.0}, Vec3{1.0, 2.0, -3.0}, Vec3{-4.0, 0.5, 0.0}})
        {
            const auto aim = AimRotation(rest, target);
            assert(unitLength(aim));
            assert(sameDirection(aim.Rotate(rest), target));
        }

        // Length must not matter: these are directions, not displacements.
        assert(sameDirection(
            AimRotation(rest, {0.0, 9.0, 0.0}).Rotate(rest),
            AimRotation(rest, {0.0, 0.001, 0.0}).Rotate(rest)));

        // A half turn has no shortest rotation. It must still be a real
        // rotation that lands on the target, not a NaN.
        const auto reversed = AimRotation(rest, {0.0, 0.0, -1.0});
        assert(unitLength(reversed));
        assert(sameDirection(reversed.Rotate(rest), {0.0, 0.0, -1.0}));

        // A collapsed joint has no direction to point. Identity beats
        // inventing one, which would spin the limb.
        const auto degenerate = AimRotation(rest, {0.0, 0.0, 0.0});
        assert(degenerate.w == 1.0 && degenerate.x == 0.0
            && degenerate.y == 0.0 && degenerate.z == 0.0);

        // Rolled aim: still hits the target, and now also carries the
        // reference up onto the requested one. This is the twist a wrist
        // needs and a forearm must not be free to invent.
        const Vec3 restUp{0.0, 1.0, 0.0};
        const Vec3 target{1.0, 0.0, 0.0};
        const Vec3 wantUp{0.0, 0.0, 1.0};
        const auto rolled = AimRotationWithRoll(rest, restUp, target, wantUp);
        assert(unitLength(rolled));
        assert(sameDirection(rolled.Rotate(rest), target));
        assert(sameDirection(rolled.Rotate(restUp), wantUp));

        // An up vector parallel to the aim carries no roll information, so the
        // result falls back to the plain aim rather than producing garbage.
        const auto noRoll = AimRotationWithRoll(rest, restUp, target, target);
        assert(unitLength(noRoll));
        assert(sameDirection(noRoll.Rotate(rest), target));

        // Determinism: the same inputs give bit-identical rotations, or a
        // replay would not reproduce the pilot.
        assert(AimRotation(rest, {1.0, 2.0, -3.0}).x
            == AimRotation(rest, {1.0, 2.0, -3.0}).x);
    }
    {
        // Torso lag. The chest and head must trail a surge and then settle on
        // it, and the lag has to live on the fixed step so a replay at any
        // frame rate produces the same lean.
        constexpr double step = 1.0 / 120.0;
        GliderRigSnapshotInput surging{};
        surging.recoverySurge = 0.4;
        auto snapshot = BuildGliderRigSnapshot(surging);
        // With nothing to lag behind, the torso starts settled rather than
        // swinging in from zero on the very first frame.
        assert(std::abs(snapshot.torsoSurge - 0.4) < 1e-12);

        GliderRigSnapshot settled = snapshot;
        auto previous = BuildGliderRigSnapshot({});
        double time = 0.0;
        double lastLean = previous.torsoSurge;
        for (int tick = 0; tick < 240; ++tick)
        {
            time += step;
            surging.simulationTimeSeconds = time;
            const auto next = BuildGliderRigSnapshot(surging, &previous);
            // Monotone approach, never an overshoot past the commanded surge.
            assert(next.torsoSurge >= lastLean - 1e-12);
            assert(next.torsoSurge <= 0.4 + 1e-12);
            // The chest leans on the filtered value, so it always trails the
            // fully surged pose until the filter has settled.
            assert(next.pilot.chestCm.x >= settled.pilot.chestCm.x - 1e-12);
            lastLean = next.torsoSurge;
            previous = next;
        }
        assert(std::abs(previous.torsoSurge - 0.4) < 1e-3);

        // One 1/60 step must land where two 1/120 steps land: the filter is a
        // function of elapsed simulation time, not of how often it was called.
        auto coarse = BuildGliderRigSnapshot({});
        GliderRigSnapshotInput coarseInput{};
        coarseInput.recoverySurge = 0.4;
        coarseInput.simulationTimeSeconds = 2.0 * step;
        coarse = BuildGliderRigSnapshot(coarseInput, &coarse);
        auto fine = BuildGliderRigSnapshot({});
        GliderRigSnapshotInput fineInput{};
        fineInput.recoverySurge = 0.4;
        fineInput.simulationTimeSeconds = step;
        fine = BuildGliderRigSnapshot(fineInput, &fine);
        fineInput.simulationTimeSeconds = 2.0 * step;
        fine = BuildGliderRigSnapshot(fineInput, &fine);
        assert(std::abs(coarse.torsoSurge - fine.torsoSurge) < 1e-12);

        // A repeated or rewound timestamp must not advance the filter, or a
        // paused replay would keep leaning while the solver stands still.
        auto paused = previous;
        GliderRigSnapshotInput pausedInput{};
        pausedInput.recoverySurge = 0.0;
        pausedInput.simulationTimeSeconds = previous.simulationTimeSeconds;
        const auto held = BuildGliderRigSnapshot(pausedInput, &paused);
        assert(held.torsoSurge == paused.torsoSurge);
        pausedInput.simulationTimeSeconds =
            previous.simulationTimeSeconds - 1.0;
        const auto rewound = BuildGliderRigSnapshot(pausedInput, &paused);
        assert(rewound.torsoSurge == paused.torsoSurge);
    }
    {
        // One layout per surveyed region, and every one of them has to hold
        // the same two contracts: a vertex budget, and a mesh that never
        // claims ground the survey does not cover.
        for (const auto& region : RouteFrame::regions)
        {
            const TerrainRenderLayout layout = LayoutFor(
                0.5 * (region.xMinM + region.xMaxM),
                0.5 * (region.yMinM + region.yMaxM));
            assert(layout.VerticesPerTile() == 2601);
            assert(layout.TrianglesPerTile() == 5000);
            assert(layout.TotalVertices() < 220000);
            assert(layout.TotalTriangles() < 420000);
            // Sample spacing should track the 20 m heightfield: fine enough to
            // resolve it, coarse enough not to spend vertices on nothing.
            assert(layout.SampleSpacingXM() > 15.0
                && layout.SampleSpacingXM() <= 22.0);
            assert(layout.SampleSpacingYM() > 15.0
                && layout.SampleSpacingYM() <= 22.0);
            // Exactly the region's own rectangle, not a rectangle that spills
            // onto ground the analytic proxy would have to invent.
            assert(layout.xMinM == region.xMinM);
            assert(layout.xMaxM == region.xMaxM);
            assert(layout.yMinM == region.yMinM);
            assert(layout.yMaxM == region.yMaxM);
        }
        // The Interlaken region is what a position off every region gets, so
        // its tile count is the one the actor reserves for.
        assert(LayoutFor(0.0, 0.0).TileCount() == 64);
        assert(LayoutFor(1.0e9, 1.0e9).TileCount() == 64);
    }
    {
        assert(TerrainModel::LoadHeightfieldAscii(
            "Content/Terrain/interlaken.asc")
            || TerrainModel::LoadHeightfieldAscii(
                "../Content/Terrain/interlaken.asc"));
        const double launchRelative = TerrainModel::HeightM(0.0, 0.0);
        const double landingX = RouteHorizontalDistanceM(
            GetRouteProfile(RouteProfileId::AmisbuehlLehn));
        const double landingRelative = TerrainModel::HeightM(landingX, 0.0);
        assert(launchRelative > 700.0 && launchRelative < 800.0);
        assert(landingRelative > -15.0 && landingRelative < 30.0);
        TerrainModel::ClearHeightfield();
    }
    {
        HeightfieldGrid grid;
        assert(grid.LoadEsriAscii(
            "../Tests/Fixtures/tiny-heightfield.asc")
            || grid.LoadEsriAscii(
                "Tests/Fixtures/tiny-heightfield.asc"));
        double elevation = 0.0;
        assert(grid.Sample(5.0, 5.0, elevation));
        assert(std::abs(elevation - 110.0) < 1e-9);
        assert(grid.Sample(15.0, 15.0, elevation));
        assert(std::abs(elevation - 130.0) < 1e-9);
        assert(!grid.Sample(-1.0, 5.0, elevation));
    }
    {
        const DiurnalState dawn =
            EvaluateDiurnalCycle(6.5, 0.0);
        const DiurnalState midday =
            EvaluateDiurnalCycle(13.5, 0.0);
        const DiurnalState eveningState =
            EvaluateDiurnalCycle(21.0, 0.0);
        const DiurnalState wrapped =
            EvaluateDiurnalCycle(23.5, 600.0);
        assert(dawn.sunElevationDegrees > 0.0);
        assert(dawn.sunElevationDegrees < 10.0);
        assert(midday.sunElevationDegrees > 55.0);
        assert(midday.surfaceHeating > 0.8);
        assert(midday.convectiveActivity > 0.8);
        assert(eveningState.sunElevationDegrees < 0.0);
        assert(eveningState.surfaceHeating < -0.1);
        assert(eveningState.convectiveActivity == 0.0);
        assert(std::abs(wrapped.localHour - 0.5) < 1e-12);
        assert(WrapLocalHour(-1.0) == 23.0);
        const DiurnalState repeated =
            EvaluateDiurnalCycle(13.5, 127.25);
        assert(repeated.localHour
            == EvaluateDiurnalCycle(13.5, 127.25).localHour);
        assert(repeated.surfaceHeating
            == EvaluateDiurnalCycle(13.5, 127.25).surfaceHeating);

        const Vec3 eastFacingNormal = Normalized({-0.35, 0.0, 1.0});
        const TerrainCirculation anabatic =
            EvaluateTerrainCirculation(eastFacingNormal, 15.0, 0.9);
        const TerrainCirculation katabatic =
            EvaluateTerrainCirculation(eastFacingNormal, 15.0, -0.9);
        const TerrainCirculation aloft =
            EvaluateTerrainCirculation(eastFacingNormal, 600.0, 0.9);
        const TerrainCirculation flat =
            EvaluateTerrainCirculation({0.0, 0.0, 1.0}, 15.0, 0.9);
        assert(anabatic.alongSlopeMps > 0.6);
        assert(anabatic.velocityWorldMps.x > 0.6);
        assert(anabatic.velocityWorldMps.z > 0.02);
        assert(katabatic.alongSlopeMps < -0.6);
        assert(katabatic.velocityWorldMps.x < -0.6);
        assert(katabatic.velocityWorldMps.z < -0.02);
        assert(std::abs(aloft.alongSlopeMps)
            < std::abs(anabatic.alongSlopeMps) * 0.03);
        assert(flat.alongSlopeMps == 0.0);

        const Vec3 north = WindVectorFromMeteorological(0.0, 5.0);
        assert(north.x > 4.9);
        assert(std::abs(north.y) < 0.3);
        const Vec3 east = WindVectorFromMeteorological(90.0, 4.0);
        assert(std::abs(east.x) < 0.3);
        assert(east.y < -3.9);

        assert(GetWeatherPresets().size() == 5);
        AtmosphereModel thermalDay;
        thermalDay.SetPreset(WeatherPresetId::ThermalDay);
        const Atmosphere thermalVolume =
            thermalDay.Sample({1660.0, 430.0, 560.0}, 0.0);
        assert(thermalVolume.thermalLiftMps > 2.0);
        assert(thermalVolume.turbulence > 0.25);
        const CloudFieldState daytimeCloud =
            thermalDay.SampleCloudField(0.0);
        const Atmosphere nightVolume =
            thermalDay.Sample({1660.0, 430.0, 560.0}, 6000.0);
        const CloudFieldState nightCloud =
            thermalDay.SampleCloudField(6000.0);
        assert(nightVolume.thermalLiftMps
            < thermalVolume.thermalLiftMps * 0.2);
        assert(nightCloud.coverage < daytimeCloud.coverage);

        AtmosphereModel foehn;
        foehn.SetPreset(WeatherPresetId::FoehnRotor);
        const Atmosphere rotorVolume =
            foehn.Sample({1580.0, -690.0, 230.0}, 0.0);
        assert(rotorVolume.rotorStrength > 0.75);
        assert(rotorVolume.turbulence > 0.7);
        assert(rotorVolume.lowFrequencyGustMps > 0.1);
        assert(rotorVolume.highFrequencyGustMps > 0.1);
        assert(rotorVolume.gustEnergyMps > 0.2);
        const Atmosphere rotorRepeated =
            foehn.Sample({1580.0, -690.0, 230.0}, 0.0);
        assert(rotorVolume.windWorldMps.x == rotorRepeated.windWorldMps.x);
        assert(rotorVolume.highFrequencyGustMps
            == rotorRepeated.highFrequencyGustMps);
        const Atmosphere rotorNextStep =
            foehn.Sample({1580.0, -690.0, 230.0}, 1.0 / 120.0);
        assert(Length(rotorNextStep.windWorldMps
            - rotorVolume.windWorldMps) < 0.25);

        AtmosphereModel evening;
        evening.SetPreset(WeatherPresetId::EveningDrainage);
        double strongestDrainageMps = 0.0;
        for (double x = 500.0; x <= 3500.0; x += 500.0)
        {
            for (double y = 6000.0; y <= 8500.0; y += 500.0)
            {
                const double localGround = TerrainModel::HeightM(x, y);
                const Atmosphere drainage = evening.Sample(
                    {x, y, localGround + 15.0}, 20.0);
                strongestDrainageMps = std::min(
                    strongestDrainageMps, drainage.slopeFlowMps);
            }
        }
        assert(strongestDrainageMps < -0.25);

        AtmosphereModel observed;
        WeatherSnapshot snapshot;
        snapshot.source = "test-station";
        snapshot.displayName = "Offline observation";
        snapshot.windFromDegrees = 90.0;
        snapshot.windSpeedMps = 6.0;
        snapshot.gustSpeedMps = 11.0;
        snapshot.thermalTopMslM = 1900.0;
        observed.ApplySnapshot(snapshot);
        assert(observed.GetPresetId() == WeatherPresetId::Custom);
        assert(observed.GetMode() == WeatherMode::LocalizedRotor);
        assert(observed.GetBaseWind().y < -5.8);
        const Atmosphere first =
            observed.Sample({900.0, 200.0, 600.0}, 12.5);
        const Atmosphere second =
            observed.Sample({900.0, 200.0, 600.0}, 12.5);
        assert(first.windWorldMps.x == second.windWorldMps.x);
        assert(first.windWorldMps.y == second.windWorldMps.y);
        assert(first.windWorldMps.z == second.windWorldMps.z);

        // Canopy sampling is deterministic and exposes the real velocity
        // difference encountered by two half-wings several metres apart.
        const Atmosphere spanFirst = observed.SampleCanopy(
            {900.0, 200.0, 600.0}, {0.0, 1.0, 0.0}, 5.6, 12.5);
        const Atmosphere spanSecond = observed.SampleCanopy(
            {900.0, 200.0, 600.0}, {0.0, 1.0, 0.0}, 5.6, 12.5);
        assert(spanFirst.leftWingWindDeltaMps.x
            == spanSecond.leftWingWindDeltaMps.x);
        assert(spanFirst.rightWingWindDeltaMps.z
            == spanSecond.rightWingWindDeltaMps.z);
        assert(spanFirst.spanwiseAirflowShearMps
            == spanSecond.spanwiseAirflowShearMps);

        AtmosphereModel calm;
        calm.SetPreset(WeatherPresetId::MorningCalm);
        const Atmosphere calmSpan = calm.SampleCanopy(
            {0.0, 0.0, 1200.0}, {0.0, 1.0, 0.0}, 5.6, 4.0);
        assert(calmSpan.spanwiseAirflowShearMps < 0.1);
        assert(calmSpan.gustEnergyMps == 0.0);
        assert(calmSpan.highFrequencyGustMps == 0.0);
        const CloudFieldState calmCloud = calm.SampleCloudField(60.0);
        assert(calmCloud.coverage < 0.04);
        assert(calmCloud.layerThicknessM == 180.0);
        assert(calmCloud.shadowStrength < 0.05);

        double minimumCloudCoverage = 1.0;
        double maximumCloudCoverage = 0.0;
        double maximumCloudDepth = 0.0;
        AtmosphereModel cloudDay;
        cloudDay.SetPreset(WeatherPresetId::ThermalDay);
        for (double time = 0.0; time <= 280.0; time += 4.0)
        {
            const CloudFieldState cloud =
                cloudDay.SampleCloudField(time);
            minimumCloudCoverage = std::min(
                minimumCloudCoverage, cloud.coverage);
            maximumCloudCoverage = std::max(
                maximumCloudCoverage, cloud.coverage);
            maximumCloudDepth = std::max(
                maximumCloudDepth, cloud.layerThicknessM);
            const CloudFieldState repeatedCloud =
                cloudDay.SampleCloudField(time);
            assert(cloud.coverage == repeatedCloud.coverage);
            assert(cloud.driftM.x == repeatedCloud.driftM.x);
        }
        assert(maximumCloudCoverage > minimumCloudCoverage + 0.05);
        assert(maximumCloudDepth > 750.0);
        assert(maximumCloudDepth < 1000.0);

        AtmosphereModel thermalEdge;
        thermalEdge.SetPreset(WeatherPresetId::ThermalDay);
        double strongestSpanShear = 0.0;
        for (double x = 1450.0; x <= 1850.0; x += 25.0)
        {
            for (double y = 250.0; y <= 650.0; y += 25.0)
            {
                const Atmosphere span = thermalEdge.SampleCanopy(
                    {x, y, 560.0}, {0.0, 1.0, 0.0}, 5.6, 18.0);
                strongestSpanShear = std::max(
                    strongestSpanShear, span.spanwiseAirflowShearMps);
            }
        }
        assert(strongestSpanShear > 0.08);
    }
    {
        const Vec3 target{1000.0, 200.0, 565.0};
        const LandingCircuit valley =
            BuildLandingCircuit(target, {-4.0, 0.0, 0.0}, true);
        assert(std::abs(valley.finalDirection.x - 1.0) < 1e-12);
        assert(std::abs(valley.finalGate.x - 720.0) < 1e-12);
        assert(valley.baseGate.y < target.y);
        const LandingGuidance stable = EvaluateLandingApproach(
            valley, valley.finalGate, {10.0, 0.0, -1.4}, 65.0);
        assert(stable.phase == LandingPhase::Final);
        assert(stable.stabilized);
        assert(stable.approachQuality > 0.9);
        const LandingGuidance crossed = EvaluateLandingApproach(
            valley, valley.finalGate, {0.0, 10.0, -5.0}, 65.0);
        assert(!crossed.stabilized);
        assert(crossed.approachQuality < stable.approachQuality);
        const LandingCircuit mountain =
            BuildLandingCircuit(target, {4.0, 0.0, 0.0}, false);
        assert(mountain.finalDirection.x < -0.99);
        assert(mountain.baseGate.y < target.y);
        const LandingGuidance flare = EvaluateLandingApproach(
            valley, {990.0, 200.0, 566.0}, {8.0, 0.0, -0.8}, 5.0);
        assert(flare.phase == LandingPhase::Flare);
    }
    {
        GroundLaunchModel launch;
        GroundLaunchState state;
        launch.Reset(state);
        GroundLaunchInput input;
        input.launchHeld = true;
        input.surfaceWindMps = {-2.5, 0.0, 0.0};
        input.slopeDownRadians = 0.14;
        GroundLaunchOutput output;
        for (int frame = 0; frame < 12 * 120 && !output.liftOff; ++frame)
            output = launch.Step(state, input, 1.0 / 120.0);
        assert(output.liftOff);
        assert(state.phase == LaunchPhase::Airborne);
        assert(state.inflation > 0.88);
        assert(state.stableOverheadS > 0.9);
        assert(output.apparentWindMps > 8.0);

        GroundLaunchState aborted;
        launch.Reset(aborted, true);
        GroundLaunchInput abortInput = input;
        abortInput.leftBrake = 1.0;
        abortInput.rightBrake = 1.0;
        GroundLaunchOutput abortOutput;
        for (int frame = 0; frame < 6 * 120 && !abortOutput.abort; ++frame)
            abortOutput = launch.Step(
                aborted, abortInput, 1.0 / 120.0);
        assert(abortOutput.abort);
        assert(aborted.phase == LaunchPhase::Aborted);

        GroundLaunchState crosswind;
        launch.Reset(crosswind);
        GroundLaunchInput crosswindInput = input;
        crosswindInput.surfaceWindMps = {-1.0, 8.0, 0.0};
        GroundLaunchOutput crosswindOutput;
        for (int frame = 0;
             frame < 12 * 120 && !crosswindOutput.abort; ++frame)
            crosswindOutput = launch.Step(
                crosswind, crosswindInput, 1.0 / 120.0);
        assert(crosswindOutput.abort);
        assert(std::abs(crosswind.canopyHeadingErrorRad) > 1.0);

        GroundLaunchState reverse;
        launch.Reset(reverse, true);
        GroundLaunchInput reverseInput = input;
        reverseInput.surfaceWindMps = {-6.0, 0.7, 0.0};
        GroundLaunchOutput reverseOutput;
        for (int frame = 0; frame < 18 * 120
             && !reverseOutput.liftOff; ++frame)
        {
            if (reverse.phase == LaunchPhase::Turning)
            {
                reverseInput.weightShift = 0.2;
                reverseInput.rightBrake = 0.16;
            }
            reverseOutput = launch.Step(
                reverse, reverseInput, 1.0 / 120.0);
        }
        assert(reverseOutput.liftOff);
        assert(reverse.turnProgress >= 1.0);
        assert(std::abs(reverse.pilotFacingYawOffsetRad) < 1e-12);
        assert(reverse.phase == LaunchPhase::Airborne);

        GroundLaunchState noTurn;
        launch.Reset(noTurn, true);
        GroundLaunchInput noTurnInput = reverseInput;
        noTurnInput.weightShift = 0.0;
        noTurnInput.rightBrake = 0.0;
        GroundLaunchOutput noTurnOutput;
        for (int frame = 0; frame < 15 * 120; ++frame)
            noTurnOutput = launch.Step(
                noTurn, noTurnInput, 1.0 / 120.0);
        assert(!noTurnOutput.liftOff);
        assert(noTurn.phase == LaunchPhase::Turning);
        assert(noTurn.turnDirection == 0);
        assert(std::abs(noTurn.pilotFacingYawOffsetRad
            - 3.14159265358979323846) < 1e-9);
    }
    {
        FlightDebrief clean;
        clean.Reset();
        FlightDebriefSample sample;
        sample.positionWorldM = {0.0, 0.0, 900.0};
        for (int frame = 0; frame < 60 * 10; ++frame)
        {
            sample.positionWorldM.x += 1.0;
            sample.positionWorldM.z -= 0.04;
            sample.verticalSpeedMps = -0.4;
            sample.airspeedMps = 11.0;
            sample.groundClearanceM = 400.0;
            if (frame >= 100 && frame < 260)
            {
                sample.thermalLiftMps = 2.2;
                sample.verticalSpeedMps = 1.4;
                sample.positionWorldM.z += 0.18;
            }
            else
                sample.thermalLiftMps = 0.0;
            if (frame >= 500)
            {
                sample.groundClearanceM = 80.0;
                sample.distanceToLandingM = 300.0;
                sample.approachQuality = 0.92;
                sample.stabilizedApproach = true;
            }
            clean.Step(sample, 0.1);
        }
        clean.FinalizeLanding(5.0, -1.0, 4.0);
        const auto& cleanSummary = clean.Summary();
        assert(cleanSummary.landed);
        assert(cleanSummary.currentPhase == FlightPhase::Landed);
        assert(cleanSummary.thermalTimeS > 15.0);
        assert(cleanSummary.thermalGainM > 20.0);
        assert(cleanSummary.landingRating > 85.0);
        assert(cleanSummary.safetyRating > 99.0);

        FlightDebrief incident;
        incident.Reset();
        FlightDebriefSample upset;
        for (int frame = 0; frame < 200; ++frame)
        {
            upset.positionWorldM.x += 0.8;
            upset.airspeedMps = 15.0;
            upset.loadFactor = frame < 120 ? 5.2 : 1.0;
            upset.leftCollapse =
                (frame >= 10 && frame < 60)
                || (frame >= 100 && frame < 150) ? 0.55 : 0.0;
            upset.leftCravat =
                frame >= 25 && frame < 80 ? 0.16 : 0.0;
            upset.spin = frame >= 35 && frame < 70 ? 0.6 : 0.0;
            incident.Step(upset, 0.1);
        }
        incident.FinalizeLanding(180.0, -6.2, 15.0);
        const auto& incidentSummary = incident.Summary();
        assert(incidentSummary.asymmetricCollapseEvents == 2);
        assert(incidentSummary.cravatEvents == 1);
        assert(incidentSummary.stallOrSpinEvents == 1);
        assert(incidentSummary.safetyRating < cleanSummary.safetyRating);
        assert(incidentSummary.landingRating < 20.0);
        assert(incidentSummary.overallRating < cleanSummary.overallRating);
    }
    {
        const auto& route = GetRouteProfile(RouteProfileId::AmisbuehlLehn);
        const Vec3 launch = RouteLaunchLocalM(route);
        const Vec3 landing = RouteLandingLocalM(route);
        const Vec3 cruise = (launch + landing) * 0.5
            + Vec3{0.0, 0.0, 300.0};

        AtmosphereModel calm;
        calm.SetPreset(WeatherPresetId::MorningCalm);
        const auto calmBrief = EvaluatePreflightBriefing(
            route, calm.GetSnapshot(), calm.GetParameters(),
            calm.GetVolumes(), calm.SampleCloudField(0.0),
            calm.Sample(launch, 0.0), calm.Sample(landing, 0.0),
            calm.Sample(cruise, 0.0));
        assert(calmBrief.overallRisk == PreflightRisk::Low);
        assert(calmBrief.suitabilityScore > 78.0);
        assert(calmBrief.authoredRotorVolumes == 0);

        AtmosphereModel foehn;
        foehn.SetPreset(WeatherPresetId::FoehnRotor);
        const auto foehnBrief = EvaluatePreflightBriefing(
            route, foehn.GetSnapshot(), foehn.GetParameters(),
            foehn.GetVolumes(), foehn.SampleCloudField(0.0),
            foehn.Sample(launch, 0.0), foehn.Sample(landing, 0.0),
            foehn.Sample(cruise, 0.0));
        assert(foehnBrief.authoredRotorVolumes == 3);
        assert(foehnBrief.rotorRisk > 0.8);
        assert(foehnBrief.suitabilityScore < calmBrief.suitabilityScore);
        assert(foehnBrief.overallRisk == PreflightRisk::High
            || foehnBrief.overallRisk == PreflightRisk::Extreme);

        WeatherSnapshot strongSnapshot{
            "manual", "strong", 0.0, route.launchFacingDegrees,
            10.0, 14.0, 2200.0, 1};
        WeatherParameters strongParameters;
        strongParameters.baseWindMps = WindVectorFromMeteorological(
            strongSnapshot.windFromDegrees, strongSnapshot.windSpeedMps);
        strongParameters.turbulence = 0.4;
        Atmosphere strongAir;
        strongAir.windWorldMps = strongParameters.baseWindMps;
        strongAir.turbulence = 0.4;
        std::array<WeatherVolume, 5> noVolumes{};
        const auto strongBrief = EvaluatePreflightBriefing(
            route, strongSnapshot, strongParameters, noVolumes,
            {2200.0, 400.0, 0.3, 0.5, 0.2, {}},
            strongAir, strongAir, strongAir);
        assert(strongBrief.launchWindAssessment
            == SiteWindAssessment::TooStrong);
        assert(strongBrief.overallRisk == PreflightRisk::Extreme);

        AtmosphereModel thermal;
        thermal.SetPreset(WeatherPresetId::ThermalDay);
        assert(std::abs(thermal.GetSnapshot().thermalTopMslM - 2450.0)
            < 1e-12);
    }
    {
        const NavigationWaypoint target{
            "TEST", {1000.0, 0.0, 0.0}, 100.0, 65.0, 400.0};
        const SteadyPolarPoint polar{
            0.0, 10.0, 1.0, 10.0, 0.6, 0.06};
        const auto calm = EvaluateGlideNavigation(
            {0.0, 0.0, 300.0}, target, {}, polar, 10.0, 0.0);
        assert(calm.reachable);
        assert(std::abs(calm.predictedGroundSpeedMps - 10.0) < 1e-12);
        assert(std::abs(calm.predictedArrivalHeightM - 200.0) < 1e-12);
        assert(std::abs(calm.availableGroundGlideRatio - 10.0) < 1e-12);

        const auto tailwind = EvaluateGlideNavigation(
            {0.0, 0.0, 300.0}, target, {4.0, 0.0, 0.0},
            polar, 10.0, 0.0);
        assert(tailwind.predictedArrivalHeightM
            > calm.predictedArrivalHeightM);

        const auto headwind = EvaluateGlideNavigation(
            {0.0, 0.0, 300.0}, target, {-9.5, 0.0, 0.0},
            polar, 10.0, 0.0);
        assert(!headwind.reachable);
        assert(headwind.speedToFly == SpeedToFlyCue::Accelerate);

        const auto crosswind = EvaluateGlideNavigation(
            {0.0, 0.0, 300.0}, target, {0.0, 11.0, 0.0},
            polar, 10.0, 0.0);
        assert(!crosswind.crosswindFeasible);
        assert(!crosswind.reachable);

        const auto lift = EvaluateGlideNavigation(
            {0.0, 0.0, 300.0}, target, {}, polar, 10.0, 1.5);
        assert(lift.speedToFly == SpeedToFlyCue::MinimumSink);
        assert(lift.predictedArrivalHeightM
            > calm.predictedArrivalHeightM);

        const NavigationRoute route = BuildNavigationRoute(
            {0.0, 0.0, 600.0}, {1000.0, 0.0, 0.0});
        NavigationProgress progress;
        const Vec3 highLanding =
            route.waypoints[2].positionWorldM + Vec3{0.0, 0.0, 300.0};
        progress.activeWaypoint = 2;
        assert(!UpdateNavigationProgress(progress, route, highLanding));
        progress.activeWaypoint = 0;
        assert(UpdateNavigationProgress(
            progress, route, route.waypoints[0].positionWorldM));
        assert(progress.activeWaypoint == 1);
        assert(UpdateNavigationProgress(
            progress, route, route.waypoints[1].positionWorldM));
        assert(progress.activeWaypoint == 2);
        assert(UpdateNavigationProgress(
            progress, route, route.waypoints[2].positionWorldM));
        assert(progress.complete);
        assert(!UpdateNavigationProgress(
            progress, route, route.waypoints[2].positionWorldM));
    }
    {
        assert(TrainingScenarioCount() == 8);
        const auto& recovery = GetTrainingScenarioByIndex(3);
        assert(ScenarioCueCrossed(recovery, 7.99, 8.01)
            == IncidentCue::LeftCollapse);
        assert(ScenarioCueCrossed(recovery, 8.01, 8.02)
            == IncidentCue::None);
        const auto& cascade = GetTrainingScenarioByIndex(5);
        assert(ScenarioCueCrossed(cascade, 12.9, 13.1)
            == IncidentCue::FrontalCollapse);
        const auto& spiral = GetTrainingScenarioByIndex(6);
        assert(spiral.id == TrainingScenarioId::SpiralRecovery);
        assert(ScenarioCueCrossed(spiral, 7.9, 8.1)
            == IncidentCue::RightSpiral);
        const auto& flare = GetTrainingScenarioByIndex(7);
        assert(flare.id == TrainingScenarioId::LandingFlare);
        assert(flare.weather == WeatherMode::Chill);
    }
    {
        ChallengeEvaluator thermal;
        thermal.Reset(TrainingScenarioId::ThermalCentering);
        ChallengeSample core;
        core.verticalSpeedMps = 2.4;
        core.thermalLiftMps = 3.0;
        core.canopyPressure = 1.0;
        for (int i = 0; i < 60 * 120; ++i)
            thermal.Step(core, 1.0 / 120.0);
        assert(thermal.IsComplete());
        assert(thermal.Score() > 850.0);
        assert(thermal.Feedback() == ChallengeFeedback::Complete);

        ChallengeEvaluator missed;
        missed.Reset(TrainingScenarioId::ThermalCentering);
        ChallengeSample sink;
        sink.verticalSpeedMps = -4.0;
        sink.canopyPressure = 1.0;
        for (int i = 0; i < 60 * 120; ++i)
            missed.Step(sink, 1.0 / 120.0);
        assert(missed.Score() < thermal.Score());

        ChallengeEvaluator landing;
        landing.Reset(TrainingScenarioId::FreeFlight);
        landing.FinalizeLanding(4.0, -1.1, 4.0);
        assert(landing.IsComplete());
        assert(landing.Score() > 850.0);

        ChallengeEvaluator hardLanding;
        hardLanding.Reset(TrainingScenarioId::FreeFlight);
        hardLanding.FinalizeLanding(180.0, -6.5, 15.0);
        assert(hardLanding.Score() < 250.0);

        ChallengeEvaluator unstableLanding;
        unstableLanding.Reset(TrainingScenarioId::FreeFlight);
        ChallengeSample unstableApproach;
        unstableApproach.groundClearanceM = 60.0;
        unstableApproach.landingPhase = LandingPhase::Final;
        unstableApproach.approachQuality = 0.1;
        for (int frame = 0; frame < 10 * 120; ++frame)
            unstableLanding.Step(unstableApproach, 1.0 / 120.0);
        unstableLanding.FinalizeLanding(4.0, -1.1, 4.0);
        assert(unstableLanding.Score() < landing.Score() - 120.0);

        ChallengeEvaluator goodFlare;
        goodFlare.Reset(TrainingScenarioId::LandingFlare);
        ChallengeSample finalSample;
        finalSample.landingPhase = LandingPhase::Final;
        finalSample.approachQuality = 0.92;
        finalSample.stabilizedApproach = true;
        finalSample.flareEnergy = 0.82;
        for (int frame = 0; frame < 10 * 120; ++frame)
        {
            finalSample.groundClearanceM =
                55.0 - 52.5 * frame / (10.0 * 120.0);
            if (finalSample.groundClearanceM < 3.0)
            {
                finalSample.leftBrake = 0.82;
                finalSample.rightBrake = 0.82;
                finalSample.flareAuthority = 0.18;
            }
            goodFlare.Step(finalSample, 1.0 / 120.0);
        }
        assert(goodFlare.FirstFlareClearanceM() > 2.4);
        assert(goodFlare.FirstFlareClearanceM() < 3.1);
        goodFlare.FinalizeLanding(5.0, -1.0, 5.0);
        assert(goodFlare.Score() > 900.0);
        const double touchdownScore = goodFlare.Score();
        goodFlare.FinalizeRollout(7.0, false);
        assert(goodFlare.Score() == touchdownScore);

        ChallengeEvaluator earlyFlare;
        earlyFlare.Reset(TrainingScenarioId::LandingFlare);
        ChallengeSample earlySample = finalSample;
        earlySample.groundClearanceM = 20.0;
        earlySample.leftBrake = 0.82;
        earlySample.rightBrake = 0.82;
        earlySample.flareAuthority = 0.10;
        earlyFlare.Step(earlySample, 1.0 / 120.0);
        earlyFlare.FinalizeLanding(5.0, -1.0, 5.0);
        assert(earlyFlare.FirstFlareClearanceM() == 20.0);
        assert(earlyFlare.Score() < goodFlare.Score() - 180.0);
        const double earlyTouchdownScore = earlyFlare.Score();
        earlyFlare.FinalizeRollout(18.0, true);
        assert(earlyFlare.Score() < earlyTouchdownScore - 180.0);

        ChallengeEvaluator controlledSpiral;
        controlledSpiral.Reset(TrainingScenarioId::SpiralRecovery);
        for (int frame = 0; frame < 35 * 120; ++frame)
        {
            ChallengeSample sample;
            if (frame >= 8 * 120 && frame < 12 * 120)
            {
                sample.yawRateRadps = 0.72;
                sample.rollRateRadps = 0.48;
                sample.loadFactor = 2.8;
            }
            controlledSpiral.Step(sample, 1.0 / 120.0);
        }
        assert(controlledSpiral.IsComplete());
        assert(controlledSpiral.Score() > 850.0);

        ChallengeEvaluator heldSpiral;
        heldSpiral.Reset(TrainingScenarioId::SpiralRecovery);
        for (int frame = 0; frame < 35 * 120; ++frame)
        {
            ChallengeSample sample;
            if (frame >= 8 * 120)
            {
                sample.yawRateRadps = 0.95;
                sample.rollRateRadps = 0.72;
                sample.pitchRateRadps = 0.35;
                sample.loadFactor = 4.8;
                sample.highLoadDeformation = 0.95;
            }
            heldSpiral.Step(sample, 1.0 / 120.0);
            if (frame == 10 * 120)
                assert(heldSpiral.Feedback() == ChallengeFeedback::EaseBrake);
        }
        assert(heldSpiral.Score() < controlledSpiral.Score() - 500.0);
    }
    {
        WingParameters parameters;
        FlightState clean;
        ControlInput rightBrake;
        rightBrake.rightBrake = 0.7;
        const auto turning = EvaluateSpanwiseCanopy(
            parameters, clean, rightBrake, parameters.trimCl,
            0.0, 72.0, 11.0, 0.0, false, 0.0, {});
        assert(std::abs(turning.loadAsymmetry) < 0.03);
        assert(std::abs(turning.rollMomentNm) < 20.0);

        FlightState folded;
        folded.rightCollapse = 0.75;
        const auto collapsed = EvaluateSpanwiseCanopy(
            parameters, folded, {}, parameters.trimCl,
            0.0, 72.0, 11.0, 0.0, false, 0.0, {});
        assert(collapsed.loadAsymmetry < -0.01);
        assert(collapsed.liftCoefficient < parameters.trimCl);
        assert(collapsed.dragCoefficient > turning.dragCoefficient * 0.5);

        FlightState rolling;
        rolling.angularVelocityBodyRadps.x = 0.7;
        const auto dynamicPanels = EvaluateSpanwiseCanopy(
            parameters, rolling, {}, parameters.trimCl,
            0.0, 72.0, 11.0, 0.0, false, 0.0, {});
        assert(std::abs(dynamicPanels.loadAsymmetry) > 0.08);
        assert(std::abs(dynamicPanels.rollMomentNm) > 100.0);

        Atmosphere leftLiftEdge;
        leftLiftEdge.leftWingWindDeltaMps = {0.0, 0.0, 2.4};
        const auto spatialPanels = EvaluateSpanwiseCanopy(
            parameters, clean, {}, parameters.trimCl,
            0.0, 72.0, 11.0, 0.0, false, 0.0, leftLiftEdge);
        assert(std::abs(spatialPanels.loadAsymmetry) > 0.015);
        assert(std::abs(spatialPanels.rollMomentNm) > 20.0);

        Atmosphere rightLiftEdge;
        rightLiftEdge.rightWingWindDeltaMps = {0.0, 0.0, 2.4};
        const auto mirroredSpatialPanels = EvaluateSpanwiseCanopy(
            parameters, clean, {}, parameters.trimCl,
            0.0, 72.0, 11.0, 0.0, false, 0.0, rightLiftEdge);
        assert(std::abs(spatialPanels.loadAsymmetry
            + mirroredSpatialPanels.loadAsymmetry) < 1e-12);
        assert(std::abs(spatialPanels.rollMomentNm
            + mirroredSpatialPanels.rollMomentNm) < 1e-9);
    }

    {
        using namespace Parapenting::Physics;
        // Both surveyed regions, so route endpoints can be held to the
        // published site elevations rather than to the analytic proxy. Loading
        // is additive; run from the repo root or one level below it.
        const auto LoadRegion = [](const char* name)
        {
            return TerrainModel::LoadHeightfieldAscii(
                       std::string("Content/Terrain/") + name)
                || TerrainModel::LoadHeightfieldAscii(
                       std::string("../Content/Terrain/") + name);
        };
        assert(LoadRegion("interlaken.asc"));
        assert(LoadRegion("grindelwald.asc"));
        assert(TerrainModel::LoadedRegionCount() == 2);
        assert(RouteProfileCount() == 10);
        const auto& primary = GetRouteProfile(RouteProfileId::AmisbuehlLehn);
        assert(RouteHorizontalDistanceM(primary) > 2300.0);
        assert(RouteHorizontalDistanceM(primary) < 2500.0);
        assert(RouteLaunchHeightM(primary) == 760.0);
        const Vec3 primaryLaunch = RouteLaunchLocalM(primary);
        const Vec3 primaryLanding = RouteLandingLocalM(primary);
        assert(std::hypot(primaryLaunch.x, primaryLaunch.y) < 0.01);
        assert(std::abs(primaryLanding.y) < 0.5);
        assert(std::abs(primaryLanding.x
                      - RouteHorizontalDistanceM(primary)) < 2.0);
        const auto& longRoute = GetRouteProfile(RouteProfileId::NiederhornHoehematte);
        assert(RouteHorizontalDistanceM(longRoute) > RouteHorizontalDistanceM(primary));
        assert(longRoute.advancedLanding);
        // Niederhorn launches well off the route axis. The sign is +Y since
        // the terrain frame was flipped to route-right; this expectation was
        // written before that and had no way to say so while the suite was
        // built with NDEBUG.
        assert(RouteLaunchLocalM(longRoute).y > 2500.0);
        const auto& firstGrund =
            GetRouteProfile(RouteProfileId::GrindelwaldFirstGrund);
        const auto& firstBodmi =
            GetRouteProfile(RouteProfileId::GrindelwaldFirstBodmi);
        assert(RouteHorizontalDistanceM(firstGrund) > 4500.0);
        assert(RouteHorizontalDistanceM(firstGrund) < 4700.0);
        assert(RouteLaunchHeightM(firstGrund) == 1173.0);
        assert(RouteLaunchHeightM(firstBodmi) == 994.0);
        const Vec3 firstLaunch = RouteLaunchLocalM(firstGrund);
        const Vec3 grundLanding = RouteLandingLocalM(firstGrund);
        const Vec3 bodmiLanding = RouteLandingLocalM(firstBodmi);
        // Grindelwald sits where Grindelwald is. These were an invented lane
        // at x = 0, y = -8500 until the valley got its own surveyed region:
        // the intra-valley geometry was right and the whole group was 20 km
        // from its real position, on analytic ground that put the Grund
        // landing field at 4683 m against a published 950 m.
        assert(firstLaunch.x > 5890.0 && firstLaunch.x < 5990.0);
        assert(firstLaunch.y < -17430.0 && firstLaunch.y > -17530.0);
        assert(grundLanding.x > 9910.0 && grundLanding.x < 10010.0);
        assert(grundLanding.y < -15230.0 && grundLanding.y > -15330.0);
        assert(bodmiLanding.x > 9030.0 && bodmiLanding.x < 9130.0);
        assert(bodmiLanding.y < -16370.0 && bodmiLanding.y > -16470.0);
        // On surveyed ground now, so the terrain can be held to the published
        // site elevations rather than to the proxy's arithmetic. Local z is
        // relative to the Lehn field at 565 m.
        assert(RouteFrame::IsInsideSurveyedBounds(firstLaunch.x, firstLaunch.y));
        assert(RouteFrame::IsInsideSurveyedBounds(
            grundLanding.x, grundLanding.y));
        assert(std::abs(firstGrund.landing.elevationM - 950.0) < 1.0);
        const double grundGroundM =
            TerrainModel::HeightM(grundLanding.x, grundLanding.y) + 565.0;
        assert(grundGroundM > 900.0 && grundGroundM < 1010.0);
        const double firstGroundM =
            TerrainModel::HeightM(firstLaunch.x, firstLaunch.y) + 565.0;
        assert(firstGroundM > 2050.0 && firstGroundM < 2200.0);
        assert(firstBodmi.advancedLanding);
        AtmosphereModel regionalAir;
        regionalAir.SetPreset(WeatherPresetId::ThermalDay);
        // Each surveyed region has its own convection triggers, so a route in
        // either valley flies in air with thermals in it. This asserted the
        // opposite until Grindelwald got real ground: the thermal field was a
        // single Interlaken set plus a lane offset, and the two Grindelwald
        // routes flew through air with no thermals at any time of day.
        //
        // Sampled over each valley's own trigger line, at a height where a
        // thermal has developed.
        const Atmosphere grindelwaldAir = regionalAir.Sample(
            {7900.0, -16300.0, 1800.0}, 90.0);
        assert(std::isfinite(grindelwaldAir.windWorldMps.x));
        assert(std::isfinite(grindelwaldAir.windWorldMps.z));
        assert(grindelwaldAir.thermalLiftMps > 0.1);
        const Atmosphere corridorAir = regionalAir.Sample(
            {1280.0, -760.0, 1800.0}, 90.0);
        assert(corridorAir.thermalLiftMps > 0.1);
        // Between the two valleys there is no surveyed ground and no trigger
        // set of its own, so the air there is still the Interlaken field seen
        // from far away - nothing. Nobody flies there; it is asserted so that
        // "thermals everywhere" cannot creep in as a fix.
        const Atmosphere betweenAir = regionalAir.Sample(
            {1280.0, -7830.0, 1800.0}, 90.0);
        assert(betweenAir.thermalLiftMps == 0.0);
        regionalAir.SetPreset(WeatherPresetId::FoehnRotor);
        // Sampled at a fixed height ABOVE GROUND, which is the only way two
        // places compare. Local z is metres relative to the Lehn field at
        // 565 m, so the fixed z = 260 these once used is 825 m MSL - open air
        // over the valley at y = -760, and thirty metres inside the hillside
        // at y = +760, where the ground is 1146 m. That buried sample returned
        // a rotor of 0.82 against the valley's 0.00, and the difference was
        // read as the field being sided against the surveyed geography. It was
        // not; it was one sample underground. The terrain frame has agreed
        // with the flight frame since the route-right flip, and
        // TerrainSurveyTests gates that directly.
        const auto RotorAboveGround = [&](double x, double y)
        {
            return regionalAir
                .Sample({x, y, TerrainModel::HeightM(x, y) + 260.0}, 0.0)
                .rotorStrength;
        };
        // Rotor comes from the terrain gradient the wind crosses, so it lives
        // on the flanks and not over flat ground - on both sides of the route,
        // rather than on a hardcoded one. Both valley walls at 1250 m out,
        // on the surveyed grid loaded above.
        const double westFlank = RotorAboveGround(760.0, 1250.0);
        const double eastFlank = RotorAboveGround(760.0, -1250.0);
        assert(westFlank > 0.0);
        assert(eastFlank > 0.0);
        // The authored rotor volumes move onto the region being flown, so a
        // foehn day at Grindelwald is a foehn day rather than smooth air. They
        // are placed in Interlaken coordinates and offset per region; without
        // that, every authored volume in every preset sat 20 km from anyone
        // flying there.
        double grindelwaldRotor = 0.0;
        for (double x = 6200.0; x <= 9800.0; x += 200.0)
        {
            for (double y = -17400.0; y <= -15000.0; y += 200.0)
                grindelwaldRotor =
                    std::max(grindelwaldRotor, RotorAboveGround(x, y));
        }
        assert(grindelwaldRotor > 0.1);
        std::size_t advancedLandings = 0;
        std::size_t routesOutsideRenderedTerrain = 0;
        for (std::size_t i = 0; i < RouteProfileCount(); ++i)
        {
            const auto& route = GetRouteProfileByIndex(i);
            const Vec3 launch = RouteLaunchLocalM(route);
            const Vec3 landing = RouteLandingLocalM(route);
            assert(std::isfinite(RouteHorizontalDistanceM(route)));
            assert(RouteHorizontalDistanceM(route) > 1500.0);
            assert(RouteLaunchHeightM(route) > 600.0);
            // Both ends must be inside the SAME rendered region: the renderer
            // draws one region at a time, so a route straddling two would fly
            // off the drawn mesh half way down. This used to count the two
            // Grindelwald routes, which sat on an invented lane at y = -8500,
            // outside the rendered extent entirely, on analytic terrain in air
            // with no thermals.
            const TerrainRenderLayout launchLayout =
                LayoutFor(launch.x, launch.y);
            const auto inside = [](const TerrainRenderLayout& layout,
                                   const Vec3& point)
            {
                return point.x >= layout.xMinM && point.x <= layout.xMaxM
                    && point.y >= layout.yMinM && point.y <= layout.yMaxM;
            };
            const bool insideRender = RouteFrame::IsInsideSurveyedBounds(
                                          launch.x, launch.y)
                && inside(launchLayout, launch)
                && inside(launchLayout, landing);
            if (!insideRender) ++routesOutsideRenderedTerrain;
            assert(route.sourceLabel != nullptr);
            assert(route.launchHazard != nullptr);
            assert(route.landingCircuit != nullptr);
            if (route.advancedLanding) ++advancedLandings;
            for (std::size_t j = i + 1; j < RouteProfileCount(); ++j)
            {
                const auto& other = GetRouteProfileByIndex(j);
                assert(std::string(route.launch.id) != other.launch.id
                    || std::string(route.landing.id) != other.landing.id);
            }
        }
        assert(advancedLandings == 5);
        // Every route, both ends, on surveyed ground inside one rendered
        // region. This counted 2 for as long as the Grindelwald pair sat on an
        // invented lane off the map; it is the number that closes that defect,
        // so it is asserted at zero rather than counted.
        assert(routesOutsideRenderedTerrain == 0);
        assert(AssessRouteWind(primary, 135.0, 3.0)
            == SiteWindAssessment::Suitable);
        assert(AssessRouteWind(primary, 315.0, 3.0)
            == SiteWindAssessment::MarginalDirection);
        assert(AssessRouteWind(primary, 135.0, 8.0)
            == SiteWindAssessment::TooStrong);
        assert(RouteWindDirectionErrorDegrees(primary, 125.0) == 10.0);
        assert(primary.sourceLabel != nullptr);
        assert(primary.launchHazard != nullptr);
        assert(primary.landingCircuit != nullptr);
    }
    {
        LandingRolloutModel rollout;
        LandingRolloutState clean;
        rollout.Begin(clean, {5.0, 0.0, -1.1}, 0.95);
        assert(clean.phase == LandingRolloutPhase::Running);
        assert(!clean.hardImpact);
        for (int frame = 0; frame < 8 * 120; ++frame)
            rollout.Step(clean, {}, 1.0 / 120.0);
        assert(clean.phase == LandingRolloutPhase::Settled);
        assert(clean.runoutDistanceM > 7.0);
        assert(clean.runoutDistanceM < 11.0);

        LandingRolloutState repeated;
        rollout.Begin(repeated, {5.0, 0.0, -1.1}, 0.95);
        for (int frame = 0; frame < 8 * 120; ++frame)
            rollout.Step(repeated, {}, 1.0 / 120.0);
        assert(repeated.runoutDistanceM == clean.runoutDistanceM);
        assert(repeated.canopyPressure == clean.canopyPressure);

        LandingRolloutState hard;
        rollout.Begin(hard, {13.0, 0.0, -1.5}, 0.9);
        assert(hard.phase == LandingRolloutPhase::Fallen);
        assert(hard.hardImpact);
        for (int frame = 0; frame < 4 * 120; ++frame)
            rollout.Step(hard, {}, 1.0 / 120.0);
        assert(hard.phase == LandingRolloutPhase::Settled);
        assert(hard.canopyPressure < 0.2);

        LandingRolloutState asymmetric;
        rollout.Begin(asymmetric, {8.5, 0.0, -1.0}, 0.95);
        LandingRolloutInput oneSided;
        oneSided.leftBrake = 1.0;
        rollout.Step(asymmetric, oneSided, 1.0 / 120.0);
        assert(asymmetric.phase == LandingRolloutPhase::Fallen);
        assert(asymmetric.hardImpact);
        assert(std::string(LandingRolloutPhaseName(
            LandingRolloutPhase::Running)) == "RUNNING OUT");
    }

    {
        GroundEffectInput lateFlare;
        lateFlare.pilotGroundClearanceM = 1.5;
        lateFlare.wingSpanM = 11.8;
        lateFlare.airspeedMps = 10.8;
        lateFlare.verticalSpeedMps = -1.6;
        lateFlare.symmetricBrake = 0.78;
        lateFlare.brakeApplicationRatePerS = 48.0;
        lateFlare.previousFlareEnergy = 0.9;
        const GroundEffectOutput flareModel =
            EvaluateGroundEffect(lateFlare);
        assert(flareModel.proximity > 0.3);
        assert(flareModel.proximity < 0.7);
        assert(flareModel.inducedDragReduction > 0.05);
        assert(flareModel.flareLiftCoefficient > 0.04);
        assert(flareModel.flareEnergy < lateFlare.previousFlareEnergy);

        GroundEffectInput highInput = lateFlare;
        highInput.pilotGroundClearanceM = 100.0;
        const GroundEffectOutput highModel =
            EvaluateGroundEffect(highInput);
        assert(highModel.proximity < 0.002);
        assert(highModel.inducedDragReduction < 0.001);
        assert(highModel.flareAuthority
            < flareModel.flareAuthority * 0.6);

        GroundEffectInput collapsedInput = lateFlare;
        collapsedInput.collapseFraction = 0.75;
        collapsedInput.canopyPressure = 0.45;
        const GroundEffectOutput collapsedFlare =
            EvaluateGroundEffect(collapsedInput);
        assert(collapsedFlare.flareAuthority
            < flareModel.flareAuthority * 0.3);

        GroundEffectInput early = lateFlare;
        early.pilotGroundClearanceM = 25.0;
        early.symmetricBrake = 0.82;
        early.brakeApplicationRatePerS = 48.0;
        GroundEffectOutput earlyState = EvaluateGroundEffect(early);
        for (int frame = 0; frame < 180; ++frame)
        {
            early.previousFlareEnergy = earlyState.flareEnergy;
            early.previousFlareLift = earlyState.flareLiftState;
            early.brakeApplicationRatePerS = 0.0;
            earlyState = EvaluateGroundEffect(early);
        }
        early.pilotGroundClearanceM = 1.5;
        const GroundEffectOutput earlyAtGround =
            EvaluateGroundEffect(early);
        assert(earlyState.flareEnergy < flareModel.flareEnergy);
        assert(earlyAtGround.flareAuthority
            < flareModel.flareAuthority * 0.35);

        ParagliderDynamics dynamics;
        FlightState state;
        const double initialAltitude = state.positionWorldM.z;
        StepFor(dynamics, state, {}, 3.0);
        assert(std::isfinite(state.positionWorldM.x));
        assert(std::isfinite(state.positionWorldM.z));
        assert(state.positionWorldM.x > 5.0);
        assert(state.positionWorldM.z < initialAltitude + 20.0);
        assert(dynamics.LastTelemetry().airspeedMps > 3.0);
    }

    {
        ParagliderDynamics dynamics;
        FlightState neutral;
        FlightState turning;
        StepFor(dynamics, neutral, {}, 2.0);
        StepFor(dynamics, turning, ControlInput{0.0, 0.65, 0.35}, 2.0);
        assert(std::abs(turning.angularVelocityBodyRadps.z)
             > std::abs(neutral.angularVelocityBodyRadps.z) + 0.05);
        assert(std::abs(turning.attitude.z) > 0.01);
    }
    {
        // Pilot-facing maneuver contract: left brake must turn and bank left,
        // its mirror must turn and bank right, and held moderate brake on the
        // full-size EPIC must settle rather than accumulate barrel rolls.
        const auto& epic =
            GetWingProfile(WingProfileId::Epic2MLResearch);
        ParagliderDynamics leftDynamics(epic.parameters);
        ParagliderDynamics rightDynamics(epic.parameters);
        FlightState left;
        FlightState right;
        ControlInput leftBrake;
        leftBrake.leftBrake = 0.62;
        ControlInput rightBrake;
        rightBrake.rightBrake = 0.62;
        StepFor(leftDynamics, left, leftBrake, 8.0);
        StepFor(rightDynamics, right, rightBrake, 8.0);
        assert(left.angularVelocityBodyRadps.z < -0.05);
        assert(right.angularVelocityBodyRadps.z > 0.05);
        assert(std::abs(left.angularVelocityBodyRadps.z) < 0.85);
        assert(std::abs(right.angularVelocityBodyRadps.z) < 0.85);
        assert(std::abs(BankAngle(left)) > 0.08);
        assert(std::abs(BankAngle(right)) > 0.08);
        assert(BankAngle(left) * BankAngle(right) < 0.0);
        assert(std::abs(BankAngle(left)) < 1.05);
        assert(std::abs(BankAngle(right)) < 1.05);
        assert(std::abs(BankAngle(left) + BankAngle(right)) < 0.08);
        assert(std::abs(HeadingAngle(left) + HeadingAngle(right)) < 0.12);
    }
    {
        // Brake commands represent the pilot's hand target; actual travel is
        // rate-limited, and release is quicker than application.
        ParagliderDynamics dynamics;
        FlightState state;
        ControlInput fullBrake;
        fullBrake.leftBrake = fullBrake.rightBrake = 1.0;
        dynamics.Step(state, fullBrake, Atmosphere{}, 1.0 / 120.0);
        assert(dynamics.LastTelemetry().effectiveLeftBrake > 0.0);
        assert(dynamics.LastTelemetry().effectiveLeftBrake < 0.05);
        StepFor(dynamics, state, fullBrake, 0.4);
        assert(dynamics.LastTelemetry().effectiveLeftBrake > 0.94);
        StepFor(dynamics, state, {}, 0.1);
        assert(dynamics.LastTelemetry().effectiveLeftBrake < 0.60);
    }
    {
        // Excessive sustained one-sided brake becomes a stalled/spinning
        // inside half, not ever-increasing roll authority.
        const auto& epic =
            GetWingProfile(WingProfileId::Epic2MLResearch);
        ParagliderDynamics dynamics(epic.parameters);
        FlightState state;
        ControlInput deepLeft;
        deepLeft.leftBrake = 1.0;
        deepLeft.rightBrake = 0.18;
        StepFor(dynamics, state, deepLeft, 1.0);
        StepFor(dynamics, state, deepLeft, 3.0);
        assert(std::abs(state.spin) > 0.35);
        assert(std::abs(BankAngle(state)) < 1.20);
        assert(std::abs(state.angularVelocityBodyRadps.x) < 0.16);
        assert(dynamics.LastTelemetry().dragCoefficient > 0.25);
        assert(dynamics.LastTelemetry().leftStalledSpan > 0.65);
        assert(dynamics.LastTelemetry().brakeRollAuthority < 0.38);
    }

    {
        ParagliderDynamics dynamics;
        FlightState state;
        state.velocityWorldMps = {2.0, 0.0, -8.0};
        StepFor(dynamics, state, ControlInput{1.0, 1.0, 0.0}, 0.4);
        assert(dynamics.LastTelemetry().stalled);
        assert(dynamics.LastTelemetry().dragCoefficient > 0.15);
    }

    {
        ParagliderDynamics dynamics;
        FlightState state;
        state.positionWorldM.z = 737.0;
        StepFor(dynamics, state, {}, 120.0);
        const auto& telemetry = dynamics.LastTelemetry();
        assert(telemetry.airspeedMps > 8.0 && telemetry.airspeedMps < 12.0);
        assert(state.velocityWorldMps.z < -0.5 && state.velocityWorldMps.z > -1.8);
        assert(std::abs(state.angularVelocityBodyRadps.x) < 0.01);
        assert(std::abs(state.angularVelocityBodyRadps.y) < 0.01);
        assert(std::abs(state.angularVelocityBodyRadps.z) < 0.01);
        assert(state.positionWorldM.x > 900.0);
        assert(state.positionWorldM.z > 500.0);
    }
    {
        // High dynamic pressure deforms the flexible canopy progressively
        // instead of allowing the rigid-strip force law to sit at the
        // structural 8 g qualification boundary.
        ParagliderDynamics dynamics;
        FlightState fast;
        fast.velocityWorldMps = {30.0, 0.0, -2.0};
        dynamics.Step(fast, {}, Atmosphere{}, 1.0 / 120.0);
        assert(dynamics.LastTelemetry().highLoadDeformation > 0.5);
        assert(dynamics.LastTelemetry().loadFactor < 8.0);
        assert(dynamics.LastTelemetry().dragCoefficient > 0.15);

        FlightState trim;
        dynamics.Step(trim, {}, Atmosphere{}, 1.0 / 120.0);
        assert(dynamics.LastTelemetry().highLoadDeformation == 0.0);
    }

    {
        AtmosphereModel atmosphere;
        double strongestRotor = 0.0;
        Vec3 rotorPoint{};
        for (double x = 100.0; x <= 1900.0; x += 100.0)
        {
            for (double y = -1200.0; y <= 1200.0; y += 100.0)
            {
                const Vec3 point{x, y, TerrainModel::HeightM(x, y) + 45.0};
                const Atmosphere sample = atmosphere.Sample(point, 3.0);
                if (sample.rotorStrength > strongestRotor)
                {
                    strongestRotor = sample.rotorStrength;
                    rotorPoint = point;
                }
            }
        }
        assert(strongestRotor > 0.25);
        assert(atmosphere.Sample(rotorPoint, 3.0).turbulence > 0.25);

        atmosphere.SetMode(WeatherMode::Chill);
        const Atmosphere chill = atmosphere.Sample(rotorPoint, 3.0);
        assert(chill.rotorStrength == 0.0);
        assert(chill.turbulence < 0.1);
    }

    {
        // Ground sources release finite-lived, deterministic thermal plumes
        // rather than permanent columns.
        AtmosphereModel atmosphere;
        atmosphere.SetMode(WeatherMode::Ridge);
        atmosphere.SetBaseWind({0.0, 0.0, 0.0});
        const double ground = TerrainModel::HeightM(1280.0, 330.0);
        const Vec3 core{1280.0, 330.0, ground + 400.0};
        double weakest = 100.0;
        double strongest = 0.0;
        double strongestTime = 0.0;
        for (double t = 0.0; t <= 280.0; t += 4.0)
        {
            const Atmosphere sample = atmosphere.Sample(core, t);
            weakest = std::min(weakest, sample.thermalLiftMps);
            if (sample.thermalLiftMps > strongest)
            {
                strongest = sample.thermalLiftMps;
                strongestTime = t;
            }
        }
        assert(strongest > 0.9);
        assert(weakest < strongest * 0.55);

        // An active plume has coherent low-level convergence, edge
        // circulation, and explicit lifecycle data. Opposite sides must feed
        // inward rather than behave like unrelated vertical gust samples.
        const Atmosphere centre = atmosphere.Sample(core, strongestTime);
        const Atmosphere east = atmosphere.Sample(
            {core.x + 120.0, core.y, core.z}, strongestTime);
        const Atmosphere west = atmosphere.Sample(
            {core.x - 120.0, core.y, core.z}, strongestTime);
        assert(centre.thermalCoreStrength > 0.8);
        assert(centre.thermalLifecycle > 0.65);
        assert(east.windWorldMps.x < west.windWorldMps.x - 0.08);
        assert(std::abs(east.windWorldMps.y - west.windWorldMps.y) > 0.01);

        // The inversion/cloud-base cap must shut the dry thermal down instead
        // of allowing an infinite lift column.
        const Atmosphere aboveCap = atmosphere.Sample(
            {core.x, core.y, 2240.0}, strongestTime);
        assert(aboveCap.thermalLiftMps < centre.thermalLiftMps * 0.25);
        assert(aboveCap.cloudBaseClearanceM < 0.0);
    }

    {
        // The terrain boundary layer slows wind near the surface and recovers
        // toward the model wind aloft.
        AtmosphereModel atmosphere;
        WeatherSnapshot snapshot{
            "test", "BOUNDARY LAYER", 0.0, 0.0, 8.0, 8.0,
            1000.0, 5
        };
        atmosphere.ApplySnapshot(snapshot);
        atmosphere.SetMode(WeatherMode::Ridge);
        const double ground = TerrainModel::HeightM(300.0, 0.0);
        const Atmosphere low = atmosphere.Sample(
            {300.0, 0.0, ground + 5.0}, 0.0);
        const Atmosphere high = atmosphere.Sample(
            {300.0, 0.0, ground + 500.0}, 0.0);
        assert(std::hypot(high.windWorldMps.x, high.windWorldMps.y)
             > std::hypot(low.windWorldMps.x, low.windWorldMps.y) + 2.0);
    }

    {
        ParagliderDynamics dynamics;
        FlightState state;
        Atmosphere rotor;
        rotor.windWorldMps = {1.5, 0.0, 0.0};
        rotor.rotorStrength = 0.9;
        rotor.turbulence = 0.9;
        rotor.lateralGust = 1.0;
        constexpr double dt = 1.0 / 120.0;
        for (int i = 0; i < 1200; ++i)
            dynamics.Step(state, {}, rotor, dt);
        assert(state.leftCollapse > 0.2);
        assert(state.frontalCollapse > 0.1);
        assert(dynamics.LastTelemetry().leftCollapse == state.leftCollapse);

        Atmosphere smooth;
        ControlInput pump;
        pump.leftBrake = 0.55;
        const double collapsed = state.leftCollapse;
        for (int i = 0; i < 1200; ++i)
            dynamics.Step(state, pump, smooth, dt);
        assert(state.leftCollapse < collapsed);
        assert(state.frontalCollapse < 0.1);
        assert(std::isfinite(state.recoverySurge));
    }
    {
        // A half-wing crossing a sharper vertical-flow boundary must unload
        // on that side. Centre-point gust is intentionally zero here so the
        // result comes solely from spanwise sampling.
        constexpr double dt = 1.0 / 120.0;
        ParagliderDynamics leftDynamics;
        FlightState leftState;
        for (int frame = 0; frame < 120; ++frame)
            leftDynamics.Step(leftState, {}, Atmosphere{}, dt);

        Atmosphere leftShear;
        leftShear.turbulence = 0.82;
        leftShear.rotorStrength = 0.58;
        leftShear.leftWingWindDeltaMps = {0.2, 0.0, 2.6};
        leftShear.rightWingWindDeltaMps = {0.0, 0.0, 0.05};
        leftShear.spanwiseAirflowShearMps = 2.55;
        for (int frame = 0; frame < 120; ++frame)
            leftDynamics.Step(leftState, {}, leftShear, dt);
        assert(leftState.leftCollapse > leftState.rightCollapse + 0.04);
        assert(leftDynamics.LastTelemetry().spanwiseAirflowShearMps == 2.55);
        assert(leftDynamics.LastTelemetry().leftAirflowDisturbance
            > leftDynamics.LastTelemetry().rightAirflowDisturbance);

        // Mirroring the sampled flow must mirror the physical response.
        ParagliderDynamics rightDynamics;
        FlightState rightState;
        for (int frame = 0; frame < 120; ++frame)
            rightDynamics.Step(rightState, {}, Atmosphere{}, dt);
        Atmosphere rightShear = leftShear;
        rightShear.leftWingWindDeltaMps = {0.0, 0.0, 0.05};
        rightShear.rightWingWindDeltaMps = {0.2, 0.0, 2.6};
        for (int frame = 0; frame < 120; ++frame)
            rightDynamics.Step(rightState, {}, rightShear, dt);
        assert(rightState.rightCollapse > rightState.leftCollapse + 0.04);
        assert(std::abs(leftState.leftCollapse - rightState.rightCollapse)
            < 1e-9);

        HapticFeedbackModel haptics;
        Telemetry leftBoundary;
        leftBoundary.canopyPressure = 1.0;
        leftBoundary.leftAirflowDisturbance = 0.9;
        leftBoundary.rightAirflowDisturbance = 0.05;
        const HapticOutput leftTexture = haptics.Evaluate(leftBoundary, 1.0);
        assert(leftTexture.left > leftTexture.right + 0.03);
    }
    {
        // Smooth, pressurized flight must not spontaneously fold, while a
        // rapid loss of dynamic pressure in disturbed air should produce a
        // measurable aerodynamic-unloading event even without an authored
        // rotor volume.
        constexpr double dt = 1.0 / 120.0;
        ParagliderDynamics stableDynamics;
        FlightState stable;
        for (int frame = 0; frame < 8 * 120; ++frame)
            stableDynamics.Step(stable, {}, Atmosphere{}, dt);
        assert(stable.leftCollapse < 0.01);
        assert(stable.rightCollapse < 0.01);
        assert(stable.frontalCollapse < 0.01);

        ParagliderDynamics unloadingDynamics;
        FlightState unloading;
        for (int frame = 0; frame < 120; ++frame)
            unloadingDynamics.Step(unloading, {}, Atmosphere{}, dt);
        Atmosphere pressureLoss;
        pressureLoss.windWorldMps = {
            unloading.velocityWorldMps.x - 1.5,
            unloading.velocityWorldMps.y,
            unloading.velocityWorldMps.z};
        pressureLoss.turbulence = 0.72;
        pressureLoss.lateralGust = 0.8;
        for (int frame = 0; frame < 45; ++frame)
            unloadingDynamics.Step(unloading, {}, pressureLoss, dt);
        assert(unloadingDynamics.LastTelemetry().aerodynamicUnloading > 0.1);
        assert(unloading.frontalCollapse > 0.015);
        assert(unloading.leftCollapse > unloading.rightCollapse);
    }
    {
        // A deliberate pulse-and-release sequence must outperform simply
        // parking a deep brake on the collapsed side.
        constexpr double dt = 1.0 / 120.0;
        ParagliderDynamics pumpingDynamics;
        ParagliderDynamics heldDynamics;
        FlightState pumping;
        FlightState held;
        pumping.leftCollapse = held.leftCollapse = 0.68;
        pumping.canopyPressure = held.canopyPressure = 0.52;
        for (int frame = 0; frame < 4 * 120; ++frame)
        {
            ControlInput pulse;
            const int phase = frame % 72;
            pulse.leftBrake = phase < 24 ? 0.62 : 0.0;
            pulse.rightBrake = 0.18;
            pulse.weightShift = 0.35;
            pumpingDynamics.Step(pumping, pulse, Atmosphere{}, dt);

            ControlInput heldBrake;
            heldBrake.leftBrake = 0.82;
            heldDynamics.Step(held, heldBrake, Atmosphere{}, dt);
        }
        assert(pumping.leftCollapse + 0.04 < held.leftCollapse);
        assert(pumpingDynamics.LastTelemetry().leftReinflationAuthority > 0.0);
        assert(heldDynamics.LastTelemetry().leftReinflationAuthority > 0.0);
    }
    {
        // Checking the forward surge only after pressure returns should lower
        // the peak without delaying the initial canopy reopening.
        constexpr double dt = 1.0 / 120.0;
        ParagliderDynamics checkedDynamics;
        ParagliderDynamics handsUpDynamics;
        FlightState checked;
        FlightState handsUp;
        checked.leftCollapse = handsUp.leftCollapse = 0.62;
        checked.rightCollapse = handsUp.rightCollapse = 0.62;
        checked.previousMeanCollapse = handsUp.previousMeanCollapse = 0.62;
        checked.canopyPressure = handsUp.canopyPressure = 0.48;
        double checkedPeak = 0.0;
        double handsUpPeak = 0.0;
        for (int frame = 0; frame < 5 * 120; ++frame)
        {
            ControlInput check;
            if (checked.recoverySurge > 0.012
                && checked.canopyPressure > 0.62)
            {
                check.leftBrake = 0.42;
                check.rightBrake = 0.42;
            }
            checkedDynamics.Step(checked, check, Atmosphere{}, dt);
            handsUpDynamics.Step(handsUp, {}, Atmosphere{}, dt);
            checkedPeak = std::max(checkedPeak, checked.recoverySurge);
            handsUpPeak = std::max(handsUpPeak, handsUp.recoverySurge);
        }
        assert(checkedPeak < handsUpPeak);
        assert(checkedDynamics.LastTelemetry().surgeContainment >= 0.0);
    }

    {
        ParagliderDynamics dynamics;
        FlightState state;
        ControlInput shifted;
        shifted.weightShift = 1.0;
        StepFor(dynamics, state, shifted, 2.0);
        assert(state.harnessRollRad > 0.15);
        assert(state.harnessRollRad < 0.30);
        assert(dynamics.LastTelemetry().lateralLineLoadImbalance > 0.12);
        assert(dynamics.LastTelemetry().rightLineTensionN[0]
            > dynamics.LastTelemetry().leftLineTensionN[0]);
        assert(dynamics.LastTelemetry().rightCarabinerLateralCm > 20.0);

        StepFor(dynamics, state, {}, 3.0);
        assert(std::abs(state.harnessRollRad) < 0.04);
        assert(std::abs(
            dynamics.LastTelemetry().lateralLineLoadImbalance) < 0.15);
    }
    {
        // Weight shift alone must create a useful bank and curve the flight
        // path; it is not merely a pilot animation.
        const auto& epic =
            GetWingProfile(WingProfileId::Epic2MLResearch);
        ParagliderDynamics neutralDynamics(epic.parameters);
        ParagliderDynamics shiftedDynamics(epic.parameters);
        FlightState neutral;
        FlightState shifted;
        ControlInput rightShift;
        rightShift.weightShift = 1.0;
        StepFor(neutralDynamics, neutral, {}, 6.0);
        StepFor(shiftedDynamics, shifted, rightShift, 6.0);
        assert(std::abs(BankAngle(shifted)) > 0.22);
        assert(std::abs(HeadingAngle(shifted)) > 0.18);
        assert(std::abs(shifted.positionWorldM.y) > 4.0);
        assert(std::abs(shifted.positionWorldM.y)
            > std::abs(neutral.positionWorldM.y) + 3.0);
        assert(std::abs(
            shiftedDynamics.LastTelemetry().coordinatedYawTargetRadps)
            > 0.15);
        // The hang angle is statics: roll moment over the pendulum stiffness
        // of the suspended load. It is properly small - under a degree - and
        // it used to read 0.78 rad only because the same moment was counted
        // twice and pinned it against its clamp.
        const auto& shiftedTelemetry = shiftedDynamics.LastTelemetry();
        assert(std::abs(shiftedTelemetry.canopyRelativeRollRad) > 0.002);
        assert(std::abs(shiftedTelemetry.canopyRelativeRollRad) < 0.10);

        // Direction, stated rather than absolute-valued. Shifting right must
        // load the right carabiner, drop the right tip and turn right. Every
        // check above this one takes std::abs, which is how a weight shift
        // that banked right and tracked left survived for so long.
        assert(shiftedTelemetry.rightCarabinerLoadN
            > shiftedTelemetry.leftCarabinerLoadN * 1.2);
        assert(shiftedTelemetry.carabinerLoadAsymmetry > 0.05);
        assert(shiftedTelemetry.pilotCgOffsetM > 0.02);
        // Right tip down is a negative span-vector z under the measured
        // convention: a positive rotation about body +X raises the right tip.
        assert(shifted.attitude.Rotate({0.0, 1.0, 0.0}).z < -0.10);
        // And the flight path curves toward +Y, the same side.
        assert(shifted.velocityWorldMps.y > 1.0);
        assert(shifted.positionWorldM.y > 4.0);
        assert(shiftedDynamics.LastTelemetry()
            .suspensionControlTransmission > 0.55);
    }
    {
        // End-to-end direction contract. Negative control is the left input;
        // its displacement must project onto body-left, not merely produce a
        // nonzero bank with an accidentally mirrored world trajectory.
        ParagliderDynamics leftDynamics;
        ParagliderDynamics rightDynamics;
        FlightState left;
        FlightState right;
        const Vec3 initialForward = left.attitude.Rotate({1.0, 0.0, 0.0});
        const Vec3 initialLeft = left.attitude.Rotate({0.0, -1.0, 0.0});
        const Vec3 origin = left.positionWorldM;
        ControlInput leftInput;
        leftInput.weightShift = -1.0;
        ControlInput rightInput;
        rightInput.weightShift = 1.0;
        StepFor(leftDynamics, left, leftInput, 3.0);
        StepFor(rightDynamics, right, rightInput, 3.0);
        const Vec3 leftDisplacement = left.positionWorldM - origin;
        const Vec3 rightDisplacement = right.positionWorldM - origin;
        assert(Dot(leftDisplacement, initialForward) > 15.0);
        assert(Dot(leftDisplacement, initialLeft) > 0.5);
        assert(Dot(rightDisplacement, initialLeft) < -0.5);
        assert(leftDynamics.LastTelemetry().coordinatedYawTargetRadps < 0.0);
        assert(rightDynamics.LastTelemetry().coordinatedYawTargetRadps > 0.0);
    }
    {
        // Weight shift cannot torque an unloaded canopy through slack lines.
        ParagliderDynamics loadedDynamics;
        ParagliderDynamics unloadedDynamics;
        FlightState loaded;
        FlightState unloaded;
        unloaded.velocityWorldMps = {2.5, 0.0, -4.0};
        unloaded.canopyPressure = 0.18;
        unloaded.frontalCollapse = 0.68;
        ControlInput shift;
        shift.weightShift = 1.0;
        StepFor(loadedDynamics, loaded, shift, 0.75);
        StepFor(unloadedDynamics, unloaded, shift, 0.75);
        assert(unloadedDynamics.LastTelemetry()
            .suspensionControlTransmission
            < loadedDynamics.LastTelemetry()
                .suspensionControlTransmission);
        assert(std::abs(unloadedDynamics.LastTelemetry()
            .canopyRelativeRollRad)
            < std::abs(loadedDynamics.LastTelemetry()
                .canopyRelativeRollRad));
    }
    {
        // THE PENDULUM, both ends of it, as a sequence rather than a state.
        //
        // A firm brake pulse and a release is the manoeuvre every pilot knows:
        // the wing goes back, the incidence rises toward the stall, and on the
        // release it surges ahead and its incidence falls toward a frontal.
        // Before the swing was connected to the aerodynamics the first half of
        // that was invisible to the model - the wing went 32 degrees behind
        // the pilot while the incidence it flew on went DOWN. This asserts the
        // couplings that make the two ends different things.
        const auto& epic =
            GetWingProfile(WingProfileId::Epic2MLResearch);
        ParagliderDynamics pendulum(epic.parameters);
        FlightState flight;
        flight.velocityWorldMps = {10.8, 0.0, -1.1};
        constexpr double dt = 1.0 / 120.0;
        for (int frame = 0; frame < 120 * 30; ++frame)
            pendulum.Step(flight, ControlInput{}, Atmosphere{}, dt);
        const double trimSwing =
            pendulum.LastTelemetry().canopyRelativePitchRad;
        const double trimAlpha = pendulum.LastTelemetry().angleOfAttackRad;
        // The wing hangs where it hangs, and it is not swinging.
        assert(std::abs(trimSwing) < 0.02);

        double deepestBack = 0.0, alphaAtBack = trimAlpha;
        double furthestForward = 0.0, alphaAtForward = trimAlpha;
        double swingReversals = 0.0;
        double previousRate = 0.0;
        for (int frame = 0; frame < 120 * 12; ++frame)
        {
            const double t = frame * dt;
            ControlInput input{};
            if (t >= 1.0 && t < 3.0)
                input.leftBrake = input.rightBrake = 0.6;
            pendulum.Step(flight, input, Atmosphere{}, dt);
            const Telemetry& tel = pendulum.LastTelemetry();
            if (tel.canopyRelativePitchRad > deepestBack)
            {
                deepestBack = tel.canopyRelativePitchRad;
                alphaAtBack = tel.angleOfAttackRad;
            }
            if (t > 3.0 && tel.canopyRelativePitchRad < furthestForward)
            {
                furthestForward = tel.canopyRelativePitchRad;
                alphaAtForward = tel.angleOfAttackRad;
            }
            if (previousRate != 0.0
                && (tel.canopyRelativePitchRateRadps < 0.0)
                    != (previousRate < 0.0))
                swingReversals += 1.0;
            previousRate = tel.canopyRelativePitchRateRadps;
        }
        // BACK: the wing ends up behind the pilot, and its incidence is UP -
        // that is the half that stalls wings, and the half this model did not
        // have. Past the stall angle it is a stall, which is what a firm
        // two-second pull does.
        assert(deepestBack > 0.15);
        assert(alphaAtBack > trimAlpha);
        assert(alphaAtBack - epic.parameters.trimAngleOfAttackRad
            > epic.parameters.stallAngleRad * 0.5);
        // FRONT: on the release it swings through and ahead, and there its
        // incidence is DOWN - which is the half that collapses wings, and why
        // a surge is checked with brake rather than watched.
        assert(furthestForward < -0.05);
        assert(alphaAtForward < trimAlpha);
        // And it is a PENDULUM: it reverses several times rather than moving
        // once and staying there. Two reversals would be one swing out and
        // one back; a wing that only creeps to a new position and holds it is
        // what the near-critical damping used to give.
        assert(swingReversals >= 4.0);
    }
    {
        // A fast symmetric brake application converts airspeed to height
        // before drag and held brake take over.
        const auto& epic =
            GetWingProfile(WingProfileId::Epic2MLResearch);
        ParagliderDynamics brakeDynamics(epic.parameters);
        FlightState braking;
        braking.velocityWorldMps = {16.0, 0.0, -1.0};
        ControlInput hardBrake;
        hardBrake.leftBrake = hardBrake.rightBrake = 0.78;
        double brakePeakVz = -100.0;
        double brakePeakAltitude = braking.positionWorldM.z;
        double brakePeakZoomEnergy = 0.0;
        double brakeMinimumAirspeed = 100.0;
        double brakePeakEnergyJ = 0.0;
        double canopyPitchPeak = 0.0;
        constexpr double dt = 1.0 / 120.0;
        for (int frame = 0; frame < 360; ++frame)
        {
            brakeDynamics.Step(braking, hardBrake, Atmosphere{}, dt);
            brakePeakVz = std::max(
                brakePeakVz, braking.velocityWorldMps.z);
            brakePeakAltitude = std::max(
                brakePeakAltitude, braking.positionWorldM.z);
            brakePeakZoomEnergy = std::max(
                brakePeakZoomEnergy,
                brakeDynamics.LastTelemetry().brakeZoomEnergy);
            brakeMinimumAirspeed = std::min(
                brakeMinimumAirspeed,
                brakeDynamics.LastTelemetry().airspeedMps);
            brakePeakEnergyJ = std::max(
                brakePeakEnergyJ,
                brakeDynamics.LastTelemetry().brakeZoomEnergyJ);
            canopyPitchPeak = std::max(
                canopyPitchPeak,
                brakeDynamics.LastTelemetry().canopyRelativePitchRad);
        }
        assert(brakePeakVz > 0.0);
        assert(brakePeakAltitude > 1.0);
        assert(braking.positionWorldM.z < brakePeakAltitude - 0.5);
        assert(brakeMinimumAirspeed < 14.5);
        assert(brakePeakZoomEnergy > 0.12);
        assert(brakePeakEnergyJ > 500.0);
        const double initialExcessKineticEnergyJ =
            0.5 * epic.parameters.allUpMassKg
            * (16.0 * 16.0
                - std::pow(39.0 / 3.6, 2.0));
        const double peakPotentialEnergyJ =
            epic.parameters.allUpMassKg * 9.80665
            * brakePeakAltitude;
        assert(peakPotentialEnergyJ
            < initialExcessKineticEnergyJ * 1.05);
        assert(canopyPitchPeak > 0.08);
    }

    {
        ParagliderDynamics trimDynamics;
        ParagliderDynamics acceleratedDynamics;
        FlightState trim;
        FlightState accelerated;
        trim.positionWorldM.z = 1000.0;
        accelerated.positionWorldM.z = 1000.0;
        ControlInput speedbar;
        speedbar.accelerator = 1.0;
        StepFor(trimDynamics, trim, {}, 20.0);
        StepFor(acceleratedDynamics, accelerated, speedbar, 20.0);
        const auto& speedTelemetry = acceleratedDynamics.LastTelemetry();
        assert(accelerated.acceleratorTravel > 0.95);
        assert(speedTelemetry.accelerator > 0.95);
        assert(speedTelemetry.airspeedMps
            > trimDynamics.LastTelemetry().airspeedMps + 0.2);
        assert(speedTelemetry.aRiserLoad > speedTelemetry.dRiserLoad);
        const double riserSum = speedTelemetry.aRiserLoad
            + speedTelemetry.bRiserLoad + speedTelemetry.cRiserLoad
            + speedTelemetry.dRiserLoad;
        assert(std::abs(riserSum - 1.0) < 1e-9);
        assert(speedTelemetry.lineLoadTotalN > 100.0);
    }

    {
        // Replaying an identical 120 Hz control stream through the
        // engine-independent solver must reproduce the exact flight state.
        ParagliderDynamics firstDynamics;
        ParagliderDynamics replayDynamics;
        FlightState first;
        FlightState replay;
        constexpr double dt = 1.0 / 120.0;
        for (int frame = 0; frame < 2400; ++frame)
        {
            ControlInput input;
            input.leftBrake = frame > 300 && frame < 900 ? 0.35 : 0.0;
            input.rightBrake = frame > 600 && frame < 1200 ? 0.55 : 0.0;
            input.weightShift = frame > 1000 && frame < 1700 ? -0.4 : 0.0;
            input.accelerator = frame > 1700 ? 0.8 : 0.0;
            firstDynamics.Step(first, input, Atmosphere{}, dt);
            replayDynamics.Step(replay, input, Atmosphere{}, dt);
        }
        assert(first.positionWorldM.x == replay.positionWorldM.x);
        assert(first.positionWorldM.y == replay.positionWorldM.y);
        assert(first.positionWorldM.z == replay.positionWorldM.z);
        assert(first.attitude.w == replay.attitude.w);
        assert(first.acceleratorTravel == replay.acceleratorTravel);
    }

    {
        // Equipment extremes must remain finite and retain distinct pendulum
        // behavior across every harness profile.
        const auto& wing = GetWingProfile(WingProfileId::Epic2MLResearch);
        for (const auto& harness : GetHarnessProfiles())
        {
            for (const double pilotMass : {55.0, 85.0, 115.0})
            {
                EquipmentSetup setup;
                setup.pilotMassKg = pilotMass;
                setup.harness = harness.id;
                setup.ballastKg = pilotMass > 100.0 ? 10.0 : 0.0;
                ParagliderDynamics dynamics(ApplyEquipmentSetup(
                    wing.parameters, setup, wing.wingMassKg));
                dynamics.SetHarnessParameters(HarnessParametersFor(setup));
                FlightState state;
                state.positionWorldM.z = 1500.0;
                ControlInput input;
                input.weightShift = 0.65;
                input.rightBrake = 0.25;
                StepFor(dynamics, state, input, 30.0);
                const auto& telemetry = dynamics.LastTelemetry();
                assert(std::isfinite(state.positionWorldM.x));
                assert(std::isfinite(state.positionWorldM.z));
                assert(telemetry.allUpMassKg >= 55.0);
                assert(telemetry.allUpMassKg <= 160.0);
                assert(telemetry.wingLoadingKgM2 > 2.0);
                assert(telemetry.wingLoadingKgM2 < 6.0);
                assert(telemetry.harnessDragN > 1.0);
            }
        }
    }

    {
        ParagliderDynamics dynamics;
        FlightState state;
        Atmosphere nearGround;
        nearGround.groundClearanceM = 2.0;
        ControlInput flare;
        flare.leftBrake = 0.8;
        flare.rightBrake = 0.8;
        double nearFlarePeak = 0.0;
        double nearFlareAuthorityPeak = 0.0;
        for (int frame = 0; frame < 30; ++frame)
        {
            dynamics.Step(state, flare, nearGround, 1.0 / 120.0);
            nearFlarePeak = std::max(
                nearFlarePeak, dynamics.LastTelemetry().flareBoost);
            nearFlareAuthorityPeak = std::max(
                nearFlareAuthorityPeak,
                dynamics.LastTelemetry().flareAuthority);
        }
        assert(dynamics.LastTelemetry().groundEffect > 0.25);
        assert(nearFlarePeak > 0.008);
        assert(dynamics.LastTelemetry().flareEnergy < 0.8);
        assert(nearFlareAuthorityPeak > 0.02);

        ParagliderDynamics highDynamics;
        FlightState highState;
        Atmosphere high;
        high.groundClearanceM = 100.0;
        double highFlarePeak = 0.0;
        for (int frame = 0; frame < 30; ++frame)
        {
            highDynamics.Step(highState, flare, high, 1.0 / 120.0);
            highFlarePeak = std::max(
                highFlarePeak, highDynamics.LastTelemetry().flareBoost);
        }
        assert(highDynamics.LastTelemetry().groundEffect < 0.002);
        assert(highFlarePeak < nearFlarePeak * 0.6);

        AtmosphereModel calm;
        calm.SetMode(WeatherMode::Chill);
        const double calmGround = TerrainModel::HeightM(400.0, 120.0);
        const auto calmSample =
            calm.Sample({400.0, 120.0, calmGround + 2.0}, 0.0);
        assert(std::abs(calmSample.groundClearanceM - 2.0) < 0.01);
    }

    {
        AtmosphereModel atmosphere;
        atmosphere.SetMode(WeatherMode::Ridge);
        atmosphere.SetBaseWind({0.0, 0.0, 0.0});
        const double ground = TerrainModel::HeightM(1280.0, 330.0);
        const double altitude = ground + 400.0;
        double strongestCore = 0.0;
        double strongestRing = 0.0;
        double coreVerticalAtPeak = -100.0;
        double ringVerticalAtPeak = 100.0;
        for (double time = 0.0; time <= 280.0; time += 2.0)
        {
            const Atmosphere core =
                atmosphere.Sample({1280.0, 330.0, altitude}, time);
            const Atmosphere ring = atmosphere.Sample(
                {1280.0 + 170.0 * 1.7, 330.0, altitude}, time);
            if (core.thermalLiftMps > strongestCore)
            {
                strongestCore = core.thermalLiftMps;
                coreVerticalAtPeak = core.windWorldMps.z;
                ringVerticalAtPeak = ring.windWorldMps.z;
            }
            strongestRing = std::max(strongestRing, ring.sinkRingMps);
        }
        assert(strongestCore > 0.8);
        assert(coreVerticalAtPeak > ringVerticalAtPeak);
        assert(strongestRing > 0.1);
    }

    {
        const auto& profiles = GetWingProfiles();
        assert(profiles.size() == WingProfileCount);
        assert(profiles[0].parameters.areaM2 > profiles[4].parameters.areaM2);
        assert(profiles[0].parameters.zeroLiftDrag
             > profiles[4].parameters.zeroLiftDrag);
        assert(profiles[0].parameters.brakeRollMoment
             < profiles[4].parameters.brakeRollMoment);
        for (const auto& profile : profiles)
        {
            assert(IsValidBrakePolar(
                profile.parameters.brakeLiftCurve,
                profile.parameters.brakeDragCurve));
            assert(profile.targetTrimSpeedKmh > 30.0);
            assert(profile.targetTopSpeedKmh > profile.targetTrimSpeedKmh);
            assert(profile.targetMinimumSinkMps > 0.5);
            assert(profile.targetBestGlide > 7.0);
            const auto trimPoint =
                EstimateSteadyPolarPoint(profile.parameters, 0.0);
            const auto quarterPoint =
                EstimateSteadyPolarPoint(profile.parameters, 0.25);
            const auto halfPoint =
                EstimateSteadyPolarPoint(profile.parameters, 0.5);
            const auto deepPoint =
                EstimateSteadyPolarPoint(profile.parameters, 1.0);
            assert(trimPoint.airspeedMps > 8.0);
            assert(trimPoint.glideRatio > 7.0);
            assert(quarterPoint.airspeedMps < trimPoint.airspeedMps);
            assert(halfPoint.airspeedMps < quarterPoint.airspeedMps);
            assert(deepPoint.dragCoefficient
                > halfPoint.dragCoefficient * 2.0);
            assert(deepPoint.glideRatio < trimPoint.glideRatio * 0.45);
        }
        const auto quarterPolar = SampleBrakePolar(
            profiles[2].parameters.brakeLiftCurve,
            profiles[2].parameters.brakeDragCurve, 0.25);
        const auto deepPolar = SampleBrakePolar(
            profiles[2].parameters.brakeLiftCurve,
            profiles[2].parameters.brakeDragCurve, 0.90);
        assert(quarterPolar.liftDelta > 0.0);
        assert(deepPolar.dragDelta > quarterPolar.dragDelta * 8.0);
        const auto epicTrim =
            EstimateSteadyPolarPoint(profiles[2].parameters, 0.0);
        assert(std::abs(epicTrim.airspeedMps * 3.6
            - profiles[2].targetTrimSpeedKmh) < 4.0);
        // The analytic polar is canopy-only. Installed harness and projected
        // line drag are verified by the integrated still-air test below.
        assert(epicTrim.glideRatio > profiles[2].targetBestGlide);

        const auto& epsilon = GetWingProfile(
            WingProfileId::AdvanceEpsilonDls28Research);
        assert(epsilon.parameters.areaM2 == 27.6);
        assert(epsilon.wingMassKg == 4.35);
        assert(epsilon.recommendedAllUpMinKg == 91.0);
        assert(epsilon.recommendedAllUpMaxKg == 118.0);
        assert(epsilon.sourceLabel != nullptr);
        assert(epsilon.dataPackagePath != nullptr);
        assert(std::string(epsilon.sourceLabel).find("ADVANCE")
            != std::string::npos);
        assert(std::string(epsilon.dataPackagePath).find(
            "advance-epsilon-dls-28-research.json") != std::string::npos);
        const auto epsilonTrim =
            EstimateSteadyPolarPoint(epsilon.parameters, 0.0);
        const auto epsilonHalf =
            EstimateSteadyPolarPoint(epsilon.parameters, 0.5);
        const auto epsilonDeep =
            EstimateSteadyPolarPoint(epsilon.parameters, 0.9);
        assert(std::abs(epsilonTrim.airspeedMps * 3.6
            - epsilon.targetTrimSpeedKmh) < 4.0);
        assert(std::abs(epsilonTrim.glideRatio
            - epsilon.targetBestGlide) < 1.0);
        assert(epsilonHalf.airspeedMps < epsilonTrim.airspeedMps);
        assert(epsilonDeep.dragCoefficient
            > epsilonHalf.dragCoefficient * 2.0);
        EquipmentSetup epsilonSetup;
        const double epsilonAllUp = AllUpMassKg(
            epsilonSetup, epsilon.wingMassKg);
        assert(epsilonAllUp >= 99.0 && epsilonAllUp <= 113.0);
        assert(epsilonAllUp >= epsilon.recommendedAllUpMinKg);
        assert(epsilonAllUp <= epsilon.recommendedAllUpMaxKg);

        const auto& defaultWing =
            GetWingProfile(WingProfileId::Epic2MLResearch);
        EquipmentSetup defaultSetup;
        const auto equipped = ApplyEquipmentSetup(
            defaultWing.parameters, defaultSetup, defaultWing.wingMassKg);
        assert(std::abs(equipped.allUpMassKg - 104.0) < 0.01);
        assert(std::abs(WingLoadingKgM2(
            defaultSetup, defaultWing.wingMassKg,
            defaultWing.parameters.areaM2) - 104.0 / 27.0) < 1e-9);
        EquipmentSetup heavy = defaultSetup;
        heavy.pilotMassKg = 100.0;
        heavy.ballastKg = 10.0;
        assert(AllUpMassKg(heavy, defaultWing.wingMassKg)
            > equipped.allUpMassKg + 20.0);
        const auto pod = GetHarnessProfile(HarnessType::Pod);
        const auto lightweight = GetHarnessProfile(HarnessType::Lightweight);
        assert(pod.dragAreaM2 < lightweight.dragAreaM2);

        const auto small = ConfigureWing(
            profiles[1], WingSize::Small, BrakeTravel::Standard);
        const auto large = ConfigureWing(
            profiles[1], WingSize::Large, BrakeTravel::Standard);
        assert(small.areaM2 < profiles[1].parameters.areaM2);
        assert(large.areaM2 > profiles[1].parameters.areaM2);
        assert(ConfiguredWingMassKg(profiles[1], WingSize::Small)
            < ConfiguredWingMassKg(profiles[1], WingSize::Large));
        assert(ConfiguredRangeMaxKg(profiles[1], WingSize::Small)
            < ConfiguredRangeMaxKg(profiles[1], WingSize::Large));

        const auto shortBrake = ConfigureWing(
            profiles[1], WingSize::Medium, BrakeTravel::Short);
        const auto longBrake = ConfigureWing(
            profiles[1], WingSize::Medium, BrakeTravel::Long);
        assert(shortBrake.brakeTravelMm < longBrake.brakeTravelMm);
        assert(shortBrake.brakeRollMoment > longBrake.brakeRollMoment);
        ParagliderDynamics shortDynamics(shortBrake);
        ParagliderDynamics longDynamics(longBrake);
        FlightState shortState;
        FlightState longState;
        ControlInput halfRight;
        halfRight.rightBrake = 0.5;
        StepFor(shortDynamics, shortState, halfRight, 2.0);
        StepFor(longDynamics, longState, halfRight, 2.0);
        assert(std::abs(shortState.angularVelocityBodyRadps.z)
            > std::abs(longState.angularVelocityBodyRadps.z));
        assert(shortDynamics.LastTelemetry().brakeTravelRightMm == 260.0);
        assert(longDynamics.LastTelemetry().brakeTravelRightMm == 360.0);

        ParagliderDynamics dynamics(profiles[1].parameters);
        FlightState state;
        ControlInput brakes;
        brakes.leftBrake = 0.8;
        brakes.rightBrake = 0.4;
        StepFor(dynamics, state, brakes, 0.25);
        assert(dynamics.LastTelemetry().leftBrakePressure
             > dynamics.LastTelemetry().rightBrakePressure);
        assert(dynamics.LastTelemetry().leftBrakeForceN
             > dynamics.LastTelemetry().rightBrakeForceN);

        // Brake force rises with dynamic pressure and unloads sharply on the
        // collapsed side, matching the cue expected at a real brake handle.
        ControlInput symmetric;
        symmetric.leftBrake = symmetric.rightBrake = 0.7;
        FlightState slow;
        slow.velocityWorldMps = {7.0, 0.0, -0.8};
        FlightState fast;
        fast.velocityWorldMps = {14.0, 0.0, -1.0};
        ParagliderDynamics slowDynamics(defaultWing.parameters);
        ParagliderDynamics fastDynamics(defaultWing.parameters);
        slowDynamics.Step(slow, symmetric, Atmosphere{}, 1.0 / 120.0);
        fastDynamics.Step(fast, symmetric, Atmosphere{}, 1.0 / 120.0);
        assert(fastDynamics.LastTelemetry().leftBrakeForceN
            > slowDynamics.LastTelemetry().leftBrakeForceN * 2.5);
        FlightState collapsedSide = fast;
        collapsedSide.leftCollapse = 0.7;
        collapsedSide.canopyPressure = 0.55;
        ParagliderDynamics collapsedDynamics(defaultWing.parameters);
        collapsedDynamics.Step(
            collapsedSide, symmetric, Atmosphere{}, 1.0 / 120.0);
        assert(collapsedDynamics.LastTelemetry().leftBrakeForceN
            < collapsedDynamics.LastTelemetry().rightBrakeForceN * 0.6);
    }

    {
        // Stability and reinflation are profile characteristics rather than
        // a global canned response. These are research envelopes pending
        // measured manufacturer manoeuvre data.
        const auto& profiles = GetWingProfiles();
        for (const auto& profile : profiles)
        {
            const auto& p = profile.parameters;
            assert(p.collapseResistance >= 0.7
                && p.collapseResistance <= 1.3);
            assert(p.passiveReinflationRate >= 0.1
                && p.passiveReinflationRate <= 0.3);
            assert(p.brakeReinflationGain > 0.0);
            assert(p.pumpReinflationGain > p.brakeReinflationGain);
            assert(p.frontalReinflationRate >= 0.2
                && p.frontalReinflationRate <= 0.5);
            assert(p.cravatSusceptibility >= 0.5
                && p.cravatSusceptibility <= 1.5);
            assert(p.recoverySurgeGain >= 1.5
                && p.recoverySurgeGain <= 3.2);
        }

        auto asymmetricRemainder = [](const WingProfile& profile)
        {
            ParagliderDynamics dynamics(profile.parameters);
            FlightState state;
            state.leftCollapse = 0.60;
            state.previousMeanCollapse = 0.30;
            StepFor(dynamics, state, ControlInput{}, 5.0);
            assert(dynamics.LastTelemetry().collapseResistance
                == profile.parameters.collapseResistance);
            assert(dynamics.LastTelemetry().passiveReinflationRate
                == profile.parameters.passiveReinflationRate);
            return state.leftCollapse;
        };
        auto frontalRemainder = [](const WingProfile& profile)
        {
            ParagliderDynamics dynamics(profile.parameters);
            FlightState state;
            state.frontalCollapse = 0.60;
            StepFor(dynamics, state, ControlInput{}, 5.0);
            assert(dynamics.LastTelemetry().frontalReinflationRate
                == profile.parameters.frontalReinflationRate);
            return state.frontalCollapse;
        };

        const auto& training =
            GetWingProfile(WingProfileId::TrainingA);
        const auto& sport =
            GetWingProfile(WingProfileId::SportB);
        const auto& epsilon = GetWingProfile(
            WingProfileId::AdvanceEpsilonDls28Research);
        const auto& epic =
            GetWingProfile(WingProfileId::Epic2MLResearch);
        assert(asymmetricRemainder(training)
            < asymmetricRemainder(sport) * 0.75);
        assert(frontalRemainder(training)
            < frontalRemainder(sport) * 0.55);
        assert(asymmetricRemainder(epsilon)
            < asymmetricRemainder(epic));

        WingParameters resistant = sport.parameters;
        WingParameters sensitive = sport.parameters;
        resistant.collapseResistance = 1.25;
        sensitive.collapseResistance = 0.72;
        ParagliderDynamics resistantDynamics(resistant);
        ParagliderDynamics sensitiveDynamics(sensitive);
        FlightState resistantState;
        FlightState sensitiveState;
        Atmosphere incident;
        incident.rotorStrength = 0.50;
        incident.turbulence = 0.55;
        incident.lateralGust = 0.65;
        incident.leftWingWindDeltaMps = {0.0, 0.0, -1.0};
        constexpr double dt = 1.0 / 120.0;
        for (int frame = 0; frame < 120; ++frame)
        {
            resistantDynamics.Step(
                resistantState, ControlInput{}, incident, dt);
            sensitiveDynamics.Step(
                sensitiveState, ControlInput{}, incident, dt);
        }
        assert(resistantState.leftCollapse
            < sensitiveState.leftCollapse * 0.80);
    }

    {
        ParagliderDynamics dynamics;
        FlightState state;
        StepFor(dynamics, state, ControlInput{1.0, 1.0, 0.0}, 2.0);
        assert(state.deepStall > 0.5);
        assert(dynamics.LastTelemetry().dragCoefficient > 0.8);
        assert(dynamics.LastTelemetry().leftStalledSpan > 0.65);
        assert(dynamics.LastTelemetry().rightStalledSpan > 0.65);
        assert(dynamics.LastTelemetry().brakeRollAuthority < 0.12);
        StepFor(dynamics, state, {}, 3.0);
        assert(state.deepStall < 0.5);
    }
    {
        // The authored spiral entry remains recoverable through ordinary
        // progressive controls; the curriculum is not merely a scoring-only
        // state injection.
        constexpr double dt = 1.0 / 120.0;
        ParagliderDynamics dynamics;
        FlightState state;
        for (int frame = 0; frame < 8 * 120; ++frame)
            dynamics.Step(state, {}, Atmosphere{}, dt);
        state.angularVelocityBodyRadps.x = 0.62;
        state.angularVelocityBodyRadps.y = -0.12;
        state.angularVelocityBodyRadps.z = 0.92;
        state.velocityWorldMps.z = std::min(
            state.velocityWorldMps.z, -4.5);

        for (int phase = 0; phase < 4; ++phase)
        {
            ControlInput exit;
            exit.rightBrake = std::max(0.0, 0.62 - phase * 0.20);
            exit.leftBrake = phase >= 2 ? 0.10 : 0.0;
            exit.weightShift = std::max(0.0, 0.4 - phase * 0.14);
            for (int frame = 0; frame < 120; ++frame)
                dynamics.Step(state, exit, Atmosphere{}, dt);
        }
        ControlInput pitchControl;
        pitchControl.leftBrake = pitchControl.rightBrake = 0.18;
        for (int frame = 0; frame < 8 * 120; ++frame)
            dynamics.Step(state, pitchControl, Atmosphere{}, dt);
        for (int frame = 0; frame < 12 * 120; ++frame)
            dynamics.Step(state, {}, Atmosphere{}, dt);
        assert(std::abs(state.angularVelocityBodyRadps.z) < 0.35);
        assert(std::abs(state.angularVelocityBodyRadps.x) < 0.35);
        assert(dynamics.LastTelemetry().loadFactor < 2.0);
        assert(std::isfinite(state.positionWorldM.z));
    }

    {
        ParagliderDynamics dynamics;
        FlightState state;
        Atmosphere violent;
        violent.rotorStrength = 1.0;
        violent.turbulence = 1.0;
        violent.lateralGust = 1.0;
        constexpr double dt = 1.0 / 120.0;
        for (int i = 0; i < 1500; ++i)
            dynamics.Step(state, {}, violent, dt);
        assert(state.leftCravat > 0.01);
        const double cravat = state.leftCravat;
        ControlInput pump{0.65, 0.0, 0.35};
        for (int i = 0; i < 1200; ++i)
            dynamics.Step(state, pump, Atmosphere{}, dt);
        assert(state.leftCravat < cravat);
    }

    {
        HapticFeedbackModel haptics;
        Telemetry neutral;
        const HapticOutput quiet = haptics.Evaluate(neutral, 0.0);
        assert(quiet.left < 0.01 && quiet.right < 0.01);

        Telemetry rightBrake;
        rightBrake.rightBrakePressure = 0.9;
        const HapticOutput brake = haptics.Evaluate(rightBrake, 0.1);
        assert(brake.right > brake.left + 0.2);

        haptics.Reset();
        Telemetry unloading;
        unloading.aerodynamicUnloading = 0.8;
        unloading.dynamicPressureDropPaPerS = 600.0;
        const HapticOutput warning = haptics.Evaluate(unloading, 0.15);
        assert(warning.left > 0.12 && warning.right > 0.12);
        assert(std::abs(warning.left - warning.right) < 0.01);

        haptics.Reset();
        Telemetry leftCollapse;
        leftCollapse.leftCollapse = 0.65;
        leftCollapse.canopyPressure = 0.5;
        const HapticOutput onset = haptics.Evaluate(leftCollapse, 0.2);
        assert(onset.left > 0.9);
        assert(onset.left > onset.right + 0.25);
        const HapticOutput sustained = haptics.Evaluate(leftCollapse, 1.0);
        assert(sustained.left < onset.left);
        assert(sustained.left > sustained.right);

        haptics.Reset();
        Telemetry rotor;
        rotor.rotorStrength = 0.9;
        const HapticOutput rough = haptics.Evaluate(rotor, 0.7);
        assert(rough.left > 0.1 && rough.right > 0.1);
        assert(rough.left <= 1.0 && rough.right <= 1.0);

        haptics.Reset();
        Telemetry strained;
        strained.highLoadDeformation = 0.9;
        const HapticOutput loadCue = haptics.Evaluate(strained, 0.7);
        assert(loadCue.left > quiet.left + 0.03);
        assert(std::abs(loadCue.left - loadCue.right) < 0.01);

        haptics.Reset();
        Telemetry fineGust;
        fineGust.highFrequencyGustMps = 1.2;
        const HapticOutput gustCue = haptics.Evaluate(fineGust, 0.7);
        assert(gustCue.left > quiet.left + 0.03);
        assert(std::abs(gustCue.left - gustCue.right) < 0.01);
    }

    {
        const AudioFeedback quiet = EvaluateAudioFeedback({});
        assert(quiet.varioLevel == 0.0);
        assert(quiet.windLevel >= 0.01 && quiet.windLevel < 0.02);

        AudioFeedbackInput climb;
        climb.verticalSpeedMps = 4.0;
        climb.airspeedMps = 16.0;
        climb.thermalCoreMps = 2.8;
        const AudioFeedback thermal = EvaluateAudioFeedback(climb);
        assert(thermal.varioFrequencyHz > 900.0);
        assert(thermal.varioBeepRateHz > 3.0);
        assert(thermal.windLevel > quiet.windLevel * 10.0);
        assert(thermal.thermalBreathLevel > 0.03);

        AudioFeedbackInput incident;
        incident.airspeedMps = 12.0;
        incident.turbulence = 0.7;
        incident.leftCollapse = 0.5;
        incident.leftCravat = 0.35;
        incident.canopyPressure = 0.45;
        incident.aerodynamicUnloading = 0.8;
        incident.lineLoadN = 1800.0;
        incident.leftBrakeForceN = 55.0;
        incident.rightBrakeForceN = 8.0;
        incident.recoverySurge = 0.3;
        const AudioFeedback event = EvaluateAudioFeedback(incident);
        assert(event.leftFabricLevel > event.rightFabricLevel + 0.08);
        assert(event.leftLineLevel > event.rightLineLevel);
        assert(event.surgeRushLevel > 0.0);
        assert(event.leftFabricLevel <= 0.32);
        assert(event.leftLineLevel <= 0.045);
        assert(event.windLevel <= 0.23);

        AudioFeedbackInput loadedAudio;
        loadedAudio.canopyPressure = 1.0;
        loadedAudio.lineLoadN = 900.0;
        loadedAudio.highLoadDeformation = 0.9;
        const AudioFeedback loadedSound =
            EvaluateAudioFeedback(loadedAudio);
        assert(loadedSound.leftFabricLevel > quiet.leftFabricLevel);
        assert(loadedSound.leftLineLevel > quiet.leftLineLevel);

        AudioFeedbackInput gustAudio;
        gustAudio.canopyPressure = 1.0;
        gustAudio.highFrequencyGustMps = 1.5;
        const AudioFeedback gustSound = EvaluateAudioFeedback(gustAudio);
        assert(gustSound.leftFabricLevel > quiet.leftFabricLevel + 0.015);
    }

    {
        std::array<double, ProgressionScenarioCount> scores{};
        const PilotProgression student = EvaluatePilotProgression(scores);
        assert(std::string(student.rankName) == "STUDENT");
        assert(student.experience == 0.0);
        assert(student.rankProgress == 0.0);

        scores = {720.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        const PilotProgression novice = EvaluatePilotProgression(scores);
        assert(std::string(novice.rankName) == "NOVICE");
        assert(novice.silverMedals == 1);
        assert(novice.masteredScenarios == 1);

        scores = {900.0, 880.0, 860.0, 840.0, 820.0, 800.0, 0.0};
        const PilotProgression master = EvaluatePilotProgression(scores);
        assert(std::string(master.rankName) == "ADVANCED");
        assert(master.experience == 5100.0);
        // Mastery is required as well as raw XP, so 5100 remains Advanced.
        assert(master.rankProgress < 1.0);

        scores = {
            900.0, 900.0, 900.0, 900.0,
            900.0, 900.0, 900.0, 900.0};
        const PilotProgression complete = EvaluatePilotProgression(scores);
        assert(std::string(complete.rankName) == "FLIGHT LAB MASTER");
        assert(complete.goldMedals == 8);
        assert(complete.rankProgress == 1.0);
        assert(MedalForScore(499.0) == MasteryMedal::None);
        assert(MedalForScore(500.0) == MasteryMedal::Bronze);
    }

    {
        const auto& profiles = GetAccessibilityProfiles();
        assert(profiles.size() == AccessibilityProfileCount);
        const auto& full = GetAccessibilityProfile(
            AccessibilityProfileId::FullMotion);
        const auto& comfort = GetAccessibilityProfile(
            AccessibilityProfileId::Comfort);
        const auto& minimal = GetAccessibilityProfile(
            AccessibilityProfileId::MinimalMotion);
        assert(full.inertialCameraScale == 1.0);
        assert(comfort.inertialCameraScale < full.inertialCameraScale);
        assert(minimal.inertialCameraScale < comfort.inertialCameraScale);
        assert(minimal.rotorBuffetScale == 0.0);
        assert(minimal.hapticScale > 0.0);
        // Accessibility profiles affect presentation only. They contain no
        // aerodynamic or control-authority parameters.
        assert(full.hapticScale <= 1.0);
        assert(comfort.hapticScale <= full.hapticScale);

        Telemetry incident;
        incident.airspeedMps = 16.0;
        incident.loadFactor = 2.2;
        incident.turbulence = 0.7;
        incident.rotorStrength = 0.8;
        incident.rightCollapse = 0.55;
        incident.rightCravat = 0.12;
        incident.frontalCollapse = 0.2;
        incident.recoverySurge = 0.24;
        incident.aerodynamicUnloading = 0.65;
        const CameraFeedback fullResponse = EvaluateCameraFeedback(
            incident, {2.0, -1.0, 3.0}, 4.25, full);
        const CameraFeedback repeated = EvaluateCameraFeedback(
            incident, {2.0, -1.0, 3.0}, 4.25, full);
        const CameraFeedback minimalResponse = EvaluateCameraFeedback(
            incident, {2.0, -1.0, 3.0}, 4.25, minimal);
        assert(fullResponse.positionOffsetCm.x
            == repeated.positionOffsetCm.x);
        assert(fullResponse.rollDegrees == repeated.rollDegrees);
        assert(fullResponse.positionOffsetCm.y > 10.0);
        assert(fullResponse.rollDegrees > 4.0);
        assert(fullResponse.pitchDegrees < 0.0);
        assert(fullResponse.fieldOfViewDeltaDegrees > 3.0);
        assert(std::abs(minimalResponse.positionOffsetCm.y)
            < std::abs(fullResponse.positionOffsetCm.y) * 0.12);
        assert(std::abs(minimalResponse.rollDegrees)
            < std::abs(fullResponse.rollDegrees) * 0.12);

        // The pendulum's two halves must move the camera in opposite
        // directions and must not be the collapse cue wearing a different
        // name. Back: the view lifts and pitches up. Front: it drops and
        // pitches down. Anything that returns the same sign for both is
        // reading the magnitude of a swing and not its direction, which is
        // the bug this cue exists to avoid.
        {
            Telemetry wingBack;
            wingBack.canopyRelativePitchRad = 0.30;
            wingBack.canopyRelativePitchRateRadps = 0.6;
            Telemetry wingFront;
            wingFront.canopyRelativePitchRad = -0.30;
            wingFront.canopyRelativePitchRateRadps = -0.6;
            const CameraFeedback back =
                EvaluateCameraFeedback(wingBack, {}, 1.0, full);
            const CameraFeedback front =
                EvaluateCameraFeedback(wingFront, {}, 1.0, full);
            assert(back.pitchDegrees > 0.0);
            assert(front.pitchDegrees < 0.0);
            assert(back.positionOffsetCm.z > 0.0);
            assert(front.positionOffsetCm.z < 0.0);
            assert(back.positionOffsetCm.x < front.positionOffsetCm.x);
            // Bounded, because this one happens all the time. A pendulum cue
            // that shouts as loudly as a collapse is a cue nobody can read.
            // Eight degrees at a swing well past what the model reaches under
            // a hard brake; stated absolutely rather than against another
            // case, because the incident case's own pitch is a sum of terms
            // that partly cancel and would make this a comparison of
            // coincidences.
            assert(std::abs(back.pitchDegrees) < 8.0);
            assert(std::abs(front.pitchDegrees) < 8.0);
            // And the comfort profiles still govern it.
            const CameraFeedback quiet =
                EvaluateCameraFeedback(wingBack, {}, 1.0, minimal);
            assert(std::abs(quiet.positionOffsetCm.z)
                < std::abs(back.positionOffsetCm.z) * 0.12);
        }

        Telemetry loadOnly;
        loadOnly.loadFactor = 4.7;
        loadOnly.highLoadDeformation = 0.9;
        const CameraFeedback loadResponse = EvaluateCameraFeedback(
            loadOnly, {}, 2.0, full);
        assert(loadResponse.positionOffsetCm.z < -30.0);
        assert(loadResponse.fieldOfViewDeltaDegrees
            < (loadOnly.loadFactor - 1.0) * 0.45);

        Telemetry gustOnly;
        gustOnly.lowFrequencyGustMps = 2.0;
        const CameraFeedback gustResponse = EvaluateCameraFeedback(
            gustOnly, {}, 4.25, full);
        assert(std::abs(gustResponse.positionOffsetCm.y) > 0.2);
    }

    {
        const auto& profiles = GetGraphicsProfiles();
        assert(profiles.size() == GraphicsProfileCount);
        for (std::size_t index = 0; index < profiles.size(); ++index)
        {
            assert(profiles[index].qualityLevel == static_cast<int>(index));
            assert(profiles[index].resolutionScale >= 50.0);
            assert(profiles[index].resolutionScale <= 100.0);
            if (index > 0)
                assert(profiles[index].resolutionScale
                    >= profiles[index - 1].resolutionScale);
        }
        assert(GetGraphicsProfile(GraphicsProfileId::High).qualityLevel == 2);
        // Graphics profiles contain render quality only. Flight integration
        // remains fixed at 120 Hz and is not coupled to frame rate or tier.
    }

    {
        const ResearchManeuverResult weightShift = RunResearchManeuver(
            ResearchManeuver::WeightShiftStep);
        const ResearchManeuverResult zoom = RunResearchManeuver(
            ResearchManeuver::BrakeZoom);
        const ResearchManeuverResult deepStall = RunResearchManeuver(
            ResearchManeuver::SymmetricDeepStall);
        const ResearchManeuverResult asymmetric = RunResearchManeuver(
            ResearchManeuver::AsymmetricStall);

        assert(weightShift.peakAbsBankRad > 0.12);
        assert(std::abs(weightShift.lateralDisplacementM) > 1.0);
        assert(zoom.peakClimbMps > 0.05);
        assert(zoom.altitudeGainM > 0.05);
        assert(zoom.minimumAirspeedMps < 15.0);
        assert(deepStall.peakSeparatedSpan > 0.45);
        assert(deepStall.minimumAirspeedMps < 10.0);
        assert(asymmetric.peakSeparatedSpan > 0.35);
        assert(asymmetric.peakAbsBankRad < 1.45);
        assert(std::isfinite(weightShift.peakAbsEnergyResidualW));
        assert(std::isfinite(zoom.peakAbsEnergyResidualW));
        assert(std::isfinite(deepStall.peakAbsEnergyResidualW));
        assert(std::isfinite(asymmetric.peakAbsEnergyResidualW));
    }

    {
        // Still-air system-identification baseline: use the integrated
        // dynamics rather than only sampling the configured aerodynamic polar.
        constexpr double dt = 1.0 / 120.0;
        const auto& epic = GetWingProfile(WingProfileId::Epic2MLResearch);
        ParagliderDynamics trimDynamics(epic.parameters);
        FlightState trimState;
        for (int frame = 0; frame < 30 * 120; ++frame)
            trimDynamics.Step(trimState, {}, Atmosphere{}, dt);
        const Vec3 trimStart = trimState.positionWorldM;
        for (int frame = 0; frame < 20 * 120; ++frame)
            trimDynamics.Step(trimState, {}, Atmosphere{}, dt);
        const double horizontalM = std::hypot(
            trimState.positionWorldM.x - trimStart.x,
            trimState.positionWorldM.y - trimStart.y);
        const double heightLossM =
            trimStart.z - trimState.positionWorldM.z;
        const double integratedGlideRatio =
            horizontalM / std::max(0.1, heightLossM);
        assert(integratedGlideRatio > 8.5);
        assert(integratedGlideRatio < 10.5);
        assert(std::abs(trimState.velocityWorldMps.y) < 0.5);

        ParagliderDynamics pendulumDynamics(epic.parameters);
        FlightState pendulumState;
        pendulumState.velocityWorldMps = {16.0, 0.0, -1.0};
        double maximumPayloadPitch = 0.0;
        double minimumPayloadPitch = 0.0;
        double maximumSystemPitchRate = 0.0;
        for (int frame = 0; frame < 8 * 120; ++frame)
        {
            ControlInput pulse;
            if (frame >= 120 && frame < 210)
                pulse.leftBrake = pulse.rightBrake = 0.72;
            pendulumDynamics.Step(
                pendulumState, pulse, Atmosphere{}, dt);
            maximumPayloadPitch = std::max(
                maximumPayloadPitch, pendulumState.harnessPitchRad);
            minimumPayloadPitch = std::min(
                minimumPayloadPitch, pendulumState.harnessPitchRad);
            maximumSystemPitchRate = std::max(
                maximumSystemPitchRate,
                std::abs(pendulumState.angularVelocityBodyRadps.y));
        }
        assert(maximumPayloadPitch > 0.01);
        assert(minimumPayloadPitch < -0.005);
        assert(maximumSystemPitchRate > 0.01);

        // A low-speed disturbance with hands up is not itself a stall. The
        // wing must lower its flight path and recover toward trim instead of
        // entering the former self-reinforcing 3 m/s state.
        ParagliderDynamics recoveryDynamics(epic.parameters);
        FlightState recoveryState;
        recoveryState.velocityWorldMps = {4.0, 0.0, -0.8};
        double minimumSpeed = 100.0;
        for (int frame = 0; frame < 15 * 120; ++frame)
        {
            recoveryDynamics.Step(
                recoveryState, {}, Atmosphere{}, dt);
            minimumSpeed = std::min(
                minimumSpeed,
                recoveryDynamics.LastTelemetry().airspeedMps);
        }
        assert(minimumSpeed < 5.2);
        assert(recoveryDynamics.LastTelemetry().airspeedMps > 8.0);
        assert(recoveryState.deepStall < 0.05);
    }

    // A stall recovery has to convert the descent back into forward flight.
    //
    // Reported by a pilot flying the game: "after a stall the recovery is fast,
    // but the heading is still pretty much going down - it stabilises as if it
    // was on the moon's gravity. If I touch the brake or weight shift it works
    // correctly." That is a precise description of a real defect and this is it
    // measured.
    //
    // The distinction the old deep-stall test missed is between the stall
    // STATE and the FLIGHT PATH. `state.deepStall` clears in about two seconds
    // and the existing gate is satisfied by that alone. The aircraft is then
    // still falling: descent goes on INCREASING after the wing has started
    // flying again, and the trajectory needs the better part of a minute to
    // come back to the glide it left.
    //
    // What a wing does instead: the pilot's weight hangs seven metres under the
    // canopy, so a wing that has stopped being stalled is a pendulum with lift
    // on it. It pitches down, converts height into speed, and is flying within
    // a few seconds. Gravity does that work whether or not the pilot touches
    // anything - which is exactly the part the report says is missing.
    {
        constexpr double dt = 1.0 / 120.0;
        ParagliderDynamics dynamics;
        FlightState state;
        StepFor(dynamics, state, {}, 20.0);
        const double trimH = std::hypot(state.velocityWorldMps.x,
                                        state.velocityWorldMps.y);
        const double trimGlide = trimH / std::max(0.01,
                                                  -state.velocityWorldMps.z);

        StepFor(dynamics, state, ControlInput{1.0, 1.0, 0.0}, 3.0);
        const double stalledSink = -state.velocityWorldMps.z;

        double peakSink = stalledSink;
        double stallClearedAt = -1.0;
        double flyingAgainAt = -1.0;
        double settledAt = -1.0;
        const int steps = static_cast<int>(60.0 / dt);
        for (int i = 0; i < steps; ++i)
        {
            dynamics.Step(state, ControlInput{}, Atmosphere{}, dt);
            const double t = (i + 1) * dt;
            const double h = std::hypot(state.velocityWorldMps.x,
                                        state.velocityWorldMps.y);
            const double sink = -state.velocityWorldMps.z;
            peakSink = std::max(peakSink, sink);
            const double glide = sink > 0.01 ? h / sink : 1000.0;
            if (stallClearedAt < 0.0 && state.deepStall < 0.05)
                stallClearedAt = t;
            if (flyingAgainAt < 0.0 && stallClearedAt >= 0.0 && glide > 1.0)
                flyingAgainAt = t;
            // Settled means back inside a tenth of the glide it left and
            // staying there - so the last excursion outside the band is what
            // sets it, not the first entry into it.
            if (std::fabs(glide - trimGlide) > 0.10 * trimGlide)
                settledAt = t;
        }

        std::cout << "Stall recovery: trim glide " << trimGlide
                  << ", stalled sink " << stalledSink
                  << ", peak sink after release " << peakSink
                  << " (x" << (peakSink / stalledSink) << ")\n"
                  << "  stall state cleared at " << stallClearedAt
                  << " s, flying forward again at " << flyingAgainAt
                  << " s, glide settled at " << settledAt << " s\n";

        // KNOWN DEFECT, bounded at what it measures today so it cannot
        // quietly get worse while the fix is decided.
        //
        // The cause is one line of `ParagliderDynamics`, and its own comment
        // gives it away: `pitchStiffness` is documented as the
        // "aerodynamic/pendular restoring moment toward the configured trim
        // INCIDENCE". The aerodynamic weathercock and the pendulum are folded
        // into a single spring that measures incidence error - and a wing
        // diving vertically is already AT trim incidence, so that spring reads
        // zero and does nothing. Nothing else in the pitch axis references
        // gravity. The recovery that does happen is the slow speed-for-height
        // phugoid exchange, which is why it takes half a minute.
        //
        // A real pendulum is referenced to gravity, not to incidence: the
        // pilot's weight hangs under the canopy and pulls the nose up out of a
        // dive whatever the incidence happens to be. The geometry-driven stack
        // has this by construction and measures the line network's pitch
        // spring at 6317 N.m/rad at 1 g, against the 165 N.m/rad this whole
        // axis runs on. That is the "moon gravity" a pilot reported, and the
        // ratio is roughly what it feels like.
        //
        // Not fixed here, deliberately. Adding a gravity-referenced pendulum
        // term to the legacy path changes an axis that eleven calibration
        // gates are written against, and the target numbers want a pilot's
        // judgement rather than a plausible-looking constant. `PHYSICS_TODO`
        // item 19.
        assert(peakSink < 2.3 * stalledSink);
        assert(flyingAgainAt > 0.0 && flyingAgainAt - stallClearedAt < 3.0);
        assert(settledAt > 0.0 && settledAt < 40.0);
    }

    std::cout << "All paraglider physics tests passed.\n";
}
