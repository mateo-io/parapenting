#include "ParapentingHUD.h"
#include "ParagliderPawn.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "GameFramework/PlayerController.h"

void AParapentingHUD::DrawFlightDeck(const AParagliderPawn* Glider)
{
    if (!Canvas || !Glider || !Glider->IsFlightDeckVisible()) return;

    // A conservative Canvas implementation is deliberate for this first
    // front-end slice: it is packaged today, scales from a reference canvas,
    // and consumes the existing engine-independent settings contracts.  The
    // visual language can migrate to UMG without duplicating any flight state.
    const float Scale = FMath::Clamp(FMath::Min(
        Canvas->SizeX / 1440.0f, Canvas->SizeY / 900.0f), 0.72f, 1.45f);
    const float Width = 680.0f * Scale;
    const float Height = 510.0f * Scale;
    const float X = (Canvas->SizeX - Width) * 0.5f;
    const float Y = (Canvas->SizeY - Height) * 0.5f;
    const FLinearColor Panel(0.006f, 0.016f, 0.028f, 0.93f);
    const FLinearColor White(0.92f, 0.97f, 1.0f);
    const FLinearColor Cyan(0.36f, 0.86f, 1.0f);
    const FLinearColor Muted(0.55f, 0.70f, 0.80f);
    const auto Text = [this, Scale](const FString& Value, const FLinearColor& Colour,
        float InX, float InY, UFont* Font, float FontScale = 1.0f)
    {
        DrawText(Value, Colour, InX, InY, Font, FontScale * Scale);
    };

    DrawRect(Panel, X, Y, Width, Height);
    DrawRect(Cyan, X, Y, Width, 3.0f * Scale);
    Text(TEXT("PARAPENTING"), White, X + 32.0f * Scale, Y + 28.0f * Scale,
        GEngine->GetMediumFont(), 1.35f);
    Text(TEXT("FLIGHT DECK  ·  A focused setup layer over the live simulator"),
        Cyan, X + 34.0f * Scale, Y + 70.0f * Scale, GEngine->GetSmallFont(), 0.88f);
    Text(TEXT("ESC  CLOSE   ·   AUTO-CLOSE 5 s"), Muted, X + Width - 245.0f * Scale,
        Y + 36.0f * Scale, GEngine->GetSmallFont(), 0.82f);

    const float Left = X + 34.0f * Scale;
    const float Row = 42.0f * Scale;
    float Cursor = Y + 125.0f * Scale;
    const auto Setting = [&Text, &Cursor, Left, Row, Scale, this](const TCHAR* Label,
        const FString& Value, const FString& Hint)
    {
        Text(Label, FLinearColor(0.55f, 0.70f, 0.80f), Left, Cursor,
            GEngine->GetSmallFont(), 0.88f);
        Text(Value, FLinearColor(0.92f, 0.97f, 1.0f), Left + 170.0f * Scale,
            Cursor, GEngine->GetSmallFont(), 0.98f);
        Text(Hint, FLinearColor(0.36f, 0.86f, 1.0f), Left + 420.0f * Scale,
            Cursor, GEngine->GetSmallFont(), 0.78f);
        Cursor += Row;
    };
    Setting(TEXT("ROUTE"), ANSI_TO_TCHAR(Glider->GetRouteDisplayName()),
        TEXT("[ / ] CHANGE"));
    Setting(TEXT("WEATHER"), ANSI_TO_TCHAR(Glider->GetWeatherPresetDisplayName()),
        TEXT("O CYCLE"));
    Setting(TEXT("TIME"), Glider->GetLocalTimeDisplay(), TEXT("F11 CYCLE"));
    Setting(TEXT("WING"), ANSI_TO_TCHAR(Glider->GetWingDisplayName()), TEXT("Q CYCLE"));
    Setting(TEXT("GRAPHICS"), ANSI_TO_TCHAR(Glider->GetGraphicsProfileName()),
        TEXT("F10 CYCLE"));
    Setting(TEXT("MOTION"), ANSI_TO_TCHAR(Glider->GetAccessibilityProfileName()),
        TEXT("F8 CYCLE"));
    Setting(TEXT("CONTROLS"), ANSI_TO_TCHAR(Glider->GetKeyboardLayoutName()),
        TEXT("F6 / F7 BIND"));

    DrawRect(FLinearColor(0.05f, 0.13f, 0.20f, 0.82f), Left,
        Y + Height - 104.0f * Scale, Width - 68.0f * Scale, 68.0f * Scale);
    Text(TEXT("START: ESC closes this layer.  N prepares a launch; SPACE commits the run."),
        White, Left + 16.0f * Scale, Y + Height - 87.0f * Scale,
        GEngine->GetSmallFont(), 0.82f);
    Text(TEXT("F5 weather briefing · TAB HUD mode · M camera · R reset"), Muted,
        Left + 16.0f * Scale, Y + Height - 61.0f * Scale,
        GEngine->GetSmallFont(), 0.82f);
}

void AParapentingHUD::DrawPreflightBriefing(
    const AParagliderPawn* Glider)
{
    if (!Canvas || !Glider || !Glider->IsBriefingVisible()) return;
    const auto B = Glider->GetPreflightBriefing();
    const float Width = FMath::Min(680.0f, Canvas->SizeX - 48.0f);
    const float Height = 390.0f;
    const float X = (Canvas->SizeX - Width) * 0.5f;
    const float Y = (Canvas->SizeY - Height) * 0.5f;
    const FLinearColor White(0.94f, 0.97f, 1.0f);
    FLinearColor RiskColor(0.35f, 1.0f, 0.55f);
    if (B.overallRisk == Parapenting::Physics::PreflightRisk::Moderate)
        RiskColor = FLinearColor(1.0f, 0.82f, 0.25f);
    else if (B.overallRisk == Parapenting::Physics::PreflightRisk::High)
        RiskColor = FLinearColor(1.0f, 0.48f, 0.12f);
    else if (B.overallRisk == Parapenting::Physics::PreflightRisk::Extreme)
        RiskColor = FLinearColor(1.0f, 0.12f, 0.08f);

    DrawRect(FLinearColor(0.006f, 0.016f, 0.028f, 0.96f),
        X, Y, Width, Height);
    DrawText(TEXT("PRE-FLIGHT WEATHER BRIEFING"),
        White, X + 24.0f, Y + 20.0f, GEngine->GetMediumFont(), 1.12f);
    DrawText(TEXT("F5 CLOSE   SIMULATOR PLANNING ONLY"),
        FLinearColor(0.6f, 0.78f, 0.9f),
        X + Width - 255.0f, Y + 25.0f, GEngine->GetSmallFont(), 0.84f);
    DrawText(FString::Printf(TEXT("%s   %s   %s LOCAL"),
        ANSI_TO_TCHAR(Glider->GetRouteDisplayName()),
        ANSI_TO_TCHAR(Glider->GetWeatherPresetDisplayName()),
        *Glider->GetLocalTimeDisplay()),
        FLinearColor(0.68f, 0.9f, 1.0f), X + 24.0f, Y + 64.0f);
    DrawText(FString::Printf(TEXT("RISK %s   SUITABILITY %.0f / 100"),
        ANSI_TO_TCHAR(Parapenting::Physics::PreflightRiskName(B.overallRisk)),
        B.suitabilityScore),
        RiskColor, X + 24.0f, Y + 96.0f,
        GEngine->GetMediumFont(), 1.0f);
    DrawText(FString::Printf(
        TEXT("WIND FROM %03.0f°   LAUNCH %.1f   CRUISE %.1f   LANDING %.1f m/s"),
        B.windFromDegrees, B.launchWindMps,
        B.cruiseWindMps, B.landingWindMps),
        White, X + 24.0f, Y + 138.0f);
    DrawText(FString::Printf(
        TEXT("DIRECTION ERROR %.0f°   GUST SPREAD %.1f m/s   %s"),
        B.launchDirectionErrorDegrees, B.gustSpreadMps,
        ANSI_TO_TCHAR(Parapenting::Physics::SiteWindAssessmentName(
            B.launchWindAssessment))),
        B.launchWindAssessment
                == Parapenting::Physics::SiteWindAssessment::Suitable
            ? FLinearColor(0.55f, 0.95f, 0.65f) : RiskColor,
        X + 24.0f, Y + 170.0f);
    DrawText(FString::Printf(
        TEXT("THERMALS %.1f m/s   TOP %.0f m MSL   CLOUD +%.0f m OVER LAUNCH"),
        B.thermalStrengthMps, B.thermalTopMslM,
        B.cloudBaseAboveLaunchM),
        FLinearColor(1.0f, 0.72f, 0.25f), X + 24.0f, Y + 210.0f);
    DrawText(FString::Printf(
        TEXT("CLOUD %.0f%%   TURBULENCE %.0f%%   ROTOR RISK %.0f%%   %d ROTOR ZONES"),
        B.cloudCoverage * 100.0, B.turbulenceRisk * 100.0,
        B.rotorRisk * 100.0, B.authoredRotorVolumes),
        B.rotorRisk > 0.65 ? RiskColor : White,
        X + 24.0f, Y + 242.0f);
    DrawText(ANSI_TO_TCHAR(Glider->GetLaunchHazardText()),
        FLinearColor(1.0f, 0.74f, 0.35f),
        X + 24.0f, Y + 282.0f, GEngine->GetSmallFont(), 0.85f);
    DrawText(ANSI_TO_TCHAR(Glider->GetLandingCircuitText()),
        FLinearColor(0.72f, 0.86f, 1.0f),
        X + 24.0f, Y + 310.0f, GEngine->GetSmallFont(), 0.82f);
    DrawText(ANSI_TO_TCHAR(
        Parapenting::Physics::PreflightRecommendation(B)),
        RiskColor, X + 24.0f, Y + 348.0f,
        GEngine->GetSmallFont(), 0.9f);
}

void AParapentingHUD::DrawCompactHUD(
    const AParagliderPawn* Glider, bool bMinimal)
{
    if (!Canvas || !Glider) return;
    const auto& T = Glider->GetFlightTelemetry();
    const auto& State = Glider->GetFlightState();
    const auto& Controls = Glider->GetControlInput();
    const auto Navigation = Glider->GetNavigationSolution();
    const FLinearColor White(0.95f, 0.98f, 1.0f, 1.0f);
    const FLinearColor Cyan(0.42f, 0.88f, 1.0f, 1.0f);
    const FLinearColor Panel(0.008f, 0.02f, 0.035f, 0.72f);
    const float Agl = static_cast<float>(Glider->GetGroundClearanceM());

    if (bMinimal)
    {
        const float Width = FMath::Min(Canvas->SizeX - 48.0f, 760.0f);
        const float X = (Canvas->SizeX - Width) * 0.5f;
        const float Y = Canvas->SizeY - 58.0f;
        DrawRect(Panel, X, Y, Width, 36.0f);
        DrawText(FString::Printf(
            TEXT("%.0f km/h  %+3.1f m/s  %.0f AGL  %s %.1fkm ARR%+.0fm  [TAB]"),
            T.airspeedMps * 3.6, State.velocityWorldMps.z, Agl,
            ANSI_TO_TCHAR(Glider->GetActiveWaypointName()),
            Navigation.distanceM / 1000.0,
            Navigation.predictedArrivalHeightM),
            White, X + 14.0f, Y + 10.0f, GEngine->GetSmallFont(), 0.95f);
    }
    else
    {
        constexpr float Left = 22.0f;
        constexpr float Top = 22.0f;
        constexpr float Width = 350.0f;
        DrawRect(Panel, Left, Top, Width, 206.0f);
        DrawText(TEXT("PARAPENTING"), White, Left + 16.0f, Top + 12.0f,
            GEngine->GetSmallFont(), 1.08f);
        DrawText(TEXT("COMPACT  [TAB]"), Cyan, Left + 220.0f, Top + 12.0f,
            GEngine->GetSmallFont(), 0.82f);
        DrawText(FString::Printf(
            TEXT("%3.0f km/h    %+3.1f m/s    %3.0f m AGL"),
            T.airspeedMps * 3.6, State.velocityWorldMps.z, Agl),
            White, Left + 16.0f, Top + 42.0f,
            GEngine->GetMediumFont(), 0.92f);
        DrawText(FString::Printf(TEXT("BRAKES  L %.0f N   R %.0f N"),
            T.leftBrakeForceN, T.rightBrakeForceN),
            Cyan, Left + 16.0f, Top + 76.0f);
        DrawRect(FLinearColor(0.10f, 0.16f, 0.20f, 1.0f),
            Left + 16.0f, Top + 100.0f, 145.0f, 8.0f);
        DrawRect(FLinearColor(0.2f, 0.76f, 1.0f, 1.0f),
            Left + 16.0f, Top + 100.0f,
            145.0f * Controls.leftBrake, 8.0f);
        DrawRect(FLinearColor(0.10f, 0.16f, 0.20f, 1.0f),
            Left + 184.0f, Top + 100.0f, 145.0f, 8.0f);
        DrawRect(FLinearColor(0.2f, 0.76f, 1.0f, 1.0f),
            Left + 184.0f, Top + 100.0f,
            145.0f * Controls.rightBrake, 8.0f);
        DrawText(FString::Printf(TEXT("%s  %.0f m  %s %2.0f%%"),
            ANSI_TO_TCHAR(Glider->GetRouteDisplayName()),
            Glider->GetDistanceToTargetM(),
            ANSI_TO_TCHAR(Glider->GetLandingPhaseName()),
            Glider->GetApproachQuality() * 100.0),
            FLinearColor(0.72f, 1.0f, 0.76f), Left + 16.0f, Top + 122.0f,
            GEngine->GetSmallFont(), 0.88f);
        DrawText(FString::Printf(TEXT("%s   %s   %s"),
            ANSI_TO_TCHAR(Glider->GetWeatherPresetDisplayName()),
            ANSI_TO_TCHAR(Glider->GetSiteWindAssessmentName()),
            *Glider->GetLocalTimeDisplay()),
            FLinearColor(0.88f, 0.88f, 0.64f), Left + 16.0f, Top + 148.0f,
            GEngine->GetSmallFont(), 0.84f);
        DrawText(FString::Printf(
            TEXT("WP%d %s  %.1fkm  ARR%+.0fm  %s"),
            Glider->GetActiveWaypointNumber(),
            ANSI_TO_TCHAR(Glider->GetActiveWaypointName()),
            Navigation.distanceM / 1000.0,
            Navigation.predictedArrivalHeightM,
            ANSI_TO_TCHAR(Parapenting::Physics::SpeedToFlyCueName(
                Navigation.speedToFly))),
            Navigation.reachable
                ? FLinearColor(0.35f, 1.0f, 0.58f)
                : FLinearColor(1.0f, 0.55f, 0.18f),
            Left + 16.0f, Top + 174.0f,
            GEngine->GetSmallFont(), 0.86f);

        const float Right = Canvas->SizeX - 302.0f;
        DrawRect(Panel, Right, Top, 280.0f, 92.0f);
        DrawText(FString::Printf(TEXT("%s   %03.0f / 1000"),
            ANSI_TO_TCHAR(Glider->GetScenarioDisplayName()),
            Glider->GetChallengeScore()),
            FLinearColor(1.0f, 0.82f, 0.34f), Right + 14.0f, Top + 13.0f,
            GEngine->GetSmallFont(), 0.9f);
        DrawText(ANSI_TO_TCHAR(Glider->IsGroundLaunching()
                ? Glider->GetLaunchPhaseName()
                : Glider->GetChallengeFeedback()),
            Glider->IsChallengeComplete()
                ? FLinearColor(0.35f, 1.0f, 0.55f) : White,
            Right + 14.0f, Top + 40.0f, GEngine->GetSmallFont(), 0.88f);
        DrawRect(FLinearColor(0.10f, 0.16f, 0.20f, 1.0f),
            Right + 14.0f, Top + 69.0f, 252.0f, 7.0f);
        DrawRect(Cyan, Right + 14.0f, Top + 69.0f,
            252.0f * Glider->GetChallengeProgress(), 7.0f);
    }

    const double Collapse =
        T.leftCollapse + T.rightCollapse + T.frontalCollapse;
    const bool bSeriousIncident =
        Collapse > 0.18 || T.deepStall > 0.35
        || FMath::Abs(T.spin) > 0.3
        || T.leftCravat + T.rightCravat > 0.08;
    if (bSeriousIncident)
    {
        FString Warning;
        if (T.deepStall > 0.35) Warning = TEXT("DEEP STALL — RELEASE BRAKES");
        else if (FMath::Abs(T.spin) > 0.3)
            Warning = TEXT("SPIN — RELEASE DEEP BRAKE");
        else if (T.leftCravat + T.rightCravat > 0.08)
            Warning = TEXT("CRAVAT — CONTROL HEADING / PUMP TIP");
        else Warning = TEXT("COLLAPSE — CONTROL HEADING");
        DrawRect(FLinearColor(0.28f, 0.005f, 0.005f, 0.86f),
            Canvas->SizeX * 0.5f - 210.0f, 22.0f, 420.0f, 42.0f);
        DrawText(Warning, FLinearColor(1.0f, 0.34f, 0.22f),
            Canvas->SizeX * 0.5f - 180.0f, 34.0f,
            GEngine->GetMediumFont(), 0.95f);
    }
    else if (T.highLoadDeformation > 0.7)
    {
        DrawRect(FLinearColor(0.24f, 0.12f, 0.0f, 0.84f),
            Canvas->SizeX * 0.5f - 210.0f, 22.0f, 420.0f, 42.0f);
        DrawText(FString::Printf(TEXT("HIGH LOAD %.1f g — EASE MANEUVER"),
            T.loadFactor), FLinearColor(1.0f, 0.72f, 0.18f),
            Canvas->SizeX * 0.5f - 170.0f, 34.0f,
            GEngine->GetMediumFont(), 0.95f);
    }
    else if (T.thermalLiftMps > 0.15)
    {
        DrawText(FString::Printf(TEXT("THERMAL  +%.1f m/s"),
            T.thermalCoreStrength),
            FLinearColor(1.0f, 0.64f, 0.16f),
            Canvas->SizeX * 0.5f - 74.0f, 30.0f,
            GEngine->GetMediumFont(), 0.9f);
    }
    if (Glider->IsCapturingBinding())
    {
        DrawRect(FLinearColor(0.02f, 0.08f, 0.14f, 0.94f),
            Canvas->SizeX * 0.5f - 260.0f, 76.0f, 520.0f, 42.0f);
        DrawText(Glider->GetBindingCaptureText(),
            FLinearColor(0.4f, 0.9f, 1.0f),
            Canvas->SizeX * 0.5f - 235.0f, 88.0f,
            GEngine->GetSmallFont(), 0.92f);
    }

    if (Glider->HasLanded())
    {
        const auto& D = Glider->GetFlightDebrief();
        DrawRect(FLinearColor(0.01f, 0.02f, 0.03f, 0.90f),
            Canvas->SizeX * 0.5f - 250.0f,
            Canvas->SizeY * 0.5f - 105.0f, 500.0f, 210.0f);
        DrawText(Glider->WasHardLanding()
                ? TEXT("HARD LANDING") : TEXT("LANDED"),
            Glider->WasHardLanding()
                ? FLinearColor::Red : FLinearColor::Green,
            Canvas->SizeX * 0.5f - 78.0f,
            Canvas->SizeY * 0.5f - 82.0f,
            GEngine->GetMediumFont(), 1.2f);
        DrawText(FString::Printf(
            TEXT("%.0f m TARGET   V/S %.1f   %s %.1f m   R RESET"),
            Glider->GetLandingDistanceM(),
            Glider->GetTouchdownVerticalSpeedMps(),
            ANSI_TO_TCHAR(Glider->GetLandingRolloutPhaseName()),
            Glider->GetRunoutDistanceM()),
            White, Canvas->SizeX * 0.5f - 180.0f,
            Canvas->SizeY * 0.5f - 42.0f);
        DrawText(FString::Printf(
            TEXT("SAFETY %.0f   EFFICIENCY %.0f   THERMAL %.0f   LANDING %.0f"),
            D.safetyRating, D.efficiencyRating,
            D.thermalRating, D.landingRating),
            FLinearColor(0.45f, 0.9f, 1.0f),
            Canvas->SizeX * 0.5f - 215.0f,
            Canvas->SizeY * 0.5f - 8.0f,
            GEngine->GetSmallFont(), 0.9f);
        DrawText(FString::Printf(
            TEXT("OVERALL %.0f   %.1f km   +%.0f m   %.0f s ROTOR"),
            D.overallRating, D.horizontalDistanceM / 1000.0,
            D.altitudeGainM, D.rotorExposureS),
            FLinearColor(1.0f, 0.8f, 0.32f),
            Canvas->SizeX * 0.5f - 180.0f,
            Canvas->SizeY * 0.5f + 24.0f);
        if (Glider->IsLandingFlareScenario())
        {
            const FString FlareResult =
                Glider->GetFirstFlareClearanceM() >= 0.0
                ? FString::Printf(
                    TEXT("FLARE %.1f m AGL   PEAK AUTHORITY %.0f%%   SCORE %.0f"),
                    Glider->GetFirstFlareClearanceM(),
                    Glider->GetPeakFlareAuthority() * 100.0,
                    Glider->GetChallengeScore())
                : FString::Printf(
                    TEXT("NO FLARE DETECTED   SCORE %.0f"),
                    Glider->GetChallengeScore());
            DrawText(FlareResult,
                White, Canvas->SizeX * 0.5f - 210.0f,
                Canvas->SizeY * 0.5f + 57.0f,
                GEngine->GetSmallFont(), 0.86f);
        }
        else
        {
            DrawText(ANSI_TO_TCHAR(Glider->GetDebriefFocusText()),
                White, Canvas->SizeX * 0.5f - 210.0f,
                Canvas->SizeY * 0.5f + 57.0f,
                GEngine->GetSmallFont(), 0.86f);
        }
    }
}

void AParapentingHUD::DrawHUD()
{
    Super::DrawHUD();
    if (!Canvas) return;

    const AParagliderPawn* Glider =
        PlayerOwner ? Cast<AParagliderPawn>(PlayerOwner->GetPawn()) : nullptr;
    const FLinearColor White(0.95f, 0.98f, 1.0f, 1.0f);

    if (Glider && Glider->GetHudMode() != 1)
    {
        DrawCompactHUD(Glider, Glider->GetHudMode() == 2);
        DrawPreflightBriefing(Glider);
        DrawFlightDeck(Glider);
        return;
    }

    DrawRect(FLinearColor(0.01f, 0.025f, 0.045f, 0.78f), 24.0f, 24.0f, 560.0f, 440.0f);
    DrawText(TEXT("PARAPENTING  v4 FLIGHT LAB"), White, 42.0f, 40.0f,
             GEngine->GetSmallFont(), 1.15f);

    if (Glider)
    {
        DrawFlightDeck(Glider);
        const auto& Telemetry = Glider->GetFlightTelemetry();
        const auto& State = Glider->GetFlightState();
        const auto& Controls = Glider->GetControlInput();
        const auto Navigation = Glider->GetNavigationSolution();
        const double GroundClearance = Glider->GetGroundClearanceM();
        DrawText(FString::Printf(TEXT("AIRSPEED   %4.1f km/h"), Telemetry.airspeedMps * 3.6),
                 White, 42.0f, 78.0f);
        DrawText(FString::Printf(TEXT("ALTITUDE   %4.0f m MSL   %3.0f AGL"),
                 State.positionWorldM.z + Glider->GetLandingElevationM(), GroundClearance),
                 White, 42.0f, 102.0f);
        DrawText(FString::Printf(TEXT("VARIO      %+4.1f m/s"), State.velocityWorldMps.z),
                 State.velocityWorldMps.z < -3.0 ? FLinearColor::Yellow : White,
                 42.0f, 126.0f);
        const FString FlightStatus = Glider->IsGroundLaunching()
            ? FString::Printf(
                TEXT("%s  %.0f%%  RUN %.1f m/s  HOLD SPACE%s"),
                ANSI_TO_TCHAR(Glider->GetLaunchPhaseName()),
                Glider->GetLaunchInflation() * 100.0,
                Glider->GetLaunchRunSpeedMps(),
                Glider->AreLaunchBrakeSidesCrossed()
                    ? TEXT("  BRAKES WING-L/R") : TEXT(""))
            : (Telemetry.stalled
                ? TEXT("STALL")
                : TEXT("ARROWS BRAKES   A/D SHIFT   N GROUND START   R RESET"));
        DrawText(FlightStatus,
                 !Glider->IsGroundLaunching() && Telemetry.stalled
                    ? FLinearColor::Red
                    : FLinearColor(0.55f, 0.8f, 1.0f),
                 42.0f, 150.0f);
        const int32 LeftSteps = FMath::RoundToInt(Controls.leftBrake * 5.0);
        const int32 RightSteps = FMath::RoundToInt(Controls.rightBrake * 5.0);
        const int32 ShiftSteps = FMath::RoundToInt(Controls.weightShift * 5.0);
        const int32 BarSteps = FMath::RoundToInt(Controls.accelerator * 5.0);
        DrawText(FString::Printf(TEXT("BRAKE L %d/5  R %d/5  SHIFT %+d  BAR %d/5"),
                 LeftSteps, RightSteps, ShiftSteps, BarSteps),
                 FLinearColor(0.65f, 0.9f, 1.0f), 42.0f, 178.0f);

        const TCHAR* WeatherName = TEXT("LOCAL ROTOR");
        switch (Glider->GetWeatherMode())
        {
            case Parapenting::Physics::WeatherMode::Chill:
                WeatherName = TEXT("CHILL"); break;
            case Parapenting::Physics::WeatherMode::Ridge:
                WeatherName = TEXT("RIDGE"); break;
            case Parapenting::Physics::WeatherMode::LocalizedRotor:
                WeatherName = TEXT("LOCAL ROTOR"); break;
            case Parapenting::Physics::WeatherMode::RotorEverywhere:
                WeatherName = TEXT("ROTOR EVERYWHERE"); break;
        }
        DrawText(FString::Printf(TEXT("AIR %s   ROTOR %3.0f%%  GUST %.1f  SHEAR %.1f m/s"),
                 WeatherName, Telemetry.rotorStrength * 100.0,
                 Telemetry.gustEnergyMps,
                 Telemetry.spanwiseAirflowShearMps),
                 Telemetry.rotorStrength > 0.45
                    ? FLinearColor::Yellow : FLinearColor(0.65f, 0.9f, 1.0f),
                 42.0f, 202.0f);
        DrawText(FString::Printf(TEXT("PRESET %s   %s  F11: TIME"),
                 ANSI_TO_TCHAR(Glider->GetWeatherPresetDisplayName()),
                 *Glider->GetLocalTimeDisplay()),
                 FLinearColor(0.66f, 0.88f, 1.0f), 292.0f, 226.0f);
        const auto Wind = Glider->GetBaseWindMps();
        DrawText(FString::Printf(TEXT("WIND %.1f m/s   ROUTE X %+3.1f  Y %+3.1f"),
                 FMath::Sqrt(Wind.x * Wind.x + Wind.y * Wind.y),
                 Wind.x, Wind.y),
                 FLinearColor(0.65f, 0.9f, 1.0f), 292.0f, 202.0f);
        DrawText(FString::Printf(TEXT("CANOPY L %2.0f%% R %2.0f%% FRONT %2.0f%% P %3.0f%% LOAD %+3.0f%%"),
                 Telemetry.leftCollapse * 100.0,
                 Telemetry.rightCollapse * 100.0,
                 Telemetry.frontalCollapse * 100.0,
                 Telemetry.canopyPressure * 100.0,
                 Telemetry.spanwiseLoadAsymmetry * 100.0),
                 (Telemetry.leftCollapse + Telemetry.rightCollapse) > 0.25
                    ? FLinearColor::Red : White,
                 42.0f, 226.0f);
        DrawText(FString::Printf(TEXT("%s  %4.0f m  %s  APPROACH %2.0f%% %s"),
                 ANSI_TO_TCHAR(Glider->GetLandingDisplayName()),
                 Glider->GetDistanceToTargetM(),
                 ANSI_TO_TCHAR(Glider->GetLandingPhaseName()),
                 Glider->GetApproachQuality() * 100.0,
                 Glider->IsApproachStabilized()
                    ? TEXT("STABLE") : TEXT("")),
                 Glider->IsApproachStabilized()
                    ? FLinearColor(0.38f, 1.0f, 0.55f)
                    : FLinearColor(0.88f, 0.94f, 0.72f),
                 42.0f, 250.0f);
        DrawText(FString::Printf(TEXT("WING [Q] %s %s   BRAKE %s  L %.0fN R %.0fN"),
                 ANSI_TO_TCHAR(Glider->GetWingDisplayName()),
                 ANSI_TO_TCHAR(Glider->GetWingSizeDisplayName()),
                 ANSI_TO_TCHAR(Glider->GetBrakeTravelDisplayName()),
                 Telemetry.leftBrakeForceN,
                 Telemetry.rightBrakeForceN),
                 FLinearColor(0.85f, 0.76f, 1.0f), 42.0f, 274.0f);
        DrawText(FString::Printf(TEXT("ROUTE %s   [ / ] CHANGE"),
                 ANSI_TO_TCHAR(Glider->GetRouteDisplayName())),
                 FLinearColor(0.72f, 1.0f, 0.76f), 42.0f, 298.0f);
        DrawText(FString::Printf(
                 TEXT("RISERS A %2.0f B %2.0f C %2.0f D %2.0f%%  %.0f N  FLEX %2.0f%%"),
                 Telemetry.aRiserLoad * 100.0,
                 Telemetry.bRiserLoad * 100.0,
                 Telemetry.cRiserLoad * 100.0,
                 Telemetry.dRiserLoad * 100.0,
                 Telemetry.lineLoadTotalN,
                 Telemetry.highLoadDeformation * 100.0),
                 FLinearColor(0.72f, 0.82f, 1.0f), 292.0f, 274.0f);
        // What the two carabiners are holding. Weight shift is a force at
        // these points, so it is worth being able to watch it.
        DrawText(FString::Printf(
                 TEXT("CARABINER L %4.0f N  R %4.0f N   CG %+.0f mm"),
                 Telemetry.leftCarabinerLoadN,
                 Telemetry.rightCarabinerLoadN,
                 Telemetry.pilotCgOffsetM * 1000.0),
                 FLinearColor(0.72f, 0.82f, 1.0f), 292.0f, 298.0f);
        DrawText(FString::Printf(
                 TEXT("%s  PILOT %.0f kg  BALLAST %.0f  ALL-UP %.0f kg  %.2f kg/m2"),
                 ANSI_TO_TCHAR(Glider->GetHarnessDisplayName()),
                 Glider->GetPilotMassKg(), Glider->GetBallastKg(),
                 Telemetry.allUpMassKg, Telemetry.wingLoadingKgM2),
                 Glider->IsWithinRecommendedWingRange()
                    ? FLinearColor(0.78f, 0.92f, 0.86f)
                    : FLinearColor(1.0f, 0.48f, 0.2f),
                 292.0f, 298.0f);
        if (Glider->IsRecordingTelemetry())
            DrawText(TEXT("REC"), FLinearColor::Red, 530.0f, 40.0f,
                     GEngine->GetSmallFont(), 1.1f);
        if (Glider->IsRecordingReplay())
            DrawText(FString::Printf(TEXT("REPLAY REC  %d FRAMES"),
                     Glider->GetReplayFrameCount()),
                     FLinearColor(1.0f, 0.25f, 0.2f), 530.0f, 62.0f,
                     GEngine->GetSmallFont(), 1.0f);
        else if (Glider->IsPlayingReplay())
            DrawText(TEXT("REPLAY PLAYBACK"),
                     FLinearColor(0.3f, 0.85f, 1.0f), 530.0f, 62.0f,
                     GEngine->GetSmallFont(), 1.0f);
        else if (Glider->IsGhostVisible())
            DrawText(TEXT("GHOST ON"),
                     FLinearColor(0.25f, 0.9f, 1.0f), 530.0f, 62.0f,
                     GEngine->GetSmallFont(), 1.0f);
        if (Glider->GetReplayLibraryCount() > 0
            && !Glider->IsRecordingReplay())
            DrawText(FString::Printf(TEXT("REPLAY %d/%d %s  ;/' BROWSE  P PLAY"),
                     Glider->GetSelectedReplayNumber(),
                     Glider->GetReplayLibraryCount(),
                     *Glider->GetReplayDisplayLabel()),
                     FLinearColor(0.45f, 0.86f, 1.0f), 350.0f, 84.0f,
                     GEngine->GetSmallFont(), 0.9f);
        DrawText(FString::Printf(TEXT("TRAINING %s"),
                 ANSI_TO_TCHAR(Glider->GetScenarioDisplayName())),
                 FLinearColor(1.0f, 0.82f, 0.34f), 42.0f, 322.0f);
        DrawText(FString::Printf(TEXT("CAM %s  [M]"),
                 ANSI_TO_TCHAR(Glider->GetCameraModeDisplayName())),
                 FLinearColor(0.62f, 0.85f, 1.0f), 362.0f, 322.0f);
        DrawText(FString::Printf(TEXT("ACCESS %s [F8]"),
                 ANSI_TO_TCHAR(Glider->GetAccessibilityProfileName())),
                 FLinearColor(0.68f, 0.9f, 0.82f), 42.0f, 346.0f);
        DrawText(FString::Printf(TEXT("KEYS %s [F9]"),
                 ANSI_TO_TCHAR(Glider->GetKeyboardLayoutName())),
                 FLinearColor(0.68f, 0.9f, 0.82f), 312.0f, 346.0f);
        DrawText(FString::Printf(TEXT("GRAPHICS %s [F10]"),
                 ANSI_TO_TCHAR(Glider->GetGraphicsProfileName())),
                 FLinearColor(0.66f, 0.82f, 1.0f), 42.0f, 370.0f);
        DrawText(Glider->GetBindingCaptureText(),
                 Glider->IsCapturingBinding()
                    ? FLinearColor(0.35f, 0.95f, 1.0f)
                    : FLinearColor(0.62f, 0.78f, 0.88f),
                 252.0f, 370.0f, GEngine->GetSmallFont(), 0.76f);
        const bool bSiteWindOkay =
            Glider->GetSiteWindAssessment()
                == Parapenting::Physics::SiteWindAssessment::Suitable;
        DrawText(FString::Printf(TEXT("%s  - TRAINING ONLY"),
                 ANSI_TO_TCHAR(Glider->GetSiteWindAssessmentName())),
                 bSiteWindOkay
                    ? FLinearColor(0.38f, 1.0f, 0.55f)
                    : FLinearColor(1.0f, 0.55f, 0.18f),
                 42.0f, 394.0f);
        DrawText(ANSI_TO_TCHAR(Glider->GetLaunchHazardText()),
                 FLinearColor(0.96f, 0.78f, 0.44f), 42.0f, 418.0f,
                 GEngine->GetSmallFont(), 0.82f);
        DrawText(ANSI_TO_TCHAR(Glider->GetLandingCircuitText()),
                 FLinearColor(0.72f, 0.86f, 1.0f), 42.0f, 442.0f,
                 GEngine->GetSmallFont(), 0.78f);
        if (Glider->IsAirflowVisualizationEnabled())
            DrawText(TEXT("AIRFLOW [F]  CYAN WIND  ORANGE LIFT  BLUE SINK  MAGENTA ROTOR"),
                     FLinearColor(0.4f, 0.95f, 1.0f),
                     Canvas->SizeX * 0.5f - 290.0f, Canvas->SizeY - 48.0f,
                     GEngine->GetSmallFont(), 1.0f);
        if (Glider->IsGeometryVisualizationEnabled())
            DrawText(TEXT("GEOMETRY [SHIFT+G]  ORANGE LE  BLUE TE  GREEN CELL CROWN"
                          "  A/A'/B/C NODES"),
                     FLinearColor(1.0f, 0.85f, 0.45f),
                     Canvas->SizeX * 0.5f - 290.0f, Canvas->SizeY - 68.0f,
                     GEngine->GetSmallFont(), 1.0f);
        const float ChallengeX = Canvas->SizeX - 344.0f;
        DrawRect(FLinearColor(0.01f, 0.025f, 0.045f, 0.82f),
                 ChallengeX, 24.0f, 320.0f, 116.0f);
        DrawText(FString::Printf(TEXT("CHALLENGE  %03.0f / 1000   BEST %03.0f"),
                 Glider->GetChallengeScore(), Glider->GetChallengeBestScore()),
                 FLinearColor(1.0f, 0.82f, 0.34f), ChallengeX + 16.0f, 40.0f);
        DrawText(ANSI_TO_TCHAR(Glider->GetChallengeFeedback()),
                 Glider->IsChallengeComplete()
                    ? FLinearColor(0.35f, 1.0f, 0.55f)
                    : FLinearColor(0.75f, 0.9f, 1.0f),
                 ChallengeX + 16.0f, 68.0f);
        DrawRect(FLinearColor(0.12f, 0.18f, 0.23f, 1.0f),
                 ChallengeX + 16.0f, 104.0f, 288.0f, 10.0f);
        DrawRect(FLinearColor(0.2f, 0.78f, 1.0f, 1.0f),
                 ChallengeX + 16.0f, 104.0f,
                 288.0f * Glider->GetChallengeProgress(), 10.0f);
        DrawRect(FLinearColor(0.01f, 0.025f, 0.045f, 0.82f),
                 ChallengeX, 148.0f, 320.0f, 76.0f);
        DrawText(FString::Printf(TEXT("WEATHER  %s"),
                 *Glider->GetLiveWeatherStatus()),
                 Glider->IsLiveWeatherActive()
                    ? FLinearColor(0.32f, 0.9f, 1.0f)
                    : FLinearColor(0.72f, 0.78f, 0.84f),
                 ChallengeX + 16.0f, 160.0f);
        if (Glider->IsLiveWeatherActive())
            DrawText(FString::Printf(TEXT("MODEL AGE %.0f MIN   [I] REFRESH"),
                     Glider->GetLiveWeatherAgeMinutes()),
                     FLinearColor(0.72f, 0.86f, 0.96f),
                     ChallengeX + 16.0f, 188.0f);
        DrawRect(FLinearColor(0.01f, 0.025f, 0.045f, 0.82f),
                 ChallengeX, 232.0f, 320.0f, 72.0f);
        DrawText(FString::Printf(TEXT("PILOT %s   XP %.0f   MEDALS %d"),
                 ANSI_TO_TCHAR(Glider->GetPilotRankName()),
                 Glider->GetPilotExperience(),
                 Glider->GetPilotMedalCount()),
                 FLinearColor(0.96f, 0.78f, 0.28f),
                 ChallengeX + 16.0f, 244.0f);
        DrawRect(FLinearColor(0.12f, 0.18f, 0.23f, 1.0f),
                 ChallengeX + 16.0f, 278.0f, 288.0f, 9.0f);
        DrawRect(FLinearColor(0.96f, 0.62f, 0.18f, 1.0f),
                 ChallengeX + 16.0f, 278.0f,
                 288.0f * Glider->GetPilotRankProgress(), 9.0f);
        if (Telemetry.deepStall > 0.35 || FMath::Abs(Telemetry.spin) > 0.3
            || Telemetry.leftCravat + Telemetry.rightCravat > 0.08)
        {
            const TCHAR* Incident = Telemetry.deepStall > 0.35
                ? TEXT("DEEP STALL - HANDS UP")
                : (FMath::Abs(Telemetry.spin) > 0.3
                    ? TEXT("SPIN - RELEASE DEEP BRAKE")
                    : TEXT("CRAVAT - CONTROL HEADING / PUMP TIP"));
            DrawText(Incident, FLinearColor::Red,
                     Canvas->SizeX * 0.5f - 155.0f, 154.0f,
                     GEngine->GetMediumFont(), 1.15f);
        }
        if (Telemetry.thermalLiftMps > 0.15)
        {
            const FString CloudBase = Telemetry.cloudBaseClearanceM < 1500.0
                ? FString::Printf(TEXT("   BASE %.0fm"),
                    Telemetry.cloudBaseClearanceM)
                : FString();
            DrawText(FString::Printf(
                     TEXT("THERMAL +%.1f m/s   LIFE %.0f%%%s"),
                     Telemetry.thermalCoreStrength,
                     Telemetry.thermalLifecycle * 100.0,
                     *CloudBase),
                     FLinearColor(1.0f, 0.62f, 0.18f),
                     Canvas->SizeX * 0.5f - 150.0f, 82.0f,
                     GEngine->GetMediumFont(), 1.05f);
        }

        const float NavX = Canvas->SizeX - 344.0f;
        const float NavY = 310.0f;
        DrawRect(FLinearColor(0.01f, 0.025f, 0.045f, 0.82f),
                 NavX, NavY, 320.0f, 112.0f);
        DrawText(FString::Printf(TEXT("NAV %d/3  %s"),
                 Glider->GetActiveWaypointNumber(),
                 ANSI_TO_TCHAR(Glider->GetActiveWaypointName())),
                 FLinearColor(0.52f, 0.9f, 1.0f),
                 NavX + 16.0f, NavY + 14.0f);
        DrawText(FString::Printf(
                 TEXT("%.1f km  BRG %03.0f  CRAB %+3.0f"),
                 Navigation.distanceM / 1000.0,
                 Navigation.bearingWorldDegrees,
                 Navigation.crabAngleDegrees),
                 White, NavX + 16.0f, NavY + 40.0f,
                 GEngine->GetSmallFont(), 0.9f);
        DrawText(FString::Printf(
                 TEXT("ARRIVAL %+.0f m   L/D %.1f / %.1f"),
                 Navigation.predictedArrivalHeightM,
                 Navigation.requiredGlideRatio,
                 Navigation.availableGroundGlideRatio),
                 Navigation.reachable
                    ? FLinearColor(0.35f, 1.0f, 0.58f)
                    : FLinearColor(1.0f, 0.5f, 0.16f),
                 NavX + 16.0f, NavY + 64.0f,
                 GEngine->GetSmallFont(), 0.9f);
        DrawText(FString::Printf(TEXT("%s   GS %.1f m/s"),
                 ANSI_TO_TCHAR(Parapenting::Physics::SpeedToFlyCueName(
                    Navigation.speedToFly)),
                 Navigation.predictedGroundSpeedMps),
                 FLinearColor(1.0f, 0.8f, 0.3f),
                 NavX + 16.0f, NavY + 88.0f,
                 GEngine->GetSmallFont(), 0.88f);

        if (Glider->HasLanded())
        {
            const auto& D = Glider->GetFlightDebrief();
            DrawRect(FLinearColor(0.01f, 0.02f, 0.03f, 0.9f),
                     Canvas->SizeX * 0.5f - 260.0f,
                     Canvas->SizeY * 0.5f - 115.0f, 520.0f, 230.0f);
            DrawText(Glider->WasHardLanding() ? TEXT("HARD LANDING") : TEXT("LANDED"),
                     Glider->WasHardLanding() ? FLinearColor::Red : FLinearColor::Green,
                     Canvas->SizeX * 0.5f - 90.0f, Canvas->SizeY * 0.5f - 88.0f,
                     GEngine->GetMediumFont(), 1.3f);
            DrawText(FString::Printf(TEXT("%.0f m FROM TARGET   V/S %.1f   SPEED %.1f — R"),
                     Glider->GetLandingDistanceM(),
                     Glider->GetTouchdownVerticalSpeedMps(),
                     Glider->GetTouchdownHorizontalSpeedMps() * 3.6),
                     White, Canvas->SizeX * 0.5f - 175.0f,
                     Canvas->SizeY * 0.5f - 48.0f);
            DrawText(FString::Printf(
                     TEXT("SAFETY %.0f   EFF %.0f   THERMAL %.0f   LANDING %.0f"),
                     D.safetyRating, D.efficiencyRating,
                     D.thermalRating, D.landingRating),
                     FLinearColor(0.45f, 0.9f, 1.0f),
                     Canvas->SizeX * 0.5f - 205.0f,
                     Canvas->SizeY * 0.5f - 12.0f);
            DrawText(FString::Printf(
                     TEXT("OVERALL %.0f   DIST %.1f km   GAIN +%.0f m"),
                     D.overallRating, D.horizontalDistanceM / 1000.0,
                     D.altitudeGainM),
                     FLinearColor(1.0f, 0.8f, 0.32f),
                     Canvas->SizeX * 0.5f - 180.0f,
                     Canvas->SizeY * 0.5f + 22.0f);
            DrawText(FString::Printf(
                     TEXT("EVENTS  ASYM %d  FRONT %d  CRAVAT %d  STALL/SPIN %d"),
                     D.asymmetricCollapseEvents, D.frontalCollapseEvents,
                     D.cravatEvents, D.stallOrSpinEvents),
                     White, Canvas->SizeX * 0.5f - 210.0f,
                     Canvas->SizeY * 0.5f + 55.0f,
                     GEngine->GetSmallFont(), 0.9f);
            DrawText(ANSI_TO_TCHAR(Glider->GetDebriefFocusText()),
                     White, Canvas->SizeX * 0.5f - 220.0f,
                     Canvas->SizeY * 0.5f + 83.0f,
                     GEngine->GetSmallFont(), 0.82f);
        }
    }

    DrawText(TEXT("LAND AT THE WHITE MARKER"), White,
             Canvas->SizeX * 0.5f - 120.0f, 30.0f);
    DrawText(TEXT("AIR: 1 CHILL  2 RIDGE  3 LOCAL ROTOR  4 ROTOR EVERYWHERE"),
             FLinearColor(0.75f, 0.88f, 1.0f),
             Canvas->SizeX * 0.5f - 250.0f, 54.0f);
    DrawText(TEXT("WINGS: 5 TRAINING   6 EPIC 2   7 SPORT B   8 EPSILON DLS"),
             FLinearColor(0.82f, 0.76f, 1.0f),
             Canvas->SizeX * 0.5f - 245.0f, 78.0f);
    DrawText(TEXT("SIV TRAINING: Z LEFT TIP   X FRONTAL   C RIGHT TIP"),
             FLinearColor(1.0f, 0.68f, 0.38f),
             Canvas->SizeX * 0.5f - 225.0f, 102.0f);
    DrawText(TEXT("ROUTES: [ PREVIOUS   ] NEXT (RESTARTS FLIGHT)"),
             FLinearColor(0.65f, 1.0f, 0.72f),
             Canvas->SizeX * 0.5f - 210.0f, 126.0f);
    DrawText(TEXT("T TELEMETRY   U RECORD   P REPLAY   G GHOST"),
             FLinearColor(0.78f, 0.85f, 0.9f),
             Canvas->SizeX * 0.5f - 205.0f, 150.0f);
    DrawText(TEXT("Y: NEXT DETERMINISTIC TRAINING SCENARIO"),
             FLinearColor(1.0f, 0.82f, 0.34f),
             Canvas->SizeX * 0.5f - 200.0f, 174.0f);
    DrawText(TEXT("WIND: ,/. ROTATE  -/= SPEED"),
             FLinearColor(0.64f, 0.86f, 1.0f),
             Canvas->SizeX * 0.5f - 145.0f, 198.0f);
    DrawText(TEXT("I: LOAD INTERLAKEN MODEL WIND — NOT FOR FLIGHT DECISIONS"),
             FLinearColor(1.0f, 0.62f, 0.24f),
             Canvas->SizeX * 0.5f - 245.0f, 222.0f);
    DrawText(TEXT("F5: PRE-FLIGHT WEATHER BRIEFING"),
             FLinearColor(0.55f, 0.9f, 1.0f),
             Canvas->SizeX * 0.5f - 175.0f, 246.0f);
    DrawPreflightBriefing(Glider);
}
