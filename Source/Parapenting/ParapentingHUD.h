#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "ParapentingHUD.generated.h"

UCLASS()
class PARAPENTING_API AParapentingHUD : public AHUD
{
    GENERATED_BODY()

public:
    virtual void DrawHUD() override;

private:
    void DrawCompactHUD(const class AParagliderPawn* Glider, bool bMinimal);
    void DrawPreflightBriefing(const class AParagliderPawn* Glider);
    void DrawFlightDeck(const class AParagliderPawn* Glider);
};
