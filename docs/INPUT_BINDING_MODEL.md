# Runtime input binding model

The simulator retains three instant keyboard presets and adds a fourth
persistent custom profile. `F6` selects one of the six discrete flight-control
actions and `F7` listens for the next keyboard key. `Escape` cancels capture.
All preset keys are disjoint from simulator commands; switching layout cannot
also change a wing, weather state, equipment item or replay state.

Bindable actions are:

- weight shift left/right;
- left/right brake step;
- both brakes one step deeper; and
- both brakes one step toward release.

The binding core is engine-independent and deterministic. If a captured key is
already assigned to another bindable flight action, the two keys are swapped
instead of silently leaving duplicate controls. Keys reserved for reset, HUD,
accessibility, layout/capture and function-key control are rejected. At runtime
the Unreal adapter additionally rejects any key owned by a non-remappable game
action, preventing a brake command from also changing weather, equipment or
replay state.

Successful capture activates the custom profile immediately, rebuilds Unreal's
key maps and persists all six key names in `Saved/PlayerProgress.ini`. The HUD
shows the selected action, current key, capture state, conflicts and result.
Controller axes remain independent and are never removed by keyboard rebinding.

The current capture surface intentionally covers the six primary flight
actions. General remapping of every simulator/debug command and a full settings
menu remain separate product-interface work.
