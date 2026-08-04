# Pilot character asset guide

Companion to [GLIDER_PILOT_VISUAL_MASTER_PLAN.md](GLIDER_PILOT_VISUAL_MASTER_PLAN.md).
Covers Stage 2 only: what to acquire, how to judge it, and how to integrate it
against the existing rig contract.

Status: **blockout implemented, asset decision still open.** The runtime now
carries a `UPoseableMeshComponent` (`PilotCharacter`) driven from
`GliderRigSnapshot`, a project-owned procedural harness mesh, and pose-family
blending. The primitive body parts hide themselves whenever a skeletal mesh
resolves.

There is still no character asset in the repository, and one further gap is
worth being explicit about: **`SKM_Manny` is not engine content.** It ships with
the Third Person feature pack, which copies it into `/Game/Characters/...`.
This project has no `Content/Characters` directory, so the mesh does not
resolve, the mannequin is invisible and the primitive blockout stays on screen.
Adding the Third Person feature pack in the editor is enough to light it up.
The pawn now logs a warning naming this document when the mesh is missing,
rather than silently falling back.

## What is already fixed, and therefore constrains the asset

These are not preferences. They come from solved physics and drawn hardware,
and the character must fit them rather than the reverse.

| Quantity | Value | Source |
| --- | --- | --- |
| Engine | UE 5.8 | `Parapenting.uproject` |
| Working units | centimetres, rig space, +X forward, +Z up | `PilotRigToActor()` |
| Pilot mass | 85 kg default | `EquipmentSetup::pilotMassKg` |
| Carabiner separation | 0.42 m (±21 cm lateral) | `HarnessGeometry::carabinerSeparationM` |
| Carabiner height | +34 cm above rig origin | `CarabinerLocalCm` |
| Riser length above carabiner | 45 cm | `RiserTopLocalCm` |
| Shoulder webbing anchor | (−9, ±12, +34) cm | `UpdateSuspensionVisual` |
| Seat webbing anchor | (+7, ±15, −24) cm | `UpdateSuspensionVisual` |
| Current head centre | (−12, 0, +53) cm | `UpdatePilotVisual` |
| Current hip | (+1, ±14, −13) cm | `UpdatePilotVisual` |
| Seated knee / boot | z ≈ −31 cm / −88 cm | `UpdatePilotVisual` |

The seated pilot therefore occupies roughly **150 cm from boot to crown** in
rig space, with the load path passing through the shoulders at the same height
as the carabiners. An asset authored for a standing 180 cm character is
correct; one authored at Mannequin scale-with-heroic-proportions is not, and
one authored in metres will arrive 100× small.

`GliderRigSnapshot` already publishes achieved shoulder, elbow and hand targets
per side plus rig offset and rotation. The character must be drivable from
those targets — it does not get to invent its own arm animation.

## Required properties

Hard requirements. An asset failing any of these costs more to fix than to
replace.

1. **Skeletal mesh with a standard humanoid hierarchy.** UE5 Mannequin
   (`SK_Mannequin` / UE5 `Manny`) bone naming preferred; anything retargetable
   through IK Rig is acceptable.
2. **Full finger bones**, three joints per finger plus thumb. Stage 3 animates
   open / acquire / wrapped / loaded / released grips. A mitten hand kills that
   stage outright.
3. **Twist bones** on forearm and upper arm, or the wrist shears when brake
   force rotates the grip.
4. **Clean seated deformation.** Test the hip and knee at the seated angles
   above before accepting; many marketplace characters are weighted for
   standing locomotion only and collapse at a 90° hip.
5. **Separable clothing/equipment**, or at minimum a mesh that a harness can be
   modelled around without interpenetration. Helmet as its own mesh or socket.
6. **Real-world scale in centimetres**, Z-up, X-forward on import.
7. **FBX or GLB source with the skeleton**, not a baked engine asset only. We
   need to re-export when the harness changes.
8. **A licence permitting commercial use in a shipped game, without
   attribution-in-runtime obligations**, and permitting modification.

Nice to have, in priority order: LODs; a blendshape set for a neutral face;
separate boot geometry; A-pose rather than T-pose source.

Explicitly *not* needed: facial performance rig, dialogue visemes, a locomotion
animation library. The plan already says silhouette, posture, grip and load
response matter first, and every pose in the game is IK-driven or hand-authored
for launch run, flare, landing run and fallen.

## Sourcing options

I have not verified current licence terms for any of these — terms change, and
the licence text at download time is the one that binds. Whichever route we
take, the licence file must be committed alongside the asset (see below) and
read before use.

**Option A — UE Marketplace / Fab character.** Best fit: assets are already
Mannequin-skeleton, UE-scaled and LODed, so integration is hours not days.
Cost is typically low tens of dollars. Risk: seated deformation is rarely
tested by the seller. Recommended if we find one with finger bones and a
usable body type.

**Option B — Mixamo.** Free, auto-rigged, well-known. Risk: the Mixamo
skeleton needs retargeting to UE5, hand quality varies, and the characters are
stylised. Workable but it costs a retarget step and probably a hand fix.

**Option C — Character Creator / MetaHuman-adjacent pipelines.** Highest
quality and full control over body type and clothing, at the cost of a much
heavier pipeline and, for MetaHuman specifically, licence terms that are worth
reading closely before committing a shipped product to them.

**Option D — Commission or author in-house.** Only worth it if the harness and
pilot need to be modelled as one garment system. Weeks, not days.

My recommendation is **Option A, with Option B as the fallback** if nothing
with proper hands turns up in budget. The plan's own position is that the
character is not the hard part — the causal chain is — so buy the cheapest
asset that passes the acceptance checklist and spend the effort on the harness
and the IK.

## What I need from you

Pick one:

1. **Provide the asset.** Drop the FBX/GLB plus its licence file in
   `Content/Characters/Pilot/Source/` and tell me it is there.
2. **Authorize me to select one.** Say so, give a budget ceiling and a
   preferred option letter above, and I will shortlist candidates with links,
   prices and licence summaries for you to approve **before** anything is
   downloaded or purchased. I will not download files or complete a purchase
   without your explicit go-ahead on the specific item.
3. **Unblock the stage without the asset** — see the blockout path below.

## Blockout path (unblocks Stage 2 today)

Stage 2's real deliverable is not "a bought character". It is: pelvis driven by
payload motion, chest and head by filtered inertial response, limbs by full-body
IK, and a pose-family state machine with inertial blending. All of that can be
built and tested against a **placeholder skeletal mesh** — the UE5 Mannequin
shipped with the engine is retargetable and has finger bones.

Proposed order, which removes the dependency from the critical path:

1. Author the harness mesh first. It is project-specific, nobody sells it, and
   its anchors are already fixed by the table above.
2. Stand up the IK Rig, IK Retargeter and pose-family state machine against the
   Mannequin.
3. Drive everything from `GliderRigSnapshot` and land the Stage 2 exit gates
   that do not mention art quality: no pops in pose transitions, hands stay
   attached under full harness roll and pitch.
4. Swap the Mannequin for the licensed pilot when it arrives. If step 2 is done
   through a retargeter, this is a data change, not a code change.

The one exit gate this cannot satisfy is "no Engine primitive is visible in the
live pilot" at *shippable* quality — the Mannequin is a placeholder, not a
paraglider pilot. Everything else is reachable now.

### Known limits of the current blockout

Both are expected at this stage and both are the IK Rig's job, not the poseable
component's:

- `UpdatePilotSkeleton` sets bone **translation only**. The skin follows the
  joints, but limbs do not twist about their own axis, so the mesh shears at
  the shoulder and wrist under brake load. Fixing this needs bone rotation,
  which is what the IK Rig replacing that function provides.
- Steps 2–3 above are still outstanding: there is no IK Rig, IK Retargeter or
  saved retargeter data asset yet, so today's swap path is "assign a
  Mannequin-skeleton mesh", not "retarget an arbitrary character".

## Acceptance checklist

Run against any candidate before committing to it.

- [ ] Imports at real scale; a 180 cm source measures ~180 cm in-editor.
- [ ] Retargets from the UE5 Mannequin with no limb-length change.
- [ ] Hip at seated angle, knee at seated angle: no candy-wrapper collapse.
- [ ] Fingers curl to a closed fist without self-intersection.
- [ ] Shoulder reaches the hands-up brake position without the deltoid tearing.
- [ ] Full harness roll and pitch: no shoulder-strap interpenetration.
- [ ] Triangle count and material count are sane for a chase-camera character.
- [ ] Licence permits commercial shipping and modification, and is committed.

## Repository placement and licence hygiene

```text
Content/Characters/Pilot/
  Source/           FBX/GLB as delivered, plus LICENSE.txt verbatim
  SK_Pilot.uasset
  IK_Pilot.uasset
  RTG_MannequinToPilot.uasset
  Materials/
```

Rules: commit the licence text verbatim next to the source, never only a URL.
Record vendor, item name, purchase date, licence version and any attribution
obligation in `docs/THIRD_PARTY_ASSETS.md`. Do not modify the delivered source
in place — re-export derived assets so the provenance stays checkable. If the
licence forbids redistributing the source in a public repository, keep the
source out of git and record the acquisition details instead.
