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

Terms below were checked in August 2026, but the licence text at download time
is the one that binds. Whichever route we take, the licence must be committed
alongside the asset (see below) and read before use.

**Option A — the Third Person feature pack. Free, zero download, already
licensed.** `SKM_Manny` / `SKM_Quinn` ship with the engine and are covered by
the licence you already hold. Correct scale, Mannequin skeleton, finger bones,
LODs, and no procurement decision at all. This is the fastest way to make the
blockout visible and it is what the code currently looks for.

**Option B — MetaHuman Creator. Free, and the licence changed in our favour.**
Since June 2025 MetaHuman is covered by the standard Unreal Engine licence:
free under $1M revenue, usable in other engines, and MetaHuman characters may
be sold on Fab. This removes the licence hesitation recorded in earlier
versions of this document. Best quality-per-effort for a *real* pilot: full
body and face rig, finger bones, LODs, clothing system. Costs a heavier
pipeline and still needs the seated-deformation check like anything else.

**Option C — Fab.** Large catalogue, often Mannequin-skeleton already. Two
traps: sellers rarely test seated deformation, and "free" on Fab does not
reliably mean "free for commercial use" — the licence tier is per listing and
must be read, not assumed.

**Option D — Mixamo. Free, but treat it as unmaintained.** Adobe has left it in
maintenance mode with repeated outages through 2025–26 and support statements
that it is no longer supported. Fine for a throwaway test, poor thing to make a
shipped product depend on. Free alternatives in the same niche: Mesh2Motion
(open source), AccuRIG, Blender Rigify for rigging your own mesh, Quaternius
(CC0) for animation.

**Option E — commission or author in-house.** Only worth it if the harness and
pilot must be modelled as one garment system. Weeks, not days.

### Recommendation

Option A now, Option B for the shipped pilot. A is free and unblocks the rig
work today; B is free, higher quality than anything we would buy, and no longer
carries the licence question that made it a last resort here before.

The plan's own position is that the character is not the hard part — the causal
chain is — so take the cheapest asset that passes the acceptance checklist and
spend the effort on the harness and the IK. On current terms that means paying
nothing.

### Why not generate the character from code

The harness is procedural because it is project-specific: no one sells a
paragliding harness built around *our* carabiner separation. A human body is
the opposite. A skinned humanoid could be emitted from code — a glTF is JSON
plus a binary buffer, and joints, weights and a Mannequin-compatible hierarchy
can all be written directly — but the result would be a smooth mannequin with
hand-guessed weights, no LODs and no clothing. That is not better than the free
engine mannequin, which is already rigged, weighted, LODed and licensed. Code
generation here would cost real time to land something strictly worse than
Option A, so it is not the recommended path.

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
  Mannequin-skeleton mesh", not "retarget an arbitrary character". Both are
  editor data assets and cannot be authored from code, which is why they are
  the remaining Stage 2 work rather than something already landed.

Step 1 of the blockout path is complete: the harness is project-owned
procedural geometry with seat, back protector, reserve volume, shoulder straps,
leg straps and carabiner hang points, rebuilt whenever the harness is cycled so
the webbing follows the active carabiner separation.

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
