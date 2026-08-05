# The pitch pendulum: both ends of it

What a paraglider does in pitch is a pendulum with an engine. The pilot is
roughly the centre of mass and the wing is what moves, so the interesting
quantity is the angle between them — the canopy ahead of the pilot or behind.
The game had the forward half of that and not the back half, and this document
is what was missing, why it matters, and what is now modelled.

## The two halves are not one signal with two signs

| | canopy BEHIND the pilot | canopy AHEAD of the pilot |
|---|---|---|
| what causes it | brake applied, thermal entry, the recovery half of a surge | brake released, thermal exit, collapse re-inflation |
| incidence | **rises** | **falls** |
| what the pilot feels | pressed into the harness, brakes go firm, wing quiet above and behind | light in the harness, brakes go soft, wing rushes ahead and the horizon drops |
| what it ends in if unchecked | **stall** — parachutal, then full | **frontal collapse** |
| the correction | let the brakes UP | brake to check the surge |

That table is the whole point. A pendulum modelled as one number with a sign
gets the *pose* right and the *consequences* backwards, because the two ends
lead to opposite incidents and opposite corrections.

### Back, and how it stalls

Brake, and the wing slows while the pilot keeps going: the canopy swings aft.
Its chord rotates nose-up against the same airflow, so its incidence rises by
the swing angle on top of whatever the brake already added. Hold it and the
wing reaches the stall from two directions at once — the brake's own camber
change and the pendulum's rotation. This is why a *fast* deep pull stalls a
wing that the same brake position, reached slowly, would only slow down: the
swing is a rate effect and it overshoots.

Measured here, on the coupled solver, 60% brake held for two seconds from
trim: the canopy swings 12° behind, the incidence climbs from 4.9° to 20.7°,
and the wing stalls. **The back-swing peaks about 0.8 s after the brake is
released** — the pendulum carries on when the input stops, which is exactly
why the recovery is to let the brakes up early rather than to hold them and
wait.

### Front, and how it collapses

Release, and the wing accelerates ahead of the pilot; its incidence falls by
the swing angle. At the bottom of a surge the wing is closest to the incidence
where the leading edge stops being pressurised, which is a frontal collapse.
An unchecked surge after a big pitch event is the classic way to fold a wing
that was already flying again.

The other half of the same mechanism: an unchecked surge is also how energy
leaves the aircraft. The wing dives, gains speed, and the pilot swings under
it — and the next back-swing is bigger than it needed to be.

## What is in the model

**The swing feeds the incidence.** `ParagliderDynamics` computes the canopy's
own incidence as the body's incidence plus the canopy's swing against the
pilot. Before this, the swing existed in the pose and nowhere else: through a
firm brake pulse the wing went 32° behind the pilot while the incidence the
model flew on went *down*, six degrees below trim to seventeen. The wing swung
and the aircraft did not notice, which is why a pendulum could be seen and
never felt.

With the loop closed, both incidents above emerge rather than being scripted —
the stall from the back half, and the collapse-proneness from the front half,
because the existing `incidenceUnloading` term reads the same incidence.

**The swing is quicker than it was.** The period came from `sqrt(g / lineLength)`
— the period of a weight on a string, 5.7 s. A canopy's pitch oscillation is
not that: what restores it is the lift it makes when its incidence changes, and
this project measured that mode on the real geometry at **1.86 s** by
linearising the coupled solver (`pitch_eigenmodes`, cross-checked against a
1200 s trace). The game now runs 4.0 s, not 1.86 — see the limits below — and
the damping is 0.55 rather than the old near-critical 0.72, so the wing swings
back and forth two or three times instead of moving once and stopping.

**The camera reads it.** The two halves drive opposite cues, bounded well below
the collapse cues because this happens all flight: back lifts and pitches the
view up, front drops and pitches it down, and the comfort profiles govern both.

## What is gated

- `physics_tests`: the brake-pulse sequence — the canopy ends up behind with
  incidence *above* trim and past half the stall angle; on the release it
  swings ahead with incidence *below* trim; and the swing reverses at least
  four times, which is what makes it a pendulum rather than a new resting
  position.
- `physics_tests`: the camera's two halves move in opposite directions, are
  bounded at 8°, and collapse under Minimal Motion.
- The existing zoom gate — a hard brake from 16 m/s must still convert speed
  into more than a metre of climb — which is what bounds the pendulum's
  aggressiveness from the other side.

## Stated limits, because they are the honest part

**The gain is 0.35, not the geometric 1.0.** Rotating the chord *is* changing
the incidence, so the physical gain is one. This lumped model already rotates
the whole aircraft toward trim incidence through `pitchStiffness`, so the full
swing on top of it counts part of the same restoring twice — and swept, gain
1.0 is unstable at *every* damping tried: the swing grows through the manoeuvre
and peaks eight to twelve seconds after the brake is released, at 25° to 48°.
That is a divergence, not a pendulum, and it is the same loop-gain-above-one
mechanism `PHYSICS_TODO` item 11 is about on the real solver.

**The period is 4.0 s, not the measured 1.86 s.** At 1.86 s the wing swings
back into a stall before a hard brake can convert its speed into height, and
the zoom gate fails at 0.86 m against a required metre. 4.0 s is the fastest
swept value that keeps it.

**The amplitude is roughly twice what the coupled solver gives.** This model's
brake deceleration is fierce enough to drive the swing to 20–30° where the
coupled solver reaches 12°. That is the legacy path's forcing, not the
pendulum's response, and it is one more reason the geometry-driven stack should
replace it — `PHYSICS_TODO` items 7 and 17.

All three are properties of the lumped legacy model that flies the game, not
claims about a wing. The coupled solver carries this angle as a real degree of
freedom, needs no gain factor, and measures the period rather than being told
it.
