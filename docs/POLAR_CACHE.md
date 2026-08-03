# The section polar cache

Level 10, strand 3. `PHYSICS_TODO` item 14.

Solving the section polar table costs ~723 ms: 21 brake stations, each a panel
factorisation and an incidence sweep. Cached, it is **4 ms**, and that 4 ms is
almost entirely the validation described below rather than the read.

| | cold | warm |
|---|---|---|
| section polar table | 723 ms | **4 ms** |
| suspension graph, trim load, stiffness curve, brake swing | 336 ms | 336 ms |
| **construction total** | **1059 ms** | **340 ms** |

## The part that is not the speedup

A cache is a file that can disagree with the geometry that produced it. A
section polar which silently disagrees with its own section is precisely what
this level was built away from — item 1 replaced the analytic table's *stated*
stall angle with a solved one, and a stale cache would put a stated number back
with none of the honesty of having declared it stated.

So the cache is validated twice, and it is the second check that carries the
design.

**1. The spec.** The whole `ComputedPolarSpec` is written into the file and
compared against the requested one on load. Catches a changed section,
Reynolds number, sweep range or brake resolution. A `static_assert` on
`sizeof` breaks the build if a field is added, so a new field cannot be
silently unserialised.

**2. The witness.** One canonical cold solve — the section at zero brake, zero
incidence, both surfaces attached, which is exactly how `Computed` seeds the
downward branch of its sweep and is therefore reproducible rather than
approximately reproducible — is stored, and **re-solved and compared on every
load**.

Check 2 is what makes this safe without a version constant somebody has to
remember to bump. If the viscous solver changes, or the boundary layer, or the
profile geometry, or the panelling — anything that leaves the inputs identical
and the answer different — the witness stops reproducing and the cache is
refused. It costs one section solve against the 723 ms it protects.

**Every failure means solve.** No file, unreadable, bad format, spec mismatch,
witness mismatch, truncation — all fall through to `Computed`. There is no path
where a doubtful cache is used anyway.

## Gated

`aerodynamics_tests` proves the four refusals every run, not just the hit:

- a round trip is **bit-identical** to the solved table across the stall and
  the brake axis, compared on what callers read rather than on a checksum;
- a section a thousandth of a chord different does not load the old table;
- a table whose **witness** no longer reproduces is refused — the test edits
  the stored witness in place, which stands in for the real hazard of a
  changed solver with an unchanged spec;
- a truncated file is refused rather than becoming a table with the wrong
  shape;
- and with the cache disabled entirely, solving from cold gives the same wing.

## Operating it

The directory is `Intermediate/PolarCache`, overridden by
`PARAPENTING_POLAR_CACHE`. Setting that variable **empty disables the cache**,
which is what a build wanting to prove it can still solve from cold should set.
Files are written to a temporary and renamed, so a process killed mid-write
leaves the previous cache rather than half a file that parses.

The cache is not committed and must never be: it is derived, machine-local, and
`Intermediate/` is ignored.

## What is left

The 336 ms that did not move. It is the suspension network solving itself cold
— trim load distribution, the line stiffness curve, and the brake swing curve,
each a 12000-iteration relaxation, about eleven of them. That is not a table
and cannot be cached the same way, because it depends on the line plan and the
payload rather than on the section alone. Warm-starting each solve from the
previous one is the obvious next move and nobody has tried it. See item 14.
