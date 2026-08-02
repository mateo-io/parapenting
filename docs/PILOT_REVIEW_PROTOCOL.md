# Expert pilot review protocol

Level 9's exit gate has two halves. The quantitative half is in
`docs/CALIBRATION_REPORT.md` and is closed. This is the other half: experienced
pilots confirming direction, control progression, pitch timing, pendulum timing
and stall approach **qualitatively**.

It has not been run. Until it has, the handling of this model is unvalidated in
the registry's sense — self-consistent, agreeing with four published numbers,
and never flown by anyone who flies.

## How to use this

Fly the reviewer, then ask. Do not show them this document first, do not show
them the calibration report, and do not tell them what the model is known to get
wrong — three of the four known disagreements are things a pilot would notice in
the first minute, and naming them in advance destroys the only evidence this
protocol can produce.

Each question is answerable without instruments, because a pilot's answer to
"how long did that take" is worth more than their answer to "what was the
period". Record the answer as given.

## Before flying: what the model can and cannot do

Tell the reviewer only this much, because flying outside it wastes the session:

- Hands-up to about a quarter brake is the usable envelope.
- Full accelerator and deep brake both leave the envelope and do not come back.
  This is known, it is measured, and it is not what is being reviewed.
- There is no thermal, no wind and no turbulence in the calibration
  configuration. Still air only.

## Session 1 — trim and the speed system

Hands up, straight, for two minutes.

1. Does the wing sit still, or is it hunting? If it is hunting, how long is one
   cycle — a breath, a few seconds, or ten?
2. Does the speed feel like an EN-B at trim, or like a slower or faster class?
3. Where does the wing sit relative to you? Overhead, ahead, behind?

*What the model does:* a slow speed-and-incidence mode near 20 s that decays.
Question 1 is asking whether it decays fast enough that a pilot would not call it
hunting.

## Session 2 — control progression

Symmetric brake, in 10% increments to 25%, holding each for ten seconds.

4. Where does the brake start to bite? Is there a dead band, and is it the
   length you would expect?
5. Between the first bite and a quarter brake, does the pressure build the way
   an EN-B's does — evenly, or does it go soft or hard somewhere?
6. Does the wing slow the way you expect for that much brake?

*What the model does:* 19% of handle travel is sewn-in slack and transmits
nothing at all. Question 4 is asking whether that dead band reads as a dead band
or as a fault.

## Session 3 — pitch and pendulum timing

Brake to 30%, hold two seconds, release cleanly. Repeat five times.

7. When you pull, how long before the wing goes back? Immediately, half a
   second, longer?
8. When you release, does the wing shoot? How far — does it go past overhead,
   and by how much?
9. Count the swings before it settles. One, two, three, more?
10. Is the timing of the shoot right for a wing this size, or does it feel fast
    or slow?

*What the model does:* the surge runs about four seconds and moves the wing
1.7 m fore-and-aft. Question 10 is the most valuable question in this document,
because the pendulum damping is the model's least defensible number — stated at
0.35 where the physics suggests nearer 0.06 — and a pilot's sense of surge
timing is the only external reference available for it.

## Session 4 — turning

Right brake to 35%, hold thirty seconds. Then the same on the left. Then weight
shift alone, both ways.

11. Which way does it turn, and does it bank into the turn or flat?
12. How long to establish, and how tight is the turn once it is?
13. Does weight shift alone do anything? Is it weaker than brake, and by roughly
    how much?
14. Does it feel coordinated, or is it skidding or slipping?

*What the model does:* direction, mirroring and bank are correct; the turn rate
is about a seventh of an EN-B's. Question 12 is expected to find that, and the
value is in how the reviewer describes it — a wing that will not turn, or a wing
that turns lazily?

## Session 5 — the stall approach

Brake ramped in slowly, over ten seconds, until something happens. Have the
reviewer talk while they do it.

15. Where does it start to feel different, and what changes first — pressure,
    sound, the wing's position, the sink?
16. Is there a warning before it lets go? What is the warning?
17. When it goes, how does it go — straight back, one side first, gently?
18. Would you have felt that coming on a real wing at the same point?

*What the model does:* separates at about 11° of incidence, which is early, and
does not come back once separated. Question 16 is asking whether the approach
carries the cues a pilot relies on, independent of whether the break point is
right.

## Closing questions

19. If this were a real wing, what class would you call it?
20. What is the single thing that feels most wrong?
21. What is the single thing that feels most right?
22. Would you use this to teach anyone anything? What, and what not?

## Recording

For each session record: reviewer's hours and classes flown, the answers as
given, and anything they said unprompted while flying — the unprompted
observations have consistently been the useful ones in aviation simulation
review, because they are not shaped by the question.

Disagreements between reviewers are data, not noise. Two pilots disagreeing
about whether the surge timing is right is worth more than either of them
agreeing with the model, and it should be recorded as a disagreement rather than
averaged.

## What a pass looks like

The plan's wording is that pilots "confirm direction, control progression, pitch
timing, pendulum timing, and stall approach qualitatively". Concretely:

- **Direction**: no reviewer reports a control acting the wrong way. This is
  binary and the model should pass it.
- **Control progression**: the dead band and the build of brake pressure read as
  ordinary rather than as faults.
- **Pitch and pendulum timing**: reviewers describe the surge as roughly right
  for the wing's size. This is the one that decides whether the stated damping
  ratio survives.
- **Stall approach**: the cues arrive before the break, even though the break is
  known to be early.

Anything a reviewer flags that is **not** already in the calibration report's
known disagreements is a finding, and should be written up there before being
acted on.
