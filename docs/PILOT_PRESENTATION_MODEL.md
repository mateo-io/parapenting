# Pilot and harness presentation

The runtime pilot is an articulated low-poly seated assembly rather than the
original stretched-sphere placeholder. It contains a harness shell, torso,
head, independently posed upper arms and forearms, brake handles, thighs and
lower legs.

`Physics/PilotPose` is the engine-independent presentation contract. It maps
the 120 Hz flight state into a deterministic visual pose:

- harness roll and pitch move the suspended body relative to the wing;
- commanded weight shift adds deliberate lateral seat displacement;
- each brake command lowers only the corresponding hand;
- measured brake force moves the hand outward and bends the elbow, making
  line pressure visible as effort rather than only as HUD text;
- collapse severity drops the body slightly as suspension load is lost;
- recovery surge pitches and displaces the seated assembly.

The Unreal pawn converts this pose into primitive-mesh segment transforms.
This deliberately remains a lightweight scalable representation: it is cheap
on Metal, casts useful shadows, reads clearly in chase cameras and can later be
replaced by a licensed skeletal character without changing the physics-facing
pose API.

Headless tests verify independent left/right brake travel, force response,
weight-shift displacement, incident drop and roll direction. The pose has no
feedback into aerodynamics, so graphics frame rate cannot alter the 120 Hz
solver.
