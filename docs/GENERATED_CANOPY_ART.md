# Generated Canopy Art (Interim)

`Content/ArtSource/Canopy/T_CanopyRipstop_Reinforcement_Source_v1.png` is an
interim, generated source texture for the canopy material. It provides a
red-orange ripstop weave with restrained stitched reinforcement cues.

`Content/ArtSource/Canopy/T_CanopyRipstop_Albedo_Source_v2.png` is the current
neutral albedo source: a clean, light ripstop field that can take every wing
colourway without baking an unrelated colour into the fabric. The material
generator imports it as `/Game/Materials/Textures/T_CanopyRipstop_Albedo` when
`M_CanopyFabric` is rebuilt. It is deliberately a subtle albedo modulation;
the procedural mesh's vertex colour remains authoritative for panels and the
runtime's selected wing colour remains authoritative for presentation.

Import it in Unreal as `T_CanopyRipstop_Reinforcement` with sRGB enabled and
Wrap addressing on both axes. Add it to `M_CanopyFabric` as a subtle colour
and roughness modulation only: it must not replace the procedural vertex
colourway, and the reinforcement cues should remain below normal-flight
silhouette scale.

This source is not a substitute for licensed final artwork. Revisit the scale,
tiling and reinforcement layout against reference canopy photography during
the Shipping visual-QA pass.
