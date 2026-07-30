# Physics-driven procedural audio

Parapenting synthesizes flight feedback in stereo from the fixed-step physics
telemetry. No audio layer changes the flight state.

## Signal mapping

- vertical speed controls bounded climb beeps and strong-sink tone;
- airspeed controls broadband wind level and filter bandwidth;
- actual turbulence controls shared fabric texture;
- short-band sampled gust energy adds fine fabric texture independently of
  broad rotor intensity;
- thermal-core strength adds a slow, centered low-frequency air breath;
- canopy pressure and aerodynamic unloading add rustle/hiss before fabric loss;
- collapses and cravats add independent left/right fabric flutter;
- total line load controls line resonance pitch;
- individual brake force adds same-side brake-line tension;
- recovery surge adds a short broadband rush.

The previous implementation passed rotor strength into the turbulence channel.
This has been corrected: ordinary turbulent or convective air now produces
texture even outside authored rotor, while rotor still influences physics,
camera and controller feedback.

## Bounds and hearing safety

The engine-independent mix model clamps wind, fabric, line, thermal and surge
levels independently. The final synth additionally hard-limits each channel to
`[-0.8, 0.8]`, leaving headroom for coincident layers. This is peak protection,
not a certified loudness standard; users should still set a comfortable system
volume.

Headless tests verify climb frequency/rate, thermal breath, airspeed response,
left/right incident localization, brake-line asymmetry, surge presence and all
documented level ceilings. The generated waveform remains deterministic for a
given telemetry stream and synth seed.
