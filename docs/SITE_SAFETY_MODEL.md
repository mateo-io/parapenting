# Bernese Oberland site-safety model

The route catalogue stores geographic anchors, launch orientation, landing
circuit guidance, a short hazard cue and source provenance. This lets the
simulator respond to route and wind selection instead of treating every launch
as equally suitable.

## Source hierarchy

The primary source is the SHV/FSVL Interlaken site board:

https://www.shv-fsvl.ch/fileadmin/files/redakteure/Allgemein/Sicherheit/SHVInfotafeln/Interlaken.pdf

The current Lehn circuit change is also described by SHV/FSVL:

https://www.shv-fsvl.ch/en/federation/news/news/neue-landevolte-am-landeplatz-lehn-in-interlaken/

The Interlaken heliport and emergency-landing restrictions are documented
separately:

https://www.shv-fsvl.ch/fileadmin/files/redakteure/0_Bilder/Sicherheit/Luftraum/Interlaken_Flugplatz.pdf

Coordinates in `Data/Sites/interlaken-route-catalogue.json` are research
anchors, not surveyed launch or landing polygons.

The live Deltaclub Interlaken launch pages list both Lehn and Höhematte as
landings within glide from Amisbühl and Hohwald:

https://deltaclub-interlaken.ch/startplatz-amisbuehl/

https://deltaclub-interlaken.ch/startplatz-hohwald/

The catalogue therefore exposes all four of those launch/landing pairings.
They are simulator routes assembled from verified site anchors, not prescribed
real-world flight paths. Hohwald remains subject to its published winter
closure, and the simulator records Höhematte's urban lee, traffic and
strong-valley-wind turbulence warning.

Grindelwald First, Grund and Bodmi are anchored from the DHV First site entry:

https://service.dhv.de/db2/details.php?item=1585&popup=1&qi=glp_details

The current Jungfrau-Tächi notice limits First launching to south of the start
tower or west of the path:

https://www.jungfrau-taechi.ch/newsfeed/startfirstkl

The published First operating sheet describes southeast through southwest
starts:

https://www.paragliding-jungfrau.ch/wp/downloads/Startplatz%20First.pdf

First–Grund and First–Bodmi are verified site pairings, not prescribed tracks.
Bodmi is marked advanced, Grund carries the published long-glide warning, and
neither route invents an official circuit. The simulator tells pilots to check
the current local procedure.

## Simulator interpretation

`RouteCatalogue` assigns each launch:

- a nominal facing direction;
- a preferred directional half-width;
- a conservative maximum simulated launch wind;
- the published qualitative hazard in concise form;
- the published landing-circuit convention.

The HUD compares the selected route with the authored or imported model wind
and displays `SIM ENVELOPE OK`, `SIM WIND MARGINAL`, or
`SIM WIND TOO STRONG`.

The numeric thresholds are simulator-authored training envelopes. They are not
published operating limits and must never be used for an actual flight
decision. Real pilots must check current local signs, closures, DABS, Meiringen
HX status, airspace, forecasts and observations, and make their own qualified
assessment.

## Validation

Unit tests cover suitable, crosswind/lee and excessive-wind classifications,
angular wraparound, route provenance, landing-circuit presence, catalogue
uniqueness and terrain coverage for all ten routes. Grindelwald checks also
cover geographic placement, published elevation differences and the regional
terrain anchors. The classification is
presentation and training logic only; it does not alter the 120 Hz
aerodynamics.
