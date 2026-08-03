#include "SectionViscousSolver.h"

#include <algorithm>
#include <cmath>

namespace Parapenting::Physics
{
namespace
{
constexpr double Pi = 3.14159265358979323846;

// Dense solve with partial pivoting. The system is (panels + 1) square and
// built once per contour, so nothing here needs to be clever.
bool SolveInPlace(std::vector<double>& matrix,
                  std::vector<std::vector<double>>& rhs, std::size_t n)
{
    const auto at = [&](std::size_t r, std::size_t c) -> double&
    {
        return matrix[r * n + c];
    };
    for (std::size_t column = 0; column < n; ++column)
    {
        std::size_t pivot = column;
        double best = std::fabs(at(column, column));
        for (std::size_t row = column + 1; row < n; ++row)
        {
            const double value = std::fabs(at(row, column));
            if (value > best) { best = value; pivot = row; }
        }
        if (best < 1.0e-14) return false;
        if (pivot != column)
        {
            for (std::size_t c = 0; c < n; ++c)
            {
                std::swap(at(column, c), at(pivot, c));
            }
            for (std::vector<double>& vector : rhs)
            {
                std::swap(vector[column], vector[pivot]);
            }
        }
        const double diagonal = at(column, column);
        for (std::size_t row = column + 1; row < n; ++row)
        {
            const double factor = at(row, column) / diagonal;
            if (factor == 0.0) continue;
            for (std::size_t c = column; c < n; ++c)
            {
                at(row, c) -= factor * at(column, c);
            }
            for (std::vector<double>& vector : rhs)
            {
                vector[row] -= factor * vector[column];
            }
        }
    }
    for (std::vector<double>& vector : rhs)
    {
        for (std::size_t i = n; i-- > 0;)
        {
            double sum = vector[i];
            for (std::size_t c = i + 1; c < n; ++c)
            {
                sum -= at(i, c) * vector[c];
            }
            vector[i] = sum / at(i, i);
        }
    }
    return true;
}

// Velocity a unit-strength constant source panel induces at a point, in the
// panel's own frame: the panel runs from the origin to (length, 0). Katz and
// Plotkin, constant-strength source, eq. 11.21-11.22. The matching vortex
// panel is the same pair swapped and signed, which is why only one of them is
// written out.
struct LocalVelocity
{
    double along = 0.0;
    double across = 0.0;
};

LocalVelocity SourcePanelVelocity(double x, double z, double length)
{
    LocalVelocity velocity;
    const double r0Squared = x * x + z * z;
    const double r1Squared = (x - length) * (x - length) + z * z;
    if (r0Squared < 1.0e-24 || r1Squared < 1.0e-24) return velocity;
    velocity.along = std::log(r0Squared / r1Squared) / (4.0 * Pi);
    velocity.across =
        (std::atan2(z, x - length) - std::atan2(z, x)) / (2.0 * Pi);
    return velocity;
}

// -- integral boundary layer -------------------------------------------

// Thwaites' correlations in the Cebeci-Bradshaw form. `l` is the shear
// parameter tau theta / (mu Ue).
void ThwaitesClosure(double lambda, double& shapeFactor, double& shear)
{
    if (lambda >= 0.0)
    {
        const double c = std::min(lambda, 0.25);
        shapeFactor = 2.61 - 3.75 * c + 5.24 * c * c;
        shear = 0.22 + 1.57 * c - 1.8 * c * c;
    }
    else
    {
        const double c = std::max(lambda, -0.1);
        shapeFactor = 2.088 + 0.0731 / (c + 0.14);
        shear = 0.22 + 1.402 * c + 0.018 * c / (c + 0.107);
    }
}

// Head's entrainment shape parameter, and its inverse. Two branches meeting
// at H = 1.6.
double EntrainmentShape(double shapeFactor)
{
    const double h = std::max(1.02, shapeFactor);
    if (h <= 1.6) return 3.3 + 0.8234 * std::pow(h - 1.1, -1.287);
    return 3.3 + 1.5501 * std::pow(h - 0.6778, -3.064);
}

double ShapeFromEntrainment(double h1)
{
    // H1 falls as H rises, so the branch boundary is the H1 at H = 1.6.
    constexpr double BranchH1 = 3.3 + 0.8234 * 2.441417;
    if (h1 >= BranchH1)
    {
        return 1.1 + std::pow(0.8234 / std::max(1.0e-6, h1 - 3.3),
                              1.0 / 1.287);
    }
    return 0.6778 + std::pow(1.5501 / std::max(1.0e-6, h1 - 3.3),
                             1.0 / 3.064);
}

struct BranchResult
{
    // Chord fraction where the flow separates, or 1 if it reaches the
    // trailing edge attached.
    double separationChordFraction = 1.0;
    double transitionChordFraction = 1.0;
    // Index into the branch's own station list, one past the last attached
    // station.
    std::size_t separationStation = 0;
    bool separated = false;
    // Momentum thickness and shape factor where the layer ends.
    double endMomentumThickness = 0.0;
    double endShapeFactor = 2.0;
    double endEdgeVelocity = 1.0;
};

// Marches one surface from the stagnation point to the trailing edge.
// `edgeVelocity` and `arcLength` run downstream; `chord` is the chord
// fraction of each station, used only for reporting where things happened.
// `turbulentFromStart` is set on the surface the flow reaches by crossing the
// cell opening. There is no laminar run there: the flow has already separated
// off the lip of the mouth and what arrives on the surface is a turbulent
// shear layer. This is the whole of what the opening contributes here, and it
// is a statement about where the hole is rather than a coefficient.
//
// What is deliberately NOT modelled is the momentum thickness that shear layer
// carries onto the surface. It is certainly not zero and it is the largest
// single candidate for the profile drag this section is missing - the model
// runs about 0.0096 at trim where paraglider sections are usually quoted at
// 0.018 to 0.025, and closing that would close most of PHYSICS_TODO item 12.
//
// It has been tried twice. Seeded at the lip with the shear layer's own
// momentum thickness, theta = h/6 for a linear profile across an opening of
// height h, it changes the section's drag by 2% when h is quadrupled - the lip
// sits in the steepest favourable gradient on the section and theta decays
// there as Ue^-(H+2), so the seed is thrown away within two percent of chord.
// Seeded instead at reattachment, six opening heights downstream, the same
// derivation gives 0.036 at a 1% opening and 0.076 at a 3% one, against the
// 0.019 wanted - three to four times too much, because h/6 measures the layer
// against the freestream while it actually forms where the local edge velocity
// is a fraction of it.
//
// So the derivation has a factor in it that is not geometry, the answer moves
// by five times across the plausible range of the opening height, and one
// value in that range lands on the published glide. That is a dial. It is left
// out and the gap is reported.
BranchResult MarchBoundaryLayer(
    const std::vector<double>& arcLength,
    const std::vector<double>& edgeVelocity,
    const std::vector<double>& chord,
    double reynolds,
    bool turbulentFromStart = false)
{
    BranchResult result;
    const std::size_t count = arcLength.size();
    result.separationStation = count;
    if (count < 3) return result;

    const double viscosity = 1.0 / std::max(1.0, reynolds);

    const auto gradient = [&](std::size_t i)
    {
        if (i == 0)
        {
            return (edgeVelocity[1] - edgeVelocity[0])
                / std::max(1.0e-9, arcLength[1] - arcLength[0]);
        }
        if (i + 1 >= count)
        {
            return (edgeVelocity[count - 1] - edgeVelocity[count - 2])
                / std::max(1.0e-9,
                           arcLength[count - 1] - arcLength[count - 2]);
        }
        return (edgeVelocity[i + 1] - edgeVelocity[i - 1])
            / std::max(1.0e-9, arcLength[i + 1] - arcLength[i - 1]);
    };

    // -- laminar run, Thwaites ----------------------------------------
    // The integral starts at the stagnation point, where the edge velocity
    // is zero and grows linearly, so the first station's contribution is
    // ue^5 s / 6 rather than the trapezoid's ue^5 s / 2.
    double integral = std::pow(edgeVelocity[0], 5.0) * arcLength[0] / 6.0;
    double momentum = 0.0;
    double shapeFactor = 2.5;
    std::size_t station = 0;
    bool transitioned = turbulentFromStart;

    for (; !transitioned && station < count; ++station)
    {
        if (station > 0)
        {
            integral += 0.5
                * (std::pow(edgeVelocity[station], 5.0)
                   + std::pow(edgeVelocity[station - 1], 5.0))
                * (arcLength[station] - arcLength[station - 1]);
        }
        const double ue = std::max(1.0e-6, edgeVelocity[station]);
        momentum = std::sqrt(
            std::max(0.0, 0.45 * viscosity * integral / std::pow(ue, 6.0)));
        const double lambda = momentum * momentum * gradient(station)
            / viscosity;
        double shear = 0.0;
        ThwaitesClosure(lambda, shapeFactor, shear);

        // A laminar layer that separates at these Reynolds numbers does not
        // stay separated: it transitions inside the bubble and reattaches
        // within a few percent of chord. Treating it as transition at the
        // laminar separation point is the standard short-bubble assumption
        // and is why no bubble geometry appears anywhere here.
        if (lambda <= -0.09)
        {
            transitioned = true;
            break;
        }

        const double reynoldsX = reynolds * ue * arcLength[station];
        const double reynoldsTheta = reynolds * ue * momentum;
        if (reynoldsX > 1.0e3)
        {
            const double michel = 1.174
                * (1.0 + 22400.0 / reynoldsX) * std::pow(reynoldsX, 0.46);
            if (reynoldsTheta > michel)
            {
                transitioned = true;
                break;
            }
        }
    }

    if (!transitioned)
    {
        // Laminar to the trailing edge. Rare on this section but possible on
        // the lower surface at speed.
        result.endMomentumThickness = momentum;
        result.endShapeFactor = shapeFactor;
        result.endEdgeVelocity = edgeVelocity[count - 1];
        return result;
    }

    station = std::min(station, count - 1);
    result.transitionChordFraction = chord[station];

    // -- turbulent run, Head's entrainment method ----------------------
    // The layer arrives with the momentum thickness the laminar run built and
    // a turbulent shape factor; the two are independent, which is why only
    // theta carries across.
    shapeFactor = 1.4;
    double entrainment = EntrainmentShape(shapeFactor);

    for (std::size_t i = station; i + 1 < count; ++i)
    {
        const double ds = arcLength[i + 1] - arcLength[i];
        if (ds <= 0.0) continue;
        // Substep so the entrainment equation stays stable through the
        // steep gradients under a braked trailing edge.
        constexpr int Substeps = 4;
        const double step = ds / Substeps;
        for (int sub = 0; sub < Substeps; ++sub)
        {
            const double blend = (sub + 0.5) / Substeps;
            const double ue = std::max(1.0e-6,
                edgeVelocity[i] + (edgeVelocity[i + 1] - edgeVelocity[i])
                    * blend);
            const double slope =
                (edgeVelocity[i + 1] - edgeVelocity[i]) / ds;
            const double reynoldsTheta =
                std::max(10.0, reynolds * ue * momentum);
            const double friction = 0.246
                * std::pow(10.0, -0.678 * shapeFactor)
                * std::pow(reynoldsTheta, -0.268);

            const double dMomentum = 0.5 * friction
                - (shapeFactor + 2.0) * momentum * slope / ue;
            const double entrainmentRate =
                0.0306 * std::pow(std::max(0.01, entrainment - 3.0), -0.6169);

            // d(ue theta H1)/ds = ue * F, expanded so H1 can be stepped.
            const double product = ue * momentum * entrainment;
            const double newProduct = product + step * ue * entrainmentRate;
            const double newMomentum = momentum + step * dMomentum;
            const double newUe = std::max(1.0e-6, ue + slope * step);
            entrainment = newProduct
                / std::max(1.0e-9, newUe * std::max(1.0e-9, newMomentum));
            entrainment = std::max(3.05, entrainment);
            momentum = std::max(1.0e-9, newMomentum);
            shapeFactor = ShapeFromEntrainment(entrainment);
            shapeFactor = std::clamp(shapeFactor, 1.02, 3.0);
        }

        // A turbulent layer that separates within the first few percent of
        // chord has not stalled the section - it has made a leading-edge
        // bubble, and on a nose this round the bubble reattaches. This is the
        // turbulent twin of the laminar short bubble above, and it is treated
        // the same way: the layer comes back down with a reattached profile
        // and the march continues.
        //
        // Without it this section stalls at the NOSE, and abruptly: measured,
        // the boundary layer went from separating at 94% of chord to
        // separating at 3% of it for one degree more incidence, and the wing
        // lost the whole upper surface in a single step. That is what
        // leading-edge stall looks like, and it is not what a 15.5% section
        // with a 2.65% nose radius does - leading-edge stall belongs to thin
        // sections with sharp noses. It was the integral method being asked a
        // question it cannot answer: just aft of the suction peak the layer is
        // a few thousandths of a chord thick, the gradient is at its steepest,
        // and Head's entrainment equation has no bubble in it.
        //
        // The limit is stated rather than solved, and it is the one number in
        // this file that is. Short bubbles run half a percent to two percent
        // of chord and long ones reach five to ten; three percent is inside
        // that and it is what a section with this nose radius supports. The
        // inverse boundary-layer formulation is what replaces it, and that is
        // the difference between this and XFOIL. PHYSICS_TODO item 13.
        constexpr double LeadingEdgeBubbleChord = 0.03;
        if (shapeFactor >= 2.4 && chord[i + 1] < LeadingEdgeBubbleChord)
        {
            shapeFactor = 1.5;
            entrainment = EntrainmentShape(shapeFactor);
            continue;
        }

        if (shapeFactor >= 2.4)
        {
            result.separated = true;
            result.separationStation = i + 1;
            result.separationChordFraction = chord[i + 1];
            result.endMomentumThickness = momentum;
            result.endShapeFactor = shapeFactor;
            result.endEdgeVelocity = edgeVelocity[i + 1];
            return result;
        }
    }

    result.endMomentumThickness = momentum;
    result.endShapeFactor = shapeFactor;
    result.endEdgeVelocity = edgeVelocity[count - 1];
    return result;
}
}

SectionViscousSolver::SectionViscousSolver(
    const SectionProfile& profile, double reynoldsNumber)
{
    const std::size_t panels = profile.PanelCount();
    if (panels < 8) return;
    Reynolds = std::max(1.0e4, reynoldsNumber);

    Geometry.resize(panels);
    double minX = profile.nodes[0].x;
    double maxX = profile.nodes[0].x;
    for (const SectionPoint& node : profile.nodes)
    {
        minX = std::min(minX, node.x);
        maxX = std::max(maxX, node.x);
    }
    Chord = std::max(1.0e-6, maxX - minX);

    for (std::size_t i = 0; i < panels; ++i)
    {
        const SectionPoint& a = profile.nodes[i];
        const SectionPoint& b = profile.nodes[i + 1];
        const double dx = b.x - a.x;
        const double dz = b.z - a.z;
        const double length = std::hypot(dx, dz);
        Panel& panel = Geometry[i];
        panel.length = length;
        panel.midX = 0.5 * (a.x + b.x);
        panel.midZ = 0.5 * (a.z + b.z);
        if (length > 0.0)
        {
            panel.tangentX = dx / length;
            panel.tangentZ = dz / length;
        }
        else
        {
            panel.tangentX = 1.0;
            panel.tangentZ = 0.0;
        }
        // With the contour running trailing edge - lower - nose - upper -
        // trailing edge, this normal points out of the section.
        panel.normalX = -panel.tangentZ;
        panel.normalZ = panel.tangentX;
    }
    PanelCount = panels;

    // What the opening costs the boundary layer. The shear layer that springs
    // off the lip spans the height of the opening; across it the velocity
    // runs from zero in the cell to the edge velocity outside, and the
    // momentum thickness of a linear profile over a height h is
    //
    //     theta = integral (u/U)(1 - u/U) dy = h / 6.
    //
    // That is the layer the upper surface starts with instead of starting
    // clean, and it is most of why a paraglider's profile drag is roughly
    // twice a closed section's. Nothing about it is fitted: it is the
    // opening's height and one integral.
    InletChordFraction = profile.inletChordFraction;

    // Influence coefficients. The unknowns are one source strength per panel
    // plus a single vortex strength shared by all of them; the extra row is
    // the Kutta condition, which sets the tangential velocities at the two
    // trailing-edge panels equal and opposite.
    const std::size_t size = panels + 1;
    std::vector<double> matrix(size * size, 0.0);
    std::vector<std::vector<double>> rhs(2, std::vector<double>(size, 0.0));
    std::vector<double>& rhsAlongX = rhs[0];
    std::vector<double>& rhsAlongZ = rhs[1];
    std::vector<double> sourceTangential(panels * panels, 0.0);
    std::vector<double> vortexTangential(panels, 0.0);

    const auto at = [&](std::size_t r, std::size_t c) -> double&
    {
        return matrix[r * size + c];
    };

    for (std::size_t i = 0; i < panels; ++i)
    {
        const Panel& target = Geometry[i];
        double vortexNormal = 0.0;
        double vortexTangent = 0.0;
        for (std::size_t j = 0; j < panels; ++j)
        {
            const Panel& source = Geometry[j];
            double sourceX = 0.0;
            double sourceZ = 0.0;
            double vortexX = 0.0;
            double vortexZ = 0.0;
            if (i == j)
            {
                // Own panel: the source sheet induces half its strength
                // outward, the vortex sheet half its strength along the
                // surface. Both are the limit of the expressions below.
                sourceX = 0.5 * target.normalX;
                sourceZ = 0.5 * target.normalZ;
                vortexX = 0.5 * target.tangentX;
                vortexZ = 0.5 * target.tangentZ;
            }
            else
            {
                const double dx = target.midX - profile.nodes[j].x;
                const double dz = target.midZ - profile.nodes[j].z;
                const double local = dx * source.tangentX
                    + dz * source.tangentZ;
                const double across = -dx * source.tangentZ
                    + dz * source.tangentX;
                const LocalVelocity s =
                    SourcePanelVelocity(local, across, source.length);
                sourceX = s.along * source.tangentX
                    - s.across * source.tangentZ;
                sourceZ = s.along * source.tangentZ
                    + s.across * source.tangentX;
                // The vortex panel of the same geometry is the source panel's
                // pair rotated: (along, across) -> (across, -along).
                vortexX = s.across * source.tangentX
                    + s.along * source.tangentZ;
                vortexZ = s.across * source.tangentZ
                    - s.along * source.tangentX;
            }
            at(i, j) = sourceX * target.normalX + sourceZ * target.normalZ;
            sourceTangential[i * panels + j] =
                sourceX * target.tangentX + sourceZ * target.tangentZ;
            vortexNormal += vortexX * target.normalX
                + vortexZ * target.normalZ;
            vortexTangent += vortexX * target.tangentX
                + vortexZ * target.tangentZ;
        }
        at(i, panels) = vortexNormal;
        vortexTangential[i] = vortexTangent;
        rhsAlongX[i] = -target.normalX;
        rhsAlongZ[i] = -target.normalZ;
    }

    // The circulation mode: the same no-penetration rows with no freestream,
    // and a unit vortex strength instead of the Kutta condition. It is the
    // flow field that adds circulation to the section without disturbing the
    // boundary condition, and it is what lets the separated solve carry less
    // circulation than the Kutta condition would give - which is the whole
    // physical content of a stall. Built before the Kutta row overwrites the
    // shared part of the matrix.
    std::vector<double> circulationMatrix = matrix;
    std::vector<std::vector<double>> circulationRhs(
        1, std::vector<double>(size, 0.0));
    for (std::size_t j = 0; j < panels; ++j)
    {
        circulationMatrix[panels * size + j] = 0.0;
    }
    circulationMatrix[panels * size + panels] = 1.0;
    circulationRhs[0][panels] = 1.0;

    const std::size_t first = 0;
    const std::size_t last = panels - 1;
    for (std::size_t j = 0; j < panels; ++j)
    {
        at(panels, j) = sourceTangential[first * panels + j]
            + sourceTangential[last * panels + j];
    }
    at(panels, panels) = vortexTangential[first] + vortexTangential[last];
    rhsAlongX[panels] =
        -(Geometry[first].tangentX + Geometry[last].tangentX);
    rhsAlongZ[panels] =
        -(Geometry[first].tangentZ + Geometry[last].tangentZ);

    if (!SolveInPlace(matrix, rhs, size)
        || !SolveInPlace(circulationMatrix, circulationRhs, size))
    {
        PanelCount = 0;
        return;
    }

    TangentialAlongX.assign(panels, 0.0);
    TangentialAlongZ.assign(panels, 0.0);
    TangentialCirculation.assign(panels, 0.0);
    for (std::size_t i = 0; i < panels; ++i)
    {
        double alongX = Geometry[i].tangentX;
        double alongZ = Geometry[i].tangentZ;
        double circulation = 0.0;
        for (std::size_t j = 0; j < panels; ++j)
        {
            const double influence = sourceTangential[i * panels + j];
            alongX += influence * rhsAlongX[j];
            alongZ += influence * rhsAlongZ[j];
            circulation += influence * circulationRhs[0][j];
        }
        alongX += vortexTangential[i] * rhsAlongX[panels];
        alongZ += vortexTangential[i] * rhsAlongZ[panels];
        circulation += vortexTangential[i] * circulationRhs[0][panels];
        TangentialAlongX[i] = alongX;
        TangentialAlongZ[i] = alongZ;
        TangentialCirculation[i] = circulation;
    }
    VortexAlongX = rhsAlongX[panels];
    VortexAlongZ = rhsAlongZ[panels];
}

std::vector<double> SectionViscousSolver::TangentialVelocity(
    double alphaRad) const
{
    std::vector<double> velocity(PanelCount, 0.0);
    const double cosAlpha = std::cos(alphaRad);
    const double sinAlpha = std::sin(alphaRad);
    for (std::size_t i = 0; i < PanelCount; ++i)
    {
        velocity[i] = cosAlpha * TangentialAlongX[i]
            + sinAlpha * TangentialAlongZ[i];
    }
    return velocity;
}

std::vector<double> SectionViscousSolver::InviscidPressure(
    double alphaRad) const
{
    const std::vector<double> velocity = TangentialVelocity(alphaRad);
    std::vector<double> pressure(velocity.size(), 0.0);
    for (std::size_t i = 0; i < velocity.size(); ++i)
    {
        pressure[i] = 1.0 - velocity[i] * velocity[i];
    }
    return pressure;
}

SectionAerodynamics SectionViscousSolver::IntegrateSurface(
    const std::vector<double>& pressure, double alphaRad) const
{
    SectionAerodynamics result;
    double forceX = 0.0;
    double forceZ = 0.0;
    double moment = 0.0;
    const double quarterChord = 0.25 * Chord;
    for (std::size_t i = 0; i < PanelCount; ++i)
    {
        const Panel& panel = Geometry[i];
        const double dFx = -pressure[i] * panel.normalX * panel.length;
        const double dFz = -pressure[i] * panel.normalZ * panel.length;
        forceX += dFx;
        forceZ += dFz;
        moment += dFx * panel.midZ - dFz * (panel.midX - quarterChord);
    }
    forceX /= Chord;
    forceZ /= Chord;
    moment /= (Chord * Chord);

    const double cosAlpha = std::cos(alphaRad);
    const double sinAlpha = std::sin(alphaRad);
    result.liftCoefficient = forceZ * cosAlpha - forceX * sinAlpha;
    result.dragCoefficient = forceX * cosAlpha + forceZ * sinAlpha;
    result.momentCoefficient = moment;
    return result;
}

SectionAerodynamics SectionViscousSolver::Solve(
    double alphaRad, double initialLowerAttached,
    double initialUpperAttached) const
{
    SectionAerodynamics result;
    if (PanelCount == 0) return result;

    const std::vector<double> inviscidVelocity = TangentialVelocity(alphaRad);
    std::vector<double> inviscidPressure(PanelCount, 0.0);
    for (std::size_t i = 0; i < PanelCount; ++i)
    {
        inviscidPressure[i] = 1.0 - inviscidVelocity[i] * inviscidVelocity[i];
    }
    const SectionAerodynamics attached =
        IntegrateSurface(inviscidPressure, alphaRad);
    result.attachedLiftCoefficient = attached.liftCoefficient;
    result.attachedMomentCoefficient = attached.momentCoefficient;

    // The stagnation point is where the surface velocity changes sign. On
    // this contour the lower surface runs against the traverse and the upper
    // runs with it, so there is one crossing and it is the nose.
    std::size_t stagnation = PanelCount / 2;
    double bestCrossing = 1.0e30;
    for (std::size_t i = 0; i + 1 < PanelCount; ++i)
    {
        if (inviscidVelocity[i] * inviscidVelocity[i + 1] > 0.0) continue;
        const double magnitude = std::fabs(inviscidVelocity[i])
            + std::fabs(inviscidVelocity[i + 1]);
        if (magnitude < bestCrossing)
        {
            bestCrossing = magnitude;
            stagnation = i;
        }
    }

    // Station lists for the two surfaces, both running downstream from the
    // stagnation point.
    std::vector<std::size_t> lowerIndices;
    std::vector<std::size_t> upperIndices;
    for (std::size_t i = stagnation + 1; i-- > 0;) lowerIndices.push_back(i);
    for (std::size_t i = stagnation + 1; i < PanelCount; ++i)
    {
        upperIndices.push_back(i);
    }

    const auto arcLengths = [&](const std::vector<std::size_t>& indices)
    {
        std::vector<double> arc(indices.size(), 0.0);
        if (indices.empty()) return arc;
        arc[0] = 0.5 * Geometry[indices[0]].length;
        for (std::size_t k = 1; k < indices.size(); ++k)
        {
            const Panel& a = Geometry[indices[k - 1]];
            const Panel& b = Geometry[indices[k]];
            arc[k] = arc[k - 1] + std::hypot(b.midX - a.midX, b.midZ - a.midZ);
        }
        return arc;
    };
    const auto chordFractions = [&](const std::vector<std::size_t>& indices)
    {
        std::vector<double> chord(indices.size(), 0.0);
        for (std::size_t k = 0; k < indices.size(); ++k)
        {
            chord[k] = std::clamp(Geometry[indices[k]].midX / Chord, 0.0, 1.0);
        }
        return chord;
    };

    // Which surface the opening feeds. The flow that crosses the nose is the
    // one that started on the other side, so a stagnation point on the lower
    // surface aft of the opening leaves the lower surface clean and the upper
    // surface fed. When the stagnation point sits inside the opening - which
    // is where a paraglider is designed to put it at trim, and where the
    // pressure solver independently measures it - both surfaces are fed,
    // because the air is dividing inside the mouth.
    const bool stagnationOnLower = stagnation < PanelCount / 2;
    const double stagnationChord =
        std::clamp(Geometry[stagnation].midX / Chord, 0.0, 1.0);
    const bool insideInlet = stagnationChord <= InletChordFraction;
    const bool hasInlet = InletChordFraction > 0.0;
    const bool lowerFed = hasInlet && (!stagnationOnLower || insideInlet);
    const bool upperFed = hasInlet && (stagnationOnLower || insideInlet);

    const std::vector<double> lowerArc = arcLengths(lowerIndices);
    const std::vector<double> upperArc = arcLengths(upperIndices);
    const std::vector<double> lowerChord = chordFractions(lowerIndices);
    const std::vector<double> upperChord = chordFractions(upperIndices);

    // -- viscous-inviscid fixed point ----------------------------------
    // Aft of separation the section carries the pressure it had where the
    // flow let go - the Kirchhoff dead-air region. That unloads the section,
    // which weakens the suction peak, which moves the separation point back.
    // Iterating the two to agreement is what produces a maximum lift
    // coefficient; nothing here states one.
    std::vector<double> pressure = inviscidPressure;
    double lowerSeparation = std::clamp(initialLowerAttached, 0.0, 1.0);
    double upperSeparation = std::clamp(initialUpperAttached, 0.0, 1.0);
    BranchResult lower;
    BranchResult upper;

    const double kuttaVortex = std::cos(alphaRad) * VortexAlongX
        + std::sin(alphaRad) * VortexAlongZ;

    // Kirchhoff's free-streamline result for a plate with the flow attached
    // over the leading fraction f: the circulation falls to
    // ((1 + sqrt(f)) / 2)^2 of the fully attached value. It is exact for a
    // plate and it is the standard closure for trailing-edge stall on a real
    // section; nothing in it is fitted.
    const auto circulationFactor = [](double attachedFraction)
    {
        const double f = std::clamp(attachedFraction, 0.0, 1.0);
        const double root = 0.5 * (1.0 + std::sqrt(f));
        return root * root;
    };

    // Past the stall the fixed point stops being a point. The boundary layer
    // hands back a separation station that jumps by whole panels, the
    // circulation follows it, and the pair settles into a cycle rather than a
    // value - which is not a numerical failure, it is the physical statement
    // that a stalled section has no steady state (the same thing the vortex
    // step method reports one level up). When that happens the mean of the
    // cycle is taken, and the sample is marked unconverged so the table above
    // knows to hand the angle to the post-stall branch instead.
    constexpr int MaxIterations = 160;
    constexpr int AveragingWindow = 40;
    constexpr double Relaxation = 0.12;
    constexpr double SettledTolerance = 2.0e-3;
    // The pressure field the two separation points imply. Two things happen
    // to it, and they are the same model seen from two sides. The Kutta
    // condition no longer belongs at the trailing edge, because the flow
    // leaves the surface before it gets there, so the circulation drops to
    // Kirchhoff's value for the attached fraction. And behind the separation
    // point the surface carries the pressure it had where the flow let go,
    // CONSTANT to the trailing edge - that is what dead air is.
    //
    // The constancy is the load-bearing part and it took a wrong version to
    // see why. Recovering the plateau smoothly to Cp = 0 over the following
    // 8% of chord looks more careful and is a positive feedback: from a
    // suction of -1.5 to zero over 8% of chord is a violent adverse gradient,
    // it lies exactly where the boundary layer has just separated, and it
    // guarantees the layer cannot reattach. Separation then walks forward
    // until it reaches the nose and the section falls into the deep-stall
    // state in a single degree, from only a fifth of its chord separated -
    // where a real section reaches its lift peak with half of it gone.
    //
    // Measured, that cost the wing its brake range: the attached branch ended
    // at 9, 12, 7 and 12 degrees at 25, 30, 35 and 40% brake, erratically,
    // because which station first crossed a shape factor of 2.4 decided the
    // whole thing. Dead air at constant pressure has no gradient in it, so
    // the layer behind the separation point is not being driven anywhere and
    // the fixed point is a point.
    const auto rebuildPressure = [&](double lowerSeparated,
                                     double upperSeparated)
    {
        const double attachedFraction =
            std::min(lowerSeparated, upperSeparated);
        const double scale = circulationFactor(attachedFraction) - 1.0;
        for (std::size_t i = 0; i < PanelCount; ++i)
        {
            const double velocity = inviscidVelocity[i]
                + scale * kuttaVortex * TangentialCirculation[i];
            pressure[i] = 1.0 - velocity * velocity;
        }
        const auto deadAir = [&](const std::vector<std::size_t>& indices,
                                 const std::vector<double>& chord,
                                 double separationChord)
        {
            if (separationChord >= 0.999) return;
            double separationPressure = 0.0;
            bool found = false;
            for (std::size_t k = 0; k < indices.size(); ++k)
            {
                if (chord[k] < separationChord) continue;
                if (!found)
                {
                    separationPressure = pressure[indices[k]];
                    found = true;
                }
                pressure[indices[k]] = separationPressure;
            }
        };
        deadAir(upperIndices, upperChord, upperSeparated);
        deadAir(lowerIndices, lowerChord, lowerSeparated);
    };

    rebuildPressure(lowerSeparation, upperSeparation);

    double lowerHistory = 0.0;
    double upperHistory = 0.0;
    int historyCount = 0;
    bool converged = false;
    for (int iteration = 0; iteration < MaxIterations; ++iteration)
    {
        const auto edgeVelocity = [&](const std::vector<std::size_t>& indices)
        {
            std::vector<double> ue(indices.size(), 0.0);
            for (std::size_t k = 0; k < indices.size(); ++k)
            {
                ue[k] = std::sqrt(
                    std::max(0.0, 1.0 - pressure[indices[k]]));
            }
            return ue;
        };

        lower = MarchBoundaryLayer(
            lowerArc, edgeVelocity(lowerIndices), lowerChord, Reynolds,
            lowerFed);
        upper = MarchBoundaryLayer(
            upperArc, edgeVelocity(upperIndices), upperChord, Reynolds,
            upperFed);

        const double targetLower = lower.separationChordFraction;
        const double targetUpper = upper.separationChordFraction;
        const double movedLower = std::fabs(targetLower - lowerSeparation);
        const double movedUpper = std::fabs(targetUpper - upperSeparation);
        lowerSeparation += Relaxation * (targetLower - lowerSeparation);
        upperSeparation += Relaxation * (targetUpper - upperSeparation);
        if (iteration >= MaxIterations - AveragingWindow)
        {
            lowerHistory += lowerSeparation;
            upperHistory += upperSeparation;
            ++historyCount;
        }

        rebuildPressure(lowerSeparation, upperSeparation);

        if (iteration > 2 && movedLower < SettledTolerance
            && movedUpper < SettledTolerance)
        {
            converged = true;
            break;
        }
    }

    if (!converged && historyCount > 0)
    {
        lowerSeparation = lowerHistory / historyCount;
        upperSeparation = upperHistory / historyCount;
        rebuildPressure(lowerSeparation, upperSeparation);
    }

    result = IntegrateSurface(pressure, alphaRad);
    result.attachedLiftCoefficient = attached.liftCoefficient;
    result.attachedMomentCoefficient = attached.momentCoefficient;
    result.converged = converged;
    result.upperTransitionChordFraction = upper.transitionChordFraction;
    result.lowerTransitionChordFraction = lower.transitionChordFraction;
    result.separatedChordFraction = std::clamp(
        std::max(1.0 - upperSeparation, 1.0 - lowerSeparation), 0.0, 1.0);
    result.lowerAttachedFraction = lowerSeparation;
    result.upperAttachedFraction = upperSeparation;

    // Profile drag by Squire-Young at the trailing edge, one surface at a
    // time. Where the layer separated first, the values at separation are
    // what carry to the wake, with the shape factor held at its separation
    // value - the standard treatment, and the reason a separated section's
    // drag rises steeply rather than smoothly.
    const auto squireYoung = [](const BranchResult& branch)
    {
        const double ue = std::clamp(branch.endEdgeVelocity, 0.05, 3.0);
        const double h = std::clamp(branch.endShapeFactor, 1.02, 3.0);
        return branch.endMomentumThickness * std::pow(ue, 0.5 * (h + 5.0));
    };
    const double frictionDrag =
        2.0 * (squireYoung(lower) + squireYoung(upper)) / Chord;
    // The pressure integration already carries the form drag the dead-air
    // region produces, and it is zero when the flow is attached, so the two
    // add without double counting.
    result.dragCoefficient =
        std::max(0.0, result.dragCoefficient) + frictionDrag;
    return result;
}
}
