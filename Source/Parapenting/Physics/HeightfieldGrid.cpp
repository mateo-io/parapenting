#include "HeightfieldGrid.h"

#include <algorithm>
#include <cmath>
#include <fstream>

namespace Parapenting::Physics
{
void HeightfieldGrid::Clear()
{
    ColumnCount = 0;
    RowCount = 0;
    CellSize = 0.0;
    Samples.clear();
}

bool HeightfieldGrid::LoadEsriAscii(const std::string& filePath)
{
    Clear();
    std::ifstream input(filePath);
    if (!input) return false;

    std::string key;
    input >> key >> ColumnCount;
    input >> key >> RowCount;
    input >> key >> OriginXM;
    input >> key >> OriginYM;
    input >> key >> CellSize;
    input >> key >> NoDataValue;
    if (!input || ColumnCount < 2 || RowCount < 2 || CellSize <= 0.0)
    {
        Clear();
        return false;
    }

    Samples.resize(static_cast<std::size_t>(ColumnCount * RowCount));
    for (double& value : Samples)
    {
        input >> value;
        if (!input)
        {
            Clear();
            return false;
        }
    }
    return true;
}

bool HeightfieldGrid::Sample(double xM, double yM, double& elevationM) const
{
    if (!IsLoaded()) return false;
    const double column = (xM - OriginXM) / CellSize;
    const double rowFromBottom = (yM - OriginYM) / CellSize;
    if (column < 0.0 || rowFromBottom < 0.0
        || column > ColumnCount - 1 || rowFromBottom > RowCount - 1)
        return false;

    const int c0 = std::min(static_cast<int>(std::floor(column)), ColumnCount - 2);
    const int r0Bottom =
        std::min(static_cast<int>(std::floor(rowFromBottom)), RowCount - 2);
    const double tx = column - c0;
    const double ty = rowFromBottom - r0Bottom;
    const int topRow0 = RowCount - 1 - r0Bottom;
    const int topRow1 = RowCount - 1 - (r0Bottom + 1);
    const auto at = [&](int row, int col)
    {
        return Samples[static_cast<std::size_t>(row * ColumnCount + col)];
    };
    const double z00 = at(topRow0, c0);
    const double z10 = at(topRow0, c0 + 1);
    const double z01 = at(topRow1, c0);
    const double z11 = at(topRow1, c0 + 1);
    if (z00 == NoDataValue || z10 == NoDataValue
        || z01 == NoDataValue || z11 == NoDataValue)
        return false;
    elevationM = (z00 * (1.0 - tx) + z10 * tx) * (1.0 - ty)
               + (z01 * (1.0 - tx) + z11 * tx) * ty;
    return std::isfinite(elevationM);
}
}
