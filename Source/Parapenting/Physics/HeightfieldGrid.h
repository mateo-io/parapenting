#pragma once

#include <string>
#include <vector>

namespace Parapenting::Physics
{
class HeightfieldGrid
{
public:
    bool LoadEsriAscii(const std::string& filePath);
    bool Sample(double xM, double yM, double& elevationM) const;
    bool IsLoaded() const { return !Samples.empty(); }
    void Clear();

    int Columns() const { return ColumnCount; }
    int Rows() const { return RowCount; }
    double CellSizeM() const { return CellSize; }

private:
    int ColumnCount = 0;
    int RowCount = 0;
    double OriginXM = 0.0;
    double OriginYM = 0.0;
    double CellSize = 0.0;
    double NoDataValue = -9999.0;
    std::vector<double> Samples;
};
}
