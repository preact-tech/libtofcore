#ifndef __TOFCORE_CARTESIAN_TRANSFORM_H__
#define __TOFCORE_CARTESIAN_TRANSFORM_H__
/**
 * @file cartesian_transformation.hpp
 *
 * Copyright 2023 PreAct Technologies
 */
#include <cstdint>
#include <vector>

namespace tofcore {

class CartesianTransform
{
public:

    enum LensType { WIDE_FIELD = 0, STANDARD_FIELD, NARROW_FIELD };

    CartesianTransform();
    ~CartesianTransform();    
    void transformPixel(uint32_t srcX, uint32_t srcY, double srcZ, double &destX, double &destY, double &destZ);
    void initLensTransform(double sensorPointSizeMM, int32_t width, int32_t height, int32_t offsetX, int32_t offsetY, int32_t lensType);
    void initLensTransform(int32_t width, int32_t height, const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& z);

private:

    int32_t distortionTableSize;
    int32_t numCols;
    int32_t numRows;
    double angle[101];
    double rp[101];
    double xUA[320][240];
    double yUA[320][240];
    double zUA[320][240];

    double getAngle(double x, double y, double sensorPointSizeMM);
    double interpolate(double x_in, double x0, double y0, double x1, double y1);
    void initLensDistortionTable(LensType lensType);
};


} //end namespace tofcore

#endif // __TOFCORE_CARTESIAN_TRANSFORM_H__
