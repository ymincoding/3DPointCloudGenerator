
#include "MathUtils.h"

#include <glog/logging.h>

#include "Constants.h"

namespace core {

std::tuple<double, double, double> cartesianToSpherical(EVector3 vec)
{
    const double KNorm(vec.norm());
    double phi{0.}, theta{0.}, radius{0.};
    if (KNorm >= Eigen::NumTraits<double>::epsilon())
    {
        vec /= KNorm;
        phi = std::acos(vec.z());
        if (std::abs(vec.z()) == 1.)
            theta = 0.;
        else
            theta = std::atan2(vec.y(), vec.x());
        radius = KNorm;
    }
    return std::make_tuple(phi, theta, radius);
}

//----------------------------------------------------------------------------------------------------------------------

std::tuple<double, double> unitCartesianToSpherical(EVector3 const& vec)
{
    double theta, phi = std::acos(vec.z());                    // after this phi is in the range [0, Pi]
    if (std::abs(vec.z()) == 1.)
        theta = 0.;
    else
    {
        theta = std::atan2(vec.y(), vec.x());    // after this theta is in the range [-Pi, Pi]. So if theta < 0
        if (theta < 0.)                            // we need to correct it by adding 2*Pi.
            theta += TwoPi;
    }
    return std::make_tuple(phi, theta);
}

//----------------------------------------------------------------------------------------------------------------------

EVector3 sphericalToCartesian(double phi, double theta, double radius)
{
    return radius * EVector3(std::cos(theta) * std::sin(phi), std::sin(theta) * std::sin(phi), std::cos(phi));
}

//----------------------------------------------------------------------------------------------------------------------

EVector3 sphericalToUnitCartesian(double phi, double theta)
{
    return sphericalToCartesian(phi, theta, 1.);
}

//----------------------------------------------------------------------------------------------------------------------

std::tuple<double, double> sphericalToSphereMapCoords(double phi, double theta, double imgWidth, double imgHeight)
{
    // Constrain (locally) phi to be in [0, Pi] and theta in [0, 2Pi)
   constrainSphericalBoundaries(phi, theta);

    // Compute image x position
    double x{imgWidth * (1. - (theta * TwoPiInverted)) - .5};
    // Now x is in (-0.5, w - 0.5]

    // Compute image y position
    double y{(imgHeight * phi) * PiInverted - .5};
    // Now y is in [-0.5, h - 0.5]

    return std::make_tuple(x, y);
}

//----------------------------------------------------------------------------------------------------------------------

std::tuple<double, double> unitCartesianToSphereMapCoords(EVector3 const& vec, double imgWidth, double imgHeight)
{
    double x, y, z(vec.z());
    z = (z > 1. ? 1. : (z < -1. ? -1. : z));

    /**
     * The atan2() returns an arch in the range [-Pi, Pi], i.e. an interval of 2Pi. Therefore we normalize it with the
     * division by 2Pi. Result: KTheta is in the range [-0.5, 0.5].
     */
    const double KTheta(std::abs(z) == 1. ?
                              0. :
                              std::atan2(vec.y(), vec.x()) * TwoPiInverted);
    if (KTheta < 0.)
        x = -KTheta * imgWidth - .5;           // -0.5 < x <= imgWidth/2 - 0.5 (left half of the pixel map)
    else
        x = (1. - KTheta) * imgWidth - .5;     // imgWidth/2 - 0.5 <= x <= imgWidth - 0.5 (right half of the pixel map)

    const double KPhi(std::min(std::acos(z) * PiInverted, 1.));
    y = KPhi * imgHeight - .5;                 // -0.5 <= y <= imgHeight - 0.5
    return std::make_tuple(x, y);
}

//----------------------------------------------------------------------------------------------------------------------

std::tuple<double, double> sphereMapCoordsToSpherical(double x, double y, double imgWidth, double imgHeight)
{
    // Constrain (locally) x to be in [-0.5, w - 0.5] and y in [-0.5, h - 0.5]
    constrainSphereMapBoundaries(imgWidth, imgHeight, x, y);

    // Compute spherical theta coordinate
    double theta = (1. - (x + .5) / imgWidth) * TwoPi;
    // Now theta is in [0, 2Pi]

    // Compute spherical phi coordinate
    double phi = ((y + .5) * Pi) / imgHeight;
    // Now phi is in [0, Pi]
    return std::make_tuple(phi, theta);
}

//----------------------------------------------------------------------------------------------------------------------

EVector3 sphereMapCoordsToUnitCartesian(double x, double y, double imgWidth, double imgHeight)
{
    double phi, theta;
    std::tie(phi, theta) = sphereMapCoordsToSpherical(x, y, imgWidth, imgHeight);
    return sphericalToCartesian(phi, theta, 1.);
}

//----------------------------------------------------------------------------------------------------------------------

void constrainSphericalBoundaries(double& phi, double& theta)
{
    while (phi < 0.)
        phi += TwoPi;
    while (phi >= TwoPi)
        phi -= TwoPi;

    if (phi >= Pi)
    {
        phi = TwoPi - phi;
        theta += Pi;
    }

    while (theta < 0.)
        theta += TwoPi;
    while (theta >= TwoPi)
        theta -= TwoPi;
}

//----------------------------------------------------------------------------------------------------------------------

void constrainSphereMapBoundaries(double imgWidth, double imgHeight, double& x, double& y)
{
    while (y < -.5)
        y += 2 * imgHeight;
    while (y > 2 * imgHeight - .5)
        y -= 2 * imgHeight;
    while (x < -.5)
        x += imgWidth;
    while (x > imgWidth - .5)
        x -= imgWidth;

    /**
     * Here we have -0.5 <= x <= imgWidth - 0.5 and -0.5 <= y <= 2 * imgHeight - 0.5.
     * If the condition below is true, the provided y (let's called it y0) was out of bounds. In this case, the "-1" in
     * the correction is necessary. To illustrate that, see the following examples:
     * a) y0 was too small, let's say y0 = -1 pixel.
     *    Then, the condition in the first while loop is met and we have y = 2h - 1. Without the "-1" in the correction,
     *    the result would be y = 1, but this is wrong! If y0 is -1, due to the geometry of the sphere, we should have
     *    y = 0! [to see why, notice that starting at y = 0, we need to move 0.5 pixel "up" and then another 0.5 pixel
     *    "down" in a location shifted in x by w/2; essentially, the new point is in the first row, thus y = 0]
     *
     * b) y0 is too big, let's say y = h (this is 0.5 off the image, beyond the bottom line)
     *    In this case, none of the while loops related to y is executed. Nevertheless, the condition below is met.
     *    Again, without the "-1" in the correction the result would be y = h, still out of bounds! If y0 = h, similar
     *    to the example above, the geometry of the sphere tells we should have y = h - 1 in a location shifted in x
     *    by w/2.
     */
    if (y > imgHeight - .5)
    {
        y = 2 * imgHeight - y - 1.;
        x += .5 * imgWidth;            // update x and
        if (x > imgWidth - .5)         // make sure we still have x <= imgWidth - 0.5
            x -= imgWidth;
    }

    /**
     * Here we have -0.5 <= x <= imgWidth - 0.5 and -0.5 <= y <= imgHeight - 0.5
     */
}

//----------------------------------------------------------------------------------------------------------------------

double pixelToAngle(double pixelDistance, int imgWidth)
{
    return TwoPi * pixelDistance / double(imgWidth);
}

//----------------------------------------------------------------------------------------------------------------------

double angleToPixel(double angle, int imgWidth)
{
    return double(imgWidth) * angle / TwoPi;
}

//----------------------------------------------------------------------------------------------------------------------

ERotationMatrix computeRotationMatrix(EVector3 const& unitRay)
{
    double phi, theta;
    std::tie(phi, theta, std::ignore) = cartesianToSpherical(unitRay);
    ERotationMatrix rotY;
    ERotationMatrix rotZ;
    rotY << std::cos(phi), 0., std::sin(phi), 0., 1., 0., -std::sin(phi), 0., std::cos(phi);
    rotZ << std::cos(theta), -std::sin(theta), 0., std::sin(theta), std::cos(theta), 0., 0., 0., 1.;
    return rotZ * rotY;
}

//----------------------------------------------------------------------------------------------------------------------

bool crossPixelMapSideBorder(EVector3 const& u1, EVector3 const& u2)
{
    /**
     * The first part of the statement below handles the situation where u1 and u2 are both on the positive-x half of
     * the sphere. The second part covers the other condition: when u1 and u2 are one on the positive-x half of the
     * sphere and the other on the negative-x half.
     * The case where both u1.x and u2.x are on the negative-x half of the sphere will never result in crossing the
     * side borders of the pixel map.
     */
    return ((u1.x() >= 0.) && (u2.x() >= 0.) && (u1.y() * u2.y() < 0.)) ||
           ((u1.x() * u2.x() < 0.) && (u1.y() * u2.y() < 0.) && (u1.x() + u2.x() > 0.));
}

//----------------------------------------------------------------------------------------------------------------------

EVector3 intersectionWithPixelMapSideBorder(EVector3 const& u1, EVector3 const& u2)
{
    if (!crossPixelMapSideBorder(u1, u2))
        return EVector3::Zero();

    if ((u1 + u2).norm() < 1.e-6)
    {
        /**
         * u1 and u2 are opposite to each other, or nearly so. In this case, there are infinite many arcs connecting
         * these two points (all same size). One possible arc passes through (1., 0., 0.) -- the x-axis -- and this is
         * the returned point. Of course, there are infinite many arcs going "the other way" and not intersecting the
         * side border, but in this case it is better to return a possible, valid, intersection than no intersection
         * at all.
         */
        LOG(WARNING) << "Input points are opposite to each other, or nearly so";
        return {1., 0., 0};
    }

    // From now on we know the arc connecting u1 and u2 intersects the side border of the pixel map
    const EVector3 KNormalXZPlane{0., 1., 0};   // the y-axis
    EVector3 normalInputPlane{u1.cross(u2)};
    normalInputPlane.normalize();
    if (std::abs(KNormalXZPlane.dot(normalInputPlane)) > .9999)   // if planes nearly the same
    {
        EVector3 avg{u1 + u2};
        avg.normalize();
        return avg;
    }

    /**
     * According to the cross product below, the intersection point must be perpendicular to KNormalXZPlane, therefore
     * it lies on the XZ-Plane.
     */
    EVector3 intersectionPoint{KNormalXZPlane.cross(normalInputPlane)};
    intersectionPoint.normalize();
    if (intersectionPoint.x() < 0.)
        intersectionPoint = -intersectionPoint; // flip vector if x < 0 (intersection with border must have x >= 0)

    /**
     * The intersection point IP must lie somewhere between u1 and u2. If not, the arc connecting u1 and u2 does not
     * cross the side border, which we know at this point that it does (see first if statement of this function).
     * To determine whether IP is indeed between u1 and u2, the following condition must be satisfied:
     * IP.cross(u1) = -IP.cross(u2).
     * In practice, however, it is better to do
     * n1 = IP.cross(u1)
     * n2 = IP.cross(u2)
     * and check if n1.dot(n2) < 0
     */
    const EVector3 KN1{intersectionPoint.cross(u1)};
    const EVector3 KN2{intersectionPoint.cross(u2)};
    CHECK(KN1.dot(KN2) < 0.) << "Failed to compute intersection point between pixel map side border and the arc "
                             << "connecting the points (" << u1.transpose() << ") and (" << u2.transpose() << ")";
    return intersectionPoint;
}

//----------------------------------------------------------------------------------------------------------------------

EVector3 centroid(std::vector<EVector3> const& points)
{
    EVector3 center{EVector3::Zero()};
    if (points.empty())
        return center;

    for (auto const& point : points)
        center += point;

    return center /= static_cast<double>(points.size());
}

} // namespace core
