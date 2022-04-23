
/*
 * These functions are based on those defined in the Argos framework
 * (Augmented Vision Research Group, German Research Center for Artificial
 * Intelligence (DFKI GmbH)). Here, some modifications to their interface
 * and documentation have been done. Concerning the implementation, only
 * minimal modifications were applied and no change in behavior is expected.
 * Finally, some functions were discarded as they do not fit to Dense3DKit.
 */

#ifndef SS22_PROJECT_TEST_MATHUTILS_H_
#define SS22_PROJECT_TEST_MATHUTILS_H_

#include <tuple>
#include <vector>

#include "EigenTypes.h"

namespace core {

 /**
  * @brief Computes spherical coordinates from a Cartesian vector
  *
  * @param vec vec Vector to get the spherical coordinates from
  * @return Tuple of <phi, theta, radius>
  */
std::tuple<double, double, double> cartesianToSpherical(EVector3 vec);

/**
 * \brief Computes spherical coordinates from a unit Cartesian vector
 *
 * In Dense3DKit, it is common to work with unit Cartesian vectors. The function
 * cartesianToSpherical() above is general and therefore not optimized for the
 * case of unit vectors. This function was added to allow efficient conversion
 * from unit vectors to spherical coordinates. Thus, make sure \a vec is a unit
 * vector before calling this function.
 *
 * \param[in] vec Unit Cartesian vector
 * @return Tuple of <phi, theta>
 *
 * Output:
 * phi will be in the range [0, Pi]
 * theta will be in the range [0, 2*Pi]
 */
std::tuple<double, double> unitCartesianToSpherical(EVector3 const& vec);

/**
 * \brief Transforms spherical coordinates into a Cartesian vector
 *
 * \param[in] phi The phi coordinate
 * \param[in] theta The theta coordinate
 * \param[in] radius The radius
 *
 * \return Cartesian vector of (phi, theta, radius)
 */
EVector3 sphericalToCartesian(double phi, double theta, double radius);

/**
 * \brief Transforms spherical coordinates into a unit Cartesian vector
 * \param[in] phi The phi coordinate
 * \param[in] theta The theta coordinate
 * \return Cartesian vector of (phi, theta, 1)
 */
EVector3 sphericalToUnitCartesian(double phi, double theta);

/**
 * \brief Converts spherical coordinates into sphere map (pixel) coordinates
 *
 * \param[in] phi The phi coordinate
 * \param[in] theta The theta coordinate
 * \param[in] imgWidth The width of the sphere map
 * \param[in] imgHeight The height of the sphere map
 * @return Tuple of <x, y>
 *
 * x Sphere map x coordinate, will be set to a value in (-0.5, imgWidth - 0.5]
 * y Sphere map y coordinate, will be set to a value in [-0.5, imgHeight - 0.5]
 */
std::tuple<double, double> sphericalToSphereMapCoords(double phi,
                                                      double theta,
                                                      double imgWidth,
                                                      double imgHeight);

/**
 * \brief Converts a unit Cartesian vector to sphere map (pixel) coordinates
 *
 * \param[in] vec Unit Cartesian vector
 * \param[in] imgWidth The width of the sphere map
 * \param[in] imgHeight The height of the sphere map
 * @return Tuple of <x, y>
 *
 * x Sphere map x coordinate, will be set to a value in (-0.5, imgWidth - 0.5]
 * y Sphere map y coordinate, will be set to a value in [-0.5, imgHeight - 0.5]
 */
std::tuple<double, double>  unitCartesianToSphereMapCoords(EVector3 const& vec,
                                                           double imgWidth,
                                                           double imgHeight);

/**
 * \brief Converts sphere map (pixel) coordinates into spherical coordinates
 *
 * \param[in] x Sphere map x coordinate
 * \param[in] y Sphere map y coordinate
 * \param[in] imgWidth The width of the sphere map
 * \param[in] imgHeight The height of the sphere map
 * @return Tuple of <phi, theta>
 *
 * Output:
 * phi will be in the range [0, Pi]
 * theta will be in the range [0, 2*Pi]
 */
std::tuple<double, double> sphereMapCoordsToSpherical(double x,
                                                      double y,
                                                      double imgWidth,
                                                      double imgHeight);

/**
 * \brief Converts sphere map (pixel) coordinates into a unit Cartesian vector
 *
 * \param[in] x Sphere map x coordinate
 * \param[in] y Sphere map y coordinate
 * \param[in] imgWidth The width of the sphere map
 * \param[in] imgHeight The height of the sphere map
 *
 * \return Unit Cartesian vector defined by (x, y)
 */
EVector3 sphereMapCoordsToUnitCartesian(double x,
                                        double y,
                                        double imgWidth,
                                        double imgHeight);

/**
 * \brief Constrains spherical coordinates phi and theta to be in the right boundaries
 *
 * phi will be in the range [0, Pi] and
 * theta will be in the range [0, 2*Pi)
 * taking the double periodicity of the sphere into account.
 *
 * \param[in,out] phi The phi coordinate
 * \param[in,out] theta The theta coordinate
 */
void constrainSphericalBoundaries(double& phi, double& theta);

/**
 * \brief Constrains sphere map coordinates x and y to be in the right boundaries
 *
 * x will be in the range [-0.5, w-0.5] and
 * y will be in the range [-0.5, h-0.5]
 * taking the double periodicity of the SphereMap into account.
 *
 * \param[in] imgWidth The width of the sphere map
 * \param[in] imgHeight The height of the sphere map
 * \param[in,out] x Sphere map x coordinate
 * \param[in,out] y Sphere map y coordinate
 */
void constrainSphereMapBoundaries(double imgWidth,
                                  double imgHeight,
                                  double& x,
                                  double& y);

/**
 * \brief Converts a pixel distance to the corresponding angle deviation
 *
 * \note The actual angle deviation depends on the resolution of the
 * image (Sphere Map) used.
 *
 * \param[in] pixelDistance The pixel distance
 * \param[in] imgWidth Width of the image, given in pixels
 *
 * \return The angle deviation defined by the pixel distance
 */
double pixelToAngle(double pixelDistance, int imgWidth);

/**
 * \brief Converts an angular deviation to the corresponding pixel distance
 *
 * \note The actual pixel distance depends on the resolution of the
 * image (Sphere Map) used.
 *
 * \param[in] angle The angular deviation, in radians
 * \param[in] imgWidth Width of the image, in pixels
 * \return The pixel distance defined by the angular deviation
 */
double angleToPixel(double angle, int imgWidth);

/**
 * \brief Computes a rotation matrix for a given point on the sphere
 *
 * This function is useful to determine the rotation required to transform not only
 * a single point but several points on the unit sphere at once.
 *
 * \param[in] unitRay Any point on the surface of the unit sphere
 *
 * \return The rotation matrix transforming the north pole to the provided \a unitRay
 */
ERotationMatrix computeRotationMatrix(EVector3 const& unitRay);

/**
 * \brief Computes scalar product
 *
 * The function does not assume the arrays are normalized; it simply
 * computes the scalar product between them. Additionally, it assumes
 * both arrays have the same number of elements, given by \a size.
 *
 * \note Currently available only for float and double.
 *
 * \param[in] size Size of both arrays
 * \param[in] array0 Template pointer to first array
 * \param[in] array1 Template pointer to second array
 *
 * \return The scalar product between the arrays
 */
template <typename T>
T scalarProduct(int size, T const* array0, T const* array1);

/**
 * \brief Computes sum of squared differences
 *
 * The function does not assume the arrays are normalized; it simply
 * computes the sum of squared differences (SSD) between them.
 * Additionally, it assumes both arrays have the same number of elements,
 * given by \a size.
 *
 * \note Currently available only for float and double.
 *
 * \param[in] size Size of both arrays
 * \param[in] array0 Template pointer to first array
 * \param[in] array1 Template pointer to second array
 *
 * \return The sum of squared differences between the arrays
 */
template <typename T>
T computeSSD(int size, T const* array0, T const* array1);

/**
 * \brief Detects whether the arc connecting \a u1 and \a u2 crosses the side border
 *        of the pixel map
 *
 * The side border mentioned above is where the x-coordinate goes from width - 1 to width
 * (or, equivalently, from 0 to -1).
 *
 * \param[in] u1 Unit Cartesian vector
 * \param[in] u2 Unit Cartesian vector
 *
 * \return true if the side border is crossed
 */
bool crossPixelMapSideBorder(EVector3 const& u1, EVector3 const& u2);

/**
 * \brief Computes the point where the arc connecting \a u1 and \a u2 crosses the side border
 *        of the pixel map
 *
 * Different from crossPixelMapSideBorder() above, this function returns the actual
 * point where the arc connecting \a u1 and \a u2 intersects the side border of the
 * pixel map. If the arc does not intersect the side border of the pixel map, a zero
 * vector is returned.
 *
 * \param[in] u1 Unit Cartesian vector
 * \param[in] u2 Unit Cartesian vector
 *
 * \return the intersection point or a zero-vector if not intersection (see description above)
 */
EVector3 intersectionWithPixelMapSideBorder(EVector3 const& u1, EVector3 const& u2);

/**
 * \brief Computes the center of mass of \a points
 */
EVector3 centroid(std::vector<EVector3> const& points);

/**
 * @brief Comparison operations with an epsilon applied. Mainly used for float and double.
 */
template <typename T>
bool equal(T a, T b);

template <typename T>
bool zero(T a);

template <typename T>
bool lessThan(T a, T b);

template <typename T>
bool lessThanOrEqual(T a, T b);

template <typename T>
bool greaterThan(T a, T b);

} // namespace core

#include "MathUtils.hpp"

#endif // SS22_PROJECT_TEST_MATHUTILS_H_
