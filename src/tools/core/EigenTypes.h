
#ifndef SS22_PROJECT_TEST_EIGENTYPES_H_
#define SS22_PROJECT_TEST_EIGENTYPES_H_

#include <Eigen/Geometry>

namespace core {

//=========================================================================
//                              VECTOR TYPES
//=========================================================================

/**
 * \brief Vector of 2 elements.
 *
 * Convenience \c Eigen::Matrix template instantiation of a vector of 2
 * elements.
 * \note This convenience instantiation should be preferred over the
 * convenience typdefs defined by the Eigen library because this
 * instantiation guarantees that the right scalar type is used.
 */
using EVector2 = Eigen::Matrix<double, 2, 1>;

/**
 * \brief Transposed vector of 2 elements.
 *
 * Convenience \c Eigen::Matrix template instantiation of a transposed
 * vector of 2 elements.
 * \note This convenience instantiation should be preferred over the
 * convenience typdefs defined by the Eigen library because this
 * instantiation guarantees that the right scalar type is used.
 */
using EVector2t = Eigen::Matrix<double, 1, 2>;

/**
 * \brief Vector of 3 elements.
 *
 * Convenience \c Eigen::Matrix template instantiation of a vector of 3
 * elements.
 * \note This convenience instantiation should be preferred over the
 * convenience typdefs defined by the Eigen library because this
 * instantiation guarantees that the right scalar type is used.
 */
using EVector3 = Eigen::Matrix<double, 3, 1>;
using ETranslationVector = EVector3;

/**
 * \brief Transposed vector of 3 elements.
 *
 * Convenience \c Eigen::Matrix template instantiation of a transposed
 * vector of 3 elements.
 * \note This convenience instantiation should be preferred over the
 * convenience typdefs defined by the Eigen library because this
 * instantiation guarantees that the right scalar type is used.
 */
using EVector3t = Eigen::Matrix<double, 1, 3>;

/**
 * \brief Vector of 4 elements.
 *
 * Convenience \c Eigen::Matrix template instantiation of a vector of 4
 * elements.
 * \note This convenience instantiation should be preferred over the
 * convenience typdefs defined by the Eigen library because this
 * instantiation guarantees that the right scalar type is used.
 */
using EVector4 = Eigen::Matrix<double, 4, 1>;

/**
 * \brief Transposed vector of 4 elements.
 *
 * Convenience \c Eigen::Matrix template instantiation of a transposed
 * vector of 4 elements.
 * \note This convenience instantiation should be preferred over the
 * convenience typdefs defined by the Eigen library because this
 * instantiation guarantees that the right scalar type is used.
 */
using EVector4t = Eigen::Matrix<double, 1, 4>;

/**
 * \brief Dynamic sized vector.
 *
 * Convenience \c Eigen::Matrix template instantiation of a dynamic sized
 * vector.
 * \note This convenience instantiation should be preferred over the
 * convenience typdefs defined by the Eigen library because this
 * instantiation guarantees that the right scalar type is used.
 */
using EVector = Eigen::Matrix<double, Eigen::Dynamic, 1>;

/**
 * \brief Transposed dynamic sized elements.
 *
 * Convenience \c Eigen::Matrix template instantiation of a transposed
 * dynamic sized.
 * \note This convenience instantiation should be preferred over the
 * convenience typdefs defined by the Eigen library because this
 * instantiation guarantees that the right scalar type is used.
 */
using EVectort = Eigen::Matrix<double, 1, Eigen::Dynamic>;


//=========================================================================
//                              MATRIX TYPES
//=========================================================================

/**
 * \brief Matrix of 2 x 2 elements.
 *
 * Convenience \c Eigen::Matrix template instantiation of a matrix of 2 x 2
 * elements.
 * \note This convenience instantiation should be preferred over the
 * convenience typdefs defined by the Eigen library because this
 * instantiation guarantees that the right scalar type is used.
 */
using EMatrix2x2 = Eigen::Matrix<double, 2, 2>;

/**
 * \brief Matrix of 2 x 3 elements.
 *
 * Convenience \c Eigen::Matrix template instantiation of a matrix of 2 x 3
 * elements.
 * \note This convenience instantiation should be preferred over the
 * convenience typdefs defined by the Eigen library because this
 * instantiation guarantees that the right scalar type is used.
 */
using EMatrix2x3 = Eigen::Matrix<double, 2, 3>;

/**
 * \brief Matrix of 3 x 2 elements.
 *
 * Convenience \c Eigen::Matrix template instantiation of a matrix of 3 x 2
 * elements.
 * \note This convenience instantiation should be preferred over the
 * convenience typdefs defined by the Eigen library because this
 * instantiation guarantees that the right scalar type is used.
 */
using EMatrix3x2 = Eigen::Matrix<double, 3, 2>;

/**
 * \brief Matrix of 3 x 3 elements.
 *
 * Convenience \c Eigen::Matrix template instantiation of a matrix of 3 x 3
 * elements.
 * \note This convenience instantiation should be preferred over the
 * convenience typdefs defined by the Eigen library because this
 * instantiation guarantees that the right scalar type is used.
 */
using EMatrix3x3 = Eigen::Matrix<double, 3, 3>;
using ERotationMatrix = EMatrix3x3;
using EEssentialMatrix = EMatrix3x3;
using EFundamentalMatrix = EMatrix3x3;

/**
 * \brief Matrix of 4 x 2 elements.
 *
 * Convenience \c Eigen::Matrix template instantiation of a matrix of 4 x 2
 * elements.
 * \note This convenience instantiation should be preferred over the
 * convenience typdefs defined by the Eigen library because this
 * instantiation guarantees that the right scalar type is used.
 */
using EMatrix4x2 = Eigen::Matrix<double, 4, 2>;

/**
 * \brief Matrix of 4 x 3 elements.
 *
 * Convenience \c Eigen::Matrix template instantiation of a matrix of 4 x 3
 * elements.
 * \note This convenience instantiation should be preferred over the
 * convenience typdefs defined by the Eigen library because this
 * instantiation guarantees that the right scalar type is used.
 */
using EMatrix4x3 = Eigen::Matrix<double, 4, 3>;

/**
 * \brief Matrix of 2 x 4 elements.
 *
 * Convenience \c Eigen::Matrix template instantiation of a matrix of 2 x 4
 * elements.
 * \note This convenience instantiation should be preferred over the
 * convenience typdefs defined by the Eigen library because this
 * instantiation guarantees that the right scalar type is used.
 */
using EMatrix2x4 = Eigen::Matrix<double, 2, 4>;

/**
 * \brief Matrix of 3 x 4 elements.
 *
 * Convenience \c Eigen::Matrix template instantiation of a matrix of 3 x 4
 * elements.
 * \note This convenience instantiation should be preferred over the
 * convenience typdefs defined by the Eigen library because this
 * instantiation guarantees that the right scalar type is used.
 */
using EMatrix3x4 = Eigen::Matrix<double, 3, 4>;
using EProjectionMatrix = Eigen::Matrix<double, 3, 4>;

/**
 * \brief Matrix of 4 x 4 elements.
 *
 * Convenience \c Eigen::Matrix template instantiation of a matrix of 4 x 4
 * elements.
 * \note This convenience instantiation should be preferred over the
 * convenience typdefs defined by the Eigen library because this
 * instantiation guarantees that the right scalar type is used.
 */
using EMatrix4x4 = Eigen::Matrix<double, 4, 4>;

/**
 * \brief Matrix of 6 x 6 elements.
 *
 * Convenience \c Eigen::Matrix template instantiation of a matrix of 6 x 6
 * elements.
 * \note This convenience instantiation should be preferred over the
 * convenience typdefs defined by the Eigen library because this
 * instantiation guarantees that the right scalar type is used.
 */
using EMatrix6x6 = Eigen::Matrix<double, 6, 6>;

/**
 * \brief Dynamic sized matrix.
 *
 * Convenience \c Eigen::Matrix template instantiation of a dynamic sized
 * matrix.
 * \note This convenience instantiation should be preferred over the
 * convenience typdefs defined by the Eigen library because this
 * instantiation guarantees that the right scalar type is used.
 */
using EMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;


//=========================================================================
//                              OTHER TYPES
//=========================================================================

/**
 * \brief Quaternion.
 *
 * Convenience \c Eigen::Quaternion template instantiation.
 * \note This convenience instantiation guarantees that the right scalar
 * type is used.
 */
using EQuaternion = Eigen::Quaternion<double>;

/**
 * \brief Angle axis.
 *
 * Convenience \c Eigen::AngleAxis template instantiation.
 * \note This convenience instantiation guarantees that the right scalar
 * type is used.
 */
using EAngleAxis = Eigen::AngleAxis<double>;

} // namespace core

/**
 * \brief Read Eigen matrix from istream.
 *
 * This operator allows for reading Eigen matrices from istream.
 *
 * \note This function must unfortunately be kept in the global
 * namespace in order to be guaranteed to be accessible when needed.
 */
template <class BaseType>
std::istream& operator>>(std::istream& is, Eigen::MatrixBase<BaseType>& m)
{
    for (int i = 0; i < m.rows(); ++i)
        for (int j = 0; j < m.cols(); ++j)
            is >> m(i, j);
    return is;
}

#endif // SS22_PROJECT_TEST_EIGENTYPES_H_
