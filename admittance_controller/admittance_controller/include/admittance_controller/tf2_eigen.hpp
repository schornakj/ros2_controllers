#pragma once

#include <tf2/convert.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_ros/buffer.h>

namespace tf2
{
/** \brief Apply a geometry_msgs TransformStamped to a 6-long Eigen vector.
 * Useful for transforming wrenches or twists.
 * Wrench: (force-x, force-y, force-z, torque-x, torque-y, torque-z)
 * Twist: (trans. vel. x, trans. vel. y, trans. vel. z, ang. vel. x, ang. vel. y, ang. vel. z)
 * This function is a specialization of the doTransform template defined in tf2/convert.h,
 * although it can not be used in tf2_ros::BufferInterface::transform because this
 * functions rely on the existence of a time stamp and a frame id in the type which should
 * get transformed.
 * \param t_in The vector to transform. Must be 6-long.
 * \param t_out The transformed vector. Will be 6-long.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
//template<>
//inline
//void doTransform(
//  const Eigen::VectorXd & t_in,
//  Eigen::VectorXd & t_out,
//  const geometry_msgs::msg::TransformStamped & transform)
//{
//  // References:
//  // https://core.ac.uk/download/pdf/154240607.pdf, https://www.seas.upenn.edu/~meam520/notes02/Forces8.pdf

//  Eigen::Isometry3d affine_transform = tf2::transformToEigen(transform);

//  // Build the 6x6 transformation matrix
//  Eigen::MatrixXd twist_transform(6, 6);
//  // upper left 3x3 block is the rotation part
//  twist_transform.block(0, 0, 3, 3) = affine_transform.rotation();
//  // upper right 3x3 block is all zeros
//  twist_transform.block(0, 3, 3, 3) = Eigen::MatrixXd::Zero(3, 3);
//  // lower left 3x3 block is tricky. See https://core.ac.uk/download/pdf/154240607.pdf
//  Eigen::MatrixXd pos_vector_3x3(3, 3);
//  // Disable formatting checks so the matrix remains human-readable
///* *INDENT-OFF* */
//  pos_vector_3x3(0, 0) = 0;  pos_vector_3x3(0, 1) = -affine_transform.translation().z();  pos_vector_3x3(0, 2) = affine_transform.translation().y(); // NOLINT
//  pos_vector_3x3(1, 0) = affine_transform.translation().z();  pos_vector_3x3(1, 1) = 0;  pos_vector_3x3(1, 2) = -affine_transform.translation().x(); // NOLINT
//  pos_vector_3x3(2, 0) = -affine_transform.translation().y();  pos_vector_3x3(2, 1) = affine_transform.translation().x();  pos_vector_3x3(1, 2) = 0; // NOLINT
///* *INDENT-ON* */
//  twist_transform.block(3, 0, 3, 3) = pos_vector_3x3 * affine_transform.rotation();
//  // lower right 3x3 block is the rotation part
//  twist_transform.block(3, 3, 3, 3) = affine_transform.rotation();

//  t_out = twist_transform * t_in;
//}

}
