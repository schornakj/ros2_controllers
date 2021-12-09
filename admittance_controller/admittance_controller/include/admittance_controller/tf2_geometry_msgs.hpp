#pragma once

#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer_interface.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <kdl/frames.hpp>

namespace tf2
{
/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Vector type.
  * This function is a specialization of the doTransform template defined in tf2/convert.h.
  * \param t_in The vector to transform, as a Vector3 message.
  * \param t_out The transformed vector, as a Vector3 message.
  * \param transform The timestamped transform to apply, as a TransformStamped message.
  */
//template <>
//inline
//void doTransform(const geometry_msgs::msg::Vector3& t_in, geometry_msgs::msg::Vector3& t_out, const geometry_msgs::msg::TransformStamped& transform)
//{
//  KDL::Vector v_out = gmTransformToKDL(transform).M * KDL::Vector(t_in.x, t_in.y, t_in.z);
//  t_out.x = v_out[0];
//  t_out.y = v_out[1];
//  t_out.z = v_out[2];
//}

/** \brief Convert a tf2 Transform type to its equivalent geometry_msgs representation.
 * \param in A tf2 Transform object.
 * \param out The Transform converted to a geometry_msgs message type.
 */
inline
/** This section is about converting */
void toMsg(const tf2::Transform& in, geometry_msgs::msg::Transform& out)
{
  out = toMsg(in);
}


/**********************/
/*** WrenchStamped ****/
/**********************/

/** \brief Extract a timestamp from the header of a Wrench message.
 * This function is a specialization of the getTimestamp template defined in tf2/convert.h.
 * \param t WrenchStamped message to extract the timestamp from.
 * \return The timestamp of the message.
 */
template <>
inline
  tf2::TimePoint getTimestamp(const geometry_msgs::msg::WrenchStamped& t) {return tf2_ros::fromMsg(t.header.stamp);}

/** \brief Extract a frame ID from the header of a Wrench message.
 * This function is a specialization of the getFrameId template defined in tf2/convert.h.
 * \param t WrenchStamped message to extract the frame ID from.
 * \return A string containing the frame ID of the message.
 */
template <>
inline
  std::string getFrameId(const geometry_msgs::msg::WrenchStamped& t) {return t.header.frame_id;}


/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Wrench type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The wrench to transform, as a Wrench message.
 * \param t_out The transformed wrench, as a Wrench message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline
  void doTransform(const geometry_msgs::msg::Wrench& t_in, geometry_msgs::msg::Wrench& t_out, const geometry_msgs::msg::TransformStamped& transform)
  {
    doTransform(t_in.force, t_out.force, transform);
    doTransform(t_in.torque, t_out.torque, transform);
  }

/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs WrenchStamped type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The wrench to transform, as a timestamped Wrench message.
 * \param t_out The transformed wrench, as a timestamped Wrench message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline
  void doTransform(const geometry_msgs::msg::WrenchStamped& t_in, geometry_msgs::msg::WrenchStamped& t_out, const geometry_msgs::msg::TransformStamped& transform)
  {
    doTransform(t_in.wrench, t_out.wrench, transform);
    t_out.header.stamp = transform.header.stamp;
    t_out.header.frame_id = transform.header.frame_id;
  }

/** \brief Trivial "conversion" function for Wrench message type.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A WrenchStamped message.
 * \return The input argument.
 */
inline
geometry_msgs::msg::WrenchStamped toMsg(const geometry_msgs::msg::WrenchStamped& in)
{
  return in;
}

/** \brief Trivial "conversion" function for Wrench message type.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param msg A WrenchStamped message.
 * \param out The input argument.
 */
inline
void fromMsg(const geometry_msgs::msg::WrenchStamped& msg, geometry_msgs::msg::WrenchStamped& out)
{
  out = msg;
}

/** \brief Convert as stamped tf2 Array with 2 Vector3 type to its equivalent geometry_msgs representation.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A instance of the tf2::Stamped<std::array<tf2::Vector3, 2>> specialization of the tf2::Stamped template.
 * \return The WrenchStamped converted to a geometry_msgs WrenchStamped message type.
 */
// inline
// geometry_msgs::msg::WrenchStamped toMsg(const tf2::Stamped<std::array<tf2::Vector3, 2>>& in)
// {
//   geometry_msgs::msg::WrenchStamped out;
//   out.header.stamp = tf2_ros::toMsg(in.stamp_);
//   out.header.frame_id = in.frame_id_;
//   out.wrench.force = toMsg(in[0]);
//   out.wrench.torque = toMsg(in[1]);
//   return out;
// }

/** \brief Convert a WrenchStamped message to its equivalent tf2 representation.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A WrenchStamped message type.
 * \param out The WrenchStamped converted to the equivalent tf2 type.
 */
// inline
// void fromMsg(const geometry_msgs::msg::WrenchStamped& in, tf2::Stamped<std::array<tf2::Vector3, 2>>& out)
// {
//   out.stamp_ = tf2_ros::fromMsg(in.header.stamp);
//   out.frame_id_ = in.header.frame_id;
//   tf2::Vector3 tmp;
//   fromMsg(in.wrench.force, tmp);
//   tf2::Vector3 tmp1;
//   fromMsg(in.wrench.torque, tmp1);
//   std::array<tf2::Vector3, 2> tmp_array;
//   tmp_array[0] = tmp;
//   tmp_array[1] = tmp1;
//   out.setData(tmp_array);
// }
}

