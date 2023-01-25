#ifndef STAGE_ROS2_PKG__VISIBILITY_H_
#define STAGE_ROS2_PKG__VISIBILITY_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__

  #ifdef __GNUC__
    #define STAGE_ROS2_PACKAGE_EXPORT __attribute__ ((dllexport))
    #define STAGE_ROS2_PACKAGE_IMPORT __attribute__ ((dllimport))
  #else
    #define STAGE_ROS2_PACKAGE_EXPORT __declspec(dllexport)
    #define STAGE_ROS2_PACKAGE_IMPORT __declspec(dllimport)
  #endif

  #ifdef STAGE_ROS2_PACKAGE_DLL
    #define STAGE_ROS2_PACKAGE_PUBLIC STAGE_ROS2_PACKAGE_EXPORT
  #else
    #define STAGE_ROS2_PACKAGE_PUBLIC STAGE_ROS2_PACKAGE_IMPORT
  #endif

  #define STAGE_ROS2_PACKAGE_PUBLIC_TYPE STAGE_ROS2_PACKAGE_PUBLIC

  #define STAGE_ROS2_PACKAGE_LOCAL

#else

  #define STAGE_ROS2_PACKAGE_EXPORT __attribute__ ((visibility("default")))
  #define STAGE_ROS2_PACKAGE_IMPORT

  #if __GNUC__ >= 4
    #define STAGE_ROS2_PACKAGE_PUBLIC __attribute__ ((visibility("default")))
    #define STAGE_ROS2_PACKAGE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define STAGE_ROS2_PACKAGE_PUBLIC
    #define STAGE_ROS2_PACKAGE_LOCAL
  #endif

  #define STAGE_ROS2_PACKAGE_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // STAGE_ROS2_PKG__VISIBILITY_H_
