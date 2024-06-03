#ifndef ROBOT_TWO__VISIBILITY_CONTROL_H_
#define ROBOT_TWO__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define ROBOT_TWO_EXPORT __attribute__((dllexport))
#define ROBOT_TWO_IMPORT __attribute__((dllimport))
#else
#define ROBOT_TWO_EXPORT __declspec(dllexport)
#define ROBOT_TWO_IMPORT __declspec(dllimport)
#endif
#ifdef ROBOT_TWO_BUILDING_DLL
#define ROBOT_TWO_PUBLIC ROBOT_TWO_EXPORT
#else
#define ROBOT_TWO_PUBLIC ROBOT_TWO_IMPORT
#endif
#define ROBOT_TWO_PUBLIC_TYPE ROBOT_TWO_PUBLIC
#define ROBOT_TWO_LOCAL
#else
#define ROBOT_TWO_EXPORT __attribute__((visibility("default")))
#define ROBOT_TWO_IMPORT
#if __GNUC__ >= 4
#define ROBOT_TWO_PUBLIC __attribute__((visibility("default")))
#define ROBOT_TWO_LOCAL __attribute__((visibility("hidden")))
#else
#define ROBOT_TWO_PUBLIC
#define ROBOT_TWO_LOCAL
#endif
#define ROBOT_TWO_PUBLIC_TYPE
#endif

#endif  // ROBOT_TWO__VISIBILITY_CONTROL_H_