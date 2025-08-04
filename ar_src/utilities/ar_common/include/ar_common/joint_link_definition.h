#ifndef __AR_COMMON_JOINT_LINK_DEFINITION_H__
#define __AR_COMMON_JOINT_LINK_DEFINITION_H__


#include <map>
#include <string>
#include <string_view>
#include <vector>
namespace ar
{
    inline std::string S(std::string_view s)
    {
        return std::string(s);
    }
    namespace jointgroup
    {
        inline constexpr std::string_view ARM = "arm";
    }
    namespace joint
    {
        // Arm Joint names
        inline constexpr std::string_view AJ1 = "Arm_joint1";
        inline constexpr std::string_view AJ2 = "Arm_joint2";
        inline constexpr std::string_view AJ3 = "Arm_joint3";
        inline constexpr std::string_view AJ4 = "Arm_joint4";
        inline constexpr std::string_view AJ5 = "Arm_joint5";
        inline constexpr std::string_view AJ6 = "Arm_joint6";

        // Arm link names
        inline constexpr std::string_view AL1 = "Arm_link1";
        inline constexpr std::string_view AL2 = "Arm_link2";
        inline constexpr std::string_view AL3 = "Arm_link3";
        inline constexpr std::string_view AL4 = "Arm_link4";
        inline constexpr std::string_view AL5 = "Arm_link5";
        inline constexpr std::string_view AL6 = "Arm_link6";

        inline const std::vector<std::string> ARM = {S(AJ1), S(AJ2), S(AJ3), S(AJ4), S(AJ5), S(AJ6)};
    }
    namespace link
    {
        // End effector link
        inline constexpr std::string_view ENDEFF_ARM = "Arm_endlink";
    }

    namespace
    {

        inline const std::vector<std::string> JointGroups = {
            S(jointgroup::ARM), //
        };

        inline const std::map<std::string, std::vector<std::vector<std::string>>> GroupToJoints = {
            // None
        };

        inline const std::map<std::string, std::vector<std::string>> GroupToEffs = {
            // None
        };
    }
} // namespace ar


#endif 
