#ifndef OROCOS_ROBOTARMSVD_COMPONENT_HPP
#define OROCOS_ROBOTARMSVD_COMPONENT_HPP

#include <rtt/RTT.hpp>
//#include <Eigen/SVD>
//#include <Eigen/Core>
//#include <Eigen/Dense>
#include <limits>
#include <string>
#include <rtt/Port.hpp>

class RobotArmSVD : public RTT::TaskContext
{
public:
    RobotArmSVD(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
private:
    // properties for easy reuse of component
    std::string path_to_urdf,
                base_left_arm,
                ee_left_arm,
                base_right_arm,
                ee_right_arm;

    // some ports to get current joint states
    // TODO port data types
    //RTT::InputPort<> torso_state;
    //RTT::InputPort<> left_arm_state;
    //RTT::InputPort<> right_arm_state;

    // 'global' vars
    // TODO add kinematic chains for arms
    // TODO add SVD left
    // TODO add SVD right

    bool setupKinematicChains(); // loads urdf specified by 'path_to_urdf'


    // some debug functions. just some printing/setting to evade a proper debugger
    void DEBUGprintSVD();
    void DEBUGprintSVDLeft();
    void DEBUGprintSVDRight();
    void DEBUGprintProperties();
    void DEBUGsetupFakeStates();
};
#endif
