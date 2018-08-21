#include "robot_arm_svd-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <rtt/os/TimeService.hpp>

RobotArmSVD::RobotArmSVD(std::string const& name) : TaskContext(name)
{
    // setup some the properties
    path_to_urdf = "NO PATH!";
    this->addProperty("path_to_urdf", path_to_urdf).doc("Path to the robots URDF.");
    base_left_arm = "NO VALUE!";
    this->addProperty("base_left_arm", base_left_arm).doc("Base link of left arm.");
    ee_left_arm = "NO VALUE!";
    this->addProperty("ee_left_arm", ee_left_arm).doc("End effector of left arm.");
    base_right_arm = "NO VALUE!";
    this->addProperty("base_right_arm", base_right_arm).doc("Base link of right arm.");
    ee_right_arm = "NO VALUE!";
    this->addProperty("ee_right_arm", ee_right_arm).doc("End effector of right arm.");

    std::cout << "RobotArmSVD constructed !" <<std::endl;
}

bool RobotArmSVD::configureHook()
{
    if (!setupKinematicChains())
    {
        return false;
    }

    DEBUGprintProperties();
    // quick check if some value is set to the properties
    if (path_to_urdf.compare("NO PATH!") == 0 ||
        base_left_arm.compare("NO VALUE!") == 0 ||
        ee_left_arm.compare("NO VALUE!") == 0 ||
        base_right_arm.compare("NO VALUE!") == 0 ||
        ee_right_arm.compare("NO VALUE!") == 0)
    {
        RTT::log(RTT::Error) << "(RobotArmSVD) Some property isn't set!" << RTT::endlog();
        return false;
    }
    return true;
}

bool RobotArmSVD::startHook()
{
    std::cout << "RobotArmSVD started !" <<std::endl;
    return true;
}

void RobotArmSVD::updateHook()
{
    DEBUGprintSVD();
    //std::cout << "RobotArmSVD executes updateHook !" <<std::endl;
    //RTT::log(RTT::Error) << "RobotArmSVD executes updateHook !" << RTT::endlog();
}

void RobotArmSVD::stopHook()
{
    std::cout << "RobotArmSVD executes stopping !" <<std::endl;
}

void RobotArmSVD::cleanupHook()
{
    std::cout << "RobotArmSVD cleaning up !" <<std::endl;
}

// TODO implement
bool RobotArmSVD::setupKinematicChains()
{
    return true;
}

// TODO implement
void RobotArmSVD::DEBUGprintSVD()
{
    std::cout << "Printing SVDs" <<std::endl;
    DEBUGprintSVDLeft();
    DEBUGprintSVDRight();
}

// TODO implement
void RobotArmSVD::DEBUGprintSVDLeft()
{
    std::cout << "  Left SVD:" <<std::endl;
    std::cout << "NOT IMPLEMENTED!!!" <<std::endl;
}

// TODO implement
void RobotArmSVD::DEBUGprintSVDRight()
{
    std::cout << "  Right SVD:" <<std::endl;
    std::cout << "NOT IMPLEMENTED!!!" <<std::endl;
}

// TODO implement
void RobotArmSVD::DEBUGprintProperties()
{
    std::cout << "##########################################" << '\n';
    std::cout << "  Path:       " << path_to_urdf << '\n';
    std::cout << "  Base Left:  " << base_left_arm << '\n';
    std::cout << "  EE Left:    " << ee_left_arm << '\n';
    std::cout << "  Base Right: " << base_right_arm << '\n';
    std::cout << "  EE Right:   " << ee_right_arm << '\n';
    std::cout << "##########################################" << std::endl;
}

// TODO implement
void RobotArmSVD::DEBUGsetupFakeStates()
{

}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Helloworld)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(RobotArmSVD)
