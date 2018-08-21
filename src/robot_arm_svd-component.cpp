#include "robot_arm_svd-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <rtt/os/TimeService.hpp>

RobotArmSVD::RobotArmSVD(std::string const& name) : TaskContext(name)
{
    this->model_loaded = false;

    // setup some the properties
    this->path_to_urdf = "NO PATH!";
    this->addProperty("path_to_urdf", path_to_urdf).doc("Path to the robots URDF.");

    this->base_left_arm = "NO VALUE!";
    this->addProperty("base_left_arm", base_left_arm).doc("Base link of left arm.");

    this->ee_left_arm = "NO VALUE!";
    this->addProperty("ee_left_arm", ee_left_arm).doc("End effector of left arm.");

    this->base_right_arm = "NO VALUE!";
    this->addProperty("base_right_arm", base_right_arm).doc("Base link of right arm.");

    this->ee_right_arm = "NO VALUE!";
    this->addProperty("ee_right_arm", ee_right_arm).doc("End effector of right arm.");

    RTT::log(RTT::Info) << "(RobotArmSVD) Constructed!" << RTT::endlog();
}

bool RobotArmSVD::configureHook()
{
    DEBUGprintProperties();
    // quick check if some value is set to the properties
    if (this->path_to_urdf.compare("NO PATH!") == 0 ||
        this->base_left_arm.compare("NO VALUE!") == 0 ||
        this->ee_left_arm.compare("NO VALUE!") == 0 ||
        this->base_right_arm.compare("NO VALUE!") == 0 ||
        this->ee_right_arm.compare("NO VALUE!") == 0)
    {
        RTT::log(RTT::Error) << "(RobotArmSVD) Some property isn't set!" << RTT::endlog();
        return false;
    }

    if (!this->loadModel())
    {
        return false;
    }

    RTT::log(RTT::Info) << "(RobotArmSVD) Configured!" << RTT::endlog();
    return true;
}

bool RobotArmSVD::startHook()
{
    RTT::log(RTT::Info) << "(RobotArmSVD) Started!" << RTT::endlog();
    return true;
}

void RobotArmSVD::updateHook()
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_left(this->j_left.data.block(0, 0, 3, this->chain_left_arm.getNrOfJoints()), Eigen::ComputeThinU | Eigen::ComputeThinV);
    this->u_left  = svd_left.matrixU().cast<double>();
    this->v_left  = svd_left.matrixV().cast<double>();
    this->sv_left = svd_left.singularValues().cast<double>();

    Eigen::JacobiSVD<Eigen::MatrixXd> svd_right(this->j_left.data.block(0, 0, 3, this->chain_right_arm.getNrOfJoints()), Eigen::ComputeThinU | Eigen::ComputeThinV);
    this->u_right  = svd_right.matrixU().cast<double>();
    this->v_right  = svd_right.matrixV().cast<double>();
    this->sv_right = svd_right.singularValues().cast<double>();

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

// based on https://github.com/dasblondegrauen/lwr-sim/blob/master/lwr_testdriver/src/lwr_testdriver-component.cpp
bool RobotArmSVD::loadModel()
{
    model_loaded = false;

    if(!model.initFile(this->path_to_urdf)) {
        RTT::log(RTT::Error) << "(RobotArmSVD) Could not load model from URDF at " << this->path_to_urdf << RTT::endlog();
        return false;
    }

    if(!kdl_parser::treeFromUrdfModel(model, model_tree)) {
        RTT::log(RTT::Error) << "(RobotArmSVD) Could not get tree from model" << RTT::endlog();
        return false;
    }

    if(!model_tree.getChain(this->base_left_arm, this->ee_left_arm, this->chain_left_arm))
    {
        RTT::log(RTT::Error) << "(RobotArmSVD) Could not get left chain from tree" << RTT::endlog();
        return false;
    }

    if(!model_tree.getChain(this->base_right_arm, this->ee_right_arm, this->chain_right_arm))
    {
        RTT::log(RTT::Error) << "(RobotArmSVD) Could not get right chain from tree" << RTT::endlog();
        return false;
    }

    this->q_left  = KDL::JntArray(this->chain_left_arm.getNrOfJoints());
    this->q_right = KDL::JntArray(this->chain_right_arm.getNrOfJoints());
    this->j_left  = KDL::Jacobian(this->chain_left_arm.getNrOfJoints());
    this->j_right = KDL::Jacobian(this->chain_right_arm.getNrOfJoints());

    // TODO do i need this?
    //fk_solver_pos = std::unique_ptr<KDL::ChainFkSolverPos_recursive>(new KDL::ChainFkSolverPos_recursive(lwr));

    this->jnt_to_jac_solver_left  = std::unique_ptr<KDL::ChainJntToJacSolver>(new KDL::ChainJntToJacSolver(this->chain_left_arm));
    this->jnt_to_jac_solver_right = std::unique_ptr<KDL::ChainJntToJacSolver>(new KDL::ChainJntToJacSolver(this->chain_right_arm));

    model_loaded = true;
return true;
}

void RobotArmSVD::DEBUGprintSVD()
{
    std::cout << "Printing SVDs" << std::endl;
    DEBUGprintSVDLeft();
    DEBUGprintSVDRight();
}

// TODO implement
void RobotArmSVD::DEBUGprintSVDLeft()
{
    std::cout << "  Left SVD:" << std::endl;
    std::cout << "   u: " << '\n' << this->u_left << '\n';
    std::cout << "   v: " << '\n' << this->v_left << '\n';
    std::cout << "   singularValues: " << '\n' << this->sv_left << '\n';
}

// TODO implement
void RobotArmSVD::DEBUGprintSVDRight()
{
    std::cout << "  Right SVD:" <<std::endl;
    std::cout << "   u: " << '\n' << this->u_right << '\n';
    std::cout << "   v: " << '\n' << this->v_right << '\n';
    std::cout << "   singularValues: " << '\n' << this->sv_right << '\n';
}

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
