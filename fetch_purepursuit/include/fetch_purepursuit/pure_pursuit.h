#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

#include <cmath>
#include <iostream>
#include "purepursuitinterface.h"

/**
 * @class PurePursuitController 
 * @brief PurePursuitController sets the characteristics required for a typical pure pursuit controller that can be applied to multiple scenario
 * @details This class is inheritted from the base PurePursuitInterface. It also has the following propertises:
 * + The sharp turn angle can be adjusted to increase the performance of the controller
 * + The linear velocity to pursuit the target is decided by taking the distance between the current pose of the pursuitter and the target pose times a p controller.
 * + The angular velocity to pursuit the target is decided by:
 *  - If the angle from the current orientation of the pursuitter to the target pose is greater than a threshold, the angular velocity will be that angle times another p controller. If not, it will be the arc from the current pose to the target pose times the linear velocity.
 * + The target pose is the pose in the coordinate frame of the pursuitter. 2D homogenous transformation is applied to transform the point from the world coordinate frame to the local coordinate frame.
 * @note In order for pure pursuit to work, the pose of the target must be within the coordinate frame of the pursuitter. Therefore, in this scenario, a 2D homogenous transformation is needed to convert from the world coordinate frame to the local coordinate frame of the pursuitter.
 */
class PurePursuitController : public PurePursuitInterface
{
    public:

        /**
         * @brief Default Constructor that sets the following:
         * + sharp turn threshold to be the default value
         * + p controller for linear velocity to be the default value
         * + p controller for angular velocity to be the default value
         * 
         */
        PurePursuitController();

        /**
         * @brief Destroy the Pure Pursuit Controller object
         * 
         */
        ~PurePursuitController();

        /**
         * @brief Member function to get the current linear velocity of the object
         * 
         * @return const double: current linear velocity  
         */
        const double getLinearVelocity(void);

        /**
         * @brief Member function to get the current angular velocity of the object
         * 
         * @return const double: current angular velocity
         */
        const double getAngularVelocity(void);

        /**
         * @brief Member function to get the current sharp turn angle threshold of the object
         * 
         * @return const double: current sharp turn angle threshold (radians)
         */
        const double getTurnThreshold(void);

        /**
         * @brief Member function to get the current target pose of the object
         * 
         * @return Pose:current target pose of the object
         */
        Pose2d getTargetPose();

        /**
         * @brief Member function that sets the pose of the target. If aim ahead is turned on, this function will recalculate the pose based on the velocities of the target.
         * 
         * @param pose: pose of the target: x y coordinate and orientation
         */
        void setCurrentPose(Pose2d current_pose);

        /**
         * @brief Member function that sets the pose of the target. If aim ahead is turned on, this function will recalculate the pose based on the velocities of the target.
         * 
         * @param pose: pose of the target: x y coordinate and orientation
         */
        void setTargetPose(Pose2d target_pose);

        /**
         * @brief Member function to set the sharp turn angle threshold of the object
         * 
         * @param angle: Threshold angle
         */
        void setSharpTurnThreshold(double angle);

        /**
         * @brief Member function to set the current target linear and angular velocity of the object
         * 
         * @param linear_vel: linear velocity (m/s)
         * @param angular_vel: angular velocity (rad/s)
         */
        void setTargetVelocity(double linear_vel, double angular_vel);

        /**
         * @brief Member function that sets the p controller value for the linear velocity of the object
         * 
         * @param p_controller_ : linear p controller value
         */
        void setLinearPController(double p_controller_);

        /**
         * @brief Member function that sets the p controller value for the angular velocity of the object
         * 
         * @param p_controller_: angular p controller value
         */
        void setAngularPController(double p_controller_);

        /**
         * @brief Member function that turns on or off aim ahead option
         * 
         * @param status: Turn on or off by setting true or false respectively
         */
        void setAimAhead(bool aim_ahead);

        /**
         * @brief Member function that calculates the linear and angular velocity based on the given current and target pose and velocites
         * @details This function will implement the following calculations:
         * + Recalculate the pose of the target by applying the 2D homogenous transformation to get the pose of the target in the  coordinate frame of the pursuitter. The recalculation is done in setTargetPose() member function.
         * + Calculate the linear and angular velocity to go to that target
         */
        void pursuit();

    protected:

        /**
         * @brief Constant public member variable that sets the default sharp turn angle threshold
         * 
         */
        static const double DEFAULT_TURN_THRESHOLD;

        /**
         * @brief Constant public member variable that sets the default p controller for the linear velocity 
         * 
         */
        static const double DEFAULT_LINEAR_P_CONTROLLER;

        /**
         * @brief Constant public member variable that sets the default p controller for the angular velocity 
         * 
         */
        static const double DEFAULT_ANGULAR_P_CONTROLLER;

        /**
         * @brief Constant public member variable that sets the constant time value to calculate the distance from the velocities for the aim ahead option
         * 
         */
        static const double TIME_CONSTRAINT;

        /**
         * @brief Member variable that contains the sharp turn angle threshold
         * 
         */
        double turn_threshold_; 

        /**
         * @brief Member variable that contains the current linear velocity of the pursuitter
         * 
         */
        double linear_velocity_;

        /**
         * @brief Member variable that contains the current angular velocity of the pursuitter
         * 
         */
        double angular_velocity_;

        /**
         * @brief Member variable that contains the current arc from the current pose to the target pose
         * 
         */
        double arc_;

        /**
         * @brief Member variable that contains the p controller value for linear velocity
         * 
         */
        double linear_p_controller_;

        /**
         * @brief Member variable that contains the p controller value for the angular velocity 
         * 
         */
        double angular_p_controller_;

        /**
         * @brief Member variable the contains the linear velocity of the target
         * 
         */
        double target_linear_;

        /**
         * @brief Member variable that contains the angular velocity of the target
         * 
         */
        double target_angular_;

        /**
         * @brief Member variable that contains the current pose of the pursuitter in the world coordinate frame
         * 
         */
        Pose2d current_pose_;

        /**
         * @brief Member variable the contains the pose of the target in the world coordinate frame
         * 
         */
        Pose2d desired_pose_;

        /**
         * @brief Member variable the contains the pose of the target in pursuitter coordinate frame
         * 
         */
        Pose2d transformed_desired_pose_;

        /**
         * @brief Member variable of the aim ahead option
         * 
         */
        bool aim_ahead_;

        /**
         * @brief Member function that sets the linear velocity for the pursuitter
         * 
         * @param linear_velocity: linear velocity 
         */
        void setDesiredLinearVelocity(double linear_velocity);

        /**
         * @brief Member function that sets the angular velocity for the pursuitter
         * 
         * @param angular_velocity: angular velocity 
         */        
        void setDesiredAngularVelocity(double angular_velocity);

        /**
         * @brief Member function that calculates the arc from the pursuitter to the target position (in the pursuitter coordinate frame)
         * 
         * @param point: position of the target in the pursuitter coordinate frame
         * @return double: lenght of the arc from the pursuitter to the target
         */
        double calculateArc(GlobalOrd point);

        /**
         * @brief Member function that calculates the distance between the pursuitter and the target in the pursuitter coordinate frame
         * 
         * @param point: position of the target in the pursuitter coordinate frame
         * @return double: distancefrom the pursuitter to the target
         * 
         */
        double calculateDistance(GlobalOrd point);

        /**
         * @brief Member function that calculates the linear velocity of the pursuitter
         * 
         * @return double: linear velocity of the pursuitter
         */
        double calculateAngularVelocity();

        /**
         * @brief Member function that calculates the angular velocity of the pursuitter
         * 
         * @return double: angular velocity of the pursuitter
         */
        double calculateLinearVelocity();

        /**
         * @brief Member function to gets the inverse matrix of a given 3x3 matrix
         * 
         * @param matrix; 3x3 matrix
         */
        void getInverseMatrix(double matrix[3][3], double (&inverse_mat)[3][3]);

        /**
         * @brief Member function that calculates the pose of the target in the coordinate frame of the pursuitter from the world coordinate frame
         * 
         * @param transformMatrix 
         * @param origin_pose 
         */
        void getTransformedPose(double transformMatrix[3][3], double origin_pose[3][1],double (&new_pose)[3][1]);

        /**
         * @brief Member function that takes in the position of the target in the world coordinate frame and calculate it to get the position of the target in the pursuitter coordinate frame
         * 
         * @param pose: The pose of the pursuitter in the world coordinate frame
         * @param point: The position of the target in the world coordinate frame
         * @return Pose: The pose of the target in the pursuitter coordinate frame
         */
        Pose2d calculateTransformedPose(Pose2d pose, GlobalOrd point);
        
};
#endif