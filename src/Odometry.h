#ifndef _ODOMETRY_H_
#define _ODOMETRY_H_

#include <vector>
#include <cstdint>
#include "Matrix.h"

namespace edu
{

    enum OdometryMode
    {
        ODOMETRY_RELATIVE_MODE = 0,
        ODOMETRY_ABSOLUTE_MODE = 1
    };

    struct Pose
    {
        double x;
        double y;
        double theta;
    };

    /**
    * @class Odometry
    * @brief Simple odometry estimation based on motor speeds or motor positions
    * @author Hannes Duske
    * @date 15.03.2024
    */
    class Odometry
    {
    public:

        /**
         * Constructor
         * @param[in] absolute_mode Set odometry to absolute or relative mode
         * @param[in] invKinematicModel Matrix of inverted kinematic vectors (Wheel spin to Twist conversion)
         * @param[in] kin_m0 Kinematic description of motor 0
         * @param[in] kin_m1 Kinematic description of motor 1
         * @param[in] kin_m2 Kinematic description of motor 2
         * @param[in] kin_m3 Kinematic description of motor 3
         */
        //Odometry(OdometryMode odometry_mode, std::vector<double> kin_m0, std::vector<double> kin_m1, std::vector<double> kin_m2, std::vector<double> kin_m3);
        Odometry(OdometryMode odometry_mode, edu::Matrix invKinematicModel);

        /**
         * Destructor
         */
        ~Odometry();

        /**
         * Reset odometry estimation
         */
        void reset();

        /**
         * Set odometry to absolute or relative mode
         * @param[in] odometry_mode
         */
        void set_mode(OdometryMode odometry_mode);

        /**
         * Get current odometry mode
         */
        OdometryMode get_mode();

        /**
         * Check if odometry postition-model is initialized
        */
       bool is_pos_init();

       /**
         * Check if odometry velocity-model is initialized
        */
       bool is_vel_init();

        /*
        * Get current pose estimate
        */
       Pose get_pose();

        /**
         * Update odometry estimation with new wheel positions
         * Absolute postion or change in position depends on odometry_mode setting
         * @param[in] p0 Absolute postion or change in position of motor 0
         * @param[in] p1 Absolute postion or change in position of motor 1
         * @param[in] p2 Absolute postion or change in position of motor 2
         * @param[in] p3 Absolute postion or change in position of motor 3
         * @retval status 1: o.k., status -1: error in last step (no update)
         */
        int update(double p0, double p1, double p2, double p3);

        /**
         * Update odometry estimation with new wheel speeds
         * Absolute time or change in time depends on odometry_mode setting
         * @param[in] time_ns Time in nanoseconds
         * @param[in] p0 Speed of motor 0 in rpm
         * @param[in] p1 Speed of motor 1 in rpm
         * @param[in] p2 Speed of motor 2 in rpm
         * @param[in] p3 Speed of motor 3 in rpm
         * @retval status 1: o.k., status -1: error in last step (no update)
         */
        int update(uint64_t time_ns, double w0, double w1, double w2, double w3);

    protected:
    private:

        /**
         * Calculate pose-cange in world coordinate system
         * @param[in] twistVec Twist vector describing change in x, y and theta in the robot ksys
         * @retval status 1: o.k., status -1: error in calculation
         */
        int propagate_position(edu::Vec twistVec);

        Pose _pose;
        OdometryMode _odometry_mode;

        const double _straigt_line_threshold = 0.001F;
        bool _is_pos_init;
        bool _is_vel_init;

        double _prev_p0;
        double _prev_p1;
        double _prev_p2;
        double _prev_p3;

        uint64_t _prev_time_ns;

        edu::Matrix _invKinematics;
        std::vector<double> _inv_kinematics_0;
        std::vector<double> _inv_kinematics_1;
        std::vector<double> _inv_kinematics_2;
    };

} //namespace

#endif // _ODOMETRY_H_