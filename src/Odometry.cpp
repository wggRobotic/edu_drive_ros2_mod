#include "Odometry.h"
#include <algorithm>
#include <cmath>

namespace edu
{

Odometry::Odometry(OdometryMode odometry_mode, edu::Matrix invKinematicModel) : _odometry_mode(odometry_mode), _invKinematics(invKinematicModel)
{
    reset();
}

Odometry::~Odometry()
{

}

void Odometry::reset()
{
    _is_pos_init = false;
    _is_vel_init = false;

    _prev_p0 = 0;
    _prev_p0 = 0;
    _prev_p0 = 0;
    _prev_p0 = 0;

    _prev_time_ns = 0;

    _pose.x = 0;
    _pose.y = 0;
    _pose.theta = 0;
}

void Odometry::set_mode(OdometryMode odometry_mode)
{
    _odometry_mode = odometry_mode;
}

OdometryMode Odometry::get_mode(){
    return _odometry_mode;
}

bool Odometry::is_pos_init()
{
    return _is_pos_init;
}

bool Odometry::is_vel_init()
{
    return _is_vel_init;
}

Pose Odometry::get_pose()
{
    return _pose;
}

int Odometry::update(double p0, double p1, double p2, double p3)
{
    int status = 1;

    if((_odometry_mode == ODOMETRY_ABSOLUTE_MODE) || (_is_pos_init)){
        
        double dp0 = 0;
        double dp1 = 0;
        double dp2 = 0;
        double dp3 = 0;

        // In case absolute values are give, calculate the difference to prev. position
        if(_odometry_mode == ODOMETRY_RELATIVE_MODE){
            dp0 = p0 - _prev_p0;
            dp1 = p1 - _prev_p1;
            dp2 = p2 - _prev_p2;
            dp3 = p3 - _prev_p3;
        }else{
            dp0 = p0;
            dp1 = p1;
            dp2 = p2;
            dp3 = p3;
        }

        edu::Vec vOmega({ dp0, dp1, dp2, dp3 });
        edu::Vec vTwist = _invKinematics * vOmega;

        if(std::none_of(vTwist.begin(), vTwist.end(), [](double i){ return std::isnan(i) || std::isinf(i); }))
        {
            // Calculate new position in odom ksys
            status = propagate_position(vTwist);
        }else{
            status = -1;
        }
    }
    
    if(status == 1){
        _is_pos_init = true;
        _prev_p0 = p0;
        _prev_p0 = p1;
        _prev_p0 = p2;
        _prev_p0 = p3;
    }

    return status;
}

int Odometry::update(uint64_t time_ns, double w0, double w1, double w2, double w3)
{
    int status = 1;

    if((_odometry_mode == ODOMETRY_RELATIVE_MODE) || (_is_vel_init)){

        // In case absolute values are give, calculate the difference to prev. time
        double dt_s = (_odometry_mode == ODOMETRY_ABSOLUTE_MODE) ? (double)(time_ns - _prev_time_ns) / 10e9F : (double)time_ns / 10e9F;
        
        // TODO: Find out where the error is and fix this!
        // Magic number fix:
        dt_s *= 10;

        // Calculate change in motor position
        // Convert rpm to rad per sec
        double dp0 = w0 * dt_s / 60.0F * 2 * M_PI;
        double dp1 = w1 * dt_s / 60.0F * 2 * M_PI;
        double dp2 = w2 * dt_s / 60.0F * 2 * M_PI;
        double dp3 = w3 * dt_s / 60.0F * 2 * M_PI;

        edu::Vec vOmega({ dp0, dp1, dp2, dp3 });
        edu::Vec vTwist = _invKinematics * vOmega;

        if(std::none_of(vTwist.begin(), vTwist.end(), [](double i){ return std::isnan(i) || std::isinf(i); }))
        {
            // Calculate new position in odom ksys
            status = propagate_position(vTwist);
        }else{
            status = -1;
        }
    }

    if(status == 1){
        _is_vel_init = true;
        _prev_time_ns = time_ns;
    }

    return status;
}

int Odometry::propagate_position(edu::Vec twistVec)
{
    int status = 1;

    auto dx     = twistVec[0];
    auto dy     = twistVec[1];
    auto dtheta = twistVec[2];

    // Euclidean distance of last move
    double ds = sqrt(dx*dx+dy*dy);

    // Direction of last move in world ksys
    double alpha = _pose.theta + atan2(dy, dx);

    // Turn radius of last move
    double r = ds/dtheta;

    // Calculate movement in world ksys
    double dx_world = 0;
    double dy_world = 0;
    if(dtheta < _straigt_line_threshold){
        // Straigt line move
        dx_world = ds * cos(alpha);
        dy_world = ds * sin(alpha);
    }else{
        // Move on curved path
        dx_world = r * (- sin(alpha) + sin(alpha + dtheta));
        dy_world = r * (+ cos(alpha) - cos(alpha + dtheta));
    }

    // Check for errors
    if(std::isnan(dx) || std::isinf(dx)) status = -2;
    if(std::isnan(dy) || std::isinf(dy)) status = -2;
    if(std::isnan(dtheta) || std::isinf(dtheta)) status = -2;

    if(status == 1){    
        // Sum up increments
        _pose.x += dx_world;
        _pose.y += dy_world;
        _pose.theta += dtheta;
    }

    return status;
}

} //namespace