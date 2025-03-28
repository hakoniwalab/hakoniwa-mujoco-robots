#pragma once

#include <mujoco/mujoco.h>
#include <sstream>
#include <iomanip>

namespace hako::robots::types
{
    class Vector3
    {
    public:
        double x;
        double y;
        double z;
        double length() const {
            return std::sqrt(x*x + y*y + z*z);
        }
        
        Vector3 normalize() const {
            double len = length();
            return (len > 0) ? (*this / len) : Vector3();
        }
        std::string to_string() const {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(3);
            oss << x << ", " << y << ", " << z;
            return oss.str();
        }
        
        //copy constructor
        Vector3(const Vector3& v)
        {
            x = v.x;
            y = v.y;
            z = v.z;
        }
        //default constructor
        Vector3()
        {
            x = 0;
            y = 0;
            z = 0;
        }
        // = operator
        Vector3& operator=(const Vector3& v)
        {
            x = v.x;
            y = v.y;
            z = v.z;
            return *this;
        }
        // + operator
        Vector3 operator+(const Vector3& v) const
        {
            Vector3 result;
            result.x = x + v.x;
            result.y = y + v.y;
            result.z = z + v.z;
            return result;
        }
        // - operator
        Vector3 operator-(const Vector3& v) const
        {
            Vector3 result;
            result.x = x - v.x;
            result.y = y - v.y;
            result.z = z - v.z;
            return result;
        }
        // * operator
        Vector3 operator*(const double& scalar) const
        {
            Vector3 result;
            result.x = x * scalar;
            result.y = y * scalar;
            result.z = z * scalar;
            return result;
        }
        // / operator
        Vector3 operator/(const double& scalar) const
        {
            Vector3 result;
            result.x = x / scalar;
            result.y = y / scalar;
            result.z = z / scalar;
            return result;
        }

    };

    /*
     * Position class
     * Unit: Meters
     * Frame: ROS standard frame
     */
    class Position: public Vector3
    {
        public:
            Position(const Vector3& v)
            {
                x = v.x;
                y = v.y;
                z = v.z;
            }
            Position()
            {
                x = 0;
                y = 0;
                z = 0;
            }
    };
    /*
     * Attitude class
     * Unit: Radians
     * Frame: ROS standard frame
     */
    class Euler: public Vector3
    {
        public:
            Euler(const Vector3& v)
            {
                x = v.x;
                y = v.y;
                z = v.z;
            }
            Euler()
            {
                x = 0;
                y = 0;
                z = 0;
            }
    };

    /*
     * Velocity class
     * Unit: Meters per second
     * Frame: ROS standard frame
     * World frame
     */
    class Velocity: public Vector3
    {
        public:
            Velocity(const Vector3& v)
            {
                x = v.x;
                y = v.y;
                z = v.z;
            }
            Velocity()
            {
                x = 0;
                y = 0;
                z = 0;
            }
    };

    /*
     * BodyVelocity class
     * Unit: Meters per second
     * Frame: ROS standard frame
     * Body frame
     */
    class BodyVelocity: public Vector3
    {
        public:
            BodyVelocity(const Vector3& v)
            {
                x = v.x;
                y = v.y;
                z = v.z;
            }
            BodyVelocity()
            {
                x = 0;
                y = 0;
                z = 0;
            }
    };

    /*
     * EulerRate class
     * Unit: Radians per second
     * Frame: ROS standard frame
     * World frame
     */
    class EulerRate: public Vector3
    {
        public:
            EulerRate(const Vector3& v)
            {
                x = v.x;
                y = v.y;
                z = v.z;
            }
            EulerRate()
            {
                x = 0;
                y = 0;
                z = 0;
            }
    };

    /*
     * BodyAngularVelocity class
     * Unit: Radians per second
     * Frame: ROS standard frame
     * Body frame
     */
    class BodyAngularVelocity: public Vector3
    {
        public:
            BodyAngularVelocity(const Vector3& v)
            {
                x = v.x;
                y = v.y;
                z = v.z;
            }
            BodyAngularVelocity()
            {
                x = 0;
                y = 0;
                z = 0;
            }
    };
}
