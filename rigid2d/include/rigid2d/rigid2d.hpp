#ifndef RIGID2D_INCLUDE_GUARD_HPP
#define RIGID2D_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for two-dimensional rigid body transformations.

#include <cmath>
#include <iosfwd> // contains forward definitions for iostream objects



namespace rigid2d
{
    /// \brief PI.  Not in C++ standard until C++20.
    constexpr double PI=3.14159265358979323846;

    /// \brief approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 - a number to compare
    /// \param d2 - a second number to compare
    /// \param epsilon - absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
    /// Note: the fabs function in <cmath> (c++ equivalent of math.h) will
    /// be useful here
    constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12)
    {
      return std::fabs(d1 - d2) < epsilon ? true : false;
    }

    /// \brief convert degrees to radians
    /// \param deg - angle in degrees
    /// \returns radians
    /// NOTE: implement this in the header file
    /// constexpr means that the function can be computed at compile time
    /// if given a compile-time constant as input
    constexpr double deg2rad(double deg)
    {
      return deg*(PI / 180.0);
    }

    /// \brief convert radians to degrees
    /// \param rad - angle in radians
    /// \returns the angle in degrees
    constexpr double rad2deg(double rad)
    {
      return rad*(180.0 / PI);
    }


    /// \brief wraps angle between -pi and pi
    /// \param rad - angle in radians
    /// \returns the wrapped angle in radians
    constexpr double normalize_angle_PI(double rad)
    {
      // floating point remainder essentially this is fmod
      const auto q  = std::floor((rad + PI) / (2.0*PI));
      rad = (rad + PI) - q * 2.0*PI;

      if (rad < 0)
      {
        rad += 2.0*PI;
      }

      return (rad - PI);
    }


    /// \brief wraps angle between 0 and 2pi or 0 to -2pi
    /// \param rad - angle in radians
    /// \returns the wrapped angle in radians
    constexpr double normalize_angle_2PI(double rad)
    {
      // if (rad >= 0)
      // {
      //   // floating point remainder essentially this is fmod
      //   double q  = std::floor(rad / (2*PI));
      //   rad = (rad) - q * 2*PI;
      //
      //   if (rad < 0)
      //     rad += 2*PI;
      //
      //   return rad;
      // }
      //
      // // angle is negative
      // // floating point remainder essentially this is fmod
      // double q  = std::floor(rad / (2*PI));
      // rad = (rad) - q * 2*PI;
      //
      // if (rad > 0)
      //   rad -= 2*PI;
      //
      // return rad;

      // floating point remainder essentially this is fmod
      const auto q  = std::floor(rad / (2.0*PI));
      rad = (rad) - q * 2.0*PI;

      if (rad < 0)
      {
        rad += 2.0*PI;
      }

      return rad;
    }


    /// static_assertions test compile time assumptions.
    /// You should write at least one more test for each function
    /// You should also purposely (and temporarily) make one of these tests fail
    /// just to see what happens
    static_assert(almost_equal(0, 0), "is_zero failed");
    static_assert(almost_equal(5.0, 5.0 + 1e-13), "is_zero failed");

    static_assert(almost_equal(0.001, 0.005, 1.0e-2), "is_zero failed");
    static_assert(almost_equal(0.001, 0.005, 1.0e-1), "is_zero failed");

    static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");
    static_assert(almost_equal(deg2rad(45.0), PI/4), "deg2rad failed");

    static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg) failed");
    static_assert(almost_equal(rad2deg(PI/6), 30.0), "rad2deg) failed");

    static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");
    static_assert(almost_equal(deg2rad(rad2deg(4.5)), 4.5), "deg2rad failed");


    static_assert(almost_equal(normalize_angle_PI(3.0/2.0*PI), -PI/2.0), "normalize_angle_PI failed");
    static_assert(almost_equal(normalize_angle_PI(7.0/6.0*PI), -5.0/6.0*PI), "normalize_angle_PI failed");
    static_assert(almost_equal(normalize_angle_PI(8.0/3.0*PI), 2.0/3.0*PI), "normalize_angle_PI failed");
    static_assert(almost_equal(normalize_angle_PI(deg2rad(350)), normalize_angle_PI(deg2rad(-10))), "normalize_angle_PI failed");

    static_assert(almost_equal(normalize_angle_2PI(2*PI+PI/6), PI/6), "normalize_angle_2PI failed");
    // static_assert(almost_equal(normalize_angle_2PI(-2*PI-PI/4), -PI/4), "normalize_angle_2PI failed");
    // static_assert(almost_equal(normalize_angle_2PI(-PI/4), -PI/4), "normalize_angle_2PI failed");
    static_assert(almost_equal(normalize_angle_2PI(PI/6), PI/6), "normalize_angle_2PI failed");


    static_assert(almost_equal(normalize_angle_2PI(0.5), 0.5), "normalize_angle_2PI failed");



    /// \brief A 2-Dimensional Vector
    struct Vector2D
    {
        double x;
        double y;

        /// \brief default constructor
        Vector2D() : x(0.0), y(0.0) {};


        /// \brief set elements of vector
        /// \param vec_x - x component
        /// \param vec_y - y component
        Vector2D(double vec_x, double vec_y) : x(vec_x), y(vec_y) {};


        /// \brief add vector components
        /// \param v - components to add
        /// \returns a reference to the newly transformed vector
        Vector2D & operator+=(const Vector2D &v)
        {
          x += v.x;
          y += v.y;
          return *this;
        }


        /// \brief subtract vector components
        /// \param v - components to subtract
        /// \returns a reference to the newly transformed vector
        Vector2D & operator-=(const Vector2D &v)
        {
          x -= v.x;
          y -= v.y;
          return *this;
        }


        /// \brief scalar multiplicaton of vector
        /// \param scalar - to multiply vector by
        /// \returns a reference to the newly transformed vector
        Vector2D & operator*=(const double scalar)
        {
          x *= scalar;
          y *= scalar;
          return *this;
        }
    };


    /// \brief A 2-Dimensional twist
    struct Twist2D
    {
        double w = 0.0;       // rotation about z-axis
        double vx = 0.0;      // linear x velocity
        double vy = 0.0;      // linear y velocity
    };


    /// \brief A 2-Dimensional normal vector
    struct NormalVec2D
    {
      double nx = 0.0;
      double ny = 0.0;
    };


    /// \brief A 2-Dimensional transform
    struct TransformData2D
    {
      double theta = 0.0;
      double x = 0.0;
      double y = 0.0;
    };

    /// \brief A 2-Dimensional screw
    struct Screw2D
    {
      double w = 0.0;
      double vx = 0.0;
      double vy = 0.0;
    };


    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// os - stream to output to
    /// v - the vector to print
    std::ostream & operator<<(std::ostream & os, const Vector2D & v);

    /// \brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as two numbers
    ///   separated by a newline or a space, or entered as [xcomponent ycomponent]
    /// is - stream from which to read
    /// v [out] - output vector
    /// Hint: The following may be useful:
    /// https://en.cppreference.com/w/cpp/io/basic_istream/peek
    /// https://en.cppreference.com/w/cpp/io/basic_istream/get
    std::istream & operator>>(std::istream & is, Vector2D & v);



    /// \brief output a 2 dimensional twisy as [angular velxcomponent velycomponent]
    /// os - stream to output to
    /// twist - the twist to print
    std::ostream & operator<<(std::ostream & os, const Twist2D & twist);


    /// \brief input a 2 dimensional twist
    ///   You should be able to read vectors entered as two numbers
    ///   separated by a newline or a space, or entered as [angular velxcomponent velycomponent]
    /// is - stream from which to read
    /// twist [out] - output twist
    std::istream & operator>>(std::istream & is, Twist2D & twist);


    /// \brief normalize a Vector2D
    /// \param v - the vector to normalize
    /// \return a normalize vector in the same coordinate system
    NormalVec2D normalize(const Vector2D & v);


    /// \brief add vector components
    /// \ param v1 - first vector to add components
    /// \ param v2 - first vector to add components
    /// \ return composition of two vectors
    Vector2D operator+(Vector2D v1, const Vector2D & v2);


    /// \brief subtract vector components
    /// \ param v1 - first vector to subtract components
    /// \ param v2 - first vector to subtract components
    /// \ return composition of two vectors
    Vector2D operator-(Vector2D v1, const Vector2D & v2);


    /// \brief scalar multiplicaton of vector
    /// \param v - vector to scale
    /// \param scalar to multiply vector by
    /// \ return scaled vector
    Vector2D operator*(Vector2D v, const double scalar);


    /// \brief scalar multiplicaton of vector
    /// \param scalar to multiply vector by
    /// \param v - vector to scale
    /// \ return scaled vector
    Vector2D operator*(const double scalar, Vector2D v);


    /// \brief length of vector
    /// \param v - find length of
    /// \return length
    double length(const Vector2D &v);


    /// \brief distance between two vectors
    /// \param v1 - first vector
    /// \param v2 - second vector
    /// \return distance between two vectors
    double distance(const Vector2D &v1, const Vector2D &v2);


    /// \brief angle between two vectors
    /// \param v1 - first vector
    /// \param v2 - second vector
    /// \return angle between two vectors in radians
    double angle(const Vector2D &v1, const Vector2D &v2);




    /// \brief a rigid body transformation in 2 dimensions
    class Transform2D
    {
    public:
        /// \brief Create an identity transformation
        Transform2D();

        /// \brief create a transformation that is a pure translation
        /// \param trans - the vector by which to translate
        explicit Transform2D(const Vector2D & trans);

        /// \brief create a pure rotation
        /// \param radians - angle of the rotation, in radians
        explicit Transform2D(double radians);

        /// \brief Create a transformation with a translational and rotational
        /// component
        /// \param trans - the translation
        /// \param rot - the rotation, in radians
        Transform2D(const Vector2D & trans, double radians);

        /// \brief apply a transformation to a Vector2D
        /// \param v - the vector to transform
        /// \return a vector in the new coordinate system
        Vector2D operator()(Vector2D v) const;

        /// \brief invert the transformation
        /// \return the inverse transformation.
        Transform2D inv() const;

        /// \brief apply adjunct to determine twist in new frame
        /// \param twist - the twist to transform
        /// \return the twist in the new coordinate system
        Twist2D operator()(Twist2D twist) const;

        /// \brief compose this transform with another and store the result
        /// in this object
        /// \param rhs - the first transform to apply
        /// \returns a reference to the newly transformed operator
        Transform2D & operator*=(const Transform2D & rhs);

        /// \brief \see operator<<(...) (declared outside this class)
        /// for a description
        friend std::ostream & operator<<(std::ostream & os, const Transform2D & tf);


        /// \brief composes displacement of transform
        /// \return displacement data of the transform
        TransformData2D displacement() const;

        /// \brief integrates a twist
        /// \return transformation correspond to a twist for one time step
        Transform2D integrateTwist(const Twist2D &twist) const;


    private:
        /// directly initialize, useful for forming the inverse
        Transform2D(double theta, double ctheta, double stheta, double x, double y);
        double theta, ctheta, stheta, x, y; // angle, sin, cos, x, and y
    };


    /// \brief should print a human readable version of the transform:
    /// An example output:
    /// dtheta (degrees): 90 dx: 3 dy: 5
    /// \param os - an output stream
    /// \param tf - the transform to print
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

    /// \brief Read a transformation from stdin
    /// Should be able to read input either as output by operator<< or
    /// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
    std::istream & operator>>(std::istream & is, Transform2D & tf);

    /// \brief multiply two transforms together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    /// HINT: This function can be implemented in terms of *=
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs);
}

#endif
