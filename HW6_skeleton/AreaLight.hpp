//
// TODO: [For Bonus] Edit the SamplePoint() function
// 

#pragma once

#include "Light.hpp"
#include "Vector.hpp"
#include "global.hpp"

class AreaLight : public Light
{
public:
    AreaLight(const Vector3f& p, const Vector3f& i)
        : Light(p, i)
    {
        normal = Vector3f(0, -1, 0);
        u = Vector3f(1, 0, 0);
        v = Vector3f(0, 0, 1);
        length = 1;
    }

    Vector3f SamplePoint() const
    {
        // TODO: [For bonus] Use this function to sample random points from the area light.
        // You can assume the position in the base class (Light) is the upper-left corner of
        // the area light source, and generate random floating value numbers using the function
        // *get_random_float()* that gives you random values between 0 and 1.
        // Use the random values to sample a point on the light source, and return it.
        return Vector3f(0.f, 0.f, 0.f);// please comment this line before implementation
    }

    float length;
    Vector3f normal;
    Vector3f u;
    Vector3f v;
};
