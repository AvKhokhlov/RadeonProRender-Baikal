/**********************************************************************
Copyright (c) 2019 Advanced Micro Devices, Inc. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
********************************************************************/

#include "math/quaternion.h"
#include "interpolation.h"
#include "utils.h"

using namespace RadeonRays;


float Dot(quaternion const& q1, quaternion const& q2)
{
    return q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
}


float3 GetVector(quaternion const& q)
{
    return float3(q.x, q.y, q.z);
}


std::vector<float3> slerp(quaternion const& begin, quaternion const& end, unsigned points_num)
{
    float cos_thera = Dot(begin, end) / (begin.norm() * end.norm() + 1E-6f);
    float thera = std::acosf(cos_thera);
    float sin_theta = std::sinf(thera);

    if (std::abs(sin_theta) < 1E-6f)
    {
        sin_theta = 1E-6f;
    }

    std::vector<float3> samples = { GetVector(begin) };
    float t = 0;
    float step = 1.f / static_cast<float>(points_num);

    while (t + step < 1)
    {
        auto qt = (std::sinf(1 - t) * thera / sin_theta) * begin + (std::sin(t * thera) / sin_theta) * end;
        samples.push_back(GetVector(qt));
        t += step;
    }

    samples.push_back(GetVector(end));
    return samples;
}


std::vector<float3> lerp(float3 const& begin, float3 const& end, unsigned points_num)
{
    std::vector<float3> samples = { begin };

    if (points_num == 1)
    {
        return samples;
    }

    auto step = (1.f / (static_cast<float>(points_num)- 1.f)) * (end - begin);

    for (auto i = 0u; i < points_num - 1; i++)
    {
        samples.push_back(begin + step * static_cast<float>(i));
    }

    samples.push_back(end);
    return samples;
}

float dist(float3 const& a, float3 const& b)
{
    return std::sqrt(std::powf(a.x - b.x, 2) +
                     std::powf(a.y - b.y, 2) +
                     std::powf(a.z - b.z, 2));
}

std::vector<CameraObject> CameraInterpolation(CameraObject* cameras, unsigned cameras_num, float step)
{
    if (cameras == nullptr)
    {
        THROW_EX("'cameras' collection is nullptr");
    }

    if (cameras_num == 0)
    {
        THROW_EX("'cameras_num' can't be zero");
    }

    if (cameras_num == 1)
    {
        std::vector<CameraObject> ret;
        ret.push_back(std::move(cameras[0]));
        return ret;
    }

    std::vector<float3> position_samples;
    std::vector<float3> directiomn_samples, up_samples;

    for (auto i = 0u; i < cameras_num; i++)
    {
        float3 begin_pos, begin_direction, begin_up;
        cameras[i].GetLookAt(begin_pos, begin_direction, begin_up);

        float3 end_pos, end_direction, end_up;
        cameras[i].GetLookAt(end_pos, end_direction, end_up);

        auto ith_camera_postions = lerp(begin_pos,
                                        end_pos,
                                        static_cast<unsigned>(std::ceilf(dist(begin_pos, end_pos) / step)));

        auto ith_camera_directions = slerp(quaternion(begin_direction.x, begin_direction.y, begin_direction.z, 0.0),
                                           quaternion(end_direction.x, end_direction.y, end_direction.z, 0.0),
                                           static_cast<unsigned>(ith_camera_postions.size()));

        auto ith_camera_ups = slerp(quaternion(begin_up.x, begin_up.y, begin_up.z, 0.0),
                                    quaternion(end_up.x, end_up.y, end_up.z, 0.0),
                                    static_cast<unsigned>(ith_camera_postions.size()));

        position_samples.insert(position_samples.end(),
                                ith_camera_postions.begin(),
                                ith_camera_postions.end());

        directiomn_samples.insert(directiomn_samples.end(),
                                  ith_camera_directions.begin(),
                                  ith_camera_directions.end());

        up_samples.insert(up_samples.end(),
                          ith_camera_ups.begin(),
                          ith_camera_ups.end());
    }

    std::vector<CameraObject> camera_samples;

    auto CreateCamera = [cameras]() 
    {
        CameraObject camera_object;
        camera_object.SetSensorSize(cameras[0].GetSensorSize());
        camera_object.SetAperture(cameras[0].GetAperture());
        camera_object.SetFocalLength(cameras[0].GetFocalLength());
        camera_object.SetFocusDistance(cameras[0].GetFocusDistance());
        camera_object.SetMode(cameras[0].GetMode());
        return camera_object;
    };

    for (auto i = 0u; i < position_samples.size(); i++)
    {
        camera_samples.push_back(CreateCamera());
        camera_samples.back().LookAt(position_samples[i],
                                     directiomn_samples[i],
                                     up_samples[i]);
    }

    // append last camera point
    camera_samples.push_back(CreateCamera());
    camera_samples.back().LookAt(position_samples.back(),
                                 directiomn_samples.back(),
                                 up_samples.back());

    return camera_samples;
}