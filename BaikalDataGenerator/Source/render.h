
/**********************************************************************
 Copyright (c) 2017 Advanced Micro Devices, Inc. All rights reserved.
 
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

#pragma once

#include "CLW.h"
#include "Renderers/renderer.h"
#include "RenderFactory/clw_render_factory.h"
#include "Output/output.h"
#include "SceneGraph/camera.h"
#include "scene_io.h"

#include "OpenImageIO/imageio.h"

#include <vector>
#include <memory>
#include <algorithm>
#include <iostream>

struct CameraInfo
{
    RadeonRays::float3 camera_pos;
    RadeonRays::float3 camera_at;
    RadeonRays::float3 camera_up;
    RadeonRays::float2 camera_sensor_size;
    RadeonRays::float2 camera_zcap;
    float camera_aperture;
    float camera_focus_distance;
    float camera_focal_length;
    Baikal::CameraType camera_type;
};


class Render
{
public:
    using Ptr = std::shared_ptr<Render>;

    static Ptr Create(const std::string &file_name,
                      const std::string &path,
                      std::uint32_t output_width = 256,
                      std::uint32_t output_height = 256);

    void LoadCameraXml(const std::string &full_path);
    void LoadLightXml(const std::string &full_path);

    void GenerateDataset(const std::string &full_path);

protected:
    Render(const std::string &file_name,
           const std::string &path,
           std::uint32_t output_width,
           std::uint32_t output_height);

private:
    std::unique_ptr<Baikal::Renderer> m_renderer;
    std::unique_ptr<Baikal::SceneController<Baikal::ClwScene>> m_controller;
    std::unique_ptr<Baikal::RenderFactory<Baikal::ClwScene>> m_factory;
    std::unique_ptr<Baikal::Output> m_output;
    Baikal::Scene1::Ptr m_scene;
    Baikal::PerspectiveCamera::Ptr m_camera;

    std::vector<CameraInfo> m_camera_states;
    Baikal::Light::Ptr m_light;
};