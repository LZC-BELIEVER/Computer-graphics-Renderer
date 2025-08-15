#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <vector>
#include <optional>
#include <iostream>
#include <chrono>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "render_engine.h"
#include "../scene/light.h"
#include "../utils/math.hpp"
#include "../utils/ray.h"
#include "../utils/logger.h"

using std::chrono::steady_clock;
using duration   = std::chrono::duration<float>;
using time_point = std::chrono::time_point<steady_clock, duration>;
using Eigen::Vector3f;
using Eigen::Matrix4f;


// 最大的反射次数
constexpr int MAX_DEPTH        = 5;
constexpr float INFINITY_FLOAT = std::numeric_limits<float>::max();
// 考虑物体与光线相交点的偏移值
constexpr float EPSILON = 0.00001f;

// 当前物体的材质类型，根据不同材质类型光线会有不同的反射情况
enum class MaterialType
{
    DIFFUSE_AND_GLOSSY,
    REFLECTION
};

// 显示渲染的进度条
void UpdateProgress(float progress)
{
    int barwidth = 70;
    std::cout << "[";
    int pos = static_cast<int>(barwidth * progress);
    for (int i = 0; i < barwidth; i++) {
        if (i < pos)
            std::cout << "=";
        else if (i == pos)
            std::cout << ">";
        else
            std::cout << " ";
    }
    std::cout << "]" << int(progress * 100.0) << " %\r";
    std::cout.flush();
}

WhittedRenderer::WhittedRenderer(RenderEngine& engine)
    : width(engine.width), height(engine.height), n_threads(engine.n_threads), use_bvh(false),
      rendering_res(engine.rendering_res)
{
    logger = get_logger("Whitted Renderer");
}

// whitted-style渲染的实现
void WhittedRenderer::render(Scene& scene)
{
    time_point begin_time = steady_clock::now();
    width                 = std::floor(width);
    height                = std::floor(height);

    // initialize frame buffer
    std::vector<Vector3f> framebuffer(static_cast<size_t>(width * height));
    for (auto& v : framebuffer) {
        v = Vector3f(0.0f, 0.0f, 0.0f);
    }

    int idx = 0;
    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {
            // generate ray
            Ray ray = generate_ray(static_cast<int>(width), static_cast<int>(height), i, j,
                                   scene.camera, 1.0f);
            // cast ray
            framebuffer[idx++] = cast_ray(ray, scene, 0);
        }
        UpdateProgress(j / height);
    }
    // save result to whitted_res.ppm
    FILE* fp = fopen("whitted_res.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", (int)width, (int)height);
    static unsigned char color_res[3];
    rendering_res.clear();
    for (long unsigned int i = 0; i < framebuffer.size(); i++) {
        color_res[0] = static_cast<unsigned char>(255 * clamp(0.f, 1.f, framebuffer[i][0]));
        color_res[1] = static_cast<unsigned char>(255 * clamp(0.f, 1.f, framebuffer[i][1]));
        color_res[2] = static_cast<unsigned char>(255 * clamp(0.f, 1.f, framebuffer[i][2]));
        fwrite(color_res, 1, 3, fp);
        rendering_res.push_back(color_res[0]);
        rendering_res.push_back(color_res[1]);
        rendering_res.push_back(color_res[2]);
    }
    time_point end_time         = steady_clock::now();
    duration rendering_duration = end_time - begin_time;
    logger->info("rendering takes {:.6f} seconds", rendering_duration.count());
}


// 菲涅尔定理计算反射光线
float WhittedRenderer::fresnel(const Vector3f& I, const Vector3f& N, const float& ior)
{
    float dot = I.dot(N);
    float I_length = I.norm();
    float N_length = N.norm();

    float cos = std::fabs(dot / I_length * N_length);

    float temp1 = 1 - ior;
    float temp2 = 1 + ior;
    float R0  = std::pow(temp1 / temp2, 2);


    return  (1 - R0) * std::pow(1 - cos, 5) + R0;
}

// 如果相交返回Intersection结构体，如果不相交则返回false
std::optional<std::tuple<Intersection, GL::Material>> WhittedRenderer::trace(const Ray& ray,
                                                                             const Scene& scene)
{
    std::optional<Intersection> payload;
    Eigen::Matrix4f M;
    GL::Material material;
    float t_box = INFINITY;
    // if use bvh(exercise 2.4): use object->bvh->intersect
    // else(exercise 2.3): use naive_intersect()
    // pay attention to the range of payload->t
    for (const auto& group : scene.groups) {
        for (const auto& object : group->objects) {

            GL::Material obj_material;
            Matrix4f model_mat = object->model();
            //Matrix4f model_matrix = model_mat*view();
            Matrix4f model_matrix = model_mat;

            std::optional<Intersection> newintersection =naive_intersect(ray, object->mesh, model_matrix);

            if (newintersection.has_value()) {
                if (newintersection.value().t < t_box) {
                    payload = newintersection;
                    t_box   = payload->t;
                    material = object->mesh.material;
                }
            }

            
        }
    }

    if (!payload.has_value()) {
        return std::nullopt;
    }
    return std::make_tuple(payload.value(), material);
}

// Whitted-style的光线传播算法实现
Vector3f WhittedRenderer::cast_ray(const Ray& ray, const Scene& scene, int depth)
{
    if (depth > MAX_DEPTH) {
        return Vector3f(0.0f, 0.0f, 0.0f);
    }
    // initialize hit color
    Vector3f hitcolor = RenderEngine::background_color;
    // get the result of trace()
    auto result = trace(ray, scene);

    // if result.has_value():
    // 1.judge the material_type
    // 2.if REFLECTION:
    //(1)use fresnel() to get kr
    //(2)hitcolor = cast_ray*kr
    // if DIFFUSE_AND_GLOSSY:
    //(1)compute shadow result using trace()
    //(2)hitcolor = diffuse*kd + specular*ks

    if (result.has_value()) {
        //auto [interaction, material] = result.value();

        GL::Material material = std::get<1>(result.value());
        Intersection interaction    = std::get<0>(result.value());

        // 交点
        Vector3f hit_point = ray.origin + interaction.t * ray.direction;
       
        // 判断材质类型
        if (material.shininess > 1000) {

            // 反射
            float kr = fresnel(ray.direction, interaction.normal, 0.5f);
            Ray reflect_ray;
            reflect_ray.direction = ray.direction - 2.0f * ray.direction.dot(interaction.normal) * interaction.normal;
            reflect_ray.direction = reflect_ray.direction / reflect_ray.direction.norm();

            float decide = reflect_ray.direction.dot(interaction.normal);
            if (decide > 0)
                decide = EPSILON;
            else if (decide == 0)
                decide = 0.f;
            else
                decide = -1 * EPSILON;

            reflect_ray = {hit_point + interaction.normal * decide, reflect_ray.direction};

            hitcolor += kr * cast_ray(reflect_ray, scene, depth + 1);
        } else {
            // DIFFUSE_AND_GLOSSY
            // 遍历光源
            hitcolor = {0, 0, 0};

            for (const auto& light : scene.lights) {
                Vector3f back     = light.position - hit_point;
                back          = back / back.norm();
       
                Ray shadow_ray{hit_point, back};
                auto shadow_result = trace(shadow_ray, scene);

                //计算的相交结果的 t 值不仅需要和光线递归时一样 >0，还应当小于和光源之间的距离。
                if (!shadow_result.has_value() ||(shadow_result.has_value()&& std::get<0>(shadow_result.value()).t > back.norm())) {
                    Vector3f ka = material.ambient;
                    Vector3f kd = material.diffuse;
                    Vector3f ks = material.specular;
                    float p     = material.shininess;
                    
                    Vector3f La = {0, 0, 0};
                    Vector3f Ld = {0, 0, 0};
                    Vector3f Ls = {0, 0, 0};
                    Vector3f L = {0, 0, 0};

                    Vector3f lightdir = back;
                    Vector3f viewdir = (scene.camera.position - hit_point);
                    viewdir  = viewdir / viewdir.norm();

                    // Half Vector
                    Vector3f halfvec = lightdir + viewdir;
                    halfvec = halfvec / halfvec.norm();
                    // Light Attenuation
                    float I = light.intensity;
                    float I_att   = I;
                    //    / (lightdis * lightdis);
                    // Ambient
                    float Ia = 0.1;
                    La = ka * Ia;
                    // Diffuse
                    Ld = kd * I_att * std::max(interaction.normal.dot(back), 0.0f);
                    // Specular
                    Ls = ks * I_att * pow(std::max(0.0f, interaction.normal.dot(halfvec)), p);
                    L  = L + La + Ld + Ls;
                    
                    

                    hitcolor += L;
                }
            }
        }
    }

    return hitcolor;
}


//----------------------------------------------------=-------------------------------------------

