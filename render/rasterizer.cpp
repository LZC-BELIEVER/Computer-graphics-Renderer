#include <algorithm>
#include <cmath>
#include <mutex>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <spdlog/spdlog.h>

#include "rasterizer.h"
#include "triangle.h"
#include "render_engine.h"
#include "../utils/math.hpp"

using Eigen::Matrix4f;
using Eigen::Vector2i;
using Eigen::Vector3f;
using Eigen::Vector4f;
using std::fill;
using std::tuple;


// 给定坐标(x,y)以及三角形的三个顶点坐标，判断(x,y)是否在三角形的内部
bool Rasterizer::inside_triangle(int x, int y, const Vector4f* vertices)
{
    Vector3f p(float(x) + 0.5, float(y) + 0.5, 1.0f);
    Vector3f v[3];
    for (int i = 0; i < 3; i++) v[i] = {vertices[i].x(), vertices[i].y(), 1.0f};
    Vector3f temp1 = p - v[0];
    Vector3f temp2 = p - v[1];
    Vector3f temp3 = p - v[2];

    Vector3f line1 = {v[1].x() - v[0].x(), v[1].y() - v[0].y(), 0.0};
    Vector3f line2 = {v[2].x() - v[1].x(), v[2].y() - v[1].y(), 0.0};
    Vector3f line3 = {v[0].x() - v[2].x(), v[0].y() - v[2].y(), 0.0};

    
    Vector3f z1 = line1.cross(temp1);
    Vector3f z2 = line2.cross(temp2);
    Vector3f z3 = line3.cross(temp3);
    //z1 = line1.cross(temp1);
    //z2 = line2.cross(temp2);
    //z3 = line3.cross(temp3);
    
    //float z1 = line1(0) * temp1(1) - line1(1) * temp1(0);
    //float z2 = line2(0) * temp2(1) - line2(1) * temp2(0);
    //float z3 = line3(0) * temp3(1) - line3(1) * temp3(0);

    
    if ((z1.z() >= 0 && z2.z() >= 0 && z3.z() >= 0) || (z1.z() <= 0 && z2.z() <= 0 && z3.z() <= 0))
        return true;

    else
        return false;
}

// 给定坐标(x,y)以及三角形的三个顶点坐标，计算(x,y)对应的重心坐标[alpha, beta, gamma]
tuple<float, float, float> Rasterizer::compute_barycentric_2d(float x, float y, const Vector4f* v)
{
    float c1 = 0.f, c2 = 0.f, c3 = 0.f;
    float xa = v[0].x();
    float ya = v[0].y();
    float xb = v[1].x();
    float yb = v[1].y();
    float xc = v[2].x();
    float yc = v[2].y();

    c1 = (-(x - xb) * (yc - yb) + (y - yb) * (xc - xb)) /
         (-(xa - xb) * (yc - yb) + (ya - yb) * (xc - xb));
    c2 = (-(x - xc) * (ya - yc) + (y - yc) * (xa - xc)) /
         (-(xb - xc) * (ya - yc) + (yb - yc) * (xa - xc));
    c3 = 1 - c1 - c2;
    return {c1, c2, c3};
}

// 对当前渲染物体的所有三角形面片进行遍历，进行几何变换以及光栅化
void Rasterizer::draw(const std::vector<Triangle>& TriangleList, const GL::Material& material,
                      const std::list<Light>& lights, const Camera& camera)
{
    // iterate over all triangles in TriangleList
    for (const auto& t : TriangleList) {
        Triangle newtriangle = t;
        VertexShaderPayload tria_point[3];
        VertexShaderPayload processed_tria_point[3];
        Triangle vertexed_tria;
        // Use vetex_shader to transform vertex attributes(position & normals) to
        // view port and set a new triangle
        for (int k = 0; k <= 2; k++) {
            tria_point[k].position = newtriangle.vertex[k];
            tria_point[k].normal   = newtriangle.normal[k];
            // printf("newtria_nor %d %.2f,%.2f,%.2f\n",k, tria_point[k].normal.x(),
            // tria_point[k].normal.y(),tria_point[k].normal.z());
            processed_tria_point[k] = vertex_shader(tria_point[k]);
            processed_tria_point[k].position.x() = processed_tria_point[k].position.x() / processed_tria_point[k].position.w();
            processed_tria_point[k].position.y() = processed_tria_point[k].position.y() / processed_tria_point[k].position.w();
            processed_tria_point[k].position.z() = processed_tria_point[k].position.z() / processed_tria_point[k].position.w();
            vertexed_tria.vertex[k] = processed_tria_point[k].position;
            // printf("x%.2f,y%.2f\n", processed_tria_point[k].position.x(),
            // processed_tria_point[k].position.y());//结果一千多
            vertexed_tria.normal[k] = processed_tria_point[k].normal;
            // printf("pro_tria_nor %d %.2f,%.2f,%.2f\n",k, processed_tria_point[k].normal.x(),
            // processed_tria_point[k].normal.y(),processed_tria_point[k].normal.z());
        }
        // transform vertex position to world space for interpolating
        std::array<Vector4f, 3> worldspace_pos_;
        for (int i = 0; i <= 2; i++) {
            worldspace_pos_[i] =
                model * newtriangle.vertex[i];
        }
        std::array<Vector3f, 3> worldspace_pos;
        worldspace_pos[0] = {worldspace_pos_[0].x(), worldspace_pos_[0].y(),
                             worldspace_pos_[0].z()};
        worldspace_pos[1] = {worldspace_pos_[1].x(), worldspace_pos_[1].y(),
                             worldspace_pos_[1].z()};
        worldspace_pos[2] = {worldspace_pos_[2].x(), worldspace_pos_[2].y(),
                             worldspace_pos_[2].z()};
        // call rasterize_triangle()
        rasterize_triangle(vertexed_tria, worldspace_pos, material, lights, camera);
    }
}

// 对顶点的某一属性插值
Vector3f Rasterizer::interpolate(float alpha, float beta, float gamma, const Eigen::Vector3f& vert1,
                                 const Eigen::Vector3f& vert2, const Eigen::Vector3f& vert3,
                                 const Eigen::Vector3f& weight, const float& Z)
{
    Vector3f interpolated_res;
    for (int i = 0; i < 3; i++) {
        interpolated_res[i] = alpha * vert1[i] / weight[0] + beta * vert2[i] / weight[1] +
                              gamma * vert3[i] / weight[2];
    }
    interpolated_res *= Z;
    return interpolated_res;
}



float get_max(float x, float y, float z)
{
    if (x > y) {
        if (x > z) {
            return x;
        } else {
            return z;
        }
    } else {
        if (y > z) {
            return y;
        } else {
            return z;
        }
    }
}

float get_min(float x, float y, float z)
{
    if (x < y) {
        if (x < z) {
            return x;
        } else {
            return z;
        }
    } else {
        if (y < z) {
            return y;
        } else {
            return z;
        }
    }
}


// 对当前三角形进行光栅化
void Rasterizer::rasterize_triangle(const Triangle& t, const std::array<Vector3f, 3>& world_pos,
                                    GL::Material material, const std::list<Light>& lights,
                                    Camera camera)
{
    // discard all pixels out of the range(including x,y,z)

    bool inside_screen;

    if (t.vertex[0].x() <= Uniforms::width && t.vertex[0].y() <= Uniforms::height &&
        t.vertex[1].x() <= Uniforms::width && t.vertex[1].y() <= Uniforms::height &&
        t.vertex[2].x() <= Uniforms::width && t.vertex[2].y() <= Uniforms::height) {
        inside_screen = 1;
    } else {
        inside_screen = 0;
    }

    

    if (inside_screen) {
        int tria_left   = 0;
        int tria_right  = 0;
        int tria_top    = 0;
        int tria_ground = 0;
        tria_left       = get_min(t.vertex[0].x(), t.vertex[1].x(), t.vertex[2].x()) - 2;
        // printf("Ax %.2f,Bx %.2f,Cx %.2f\n", t.vertex[0].x(), t.vertex[1].x(), t.vertex[2].x());
        tria_right  = get_max(t.vertex[0].x(), t.vertex[1].x(), t.vertex[2].x()) + 2;
        tria_top    = get_max(t.vertex[0].y(), t.vertex[1].y(), t.vertex[2].y()) + 2;
        tria_ground = get_min(t.vertex[0].y(), t.vertex[1].y(), t.vertex[2].y()) - 2;
        // printf("left %d,right %d,top %d,ground%d\n", tria_left, tria_right, tria_top,
        // tria_ground);

        // if current pixel is in current triange:
        for (int x_ = tria_left; x_ <= tria_right; x_++) {
            for (int y_ = tria_ground; y_ <= tria_top; y_++) {
                // printf("x %d,y %d\n", x_, y_);
                if (inside_triangle(x_, y_, t.vertex)) {
                    // 2. interpolate vertex positon & normal(use function:interpolate())
                    float al = std::get<0>(compute_barycentric_2d(x_+0.5, y_+0.5, t.vertex));
                    float be = std::get<1>(compute_barycentric_2d(x_+0.5, y_+0.5, t.vertex));
                    float ga = std::get<2>(compute_barycentric_2d(x_+0.5, y_+0.5, t.vertex));

                    
                    
                    std::array<Vector3f, 3> camera_pos;
                    for (int k1 = 0; k1 <= 2; k1++) {
                        camera_pos[k1] = view.block(0, 0, 3, 3) * world_pos[k1];
                    }

                    Vector3f w = {t.vertex[0].w(), t.vertex[1].w(), t.vertex[2].w()};
                    // printf("w:%.2f,%.2f,%.2f",camera_pos[0].z(), camera_pos[1].z(),
                    // camera_pos[2].z());
                    float z = 1 / (al / w.x() + be / w.y() + ga / w.z());

                    std::array<Vector4f, 3> world_normal_;
                    for (int k2 = 0; k2 <= 2; k2++) {
                        world_normal_[k2] = {t.normal[k2].x(), t.normal[k2].y(), t.normal[k2].z(),
                                             0.0};
                    }
                    Vector4f ntemp0 = view * world_normal_[0];
                    Vector4f ntemp1 = view * world_normal_[1];
                    Vector4f ntemp2 = view * world_normal_[2];

                    std::array<Vector3f, 3> camera_normal;
                    camera_normal[0] = {ntemp0.x(), ntemp0.y(), ntemp0.z()};
                    camera_normal[1] = {ntemp1.x(), ntemp1.y(), ntemp1.z()};
                    camera_normal[2] = {ntemp2.x(), ntemp2.y(), ntemp2.z()};
                   
                    

                    
                    float zt = z;
                    // 1. interpolate depth(use projection correction algorithm)
                    if (zt < depth_buf[get_index(x_, y_)]) {
                        depth_buf[get_index(x_, y_)] = zt;
                        Vector3f camera_pixelnormal;
                        camera_pixelnormal = interpolate(al, be, ga, camera_normal[0],
                                                         camera_normal[1], camera_normal[2], w, zt);

                        Vector3f camera_pixelpos;
                        camera_pixelpos = interpolate(al, be, ga, camera_pos[0], camera_pos[1],
                                                      camera_pos[2], w, zt);
                        //   3. fragment shading(use function:fragment_shader())
                        Vector3f pixelcolor;
                        
                        FragmentShaderPayload cam(camera_pixelpos, camera_pixelnormal);
                        pixelcolor     = phong_fragment_shader(cam, material, lights, camera);
                        Vector2i point = {x_, y_};
                        //   4. set pixel
                        set_pixel(point, pixelcolor);
                        
                        // printf("zbuff:%.2f", depth_buf[get_index(x_, y_)]);
                    }
                }
            }
        }
    }
}

// 初始化整个光栅化渲染器
void Rasterizer::clear(BufferType buff)
{
    if ((buff & BufferType::Color) == BufferType::Color) {
        fill(frame_buf.begin(), frame_buf.end(), RenderEngine::background_color * 255.0f);
    }
    if ((buff & BufferType::Depth) == BufferType::Depth) {
        fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

Rasterizer::Rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

// 给定像素坐标(x,y)，计算frame buffer里对应的index
int Rasterizer::get_index(int x, int y)
{
    return (height - 1 - y) * width + x;
}

// 给定像素点以及fragement shader得到的结果，对frame buffer中对应存储位置进行赋值
void Rasterizer::set_pixel(const Vector2i& point, const Vector3f& res)
{
    int idx        = get_index(point.x(), point.y());
    frame_buf[idx] = res;
}





//-----------------------------------------------------------------------------------------------------------------------------------------------------

