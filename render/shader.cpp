#include "shader.h"
#include "../utils/math.hpp"

#ifdef _WIN32
#undef min
#undef max
#endif

using Eigen::Matrix3f;
using Eigen::Matrix4f;
using Eigen::Vector3f;
using Eigen::Vector4f;

// vertex shader & fragement shader can visit
// all the static variables below from Uniforms structure
Eigen::Matrix4f Uniforms::MVP;
Eigen::Matrix4f Uniforms::inv_trans_M;
int Uniforms::width;
int Uniforms::height;


// vertex shader
VertexShaderPayload vertex_shader(const VertexShaderPayload& payload)
{

    // Vertex position transformation
    Vector4f cube_position;
    cube_position = Uniforms::MVP * payload.position;
    
    //  Viewport transformation
    Matrix4f Mviewport = Matrix4f::Identity();
    Mviewport(0, 0)    = 0.5 * Uniforms::width;
    Mviewport(0, 3)    = 0.5 * Uniforms::width;
    Mviewport(1, 1)    = 0.5 * Uniforms::height;
    Mviewport(1, 3)    = 0.5 * Uniforms::height;

    Vector4f Viewportposition = Mviewport * cube_position;
    // printf("x%.2f,y%.2f\n", Viewportposition.x(),Viewportposition.y());
    //  Vertex normal transformation
    Vector4f normal_;
    normal_(0)           = payload.normal(0);
    normal_(1)           = payload.normal(1);
    normal_(2)           = payload.normal(2);
    normal_(3)           = 0.0;
    Vector4f worldnormal = Uniforms::inv_trans_M * normal_;

    // printf("wnormal %.2f,%.2f,%.2f\n", worldnormal(0), worldnormal(1), worldnormal(2));
    VertexShaderPayload output_payload;
    output_payload.position = Viewportposition;
    output_payload.normal   = {worldnormal(0), worldnormal(1), worldnormal(2)};
    // printf("w%d,h%d", Uniforms::width, Uniforms::height);
    return output_payload;
}

float max(float x, float y)
{
    if (x >= y) {
        return x;
    } else {
        return y;
    }
}


Vector3f phong_fragment_shader(const FragmentShaderPayload& payload, GL::Material material,
                               const std::list<Light>& lights, Camera camera)
{
    Vector3f result = {0, 0, 0};
    // ka,kd,ks can be got from material.ambient,material.diffuse,material.specular
    Vector3f ka = material.ambient;
    Vector3f kd = material.diffuse;
    Vector3f ks = material.specular;
    float p     = material.shininess;
    // set ambient light intensity
    float Ia    = 0.1;
    float I     = 0.0;
    float I_att = 0.0;
    Vector3f lightpos;
    Vector3f lightdir;
    Vector3f viewdir;
    Vector3f camerapos = {0, 0, 0};
    Vector3f halfvec;
    float lightdis = 0.0;
    float viewdis  = 0.0;
    Vector3f La    = {0, 0, 0};
    Vector3f Ld    = {0, 0, 0};
    Vector3f Ls    = {0, 0, 0};
    Vector3f L     = {0, 0, 0};
    Vector3f t     = {1,1,1};
    t              = t * 2.0f;
    //printf("t%.2f,%.2f,%.2f", t(0), t(1), t(2));
   


    for (const Light& light : lights) {
        I                  = light.intensity;
        
        lightpos = camera.view().block(0, 0, 3, 3) * light.position;

        camerapos = camera.view().block(0, 0, 3, 3) * camera.position;

        lightdis = (lightpos - payload.world_pos).norm();
        viewdis  = (camerapos - payload.world_pos).norm();

        Vector3f n = payload.world_normal;
        n          = n / n.norm();
        // Light Direction
        lightdir = (lightpos - payload.world_pos) / lightdis;
        // View Direction
        viewdir = (camerapos - payload.world_pos) / viewdis;
        // Half Vector
        halfvec = lightdir + viewdir;
        halfvec = halfvec / halfvec.norm();
        // Light Attenuation
        I_att = I / (lightdis * lightdis);
        // Ambient
        La = ka * Ia;
        // Diffuse
        Ld = kd * I_att * max(n.dot(lightdir), 0.0);
        // Specular
        Ls = ks * I_att * pow(max(0.0f, n.dot(halfvec)), p);
        L  = L + La + Ld + Ls;
        //printf("pow %.2f\n", std::pow(std::max(0.0f, n.dot(halfvec)), material.shininess));

    }
    result = L;
    
    //printf("L;%.2f,%.2f,%.2f\n", result(0), result(1), result(2));
    
    for (int i = 0; i <=2; i++) {
        if (result[i] >= 1)
            result[i] = 1;
    }
   
    // set rendering result max threshold to 255

    return result * 255.f;
}








//分割线--------------------------------------------------------------------------------------------------------------


