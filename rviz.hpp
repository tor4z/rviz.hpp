#ifndef RVIZ_HPP_
#define RVIZ_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <raylib.h>
#include <vector>
#include <mutex>

#define RVIZ_DEF_SINGLETON(classname)                         \
public:                                                       \
    static inline classname* instance()                       \
    {                                                         \
        static classname *instance_ = nullptr;                \
        static std::once_flag flag;                           \
        if (!instance_) {                                     \
            std::call_once(flag, [&](){                       \
                instance_ = new (std::nothrow) classname();   \
            });                                               \
        }                                                     \
        return instance_;                                     \
    }                                                         \
private:                                                      \
    classname(const classname&) = delete;                     \
    classname& operator=(const classname&) = delete;          \
    classname(const classname&&) = delete;                    \
    classname& operator=(const classname&&) = delete;

namespace rviz {

struct PointI
{
    float x;
    float y;
    float z;
    float i;
}; // struct PointI

struct PointRGB
{
    float x;
    float y;
    float z;
    float r;
    float g;
    float b;
}; // struct PointRGB

using PointICloud = std::vector<PointI>;
using PointRGBCloud = std::vector<PointRGB>;


class Viz
{
    RVIZ_DEF_SINGLETON(Viz)
public:
    ~Viz();
    void draw_pointcloud(const PointICloud& pc);
    void draw_pointcloud(const PointRGBCloud& pc);
    void draw_se3();
    void draw_image();
    void render();
    bool closed();
private:
    Viz();

    Camera camera_;
    Model model_;
    Mesh mesh_;
    Vector3 model_center_;
}; // struct Viz

} // namespace rviz

#endif // RVIZ_HPP_


#define RVIZ_IMPLEMENTATION

#ifdef RVIZ_IMPLEMENTATION
#ifndef RVIZ_CPP_
#define RVIZ_CPP_

#ifndef RVIZ_WIN_WIDTH
#   define RVIZ_WIN_WIDTH 800
#endif // RVIZ_WIN_WIDTH

#ifndef RVIZ_WIN_HEIGHT
#   define RVIZ_WIN_HEIGHT 600
#endif // RVIZ_WIN_HEIGHT

#ifndef RVIZ_WIN_NAME
#   define RVIZ_WIN_NAME "RVIZ"
#endif // RVIZ_WIN_NAME

#ifndef RVIZ_TARGET_FPS
#   define RVIZ_TARGET_FPS 20
#endif // RVIZ_TARGET_FPS

namespace rviz {

void heatmap(float intensity, Color& color)
{
    static Color palette[] {
        {29, 72, 119, 255},
        {27, 138, 90, 255},
        {251, 176, 33, 255},
        {246, 136, 56, 255},
        {238, 62, 50, 255}
    };
    static const auto palette_size{sizeof(palette) / sizeof(palette[0])};

    const float palette_index{palette_size * intensity};
    const auto palette_index_floor{static_cast<int>(std::floor(palette_index))};
    const auto palette_index_ceil{static_cast<int>(std::ceil(palette_index))};

    const auto inter{palette_index - static_cast<float>(palette_index_floor)};
    const auto one_minus_inter{1.0f - inter};

    color.a = 255;
    color.r = one_minus_inter * palette[palette_index_floor].r + inter * palette[palette_index_ceil].r;
    color.g = one_minus_inter * palette[palette_index_floor].g + inter * palette[palette_index_ceil].g;
    color.b = one_minus_inter * palette[palette_index_floor].b + inter * palette[palette_index_ceil].b;
}

void set_pointcloud_mesh_buffer(size_t num_point, Mesh& mesh)
{
    const auto vertices_buf_size{num_point * 3};
    const auto color_buf_size{num_point * 4};

    if (mesh.vertices == nullptr || (mesh.vertexCount * 3) < vertices_buf_size) {
        delete [] mesh.vertices;
        mesh.vertices = new float[vertices_buf_size];
    }
    if (mesh.colors == nullptr || (mesh.vertexCount * 4) < color_buf_size) {
        delete [] mesh.colors;
        mesh.colors = new unsigned char[color_buf_size];
    }
    mesh.vertexCount = static_cast<int>(num_point);
    mesh.triangleCount = 1;
}

Viz::Viz()
{
    InitWindow(RVIZ_WIN_WIDTH, RVIZ_WIN_HEIGHT, RVIZ_WIN_NAME);    
    SetTargetFPS(RVIZ_TARGET_FPS);

    mesh_.vertexCount = 0;
    mesh_.vertices = nullptr;
    mesh_.colors = nullptr;

    camera_.position   = {0.0f, 0.0f, 10.0f};
    camera_.target     = {0.0f, 0.0f, 0.0f};
    camera_.up         = {0.0f, 1.0f, 0.0f};
    camera_.fovy       = 45.0f;
    camera_.projection = CAMERA_PERSPECTIVE;

    model_center_.x = 0.0f;
    model_center_.y = 0.0f;
    model_center_.z = 0.0f;
}

Viz::~Viz()
{
    delete [] mesh_.colors;
    delete [] mesh_.vertices;
    UnloadModel(model_);
    CloseWindow();
}

bool Viz::closed()
{
    return WindowShouldClose();
}

void Viz::render()
{
    camera_.fovy += GetMouseWheelMove() * -5;
    camera_.fovy = std::clamp(camera_.fovy, 0.0f, 170.0f);

    static int last_left_down_x{-1};
    static int last_left_down_y{-1};
    static int last_right_down_x{-1};
    static int last_right_down_y{-1};

    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
        last_left_down_x = -1;
        last_left_down_y = -1;
    } else if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
        int this_left_down_x{GetMouseX()};
        int this_left_down_y{GetMouseY()};
        if (last_left_down_x > 0 || last_left_down_y > 0) {
            const auto delta_x{this_left_down_x - last_left_down_x};
            const auto delta_y{this_left_down_y - last_left_down_y};
            camera_.position.x -= delta_x * 0.03f;
            camera_.position.y += delta_y * 0.03f;
            camera_.target.x -= delta_x * 0.03f;
            camera_.target.y += delta_y * 0.03f;
        }
        last_left_down_x = this_left_down_x;
        last_left_down_y = this_left_down_y;
    } else if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
        last_right_down_x = -1;
        last_right_down_y = -1;
    } else if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
        int this_right_down_x{GetMouseX()};
        int this_right_down_y{GetMouseY()};
        if (last_right_down_x > 0 || last_right_down_y > 0) {
            const auto delta_x{this_right_down_x - last_right_down_x};
            const auto delta_y{this_right_down_y - last_right_down_y};
            camera_.target.z += std::min(delta_y, delta_x) * 0.1f;
            camera_.target.y += delta_y * 0.1f;
            camera_.target.x -= delta_x * 0.1f;
        }
        last_right_down_x = this_right_down_x;
        last_right_down_y = this_right_down_y;
    }

    UpdateCamera(&camera_, CAMERA_CUSTOM);

    BeginDrawing();
        ClearBackground(BLACK);
        BeginMode3D(camera_);
            DrawModelPoints(model_, model_center_, 1.0f, WHITE);
        EndMode3D();
    EndDrawing();
}

#define RVIZ_MAX_COLOR_VEL 255

void Viz::draw_pointcloud(const PointRGBCloud& pc)
{
    set_pointcloud_mesh_buffer(pc.size(), mesh_);
    for (size_t i = 0; i < pc.size(); ++i) {
        const auto& point{pc.at(i)};
        mesh_.vertices[i * 3 + 0] = point.y;
        mesh_.vertices[i * 3 + 1] = point.x;
        mesh_.vertices[i * 3 + 2] = point.z;
        mesh_.colors[i * 4 + 0] = static_cast<uint8_t>(std::clamp(point.r, 0.0f, 1.0f) * RVIZ_MAX_COLOR_VEL);
        mesh_.colors[i * 4 + 1] = static_cast<uint8_t>(std::clamp(point.g, 0.0f, 1.0f) * RVIZ_MAX_COLOR_VEL);
        mesh_.colors[i * 4 + 2] = static_cast<uint8_t>(std::clamp(point.b, 0.0f, 1.0f) * RVIZ_MAX_COLOR_VEL);
        mesh_.colors[i * 4 + 3] = RVIZ_MAX_COLOR_VEL;
    }

    UploadMesh(&mesh_, true);
    model_ = LoadModelFromMesh(mesh_);
}

void Viz::draw_pointcloud(const PointICloud& pc)
{
    set_pointcloud_mesh_buffer(pc.size(), mesh_);
    Color color;
    for (size_t i = 0; i < pc.size(); ++i) {
        const auto& point{pc.at(i)};
        heatmap(point.i, color);
        mesh_.vertices[i * 3 + 0] = point.y;
        mesh_.vertices[i * 3 + 1] = point.x;
        mesh_.vertices[i * 3 + 2] = point.z;
        mesh_.colors[i * 4 + 0] = color.r;
        mesh_.colors[i * 4 + 1] = color.g;
        mesh_.colors[i * 4 + 2] = color.b;
        mesh_.colors[i * 4 + 3] = color.a;
    }

    UploadMesh(&mesh_, true);
    model_ = LoadModelFromMesh(mesh_);
}

void Viz::draw_se3()
{

}

void Viz::draw_image()
{

}

} // namespace rviz

#endif // RVIZ_CPP_
#endif // RVIZ_IMPLEMENTATION
