// clang-format off
#include <iostream>
#include <opencv2/opencv.hpp>
#include "rasterizer.hpp"
#include "global.hpp"
#include "Triangle.hpp"

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    return model;
}

// ─────────────────────────────────────────────────────────────────────────────
// 透视投影矩阵
//
// 思路： M_proj = M_ortho * M_persp
//   1) M_persp : 把视锥体「挤压」成一个长方体（保持近、远平面位置，远端缩到与近端等宽）
//   2) M_ortho : 把这个长方体平移并缩放到标准 NDC 立方体 [-1, 1]^3
//
// 符号约定（GAMES101 / OpenGL 风格）：相机看向 -z 方向，
//   所以 zNear、zFar 作为「距离」是正数，但实际平面坐标是 -zNear、-zFar。
// ─────────────────────────────────────────────────────────────────────────────
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // 把传入的「正距离」转成实际的 z 平面坐标（负值）
    float n = -zNear;
    float f = -zFar;

    // 视锥近平面的尺寸（由 fov 与 aspect 决定）
    //   t (top)    = |n| * tan(fov/2)
    //   r (right)  = t * aspect
    //   b, l 取负值（关于原点对称）
    float half_fov = eye_fov * MY_PI / 180.0f / 2.0f;
    float t = std::tan(half_fov) * std::abs(n);
    float b = -t;
    float r = t * aspect_ratio;
    float l = -r;

    // Step 1: 透视 → 正交「挤压」矩阵
    //   把视锥体压成长方体：x、y 在远处会按 z 的比例缩小，从而产生近大远小的效果。
    Eigen::Matrix4f M_persp;
    M_persp << n, 0, 0,     0,
               0, n, 0,     0,
               0, 0, n + f, -n * f,
               0, 0, 1,     0;

    // Step 2: 正交投影矩阵（平移到原点 + 缩放到 [-1, 1]）
    //   一步合成形式：把 [l,r] x [b,t] x [f,n] 的长方体映射成 [-1,1]^3
    Eigen::Matrix4f M_ortho;
    M_ortho << 2 / (r - l), 0,           0,           -(r + l) / (r - l),
               0,           2 / (t - b), 0,           -(t + b) / (t - b),
               0,           0,           2 / (n - f), -(n + f) / (n - f),
               0,           0,           0,           1;

    // 先挤压再正交，最终得到透视投影矩阵
    projection = M_ortho * M_persp;
    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc == 2)
    {
        command_line = true;
        filename = std::string(argv[1]);
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0,0,5};


    std::vector<Eigen::Vector3f> pos
            {
                    {2, 0, -2},
                    {0, 2, -2},
                    {-2, 0, -2},
                    {3.5, -1, -5},
                    {2.5, 1.5, -5},
                    {-1, 0.5, -5}
            };

    std::vector<Eigen::Vector3i> ind
            {
                    {0, 1, 2},
                    {3, 4, 5}
            };

    std::vector<Eigen::Vector3f> cols
            {
                    {217.0, 238.0, 185.0},
                    {217.0, 238.0, 185.0},
                    {217.0, 238.0, 185.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0}
            };

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);
    auto col_id = r.load_colors(cols);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while(key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';
    }

    return 0;
}
// clang-format on