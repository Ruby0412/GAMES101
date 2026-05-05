// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


// ─────────────────────────────────────────────────────────────────────────────
// 判断点 (x, y) 是否在三角形 _v[0], _v[1], _v[2] 内部
//
// 参数用 float 是因为我们传入的是「像素中心」(x + 0.5, y + 0.5)，不是整数坐标。
//
// 原理：2D 叉积法
//   对三角形的每条边（V0->V1, V1->V2, V2->V0），用 2D 叉积判断点 P
//   在边的「左侧」还是「右侧」。如果 P 对三条边都在同一侧，就说明 P 在内部。
//
// 2D 叉积公式（z 分量）：A × B = A.x * B.y - A.y * B.x
//   - 结果 > 0  →  B 在 A 的逆时针方向（左侧）
//   - 结果 < 0  →  B 在 A 的顺时针方向（右侧）
//   - 结果 = 0  →  共线
//
// 注意：顶点顺序（顺时针 vs 逆时针）会让所有符号整体翻转，所以
//      「全正」和「全负」都算在内部。
// ─────────────────────────────────────────────────────────────────────────────
static bool insideTriangle(float x, float y, const Vector3f* _v)
{
    Vector3f P(x, y, 0.0f);

    // 三条有向边
    Vector3f AB = _v[1] - _v[0];
    Vector3f BC = _v[2] - _v[1];
    Vector3f CA = _v[0] - _v[2];

    // 从每条边的「起点」指向 P 的向量
    Vector3f AP = P - _v[0];
    Vector3f BP = P - _v[1];
    Vector3f CP = P - _v[2];

    // 三个 2D 叉积的 z 分量
    float c1 = AB.x() * AP.y() - AB.y() * AP.x();
    float c2 = BC.x() * BP.y() - BC.y() * BP.x();
    float c3 = CA.x() * CP.y() - CA.y() * CP.x();

    // 三个符号一致 → P 在三角形内部
    return (c1 > 0 && c2 > 0 && c3 > 0) || (c1 < 0 && c2 < 0 && c3 < 0);
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 屏幕空间光栅化：把一个三角形画到 frame_buffer 上，并用 depth_buffer 处理遮挡。
//
// 进入此函数时：
//   - t.v[0..2] 已经过 MVP + 齐次除法 + 视口变换，xy 是屏幕像素坐标
//   - 在本作业的框架下，齐次除法已经做过，所以 v[i].w() 全是 1
//
// 算法步骤：
//   1) 计算三角形的轴对齐包围盒（AABB），只在盒内遍历像素，节省时间
//   2) 对盒内每个像素的「中心」(x+0.5, y+0.5)：
//        a. 用叉积法判断是否在三角形内
//        b. 用重心坐标插值出该像素的深度 z（透视校正写法）
//        c. 做深度测试：z 比当前 depth_buf 更近才更新
//        d. 通过则用 set_pixel 写入颜色
// ─────────────────────────────────────────────────────────────────────────────
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();   // v[i] 是 Eigen::Vector4f：(x_screen, y_screen, z, w)

    // ─── Step 1: 求三角形的轴对齐包围盒 (AABB) ───
    // 包围盒 = 能完全包住三角形的最小矩形。这样我们只需遍历 (x_max-x_min)*(y_max-y_min)
    // 个像素，而不是整个 700x700 屏幕。
    float min_x = std::min({v[0].x(), v[1].x(), v[2].x()});
    float max_x = std::max({v[0].x(), v[1].x(), v[2].x()});
    float min_y = std::min({v[0].y(), v[1].y(), v[2].y()});
    float max_y = std::max({v[0].y(), v[1].y(), v[2].y()});

    // 像素是整数坐标，所以下界 floor、上界 ceil，确保把整个三角形都包住
    int x_min = static_cast<int>(std::floor(min_x));
    int x_max = static_cast<int>(std::ceil (max_x));
    int y_min = static_cast<int>(std::floor(min_y));
    int y_max = static_cast<int>(std::ceil (max_y));

    // ─── Step 2: 遍历包围盒内每个像素，对每个像素做 2x2 = 4 次子样本 ───
    //
    // MSAA 4x：在像素内部 (x+0.25/0.75, y+0.25/0.75) 这 4 个位置分别采样。
    //          每个子样本独立做 inside 测试 + 深度测试 + 颜色记录。
    //          最后把 4 个样本的颜色平均，作为像素的最终颜色。
    //
    // 关键：每个子样本都有自己独立的深度和颜色（避免边缘黑边）。
    static const float dx[4] = {0.25f, 0.75f, 0.25f, 0.75f};
    static const float dy[4] = {0.25f, 0.25f, 0.75f, 0.75f};

    Eigen::Vector3f tri_color = t.getColor();

    for (int x = x_min; x <= x_max; ++x) {
        for (int y = y_min; y <= y_max; ++y) {

            int pixel_idx = get_index(x, y);
            bool any_updated = false;

            // 遍历 4 个子样本
            for (int s = 0; s < 4; ++s) {
                float px = x + dx[s];
                float py = y + dy[s];

                // 2a. 子样本是否在三角形内？
                if (!insideTriangle(px, py, t.v)) continue;

                // 2b. 透视校正深度插值（同前）
                auto [alpha, beta, gamma] = computeBarycentric2D(px, py, t.v);
                float w_reciprocal = 1.0f / (alpha/v[0].w() + beta/v[1].w() + gamma/v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w()
                                     + beta  * v[1].z() / v[1].w()
                                     + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;

                // 2c. 子样本的深度测试（每个样本有自己的深度）
                int sample_idx = pixel_idx * 4 + s;
                if (z_interpolated < sample_depth_buf[sample_idx]) {
                    sample_depth_buf[sample_idx] = z_interpolated;
                    sample_color_buf[sample_idx] = tri_color;
                    any_updated = true;
                }
            }

            // 2d. 任意子样本被更新了，就把 4 个子样本平均，写入像素
            if (any_updated) {
                Eigen::Vector3f avg = (sample_color_buf[pixel_idx*4 + 0]
                                     + sample_color_buf[pixel_idx*4 + 1]
                                     + sample_color_buf[pixel_idx*4 + 2]
                                     + sample_color_buf[pixel_idx*4 + 3]) / 4.0f;

                Eigen::Vector3f point(static_cast<float>(x), static_cast<float>(y), 0.0f);
                set_pixel(point, avg);
            }
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        // MSAA：同时清空子样本颜色
        std::fill(sample_color_buf.begin(), sample_color_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        // MSAA：同时清空子样本深度
        std::fill(sample_depth_buf.begin(), sample_depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);

    // MSAA 4x：每像素 4 个子样本
    sample_depth_buf.resize(w * h * 4);
    sample_color_buf.resize(w * h * 4);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on