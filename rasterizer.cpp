// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <tuple>


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


static bool insideTriangle(float x, float y, const Vector3f* _v)
{
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    const Eigen::Vector2f P(x, y);
    const Eigen::Vector2f A = _v[0].head(2), B = _v[1].head(2), C = _v[2].head(2);

    const Eigen::Vector2f AP = P - A;
    const Eigen::Vector2f BP = P - B;
    const Eigen::Vector2f CP = P - C;
    const Eigen::Vector2f AB = B - A;
    const Eigen::Vector2f BC = C - B;
    const Eigen::Vector2f CA = A - C;

    float eq1 = AB[0] * AP[1] - AB[1] * AP[0];   // ABXAP
    float eq2 = BC[0] * BP[1] - BC[1] * BP[0];   // BCXBP
    float eq3 = CA[0] * CP[1] - CA[1] * CP[0];   // CAXCP

    if (eq1 > 0 && eq2 > 0 && eq3 > 0)    // 有无在三角形内
        return true;
    else
        return false;

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

std::vector<Eigen::Vector3f> frame_buf;
std::vector<Eigen::Vector3f> super_frame_buf;

std::vector<float> depth_buf;
std::vector<float> super_depth_buf;



rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    super_frame_buf.resize(w * h * 4);
    super_depth_buf.resize(w * h * 4);
}





//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();

    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    //1. 找到Bounding Box，也就是最大最小的x,y坐标然后再进行向上向下取证。
    float xmin = std::min(std::min(v[0].x(), v[1].x()), v[2].x());
    float ymin = std::min(std::min(v[0].y(), v[1].y()), v[2].y());
    float xmax = std::max(std::max(v[0].x(), v[1].x()), v[2].x());
    float ymax = std::max(std::max(v[0].y(), v[1].y()), v[2].y());

    //我们将bounding box稍微扩大一点，得到整数值，方便循环
    xmin = (int)std::floor(xmin);
    xmax = (int)std::ceil(xmax);
    ymin = (int)std::floor(ymin);
    ymax = (int)std::ceil(ymax);

    // 超采样   super_sample_step 将一个正方形内再细分四个点，计算点的个数，着色的时候取均值t.getColor()*count/4
    std::vector<Eigen::Vector2f> super_sample_step
    {
        {0.25,0.25},
        {0.75,0.25},
        {0.25,0.75},
        {0.75,0.75},
    };
    // 遍历变量进行深度检测并赋予颜色color值
    for (int x = xmin; x <= xmax; x++)
    {
        for (int y = ymin; y <= ymax; y++)
        {
            int judge = 0;
            //具体思路就是记录四倍的数据量，把超采样的数据都记下来
            for (int i = 0; i < 4; i++)
            {
                //  如果在在三角形内部进行深度判断
                if (insideTriangle(x + super_sample_step[i][0], y + super_sample_step[i][1], t.v))
                {
                    float alpha, beta, gamma;
                    std::tie(alpha, beta, gamma) = computeBarycentric2D(x, y, t.v);
                    float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;
                    // 再次进行 坐标变换
                    //  在x,y轴上坐标处理方式不一样，在四个块状正方形 从0到3  x轴会变化四次,y轴变化两次因此如下处理。
                    if (super_depth_buf[get_super_index(x * 2 + i % 2, y * 2 + i / 2)] > z_interpolated)
                    {
                        judge = 1;
                        super_depth_buf[get_super_index(x * 2 + i % 2, y * 2 + i / 2)] = z_interpolated;
                        super_frame_buf[get_super_index(x * 2 + i % 2, y * 2 + i / 2)] = t.getColor();
                    }
                }
            }
            if (judge)
                //若像素的四个样本中有一个通过了深度测试，就需要对该像素进行着色。
            {
                Vector3f point = { (float)x,(float)y,0 };
                Vector3f color = (super_frame_buf[get_super_index(x * 2, y * 2)] + super_frame_buf[get_super_index(x * 2 + 1, y * 2)] + super_frame_buf[get_super_index(x * 2, y * 2 + 1)] + super_frame_buf[get_super_index(x * 2 + 1, y * 2 + 1)]) / 4;
                set_pixel(point, color);
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
        std::fill(super_frame_buf.begin(), super_frame_buf.end(), Eigen::Vector3f{ 0, 0, 0 });
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(super_depth_buf.begin(), super_depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

int rst::rasterizer::get_index(int x, int y)
{
    //  从0行0列开始记录
    return (height-1-y)*width + x;
}

int rst::rasterizer::get_super_index(int x, int y)
{
    //  扩大四倍的域都进行记录
    return (height * 2 - 1 - y) * width * 2 + x;
}


void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //   old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;
}

// clang-format on