// clang-format off
//
// Created by goksu on 4/6/19.
//

// 关于抗锯齿
// 注意应该使用浮点数来偏移像素。
// SSAA 把一个像素分为多个子像素，并且每个子像素都要进行着色计算。在这个作业里，每一个子像素都需要执行getColor()。
// MSAA 把一个像素分为多个子像素，只计算这些子像素是否在三角形内，最后只需要进行一次着色计算，并把结果乘以子像素在三角形内的比例。
// 为了避免黑边的出现，需要额外创建和维护子像素的颜色和深度缓存。这部分需要修改作业中未指出的代码部分。

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

// edited
// 判断正负
int sign(const float number)
{
    return number > 0 ? 1 : (number < 0 ? -1 : 0); 
}
// endedited

static bool insideTriangle(float x, float y, const Vector3f* _v)
{   

    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]

    // edited
    Vector2i ab = {_v[1].x() - _v[0].x(), _v[1].y() - _v[0].y()};
    Vector2i bc = {_v[2].x() - _v[1].x(), _v[2].y() - _v[1].y()};
    Vector2i ca = {_v[0].x() - _v[2].x(), _v[0].y() - _v[2].y()};

    Vector2i ap = {x - _v[0].x(), y - _v[0].y()};
    Vector2i bp = {x - _v[1].x(), y - _v[1].y()};
    Vector2i cp = {x - _v[2].x(), y - _v[2].y()};

    float cross1 = ab.x() * ap.y() - ab.y() * ap.x();
    float cross2 = bc.x() * bp.y() - bc.y() * bp.x();
    float cross3 = ca.x() * cp.y() - ca.y() * cp.x();

    int sign_cross1 = sign(cross1);
    int sign_cross2 = sign(cross2);
    int sign_cross3 = sign(cross3);

    return sign_cross1 == sign_cross2 && sign_cross2 == sign_cross3;
    // endedited

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

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();

    std::array<Eigen::Vector3f, 3> vert = {v[0].head<3>(), v[1].head<3>(), v[2].head<3>()};


    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // edited
    int BBMin_x, BBMin_y, BBMax_x, BBMax_y;
    BBMin_x = v[0].x() < v[1].x() ? v[0].x() : (v[1].x() < v[2].x() ? v[1].x(): v[2].x());
    BBMin_y = v[0].y() < v[1].y() ? v[0].y() : (v[1].y() < v[2].y() ? v[1].y(): v[2].y());
    BBMax_x = v[0].x() > v[1].x() ? v[0].x() : (v[1].x() > v[2].x() ? v[1].x(): v[2].x());
    BBMax_y = v[0].y() > v[1].y() ? v[0].y() : (v[1].y() > v[2].y() ? v[1].y(): v[2].y());

#ifdef SSAA_COUNT
    for (int y = BBMin_y; y <= BBMax_y; y++)
    {
        for (int x = BBMin_x; x <= BBMax_x; x++)
        {
            bool inside = false;
            float depth = 0.0;

            int pixel_ind = get_index(x,y);

            Eigen::Vector3f color(0,0,0);
            for (int j = 0; j < SSAA_COUNT; j++ )
            {
                float ss_y = y + (1.0/SSAA_COUNT) * j;

                for (int i = 0; i < SSAA_COUNT; i++)
                {
                    float ss_x = x + (1.0/SSAA_COUNT) * i;

                    if (insideTriangle(ss_x + 0.5/SSAA_COUNT, ss_y + 0.5/SSAA_COUNT, vert.data()))
                    {
                        int ss_pixel_ind = pixel_ind * SSAA_COUNT * SSAA_COUNT + j*SSAA_COUNT + i;
                        inside = true;

                        auto[alpha, beta, gamma] = computeBarycentric2D(ss_x, ss_y, t.v);
                        float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;
                        
                        if (z_interpolated < ss_depth_buf[ss_pixel_ind])
                        {
                            ss_frame_buf[ss_pixel_ind] = t.getColor() / (SSAA_COUNT * SSAA_COUNT);
                            ss_depth_buf[ss_pixel_ind] = z_interpolated;
                        }
                        

                    }
                }
            }

            if (inside)
            {
                Eigen::Vector3f color(0,0,0);

                for (int j = 0; j < SSAA_COUNT; j++ )
                {
                    for (int i = 0; i < SSAA_COUNT; i++)
                    {
                        int ss_pixel_ind = pixel_ind * SSAA_COUNT * SSAA_COUNT + j*SSAA_COUNT + i;
                        color += ss_frame_buf[ss_pixel_ind];
                    }
                }

                set_pixel(Eigen::Vector3f(x, y, 0), color);
            }
            
        }
    }
#endif

#if !defined(SSAA_COUNT) && !defined(MSAA)
    for (int y = BBMin_y; y <= BBMax_y; y++)
    {
        for (int x = BBMin_x; x <= BBMax_x; x++)
        {

            if (insideTriangle(x + 0.5, y + 0.5, vert.data()))
            {
                auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;

                if (z_interpolated < depth_buf[get_index(x, y)])
                {   
                    set_pixel(Eigen::Vector3f(x,y,z_interpolated), t.getColor());
                    depth_buf[get_index(x, y)] = z_interpolated;
                }

                // set_pixel(Eigen::Vector3f(x,y,z_interpolated), t.getColor());
 
            }
            
        }
    }
#endif
    // endedited

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.

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
        // edited
#ifdef SSAA_COUNT
        std::fill(ss_frame_buf.begin(), ss_frame_buf.end(), Eigen::Vector3f{0, 0, 0});
#endif
        // endedited
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        // edited
#ifdef SSAA_COUNT
        std::fill(ss_depth_buf.begin(), ss_depth_buf.end(), std::numeric_limits<float>::infinity());
#endif
        // endedited
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);

    // edited
#ifdef SSAA_COUNT
    ss_frame_buf.resize(w * h * SSAA_COUNT * SSAA_COUNT);
    ss_depth_buf.resize(w * h * SSAA_COUNT * SSAA_COUNT);
#endif
    // endedited

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