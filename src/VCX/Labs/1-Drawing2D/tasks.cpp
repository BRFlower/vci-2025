#include <random>
#include <functional>
#include <spdlog/spdlog.h>

#include "Labs/1-Drawing2D/tasks.h"
// #include <iostream>
using VCX::Labs::Common::ImageRGB;

namespace VCX::Labs::Drawing2D {
    /******************* 1.Image Dithering *****************/
    void DitheringThreshold(
        ImageRGB &       output,
        ImageRGB const & input) {
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color = input.At(x, y);
                output.At(x, y) = {
                    color.r > 0.5 ? 1 : 0,
                    color.g > 0.5 ? 1 : 0,
                    color.b > 0.5 ? 1 : 0,
                };
            }
    }

    void DitheringRandomUniform(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        static std::random_device rd;  
        static std::mt19937 gen(rd()); 
        static std::uniform_real_distribution<> dist_real(-0.5, 0.5);
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                float p = (input.At(x, y).r + dist_real(gen)) > 0.5 ? 1 : 0;
                glm::vec3 color = input.At(x, y);
                output.At(x, y) = {p, p, p};
            }
    }

    void DitheringRandomBlueNoise(
        ImageRGB &       output,
        ImageRGB const & input,
        ImageRGB const & noise) {
        // your code here:
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 _i = input.At(x, y) , _n = noise.At(x, y);
                output.At(x,y) = {
                    (_i.r+_n.r) > 1 ? 1 : 0,
                    (_i.g+_n.g) > 1 ? 1 : 0,
                    (_i.b+_n.b) > 1 ? 1 : 0,
                };
            }
    }

    void DitheringOrdered(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        static const float get_sq[10][3][3] = {
            {{0,0,0},{0,0,0},{0,0,0}},
            {{0,0,0},{0,1,0},{0,0,0}},
            {{0,0,0},{1,1,0},{0,0,0}},
            {{},{1,1,0},{0,1,0}},
            {{},{1,1,1},{0,1,0}},
            {{0,0,1},{1,1,1},{0,1,0}},
            {{0,0,1},{1,1,1},{1,1,0}},
            {{1,0,1},{1,1,1},{1,1,0}},
            {{1,0,1},{1,1,1},{1,1,1}},
            {{1,1,1},{1,1,1},{1,1,1}},
        };
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color = input.At(x, y);
                int type = std::ceil(9 * color.r);
                for (int i = 0; i < 3; i ++)
                    for (int j = 0; j < 3; j ++){
                        float value = get_sq[type][i][j];
                        output.At(x*3+i,y*3+j) = {value,value,value};
                    }
            }
    }

    void DitheringErrorDiffuse(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        float cumulate[input.GetSizeX()+1][input.GetSizeY()+1];
        std::memset(cumulate,0,sizeof(cumulate));
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                cumulate[x][y]+= input.At(x,y).r;
                float q = cumulate[x][y] > 0.5 ? 1 : 0;
                output.At(x,y) = {q,q,q};
                q = (cumulate[x][y]-q) / 16;

                cumulate[x][y+1] += q * 7;
                cumulate[x+1][y-1] += q * 3;
                cumulate[x+1][y] += q * 5;
                cumulate[x+1][y+1] += q;
            }
    }
    

    /******************* 2.Image Filtering *****************/
    void Blur(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        int X = input.GetSizeX();
        int Y = input.GetSizeY();
        for (int x = 0; x < X; x ++)
            for (int y = 0; y < Y; y ++){
                glm::vec3 o = {0,0,0};
                int xr = std::min(X-1,x+1);
                int yr = std::min(Y-1,y+1);
                for (int xx = std::max(0,x-1); xx <= xr; xx ++)
                    for (int yy = std::max(0,y-1); yy <= yr; yy++)
                        o += input.At(xx,yy);
                output.At(x,y) = {o.r/9, o.g/9, o.b/9};
            }
    }

    void Edge(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        int X = input.GetSizeX();
        int Y = input.GetSizeY();
        float kernel[2][3][3] = { {{-1,0,1},{-2,0,2},{-1,0,1}} , {{-1,-2,-1},{0,0,0},{1,2,1}} };
        for (int x = 0; x < X; x ++){
            for (int y = 0; y < Y; y ++){
                glm::vec3 o[2] = {{0,0,0},{0,0,0}};
                int xr = std::min(1,X-x-1);
                int yr = std::min(1,Y-y-1);
                for (int xx = std::max(-1,-x); xx <= xr; xx ++)
                    for (int yy = std::max(-1,-y); yy <= yr; yy++)
                        for (int k = 0; k < 2; k ++)
                            o[k] += input.At(xx + x,yy + y) * kernel[k][xx+1][yy+1];
                o[0] = o[0] * o[0] + o[1] * o[1];
                output.At(x,y) = {sqrt(o[0].r),sqrt(o[0].g),sqrt(o[0].b)};
            }
        }
    }

    /******************* 3. Image Inpainting *****************/
    void Inpainting(
        ImageRGB &         output,
        ImageRGB const &   inputBack,
        ImageRGB const &   inputFront,
        const glm::ivec2 & offset) {
        output             = inputBack;
        std::size_t width  = inputFront.GetSizeX();
        std::size_t height = inputFront.GetSizeY();
        glm::vec3 * g      = new glm::vec3[width * height];
        memset(g, 0, sizeof(glm::vec3) * width * height);
        // set boundary condition
        for (std::size_t y = 0; y < height; ++y) {
            // set boundary for (0, y), your code: g[y * width] = ?
            // set boundary for (width - 1, y), your code: g[y * width + width - 1] = ?
            g[y * width] = inputBack.At(0 + offset.x, y + offset.y) - inputFront.At(0, y);
            g[y * width + width - 1] = inputBack.At(width - 1 + offset.x, y + offset.y) - inputFront.At(width - 1, y);
        }
        for (std::size_t x = 0; x < width; ++x) {
            // set boundary for (x, 0), your code: g[x] = ?
            // set boundary for (x, height - 1), your code: g[(height - 1) * width + x] = ?
            g[x] = inputBack.At(x + offset.x, 0 + offset.y) - inputFront.At(x, 0);
            g[(height - 1) * width + x] = inputBack.At(x + offset.x, height - 1 + offset.y) - inputFront.At(x, height - 1);
        }

        // Jacobi iteration, solve Ag = b
        for (int iter = 0; iter < 8000; ++iter) {
            for (std::size_t y = 1; y < height - 1; ++y)
                for (std::size_t x = 1; x < width - 1; ++x) {
                    g[y * width + x] = (g[(y - 1) * width + x] + g[(y + 1) * width + x] + g[y * width + x - 1] + g[y * width + x + 1]);
                    g[y * width + x] = g[y * width + x] * glm::vec3(0.25);
                }
        }

        for (std::size_t y = 0; y < inputFront.GetSizeY(); ++y)
            for (std::size_t x = 0; x < inputFront.GetSizeX(); ++x) {
                glm::vec3 color = g[y * width + x] + inputFront.At(x, y);
                output.At(x + offset.x, y + offset.y) = color;
            }
        delete[] g;
    }

    /******************* 4. Line Drawing *****************/
    auto conduct_line = [](const glm::ivec2 &p0, const glm::ivec2 &p1, const std::function<void(int, int)> &add_point){
        glm::ivec2 A,B;
        if (p0.x < p1.x){
            A = p0, B = p1;
        }
        else{
            A = p1, B = p0;
        }
        auto make_change = [&add_point, &A, &B](const int &delta_x, const int &delta_y, const std::function<void(int, int)> &func) { // x > y
            int dx = 2 * delta_x;
            int dy = 2 * delta_y;
            int dxdy = dy - dx;

            int F = - dx/2;
            int y = 0;
            for (int x = 0; x <= delta_x; x++){
                if (F < 0){
                    F += dy;
                }
                else{
                    y++;
                    F += dxdy;
                }
                func(x,y);
            }
        };
        if (A.y < B.y){
            int delta_x = B.x - A.x;
            int delta_y = B.y - A.y;
            if (delta_x >= delta_y){
                // std::cout << "a" << std::endl;
                auto func = [&] (int x, int y){
                    add_point(x+A.x,y+A.y);
                };
                make_change(delta_x, delta_y, func);
            }
            else{
                // std::cout << "b" << std::endl;
                auto func = [&] (int y, int x){
                    add_point(x+A.x,y+A.y);
                };
                make_change(delta_y, delta_x, func);
            }
        }
        else{
            int delta_x = B.x - A.x;
            int delta_y = A.y - B.y;
            if (delta_x >= delta_y){
                // std::cout << "c" << std::endl;
                auto func = [&] (int x, int y){
                    add_point(x+A.x,-y+A.y);
                };
                make_change(delta_x, delta_y, func);
            }
            else{
                // std::cout << "d" << std::endl;
                auto func = [&] (int y, int x){
                    add_point(x+A.x,-y+A.y);
                };
                make_change(delta_y, delta_x, func);
            }
        }
    };

    void DrawLine(
        ImageRGB &       canvas,
        glm::vec3 const  color,
        glm::ivec2 const p0,
        glm::ivec2 const p1) {
        // your code here:
        glm::ivec2 A,B;
        auto add_point = [&canvas, &color](int x, int y){
            canvas.At(x,y) = color;
        };
        conduct_line(p0, p1, add_point);
    }

    /******************* 5. Triangle Drawing *****************/
    void DrawTriangleFilled(
        ImageRGB &       canvas,
        glm::vec3 const  color,
        glm::ivec2 const p0,
        glm::ivec2 const p1,
        glm::ivec2 const p2) {
        // your code here:
        int x_min = std::min(p0.x, std::min(p1.x, p2.x));
        int x_max = std::max(p0.x, std::max(p1.x, p2.x));
        int edge[x_max - x_min + 1][2];
        for (int x = x_min; x <= x_max; x++ ){
            edge[x - x_min][0] = 1e9;
            edge[x - x_min][1] = -1e9;
        }
        auto add_point = [&](int x, int y){
            edge[x - x_min][0] = std::min(edge[x - x_min][0], y);
            edge[x - x_min][1] = std::max(edge[x - x_min][1], y);
        };
        conduct_line(p0, p1, add_point);
        conduct_line(p1, p2, add_point);
        conduct_line(p2, p0, add_point);
        for (int x = x_min; x <= x_max; x++)
            for (int y = edge[x - x_min][0]; y <= edge[x - x_min][1]; y++)
                canvas.At(x, y) = color;
        return;
    }
    /******************* 6. Image Supersampling *****************/
    void Supersample(
        ImageRGB &       output,
        ImageRGB const & input,
        int              rate) {
        // your code here:
        int ix = input.GetSizeX();
        int iy = input.GetSizeY();
        int ox = output.GetSizeX();
        int oy = output.GetSizeY();
        float delta_x = 1.0 * ix / ox / rate;
        float delta_y = 1.0 * iy / oy / rate;
        auto query = [&](float x, float y){
            int x1 = floor(x), y1 = floor(y);
            int x2 = ceil(x), y2 = ceil(y);
            if (x < 0)
                x1 = 0,x2 = 1;
            else if (x >= ix - 1)
                x1 = ix - 2, x2 = ix - 1;
            if (y < 0)
                y1 = 0,y2 = 1;
            else if (y >= iy - 1)
                y1 = iy - 2, y2 = iy - 1;
            return input.At(x1, y1) * (x2 - x) * (y2 - y) + input.At(x2, y1) * (x - x1) * (y2 - y) + input.At(x1, y2) * (x2 - x) * (y - y1) + input.At(x2, y2) * (x - x1) * (y - y1);
        };
        float av = 1.0 / rate / rate;
        for (int i = 0; i < ox; i ++){
            for (int j = 0; j < oy; j ++){
                glm::vec3 color = glm::vec3(0.0);
                for (int k = 0; k < rate; k ++){
                    for (int l = 0; l < rate; l ++){
                        color += query((i * rate + k - rate * 0.5) * delta_x, (j * rate + l - rate * 0.5) * delta_y);
                    }
                }
                output.At(i, j) = color * av;

            }
        }
    }

    /******************* 7. Bezier Curve *****************/
    // Note: Please finish the function [DrawLine] before trying this part.
    glm::vec2 CalculateBezierPoint(
        std::span<glm::vec2> points,
        float const          t) {
        // your code here:
        int len = points.size();
        glm::vec2 p0, p1;
        glm::vec2 new_points[len];
        for (int i = 0; i < len; i ++)
            new_points[i] = points[i];

        for (int s = len; s > 1; s --){
            p0 = new_points[0];
            for (int i = 1; i < s; i ++){
                p1 = new_points[i];
                new_points[i-1] = p0 * (1-t) + p1 * t;
                p0 = p1;
            }
        }
        return new_points[0];
    }
} // namespace VCX::Labs::Drawing2D