#include "Labs/5-Visualization/tasks.h"

#include <numbers>

using VCX::Labs::Common::ImageRGB;
namespace VCX::Labs::Visualization {

    struct CoordinateStates {
        // your code here
        glm::vec3 hsv2rgb(float h, float s, float v) {
            h = fmod(h, 360.0f);
            float c = v * s;
            float x = c * (1 - fabs(fmod(h / 60.0f, 2) - 1));
            float m = v - c;

            glm::vec3 rgb;
            if      (h < 60)  rgb = {c, x, 0};
            else if (h < 120) rgb = {x, c, 0};
            else if (h < 180) rgb = {0, c, x};
            else if (h < 240) rgb = {0, x, c};
            else if (h < 300) rgb = {x, 0, c};
            else              rgb = {c, 0, x};

            return rgb + glm::vec3(m);
        }

        glm::vec4 get_color(float v, float focus) {
            v = glm::clamp(v, 0.0f, 1.0f);
            focus = glm::clamp(focus, 0.0f, 1.0f);

            // 非线性拉开黄-绿
            float t = pow(v, 0.7f);

            // 红 → 青绿（跳过纯黄）
            float hue = 10.0f + (1.0f - t) * 140.0f;

            const float sat_min = 0.25f, sat_max = 0.5f;
            const float val_min = 0.5f, val_max = 1.0f;
            float saturation = sat_min + focus * sat_max;  // focus 高时增加饱和度;和数字区分
            float value      = val_min + focus * sat_max;  // focus 高时增加亮度

            glm::vec3 rgb = hsv2rgb(hue, saturation, value);

            float alpha = 0.05f + 0.5f * focus;
            return glm::vec4(rgb, alpha);
        }

        float graph_top;
        float graph_bottom;
        float graph_left;
        float graph_right;
        glm::vec2 transfer(glm::vec2 r){
            float rx = glm::clamp(r[0], 0.0f, 1.0f);
            float ry = glm::clamp(r[1], 0.0f, 1.0f);
            rx = rx * (graph_right - graph_left) + graph_left;
            ry = ry * (graph_bottom - graph_top) + graph_top;
            return glm::vec2(rx, ry);
        }
        glm::vec2 transfer_inv(glm::vec2 r){
            // float rx = glm::clamp(r[0], graph_left, graph_right);
            // float ry = glm::clamp(r[1], graph_top, graph_bottom);
            float rx = r.x, ry = r.y;
            rx = (rx - graph_left) / (graph_right - graph_left);
            ry = (ry - graph_top) / (graph_bottom - graph_top);
            return glm::vec2(rx, ry);
        }

        struct data_line{
            Car data_ori;
            float color_state;
            float focus;
        };
        int numAxes;
        std::vector<data_line> cars;
        std::vector<std::string> axisNames;
        std::vector<int> axisOrder;
        std::vector<std::pair<float, float>> axisRanges;
        // bool isDragging = false;
        // int draggedAxis = -1;
        // float dragStartX = 0.0f;
        float focus_high = 1.0f, focus_low = 0.0f;
        int focus_column;
        CoordinateStates(std::vector<Car> const & data){
            last_swap_a = -1;
            last_swap_b = -1;
            // ori_id = -1;
            // onSwapping = false;
            numAxes = 7;
            axisNames = {"Mileage", "Cylinders", "Displacement", 
                        "Horsepower", "Weight", "Acceleration", "Year"};
            
            axisOrder = {0, 1, 2, 3, 4, 5, 6}; // ith is axisNames[axisOrder[i]]
            
            graph_top = 0.2f;
            graph_bottom = 0.8f;
            graph_left = 0.0f;
            graph_right = 1.0f;

            cars.resize(data.size());
            
            for (size_t i = 0; i < data.size(); i++){
                cars[i].data_ori = data[i];
                cars[i].focus = focus_high;
            }
            calculateAxisRanges();
            calculateColorByAxis(0);
            focus_column = 0;
        }

        float getValueByIndex(Car const & car, int index) const {
            switch (index) {
                case 0: return car.mileage;
                case 1: return static_cast<float>(car.cylinders);
                case 2: return car.displacement;
                case 3: return car.horsepower;
                case 4: return car.weight;
                case 5: return car.acceleration;
                case 6: return static_cast<float>(car.year);
                default: return 0.0f;
            }
        }

        void calculateAxisRanges() {
            axisRanges.clear();
            axisRanges.resize(numAxes);

            if (cars.empty()) return;
            
            // 计算每个属性的最小最大值
            for (int i = 0; i < numAxes; ++i) {
                float minVal = std::numeric_limits<float>::max();
                float maxVal = std::numeric_limits<float>::lowest();
                
                for (const auto &car : cars) {
                    float val = getValueByIndex(car.data_ori, i);
                    minVal = std::min(minVal, val);
                    maxVal = std::max(maxVal, val);
                }
                
                axisRanges[i] = {minVal, maxVal};
            }
        }

        void calculateColorByAxis(int axisIndex) {
            for (size_t i = 0; i < cars.size(); i++) {
                cars[i].color_state = normalizeValue(axisIndex, getValueByIndex(cars[i].data_ori, axisIndex));
            }
            // needUpdateColors = true;
        }
        
        float normalizeValue(int axisIndex, float value) {
            float minVal = axisRanges[axisIndex].first;
            float maxVal = axisRanges[axisIndex].second;
            
            if (minVal == maxVal) {
                return 0.5f;
            }
            
            return (value - minVal) / (maxVal - minVal);
        }

        int get_column(glm::vec2 r){
            // float x = (i + 0.5f) / numAxes;
            // DrawLine(canvas, glm::vec4(1.0f, 1.0f, 1.0f, 1.0f), 
            //             transfer(glm::vec2(x, 0.0f)), transfer(glm::vec2(x, 1.0f)), 2.0f);
                
            // // 轴标签
            // if (axisOrder[i] < axisNames.size()) {
            //     PrintText(canvas, glm::vec4(1.0f, 1.0f, 1.0f, 1.0f), 
            //                 transfer(glm::vec2(x, 0.0f))- glm::vec2(0.0f,0.05f), 0.025f, axisNames[axisOrder[i]]);
            // }
            auto r_ = transfer_inv(r);
            float x = r_.x, y = r_.y;
            // if (y < -0.2f || y > 1.01f)
            //     return -1;
            float delta = 0.1f / numAxes;
            // abs(x - (i + 0.5f) / numAxes ) <= delta
            for (int i = 0; i < numAxes; i++)
                if (abs(x - (i + 0.5f) / numAxes ) <= delta)
                    return i;
            return -1;
        }

        void clearFocus(){
            for (size_t i = 0; i < cars.size(); i++){
                cars[i].focus = focus_high;
            }
            return;
        }
        // bool onSwapping = false;
        // int ori_id;
        int last_swap_a, last_swap_b;
        bool update(InteractProxy const & proxy){
            auto mousePos = proxy.MousePos();

            if (! proxy.IsHovering())
                return false;
            bool change = false;
            if (proxy.IsClicking()){
                //如果在坐标轴附近点击，或者在标签上点击
                int q = get_column(mousePos);
                if (q >= 0){
                    clearFocus();
                    if (q != focus_column){
                        calculateColorByAxis(axisOrder[q]);
                        focus_column = q;
                    }
                    // return true;
                    change = true;
                }
            }
            if (proxy.IsDragging()){
                auto start = proxy.DraggingStartPoint();
                int q = get_column(start);
                if (q >= 0){
                    float y_start = transfer_inv(start).y;
                    float y_end = transfer_inv(mousePos).y;
                    if (y_start > -0.05f && y_end > -0.05f/* && y_start < 1.05f && y_end < 1.05f*/){
                        y_end = glm::clamp(y_end, -0.05f, 1.05f);
                        if (q != focus_column){
                            calculateColorByAxis(axisOrder[q]);
                            focus_column = q;
                        }
                        if (y_end < y_start)
                            std::swap(y_start, y_end);
                        for (size_t i = 0; i < cars.size(); i++){
                            float s = 1 - normalizeValue(axisOrder[q], getValueByIndex(cars[i].data_ori, axisOrder[q]));
                            cars[i].focus = (s >= y_start && s <= y_end) ? focus_high : focus_low;
                        }
                        change = true;

                    }
                    else if (y_start < -0.05f && y_start > -0.1f){
                        // if (!onSwapping){
                        //     ori_id = q;
                        //     onSwapping = true;
                        //     change = true;
                        // }
                        float x_end = transfer_inv(mousePos).x;
                        float qprime = -1;
                        for (int i = 0; i < numAxes; i ++){
                            if (std::abs(x_end - (i + 0.5)/numAxes) < 0.3f / numAxes){
                                qprime = i;
                                break;
                            }
                        }
                        if (qprime >= 0 && qprime != q/*ori_id*/ && !(q == last_swap_a && qprime == last_swap_b)){
                            std::swap(axisOrder[q], axisOrder[qprime]);
                            // ori_id = qprime;
                            last_swap_a = q, last_swap_b = qprime;
                            change = true;
                        }//
                    }
                }
            }
            // else{
            //     if (onSwapping){
            //         ori_id = -1;
            //         onSwapping = false;
            //         change = true;
            //     }
            // }
            return change;
        }

        // 绘制平行坐标图
        void paint(Common::ImageRGB & canvas) {
            // 清空画布
            SetBackGround(canvas, glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));

            
            // 绘制数据线
            for (size_t i = 0; i < cars.size(); ++i) {
                drawCarLine(canvas, i);
            }
            
            // 绘制轴线和标签
            drawAxes(canvas);
            
            // if (onSwapping){
            //     DrawRect(canvas, glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),  transfer(glm::vec2((ori_id + 0.5f) / numAxes, 0.0f))- glm::vec2(0.05f,0.05f), glm::vec2(0.1f, 0.1f + graph_bottom - graph_top), 4.0f);
            // }
        }

        void drawCarLine(Common::ImageRGB & canvas, size_t carIndex) {
            
            // 绘制连接线段
            for (int i = 0; i < numAxes - 1; ++i) {
                // 获取当前轴和下一个轴的索引（考虑重新排序）
                int axis1Idx = axisOrder[i];
                int axis2Idx = axisOrder[i + 1];
                
                // 获取对应的值
                float val1 = getValueByIndex(cars[carIndex].data_ori, axis1Idx);
                float val2 = getValueByIndex(cars[carIndex].data_ori, axis2Idx);
                
                // 归一化值
                float normVal1 = normalizeValue(axis1Idx, val1);
                float normVal2 = normalizeValue(axis2Idx, val2);
                
                // 转换为画布坐标（百分比坐标）
                glm::vec2 point1 = glm::vec2((i + 0.5f) / numAxes, 1.0f - normVal1);
                glm::vec2 point2 = glm::vec2((i + 1 + 0.5f) / numAxes, 1.0f - normVal2);
                
                glm::vec4 color = get_color(cars[carIndex].color_state, cars[carIndex].focus);
                // 绘制线段
                DrawLine(canvas, color, transfer(point1), transfer(point2), 1.0f);
            }
        }


        void drawAxes(Common::ImageRGB & canvas) {
            
            // 绘制轴线
            for (int i = 0; i < numAxes; ++i) {
                float x = (i + 0.5f) / numAxes;
                
                // 轴线
                DrawLine(canvas, glm::vec4(1.0f, 1.0f, 1.0f, 1.0f), 
                        transfer(glm::vec2(x, 0.0f)), transfer(glm::vec2(x, 1.0f)), 2.0f);
                
                // 轴标签
                if (axisOrder[i] < axisNames.size()) {
                    PrintText(canvas, glm::vec4(1.0f, 1.0f, 1.0f, 1.0f), 
                             transfer(glm::vec2(x, 0.0f))- glm::vec2(0.0f,0.05f), 0.025f, axisNames[axisOrder[i]]);
                }
                
                // 绘制刻度
                drawAxisTicks(canvas, i, axisOrder[i]);
            }
        }


        void drawAxisTicks(Common::ImageRGB & canvas, int displayPos, int axisIdx) {
            auto [minVal, maxVal] = axisRanges[axisIdx];
            float x = (displayPos + 0.5f) / numAxes;
            
            // 绘制刻度线和标签
            for (int tick = 0; tick <= 4; ++tick) {
                float ratio = tick / 4.0f;
                float value = minVal + ratio * (maxVal - minVal);
                float y = 1.0f - ratio; // Y轴翻转
                
                // 刻度线
                DrawLine(canvas, glm::vec4(0.7f, 0.7f, 0.7f, 1.0f),
                        transfer(glm::vec2(x - 0.01f, y)), transfer(glm::vec2(x + 0.01f, y)), 1.5f);
                
                if (tick % 1 == 0) {
                    char buffer[32];
                    snprintf(buffer, sizeof(buffer), "%.0f", value);
                    PrintText(canvas, glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                             transfer(glm::vec2(x + 0.04f, y)), 0.03f, std::string(buffer));
                }
            }
        }
    };

    bool PaintParallelCoordinates(Common::ImageRGB & input, InteractProxy const & proxy, std::vector<Car> const & data, bool force) {
        // your code here
        // for example: 
        static CoordinateStates states(data);
        
        // 检查交互
        bool interactionChanged = states.update(proxy);
        
        // 如果强制重绘或交互改变，重新绘制
        if (force || interactionChanged) {
            states.paint(input);
            return true;
        }

        return false;
    }

    void LIC(ImageRGB & output, Common::ImageRGB const & noise, VectorField2D const & field, int const & step) {
        // your code here
        int X = output.GetSizeX();
        int Y = output.GetSizeY();

        auto get_velocity =[&](float x, float y){
            auto get = [&](int x, int y){
                return field.At(glm::clamp(x, 0, X-1), glm::clamp(y, 0, Y-1));
            };
            int x0 = floor(x);
            int y0 = floor(y);
            float dx = x - x0;
            float dy = y - y0;
            
            return (1-dx) * (1-dy) * get(x0, y0) + dx * (1-dy) * get(x0+1, y0) + (1-dx) * dy * get(x0, y0+1) + dx * dy * get(x0+1, y0+1);
        };
        auto get_noise = [&](float x, float y){
            auto get = [&](int x, int y){
                return noise.At(glm::clamp(x, 0, X-1), glm::clamp(y, 0, Y-1));
            };
            int x0 = floor(x);
            int y0 = floor(y);
            float dx = x - x0;
            float dy = y - y0;
            return (1-dx) * (1-dy) * get(x0, y0) + dx * (1-dy) *get(x0+1, y0) + (1-dx) * dy * get(x0, y0+1) + dx * dy * get(x0+1, y0+1);
        };
        float alpha = 1;
        for (int x = 0; x < X; x ++)
            for (int y = 0; y < Y; y ++){
                int counts = 1;
                glm::vec3 colors = glm::vec3(0.0f);
                // reverse;
                float _x = x;
                float _y = y;
                for (int i = 0; i < step; i ++){
                    glm::vec2 v = get_velocity(_x, _y);
                    glm::vec3 n = get_noise(_x, _y);
                    v = v / glm::length(v);
                    counts++;
                    colors += n;
                    _x -= alpha * v.x;
                    _y -= alpha * v.y;
                    if (_x < 0 || _x >= X || _y < 0 || _y >= Y)
                        break;
                }
                //forward
                for (int i = 0; i < step; i ++){
                    glm::vec2 v = get_velocity(_x, _y);
                    glm::vec3 n = get_noise(_x, _y);
                    v = v / glm::length(v);
                    counts++;
                    colors += n;
                    _x += alpha * v.x;
                    _y += alpha * v.y;
                    if (_x < 0 || _x >= X || _y < 0 || _y >= Y)
                        break;
                }
                colors = colors / (1.0f * counts);
                output.At(x,y) = colors;
            }
    }
}; // namespace VCX::Labs::Visualization