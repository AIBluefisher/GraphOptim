#ifndef GRAPH_COLOR_GRADIENT_H_
#define GRAPH_COLOR_GRADIENT_H_

#include <vector>

namespace gopt {

// Port made from
// http://www.andrewnoske.com/wiki/Code_-_heatmaps_and_color_gradients
// - Use c++11 style
// - Add multiple possible color gradient initialization
//
// --------------------
// How to use the code:
// --------------------
// 1. Create the object and configure the color gradient
// --------------------
// ColorGradient heatMapGradient(ColorGradient::k2BlueRedHeatMap());
// // Re-Initialize to use a gradient based on 5 colors
// //heatMapGradient = ColorGradient(ColorGradient::k5ColorHeatMap());
// --------------------
// 2. Get the color corresponding to the % range you want
// --------------------
// float r,g,b;
// const float interpolated_ratio = .5f;
// heatMapGradient.getColor(interpolated_ratio, r, g, b);
// // [r,g,b] are in the range [0, 1]
//
class ColorGradient {
public:
  // Internal class used to store colors at different points
  // in the gradient.
  struct ColorPoint {
    float r, g, b; // Red, green and blue values of our color.
    float val; // Position of our color along the gradient (between 0 and 1).

    ColorPoint(float red, float green, float blue, float value)
        : r(red), g(green), b(blue), val(value) {}
  };

  using ColorPoints = std::vector<ColorPoint>;

  //-- Places a 5 color heatmap gradient into the "color" vector:
  static const ColorPoints K5ColorHeatMap() {
    return {
        {0, 0, 1, 0.0f},  // Blue
        {0, 1, 1, 0.25f}, // Cyan
        {0, 1, 0, 0.5f},  // Green
        {1, 1, 0, 0.75f}, // Yellow
        {1, 0, 0, 1.0f}   // Red
    };
  }

  //-- Places a 2 color heatmap gradient (Blue to Red):
  static const ColorPoints K2BlueRedHeatMap() {
    return {
        {0, 0, 1, 0.0f}, // Blue
        {1, 0, 0, 1.0f}  // Red
    };
  }

public:
  //-- Default constructor:
  explicit ColorGradient(const ColorPoints &rhs_color = K5ColorHeatMap())
      : color_map_(rhs_color) {}

  //-- Inputs a (value) between 0 and 1 and outputs the (red), (green) and
  //(blue)
  //-- values representing that position in the gradient.
  void GetColor(const float value, float *red, float *green,
                float *blue) const {
    if (color_map_.empty()) {
      return;
    }

    const int color_num = static_cast<int>(color_map_.size());
    for (int i = 0; i < color_num; ++i) {
      const ColorPoint &cur_color = color_map_[i];
      if (value < cur_color.val) {
        const ColorPoint &prev_color = color_map_[std::max(0, i - 1)];
        const float value_diff = (prev_color.val - cur_color.val);
        const float fract_between =
            (value_diff == 0) ? 0 : (value - cur_color.val) / value_diff;
        *red = (prev_color.r - cur_color.r) * fract_between + cur_color.r;
        *green = (prev_color.g - cur_color.g) * fract_between + cur_color.g;
        *blue = (prev_color.b - cur_color.b) * fract_between + cur_color.b;
        return;
      }
    }

    *red = color_map_.back().r;
    *green = color_map_.back().g;
    *blue = color_map_.back().b;
  }

private:
  ColorPoints color_map_; // An array of color points in ascending value
};

} // namespace gopt

#endif  // GRAPH_COLOR_GRADIENT_H_
