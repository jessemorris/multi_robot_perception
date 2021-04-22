#include "vdo_slam/visualizer/colour.h"
#include "vdo_slam/visualizer/colour-map.h"


namespace VDO_SLAM {



    HashableColor::HashableColor(const cv::viz::Color& color) : cv::viz::Color(color) {}
    HashableColor::HashableColor() : cv::viz::Color() {}
    HashableColor::HashableColor(uint8_t r, uint8_t g, uint8_t b)
        : HashableColor(r, g, b, 255) {}
    HashableColor::HashableColor(uint8_t r, uint8_t g, uint8_t b, uint8_t a)
        : cv::viz::Color(r, g, b, a) {}

    bool HashableColor::operator==(const HashableColor& other) const {
    return (r == other.r && g == other.g && b == other.b && a == other.a);
    }

    bool HashableColor::equal(const HashableColor& color) const {
    return r == color.r && g == color.g && b == color.b && a == color.a;
    }

}