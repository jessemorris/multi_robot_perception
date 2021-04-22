#ifndef VDO_SLAM_COLOR_H
#define VDO_SLAM_COLOR_H

#include <opencv2/viz/types.hpp>
#include <string>
#include <unordered_map>


constexpr size_t MAX_COLOUR_PALETTE = 150; //size of colour pallete as defined in colour-map.h

namespace VDO_SLAM {


    //https://github.com/MIT-SPARK/Kimera-Semantics/blob/master/kimera_semantics/include/kimera_semantics/color.h
    struct HashableColor : public cv::viz::Color {
        HashableColor();
        HashableColor(const cv::viz::Color& color);
        HashableColor(uint8_t r, uint8_t g, uint8_t b);
        HashableColor(uint8_t r, uint8_t g, uint8_t b, uint8_t a);

        bool operator==(const HashableColor& other) const;
        bool equal(const HashableColor& color) const;
    };

    typedef std::vector<HashableColor> HashableColors;
    typedef std::string ClassLabel;
    typedef int TrackingId;

    // For unordered map using Color as a Key.
    struct ColorHasher {
        size_t operator()(const HashableColor& k) const;
    };

    typedef std::unordered_map<ClassLabel, HashableColor>
        ClassLabelToColorMap;

    typedef std::unordered_map<TrackingId, HashableColor>
        TrackingIdToColorMap;


    class ColourManager {

        public:
            ColourManager();

            HashableColor get_colour_for_class_label(const std::string& label);
            HashableColor get_colour_for_tracking_id(const int tracking_id);

    };



}

#endif