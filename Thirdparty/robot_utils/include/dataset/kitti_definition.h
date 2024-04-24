/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/
#ifndef SEMANTIC_DEFINITION_H
#define SEMANTIC_DEFINITION_H

#include <map>
#include <string>
#include <tuple>

namespace kitti_utils {

typedef std::tuple<int, int, int> COLOR_TUPLE;
	
    #define NONE_FEATURE_ID   0

    //    ground
    #define ROAD_ID           40
    #define PARKING_ID        44
    #define SIDEWALK_ID       48
    #define OTHER_GROUND_ID   49
    #define LANE_MARKING_ID   60

    //    pillar
    #define POLE_ID           80
    #define TRUNK_ID          71

    //    plane
    #define BUILDING_ID       50
    #define FENCE_ID          51
    #define OTHER_STRUCT_ID   52

    #define CV_COLOR_BLACK      std::tuple(0,0,0)          // 纯黑
    #define CV_COLOR_WHITE      std::tuple(255,255,255)    // 纯白
    #define CV_COLOR_RED        std::tuple(0,0,255)        // 纯红
    #define CV_COLOR_GREEN      std::tuple(0,255,0)        // 纯绿
    #define CV_COLOR_BLUE       std::tuple(255,0,0)        // 纯蓝

    #define CV_COLOR_DARKGRAY   std::tuple(169,169,169)    // 深灰色
    #define CV_COLOR_DARKRED    std::tuple(0,0,169)        // 深红色
    #define CV_COLOR_ORANGERED  std::tuple(0,69,255)       // 橙红色

    #define CV_COLOR_CHOCOLATE  std::tuple(30,105,210)     // 巧克力色
    #define CV_COLOR_GOLD       std::tuple(10,215,255)     // 金色
    #define CV_COLOR_YELLOW     std::tuple(0,255,255)      // 纯黄色

    #define CV_COLOR_OLIVE      std::tuple(0,128,128)      // 橄榄色
    #define CV_COLOR_LIGHTGREEN std::tuple(144,238,144)    // 浅绿色
    #define CV_COLOR_DARKCYAN   std::tuple(139,139,0)      // 深青色
    #define CV_COLOR_CYAN       std::tuple(255,255,0)      // 青色

    #define CV_COLOR_SKYBLUE    std::tuple(235,206,135)    // 天蓝色
    #define CV_COLOR_INDIGO     std::tuple(130,0,75)       // 藏青色
    #define CV_COLOR_PURPLE     std::tuple(128,0,128)      // 紫色

    #define CV_COLOR_PINK       std::tuple(203,192,255)    // 粉色
    #define CV_COLOR_DEEPPINK   std::tuple(147,20,255)     // 深粉色
    #define CV_COLOR_VIOLET     std::tuple(238,130,238)    // 紫罗兰

    static std::vector<COLOR_TUPLE> cs_map_color = {
            CV_COLOR_DARKCYAN,
            CV_COLOR_PURPLE,
            CV_COLOR_ORANGERED,
            CV_COLOR_YELLOW,
            CV_COLOR_INDIGO,
            CV_COLOR_DEEPPINK,
            CV_COLOR_OLIVE,
            CV_COLOR_DARKRED,
            CV_COLOR_GREEN,
            CV_COLOR_BLUE
    };

    static std::vector<COLOR_TUPLE> random_color_vec = {
            std::tuple(0, 255, 0),
            std::tuple(0, 0, 255),
            std::tuple(245, 150, 100),
            std::tuple(245, 230, 100),
            std::tuple(250, 80, 100),
            std::tuple(150, 60, 30),
            std::tuple(255, 0, 0),
            std::tuple(180, 30, 80),
            std::tuple(50, 50, 255),
            std::tuple(200, 40, 255),
            std::tuple(90, 30, 150),
            std::tuple(255, 0, 255),
            std::tuple(255, 150, 255),
            std::tuple(75, 0, 75),
            std::tuple(75, 0, 175),
            std::tuple(0, 200, 255),
            std::tuple(50, 120, 255),
            std::tuple(0, 150, 255),
            std::tuple(170, 255, 150),
            std::tuple(0, 175, 0),
            std::tuple(0, 60, 135),
            std::tuple(80, 240, 150),
            std::tuple(150, 240, 255),
            std::tuple(0, 0, 255),
            std::tuple(255, 255, 50),
            std::tuple(245, 150, 100),
    };

    static std::map<uint16_t, std::string> kitti_sem_cls_info = {
            {0,   "unlabeled"},
            {1,   "outlier"},
            {10,  "car"},
            {11,  "bicycle"},
            {13,  "bus"},
            {15,  "motorcycle"},
            {16,  "on-rails"},
            {18,  "truck"},
            {20,  "other-vehicle"},
            {30,  "person"},
            {31,  "bicyclist"},
            {32,  "motorcyclist"},
            {40,  "road"},
            {44,  "parking"},
            {48,  "sidewalk"},
            {49,  "other-ground"},
            {50,  "building"},
            {51,  "fence"},
            {52,  "other-structure"},
            {60,  "lane-marking"},
            {70,  "vegetation"},
            {71,  "trunk"},
            {72,  "terrain"},
            {80,  "pole"},
            {81,  "traffic-sign"},
            {99,  "other-object"},
            {252, "moving-car"},
            {253, "moving-bicyclist"},
            {254, "moving-person"},
            {255, "moving-motorcyclist"},
            {256, "moving-on-rails"},
            {257, "moving-bus"},
            {258, "moving-truck"},
            {259, "moving-other-vehicle"}
    };

    static std::map<uint16_t, COLOR_TUPLE> kitti_sem_color_info = {
            {0,   std::tuple(0, 0, 0)},
            {1,   std::tuple(0, 0, 255)},
            {10,  std::tuple(245, 150, 100)},
            {11,  std::tuple(245, 230, 100)},
            {13,  std::tuple(250, 80, 100)},
            {15,  std::tuple(150, 60, 30)},
            {16,  std::tuple(255, 0, 0)},
            {18,  std::tuple(180, 30, 80)},
            {20,  std::tuple(255, 0, 0)},
            {30,  std::tuple(30, 30, 255)},
            {31,  std::tuple(200, 40, 255)},
            {32,  std::tuple(90, 30, 150)},
            {40,  std::tuple(255, 0, 255)},
            {44,  std::tuple(255, 150, 255)},
            {48,  std::tuple(75, 0, 75)},
            {49,  std::tuple(75, 0, 175)},
            {50,  std::tuple(0, 200, 255)},
            {51,  std::tuple(50, 120, 255)},
            {52,  std::tuple(0, 150, 255)},
            {60,  std::tuple(170, 255, 150)},
            {70,  std::tuple(0, 175, 0)},
            {71,  std::tuple(0, 60, 135)},
            {72,  std::tuple(80, 240, 150)},
            {80,  std::tuple(150, 240, 255)},
            {81,  std::tuple(0, 0, 255)},
            {99,  std::tuple(255, 255, 50)},
            {252, std::tuple(245, 150, 100)},
            {253, std::tuple(200, 40, 255)},
            {254, std::tuple(30, 30, 255)},
            {255, std::tuple(90, 30, 150)},
            {256, std::tuple(255, 0, 0)},
            {257, std::tuple(250, 80, 100)},
            {258, std::tuple(180, 30, 80)},
            {259, std::tuple(255, 0, 0)}
    };

} // namespace g3reg

#endif