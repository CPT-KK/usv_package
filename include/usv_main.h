#pragma once

// All states
#define USV_WAIT 0
#define PURSUE 10
#define PURSUE_DETECT_OBS 11
#define PURSUE_EMERGENCY 19
#define DOCK_PREPARE 20
#define DOCK_APPROACH 21
#define DOCK_ORBIT 22
#define DOCK_TRANSFER 23
#define DOCK_ADJUST 24
#define DOCK_FINAL 25
#define DOCK_EMERGENCY 29
#define ATTACH 31
#define USV_STOP 9

// 上移光标
#define MOVEUP(x) printf("\033[%dA", (x))

// 下移光标
#define MOVEDOWN(x) printf("\033[%dB", (x))

// 定位光标
#define MOVETO(x,y) printf("\033[%d;%dH", (x), (y))

#define CLEAR() printf("\033[2J")

#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <functional>

// USV LIDAR
#include "usv_lidar.h"

// USV PATH PLANNER
#include "usv_path_planner.h"

// USV POSE
#include "usv_pose.h"

// USV GUIDANCE AND CONTROLLER
#include "usv_controller.h"
#include "usv_guidance.h"