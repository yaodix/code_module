#ifndef CONFIG_H_
#define CONFIG_H_

constexpr int kScreenWidth = 1200;
constexpr int kScreenHeight = 800;

constexpr int kBallRadius = 12;

constexpr int kNumBalls = 200;
constexpr int kNumRedBalls = 5;// NUM_BALLS / 500;

constexpr int kModulo = 1000;

constexpr double kBallDx = 0.1;
constexpr double kBallDy = 0.1;

constexpr int kCollisionOffset =  0;// in pixels

constexpr int kTimeAdjustment = 800;

constexpr int kQuadtreeNodeCapacity = 5;

constexpr int KQuadtreeOutlineThickness = 1;

constexpr int kBallOutlineThickness = 0;

constexpr int kFrameRateLimit = 60;

constexpr bool kDrawQuadtreeBoundaries = true;

constexpr bool kPrintStatsToConsole = true;

constexpr long long kOutputTimeCutoff = 500;// in milliseconds

#endif // CONFIG_H_
