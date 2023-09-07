#include <time.h>
#include <random>

#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

// https://blog.csdn.net/mightbxg/article/details/118338302
int main() {
// generate points
  constexpr size_t num = 100;
  constexpr int width = 640, height = 480;
  vector<Point2f> pts;
  Mat image(height, width, CV_8UC3, cv::Scalar::all(255));
  {
      mt19937 gen(0);
      uniform_real_distribution<float> dis_x(0.f, float(width));
      uniform_real_distribution<float> dis_y(0.f, float(height));
      pts.reserve(num);
      for (size_t i = 0; i < num; ++i) {
          pts.emplace_back(dis_x(gen), dis_y(gen));
          circle(image, pts.back(), 2, cv::Scalar::all(0), cv::FILLED);
      }
  }
  cout << "num of points: " << pts.size() << '\n';

  Point2f query(width / 2.f, height / 2.f);
  cout << "query: " << query.x << " " << query.y << '\n';
  circle(image, query, 2, { 0, 0, 255 }, cv::FILLED);

  // build kd-tree
  auto kdtree = flann::GenericIndex<flann::L2<float>>(Mat(pts).reshape(1), cvflann::KDTreeIndexParams{2});

  // knn search
  constexpr int K = 4;
  vector<int> indices(K);
  vector<float> dists(K);
  kdtree.knnSearch({ query.x, query.y }, indices, dists, K, cvflann::SearchParams {});
  cout << " nearest " << K << ": " << Mat(indices).t() << endl;
  {
      Mat result = image.clone();
      for (int i : indices) {
          line(result, query, pts[i], { 0, 255, 0 });
          circle(result, pts[i], 2, { 255, 0, 0 }, cv::FILLED);
      }
      imwrite("knn_result.png", result);
  }

  // radius search
  constexpr float radius = 50.f;
  constexpr size_t max_num = 10;
  indices.resize(max_num, -1);
  dists.resize(max_num);
  kdtree.radiusSearch({ query.x, query.y }, indices, dists, radius * radius, cvflann::SearchParams {});  // L2距离，radius参数是半径的平方
  cout << "in radius " << radius << ": " << Mat(indices).t() << endl;
  {
      Mat result = image.clone();
      for (int i : indices) {
          if (i < 0)
              break;
          line(result, query, pts[i], { 0, 255, 0 });
          circle(result, pts[i], 2, { 255, 0, 0 }, cv::FILLED);
      }
      circle(result, query, radius, { 0, 0, 255 });
      imwrite("radius_result.png", result);
  }
  return 0;
}