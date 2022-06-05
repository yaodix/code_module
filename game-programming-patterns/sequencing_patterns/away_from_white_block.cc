#include <iostream>
#include <random>
#include <vector>
#include <cstdlib>
#include <time.h>
#include <algorithm>

#include "opencv4/opencv2/opencv.hpp"

class AwayWhiteBlock {
  public:
    AwayWhiteBlock(int rect_width, int rect_height) {
    for (int c = 0; c < kColCnt; c++) {
        std::vector<cv::Point> row_rect_tmp;
      for(int r = 0; r < kRowCnt; r++) {
          row_rect_tmp.emplace_back(r * rect_width, c * rect_height); 
        }      
        rect_topleft_point_vvec_.push_back(row_rect_tmp);
      }

      rect_width_ = rect_width;
      rect_height_ = rect_height;

      // 1/4概率分布的设定 
      std::mt19937 rng;
      rng.seed(std::random_device()());
      for(int i = 0; i< kRowCnt; i++) {
        std::vector<int> tmp(4, 1);
        std::uniform_int_distribution<int> dist4(0, 3);
         int black_block_index = dist4(rng);
        if(i != kRowCnt -1) 
          tmp[black_block_index] = 0;
        draw_color_vec_.push_back(tmp);
      }

      canvas_ = cv::Mat::zeros(cv::Size((kColCnt)*rect_width, (kRowCnt)*rect_height), CV_8UC3);
    }


  void Update() {
    draw_color_vec_.erase(draw_color_vec_.begin() + kColCnt-1);
    for(int i = 0; i < kColCnt; i++) {
      if (draw_color_vec_.back()[i] == 0)
        draw_color_vec_.back()[i] = 2;
    }
    srand((unsigned)time(NULL));
    std::vector<int> tmp(4, 1);
    std::mt19937 rng;
    rng.seed(std::random_device()());
    std::uniform_int_distribution<int> dist4(0, 3);
    int black_block_index = dist4(rng);

    tmp[black_block_index] = 0;
    draw_color_vec_.insert(draw_color_vec_.begin(), tmp);
    DrawCanvas();
  }

  cv::Mat GetCanvs() {
    // DrawCanvas();
    return canvas_;
  }  
  cv::Mat DrawAndGetCanvs() {
    DrawCanvas();
    return canvas_;
  }

  cv::Rect CurrentBlackBlock() {
    // std::cout<< "in get block" <<std::endl;
    // OutputColorIndex();
    auto black_index_iterator = find_if(draw_color_vec_[2].begin(), draw_color_vec_[2].end(), [](int i) { return i == 0;});
    int black_index = std::distance(draw_color_vec_[2].begin(), black_index_iterator);
    std::cout<< "black index "<<black_index <<std::endl;
    cv::Point black_topleft_point = rect_topleft_point_vvec_[2][black_index];
    return cv::Rect(black_topleft_point.x, black_topleft_point.y, rect_width_, rect_height_);
  }
  void OutputColorIndex() {
    for (int i = 0; i < kRowCnt; i++) {
      for (int j =0 ; j < kColCnt; j++) {
        std::cout << draw_color_vec_[i][j] << " ";
      }
      std::cout<< std::endl;
    }
    std::cout << "_________________________" << std::endl;
  }
  private:

    void DrawCanvas() {
      const int kEdgeLeaveDistance = 2;
      canvas_.setTo(0);
      for (int r=0; r < kRowCnt; r++) {
        for (int c = 0; c < kColCnt; c++) {
          cv::Rect draw_rect(rect_topleft_point_vvec_[r][c].x + kEdgeLeaveDistance,
          rect_topleft_point_vvec_[r][c].y + kEdgeLeaveDistance, rect_width_ - 2* kEdgeLeaveDistance,
          rect_height_ - 2*kEdgeLeaveDistance);
            cv::rectangle(canvas_, draw_rect, color_[draw_color_vec_[r][c]], -1);  // 
        }      
      }
    }
    // 产生均匀的随机数


  private:
      std::vector<std::vector<cv::Point>> rect_topleft_point_vvec_;
      std::vector<std::vector<int>> draw_color_vec_;  // 暂时支持列数为4
      cv::Mat canvas_;
      int rect_width_;
      int rect_height_;
      std::vector<cv::Scalar> color_{cv::Scalar::all(0), cv::Scalar::all(255), cv::Scalar::all(123)};

      static const int kRowCnt = 4;
      static const int kColCnt = 4;
};

void MouseHandle(int event, int x, int y, int flags, void* param) {
  AwayWhiteBlock* game = (AwayWhiteBlock*)param;  // 必须是指针变量，否则无法改变原对象
  // AwayWhiteBlock game2 = *(AwayWhiteBlock*)param;  // game2为新对象
  if (event == cv::MouseEventTypes::EVENT_LBUTTONUP) {

    cv::Rect current_block_rect = game->CurrentBlackBlock();
    std::cout << current_block_rect << std::endl;
    std::cout << x << " "<< y <<std::endl;
    if (current_block_rect.contains({x, y})) {

      game->Update();
      cv::Mat canvs = game->GetCanvs();
      cv::imshow("game", canvs);
      cv::waitKey(20);
    } else {
      // TODO: 错误处理
      std::cout <<"click wrong block" <<std::endl;
      cv::Mat canvs = game->GetCanvs();
      canvs.setTo(0);
      cv::putText(canvs, "click error", cv::Point(50,200),1, 3, cv::Scalar::all(255));
      cv::imshow("game", canvs);

    }
  }

}

int main() {
    
    AwayWhiteBlock game(80, 120);
    std::string win_name = "game";
    cv::namedWindow(win_name);

    cv::setMouseCallback(win_name, MouseHandle, (void*)&game);
    cv::Mat canv = game.DrawAndGetCanvs();

    cv::imshow(win_name, canv);
    cv::waitKey(0);
    return 0;
}