
#include <vector>
#include <numeric>

#include <opencv2/opencv.hpp>


using namespace std;
using namespace cv;


// Finding closest points
float flann_knn(std::vector<cv::Point2d>& ref_pts, std::vector<cv::Point2d>& to_reg_pts, vector<int>& ptpairs) {
    // find nearest neighbors using FLANN
  ptpairs.clear();
  auto kdtree = flann::GenericIndex<flann::L2<double>>(Mat(ref_pts).reshape(1), cvflann::KDTreeIndexParams{2});

  cv::Mat index(to_reg_pts.size(), 1, CV_32S);
  cv::Mat dist_mat(to_reg_pts.size(), 1, CV_64F);
 
  kdtree.knnSearch(cv::Mat(to_reg_pts).reshape(1), index, dist_mat, 1, cvflann::SearchParams{});

  for (int i = 0; i < index.rows; ++i) {
      ptpairs.push_back(index.at<int>(i, 0));
  }

  float dist = 0;
  for (int i = 0; i < ptpairs.size(); ++i) {
    dist += cv::norm(to_reg_pts[i] - ref_pts[ptpairs[i]]);
  }
  return dist;
}

// 有错误, Compute transform
void findBestReansformSVD(Mat& _m, Mat& _d) {
    Mat m; _m.convertTo(m,CV_32F);
    Mat d; _d.convertTo(d,CV_32F);
    Scalar d_bar = mean(d);
    Scalar m_bar = mean(m);
    Mat mc = m - m_bar;
    Mat dc = d - d_bar;
    mc = mc.reshape(1); dc = dc.reshape(1);
    Mat H(2,2,CV_32FC1);
    for(int i=0;i<mc.rows;i++) {
        Mat mci = mc(Range(i,i+1),Range(0,2));
        Mat dci = dc(Range(i,i+1),Range(0,2));
        H = H + mci.t() * dci;
    }
    cv::SVD svd(H);
    Mat R = svd.vt.t() * svd.u.t();
    double det_R = cv::determinant(R);
    if(abs(det_R + 1.0) < 0.0001) {
        float _tmp[4] = {1,0,0,cv::determinant(svd.vt*svd.u)};
        R = svd.u * Mat(2,2,CV_32FC1,_tmp) * svd.vt;
    }
// #ifdef BTM_DEBUG
    //for some strange reason the debug version of OpenCV is flipping the matrix
    R = -R;
// #endif
    float* _R = R.ptr<float>(0);
    Scalar T(d_bar[0] - (m_bar[0]*_R[0] + m_bar[1]*_R[1]),d_bar[1] - (m_bar[0]*_R[2] + m_bar[1]*_R[3]));
    m = m.reshape(1);
    m = m * R;
    m = m.reshape(2);
    m = m + T;// + m_bar;
    m.convertTo(_m,CV_32S);
}


cv::Point2d getMean (std::vector<cv::Point2d> &pt) {
	cv::Point2d mean(0,0);
	for (unsigned i=0;i<pt.size();++i)
		mean += pt[i];
	mean /= (double)pt.size();
	return mean;
}


void findRT(std::vector<cv::Point2d>& src, std::vector<cv::Point2d>& closestPt) {
	cv::Point2d mean_closest = getMean(closestPt);
	cv::Point2d mean_src = getMean(src);
cv::Point2d src2closest(0,0), src2closest_inv(0,0);
	for (unsigned i=0;i<src.size();++i) {
		src2closest.x     += (src[i].x - mean_src.x)*(closestPt[i].x - mean_closest.x);
		src2closest.y     += (src[i].y - mean_src.y)*(closestPt[i].y - mean_closest.y);
		src2closest_inv.x += (src[i].y - mean_src.y)*(closestPt[i].x - mean_closest.x);
		src2closest_inv.y += (src[i].x - mean_src.x)*(closestPt[i].y - mean_closest.y);
	}

	// key:
	// After centering at each mean, every point lives in its own coordinate.
	// Suppose the rotation we want to compute is r.
	// r = r_closest - r_src.
	// Here, r_closest is the shifted closest points' coordinate relative to the global coordinate
	// Here, r_src is the shifted source points' coordinate relative to the global coordinate
	// the formula below is an approximation to r.
	// r = atan2(y'/x') - atan2(y/x), (x, y) is the source coordinate and (x', y') is the closest points' coordinate.
	// app(r) = (atan2(y'/x') - atan2(y/x))/(1+yy'/xx')
	// app(r) is proportional to r.
	double r = atan2(src2closest_inv.y, src2closest.x+src2closest.y)-
		atan2(src2closest_inv.x, src2closest.x+src2closest.y);
  
  cv::Point2d t;
	t.x = mean_closest.x - ((mean_src.x * cos(r)) - (mean_src.y * sin(r)));
	t.y = mean_closest.y - ((mean_src.x * sin(r)) + (mean_src.y * cos(r)));

  // apply rt inplace
  for (unsigned i = 0; i < src.size(); ++i) {
    float a = ((src[i].x - mean_src.x) * cos(r)) - ((src[i].y - mean_src.y)*sin(r)) + mean_src.x + t.x ;
    float b = ((src[i].x - mean_src.x) * sin(r)) + ((src[i].y - mean_src.y)*cos(r)) + mean_src.y + t.y ;
    float a1 = ((src[i].x) * cos(r)) - ((src[i].y)*sin(r)) + t.x ;
    float b1 = ((src[i].x) * sin(r)) + ((src[i].y)*cos(r)) + t.y ;
    src[i].x = a1;  // a1,b1 more accurate
    src[i].y = b1;
  }
}

float FarestPt(std::vector<cv::Point2d>& pts) {
  float max_dist_ = 0;
  for (auto& pt : pts) {
    for (auto& pt2 : pts) {
      float dist = cv::norm(pt - pt2);
      if (dist > max_dist_) {
        max_dist_ = dist;
      }
    }
  }
  return max_dist_;
}

int main() {
  int i, num_pts;
  cv::Mat image_base = cv::Mat::zeros(500, 800, CV_8UC3);
  cv::Mat image = cv::Mat::zeros(500, 800, CV_8UC3);
  std::vector<cv::Point2d> ref_points, new_points;
	num_pts = 200;

  double norm = 200;
  double a = 0;
	for( i=0; i<num_pts; i++ ) {
		double xx = (double)(norm/2.f)*cos(a);
		double yy = (double)(norm)*sin(a);
		double x = (double)(xx * cos(CV_PI/4) + yy *sin(CV_PI/4) +550);
		double y = (double)(xx * -sin(CV_PI/4) + yy *cos(CV_PI/4)+350);
		ref_points.push_back(cvPoint2D32f(x,y));
		cv::circle(image_base, cv::Point((int)x,(int)y),1,cv::Scalar(0,255,255),1);
		a += (double)(2*CV_PI/num_pts);
	}
	a = 0.;
	for( i=0; i< num_pts/5; i++ ) {
		double xx = (double)((norm/2.f)*cos(a));
		double yy = (double)((norm/1.)*sin(a));
		double x = (double)(xx * cos(-CV_PI/8) + yy *sin(-CV_PI/8) +150);
		double y = (double)(xx * -sin(-CV_PI/8) + yy *cos(-CV_PI/8)+250);
		new_points.push_back(cvPoint2D32f(x,y));
		a += (double)(2*CV_PI/(double)(num_pts/5));
	}
  // std::cout << "end " << std::endl;
  std::vector<int> pair;
  double lastDist = 1e20;
  cv::Mat lastGood;

   std::vector<cv::Point2d> x_pts;
   x_pts = new_points;

  for (auto pt : x_pts)
		cv::circle(image_base, pt, 1,cv::Scalar(255,255,0),1);

  while(true) {
      pair.clear();
      double dist = flann_knn(ref_points, x_pts, pair);
      image_base.copyTo(image);
      
      for (int i = 0; i < pair.size(); i++) {
        cv::circle(image,x_pts[i],1,cv::Scalar(125,255,0),1);
        cv::line(image, ref_points[pair[i]], x_pts[i], cv::Scalar(0,125,0));
      }
      if(lastDist <= dist) {
          cv::imwrite("./res.png", image);
          break;  //converged?
      }
      lastDist = dist;

      cout << "distance: " << dist << endl;
      std::vector<cv::Point2d> x_close_pts;
      for(int i = 0; i < pair.size(); i++) {
          x_close_pts.push_back(ref_points[pair[i]]);
      }
      findRT(x_pts, x_close_pts);
  }
  return 0;
}