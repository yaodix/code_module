
#include <vector>

#include <opencv2/opencv.hpp>


using namespace std;
using namespace cv;


// Finding closest points
float flann_knn(Mat& m_destinations, Mat& m_object, vector<int>& ptpairs, vector<float>& dists /*= vector<float>()*/) {
    // find nearest neighbors using FLANN
    cv::Mat m_indices(m_object.rows, 1, CV_32S);
    cv::Mat m_dists(m_object.rows, 1, CV_32F);
    Mat dest_32f; m_destinations.convertTo(dest_32f,CV_32FC2);
    Mat obj_32f; m_object.convertTo(obj_32f,CV_32FC2);
    assert(dest_32f.type() == CV_32F);
    cv::flann::Index flann_index(dest_32f, cv::flann::KDTreeIndexParams(2));  // using 2 randomized kdtrees
    flann_index.knnSearch(obj_32f, m_indices, m_dists, 1, cv::flann::SearchParams(64) );
    int* indices_ptr = m_indices.ptr<int>(0);
    //float* dists_ptr = m_dists.ptr<float>(0);
    for (int i=0;i<m_indices.rows;++i) {
        ptpairs.push_back(indices_ptr[i]);
    }
    dists.resize(m_dists.rows);
    m_dists.copyTo(Mat(dists));
    return cv::sum(m_dists)[0];
}

// Compute transform,有错误
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


cv::Point2f getMean (std::vector<cv::Point2f> &pt) {
	cv::Point2f mean(0,0);
	for (unsigned i=0;i<pt.size();++i)
		mean += pt[i];
	mean /= (double)pt.size();
	return mean;
}


void findRT(std::vector<cv::Point2f>& src, std::vector<cv::Point2f>& closestPt) {
	cv::Point2f mean_closest = getMean(closestPt);
	cv::Point2f mean_src = getMean(src);
cv::Point2f src2closest(0,0), src2closest_inv(0,0);
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
  
  cv::Point2f t;
	t.x = mean_closest.x - ((mean_src.x * cos(r)) - (mean_src.y * sin(r)));
	t.y = mean_closest.y - ((mean_src.x * sin(r)) + (mean_src.y * cos(r)));

  // apply rt inplace
		for (unsigned i=0;i<src.size();++i) {
			src[i].x = (src[i].x*cos(r)) - (src[i].y*sin(r)) + t.x;
			src[i].y = (src[i].x*sin(r)) + (src[i].y*cos(r)) + t.y;
    }
}


int main() {
  int i, num_pts;
  cv::Mat image_base = cv::Mat::zeros(500, 500, CV_8UC3);
  cv::Mat image = cv::Mat::zeros(500, 500, CV_8UC3);
  std::vector<cv::Point2f> ref_points, new_points;
	num_pts = 200;

  double norm = 200;
  float a = 0;
	for( i=0; i<num_pts; i++ ) {
		float xx = (float)(norm/2.f)*cos(a);
		float yy = (float)(norm)*sin(a);
		float x = (float)(xx * cos(CV_PI/4) + yy *sin(CV_PI/4) +250);
		float y = (float)(xx * -sin(CV_PI/4) + yy *cos(CV_PI/4)+250);
		ref_points.push_back(cvPoint2D32f(x,y));
		cv::circle(image_base, cv::Point((int)x,(int)y),1,cv::Scalar(0,255,255),1);
		a += (float)(2*CV_PI/num_pts);
	}
	a = 0.;
	for( i=0; i< num_pts/5; i++ ) {
		float xx = (float)((norm/1.9)*cos(a));
		float yy = (float)((norm/1.1)*sin(a));
		float x = (float)(xx * cos(-CV_PI/8) + yy *sin(-CV_PI/8) +150);
		float y = (float)(xx * -sin(-CV_PI/8) + yy *cos(-CV_PI/8)+250);
		new_points.push_back(cvPoint2D32f(x,y));
		a += (float)(2*CV_PI/(float)(num_pts/5));
		cv::circle(image_base,cv::Point((int)x,(int)y),1,cv::Scalar(255,255,0),1);
	}
  // std::cout << "end " << std::endl;
  cv::Mat destination(ref_points);
  cv::Mat X(new_points);
  std::vector<int> pair;
  std::vector<float> dists;
  double lastDist = 1e20;
  cv::Mat lastGood;

  destination = destination.reshape(1);
  X = X.reshape(1);
   std::vector<cv::Point2f> x_pts;
   x_pts.assign(new_points.begin(), new_points.end());
  while(true) {
      pair.clear(); dists.clear();
      double dist = flann_knn(destination, X, pair, dists);
      image_base.copyTo(image);
      
      for (int i = 0; i < pair.size(); i++) {
        cv::circle(image,x_pts[i],1,cv::Scalar(125,255,0),1);
        cv::line(image, ref_points[pair[i]], x_pts[i], cv::Scalar(0,125,0));
      }
      if(lastDist <= dist) {
          X = lastGood;
          cv::imwrite("./res.png", image);
          break;  //converged?
      }
      lastDist = dist;
      X.copyTo(lastGood);
      cout << "distance: " << dist << endl;
      Mat X_bar(X.size(),X.type());
      for(int i=0;i<X.rows;i++) {
          Point p = destination.at<Point>(pair[i], 0);
          X_bar.at<Point>(i,0) = p;
      }
      // ShowQuery(destination,X,X_bar);
      // X = X.reshape(2);
      // X_bar = X_bar.reshape(2);
      // findBestReansformSVD(X,X_bar);
      x_pts.clear();
      for (int i = 0; i < X.rows; i++) {
        x_pts.emplace_back(X.at<float>(i,0), X.at<float>(i,1));
      }    
      std::vector<cv::Point2f> x_close_pts;
      for (int i = 0; i < X_bar.rows; i++) {
        x_close_pts.emplace_back(X_bar.at<float>(i,0), X_bar.at<float>(i,1));
      }
      findRT(x_pts, x_close_pts);
      image_base.copyTo(image);
      X = Mat(x_pts);
      X = X.reshape(1); // back to 1-channel
  }
  return 0;
}
