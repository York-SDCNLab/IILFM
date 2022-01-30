#include "util.hpp"

void pose_estimation_3d3d(const vector<cv::Point3f>& pts1, const vector<cv::Point3f>& pts2, cv::Mat& R, cv::Mat& t){
	cv::Point3f p1, p2; // center of mass
    int N = pts1.size();
    for (int i=0; i<N; i++){
        p1 += pts1[i];
        p2 += pts2[i];
    }
    p1 = cv::Point3f(cv::Vec3f(p1) / N);
    p2 = cv::Point3f(cv::Vec3f(p2) / N);
    
    vector<cv::Point3f>  q1(N), q2(N); // remove the center
    for (int i=0; i<N; i++){
        q1[i] = pts1[i] - p1;
        q2[i] = pts2[i] - p2;
    }

    // compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (int i=0; i<N; i++){
        W += Eigen::Vector3d ( q1[i].x, q1[i].y, q1[i].z ) * Eigen::Vector3d ( q2[i].x, q2[i].y, q2[i].z ).transpose();
    }

    // SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd ( W, Eigen::ComputeFullU|Eigen::ComputeFullV );
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    
    if (U.determinant() * V.determinant() < 0){
        for (int x = 0; x < 3; ++x){
            U(x, 2) *= -1;
        }
	}

    Eigen::Matrix3d R_ = U* ( V.transpose() );
    Eigen::Vector3d t_ = Eigen::Vector3d ( p1.x, p1.y, p1.z ) - R_ * Eigen::Vector3d ( p2.x, p2.y, p2.z );

    // convert to cv::Mat
    R = ( cv::Mat_<double> ( 3,3 ) <<
          R_ ( 0,0 ), R_ ( 0,1 ), R_ ( 0,2 ),
          R_ ( 1,0 ), R_ ( 1,1 ), R_ ( 1,2 ),
          R_ ( 2,0 ), R_ ( 2,1 ), R_ ( 2,2 )
        );
    t = ( cv::Mat_<double> ( 3,1 ) << t_ ( 0,0 ), t_ ( 1,0 ), t_ ( 2,0 ) );
}

Eigen::Quaterniond rotation_to_quaternion(Eigen::Matrix3d R){  
    Eigen::Quaterniond q = Eigen::Quaterniond(R);  
    q.normalize();  
    return q;  
}
