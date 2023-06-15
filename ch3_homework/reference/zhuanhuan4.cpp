#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <fstream>

// 将雷达坐标点转换为相机坐标点
std::vector<Eigen::Vector3d> convertToCameraCoordinates(const std::vector<Eigen::Vector3d>& radar_points,
                                                       const Eigen::Isometry3d T1
                                                       )
{
    std::vector<Eigen::Vector3d> camera_coordinates;

    for (const auto& radar_point : radar_points) {
        // 雷达坐标系到相机坐标系的转换
        Eigen::Vector3d camera_point = T1 * radar_point;
        camera_point.x()=camera_point.x()/camera_point.z();
        // 存储相机坐标
        camera_coordinates.push_back(camera_point);
    }

    return camera_coordinates;
}

// 将相机坐标点转换为像素坐标点
std::vector<Eigen::Vector2i> convertToPixelCoordinates(const std::vector<Eigen::Vector3d>& camera_points,
                                                      const Eigen::Matrix3d& intrinsic_matrix
                                                     )
{
    std::vector<Eigen::Vector2i> pixel_coordinates;

    for (const auto& camera_point : camera_points) {
        // 相机坐标系到像素坐标系的转换
        // Eigen::Vector2d pixel_point =intrinsic_matrix* camera_point ;

        // 将像素坐标映射到图像尺寸
        double u = camera_point(0)*intrinsic_matrix(0,0)+intrinsic_matrix(0,2);
        double v = camera_point(1)*intrinsic_matrix(1,1)+intrinsic_matrix(1,2);

        // 存储像素坐标
        pixel_coordinates.emplace_back(u, v);
    }

    return pixel_coordinates;
}

int main() {
    // 定义雷达坐标点
    std::vector<Eigen::Vector3d> radar_points;
    radar_points.emplace_back(10.0, 5.0, 20.0);
    radar_points.emplace_back(-2.0, 8.0, 15.0);
    radar_points.emplace_back(6.0, -3.0, 12.0);
    std::ifstream file("/home/wkw/rotation/Practical_Homework_for_slambook14/ch3_homework/reference/pose.txt");
    std::ofstream fout("/home/wkw/rotation/Practical_Homework_for_slambook14/ch3_homework/reference/uvresult.txt");
    // 定义雷达坐标系到相机坐标系的转换参数
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix << 0.866, -0.5, 0,
                       0.5, 0.866, 0,
                       0, 0, 1;

    Eigen::Vector3d translation_vector(1.0, 2.0, 3.0);
    Eigen::Quaterniond q = Eigen::Quaterniond(rotation_matrix);
    std::cout<<q.coeffs().transpose()<<std::endl;

     Eigen::Isometry3d T = Eigen::Isometry3d::Identity();                // 虽然称为3d，实质上是4＊4的矩阵
     T.rotate(q);                                     // 按照rotation_vector进行旋转 xyzw
     T.pretranslate(translation_vector); 

    double data[7] = {0};
    Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();


        
    for (int i = 0; i < 1; i++) {
        for (auto &d : data) 
        {
            file >> d;
        }
        
        Eigen::Quaterniond quat(data[6], data[3], data[4], data[5]);
        Eigen::Vector3d vec(data[0], data[1], data[2]);
        
        std::cout << quat.coeffs().transpose() << std::endl;
        std::cout << vec.transpose() << std::endl;
        std::cout << data[0] << std::endl;
        
        T1.rotate(quat);
        T1.pretranslate(vec);
    }

        
        
    


    // 定义相机的内参矩阵参数
    Eigen::Matrix3d intrinsic_matrix;
    intrinsic_matrix << 1000, 0, 320,
                        0, 1000, 240,
                        0, 0, 1;

    // 定义图像的尺寸
    int image_width = 640;
    int image_height = 480;

    // 转换为相机坐标
    std::vector<Eigen::Vector3d> camera_coordinates = convertToCameraCoordinates(radar_points, T1);

    // 转换为像素坐标
    std::vector<Eigen::Vector2i> pixel_coordinates = convertToPixelCoordinates(camera_coordinates, intrinsic_matrix);

    // 输出像素坐标
    for (const auto& pixel_coordinate : pixel_coordinates) {
        std::cout << "Pixel Coordinate: (" << pixel_coordinate.x() << ", " << pixel_coordinate.y() << ")" << std::endl;
        fout<<pixel_coordinate.x()<<"  "<<pixel_coordinate.y()<<std::endl;
    }
    fout.close();

    return 0;
}
