#include <iostream>
#include <Eigen/Dense>
#include <vector>

int main() {
    // 定义雷达坐标系中的点的列表
    std::vector<Eigen::Vector3d> radar_points;
    radar_points.push_back(Eigen::Vector3d(10.0, 5.0, 20.0));
    radar_points.push_back(Eigen::Vector3d(-2.0, 8.0, 15.0));
    // 添加更多雷达坐标系中的点...

    // 定义相机坐标系的旋转矩阵（3x3）
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix << 0.866, -0.5, 0,
                       0.5, 0.866, 0,
                       0, 0, 1;

    // 定义相机坐标系的平移向量
    Eigen::Vector3d translation_vector(1, 2, 3);

    // 定义相机内参矩阵（3x3）
    Eigen::Matrix3d intrinsic_matrix;
    intrinsic_matrix << 1000, 0, 320,
                        0, 1000, 240,
                        0, 0, 1;

    // 存储转换后的像素坐标
    std::vector<Eigen::Vector2i> pixel_coordinates;

    // 循环处理每个雷达坐标系中的点
    for (const auto& radar_point : radar_points) {
        // 进行坐标系转换
        Eigen::Vector3d camera_point = rotation_matrix * radar_point + translation_vector;
        camera_point(0)=camera_point(0)/camera_point(2);
        camera_point(1)=camera_point(1)/camera_point(2);
        camera_point(2) = 1.0;
        // 进行像素坐标系转换
        Eigen::Vector3d pixel_coordinates_3d = intrinsic_matrix * camera_point;
        // int pixel_x = static_cast<int>(pixel_coordinates_3d(0));
        // int pixel_y = static_cast<int>(pixel_coordinates_3d(1) );
        int pixel_x = pixel_coordinates_3d(0);
        int pixel_y = pixel_coordinates_3d(1);
        // 存储像素坐标
        pixel_coordinates.push_back(Eigen::Vector2i(pixel_x, pixel_y));
    }

    // 输出像素坐标
    for (const auto& pixel_coord : pixel_coordinates) {
        std::cout << "Pixel coordinates: (" << pixel_coord.x() << ", " << pixel_coord.y() << ")" << std::endl;
    }

    return 0;
}
