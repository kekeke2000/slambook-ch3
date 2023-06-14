#include <iostream>
#include <vector>
#include <Eigen/Dense>

// 将雷达坐标点转换为像素坐标点
std::vector<Eigen::Vector2d> convertToPixelCoordinates(const std::vector<Eigen::Vector3d>& radar_points,
                                                      const Eigen::Matrix3d& rotation_matrix,
                                                      const Eigen::Matrix3d& translation_vector,
                                                      const Eigen::Matrix3d& intrinsic_matrix
                                                      )
{
    std::vector<Eigen::Vector2d> pixel_coordinates;

    for (const auto& radar_point : radar_points) {
        // 雷达坐标系到像素坐标系的转换
         Eigen::Vector3d transformed_point = rotation_matrix * radar_point + translation_vector;
        // Eigen::Vector3d transformed_point =translation_vector;
        // 归一化坐标系
        double x_n = transformed_point(0) / transformed_point(2);
        double y_n = transformed_point.y() / transformed_point.z();

        // 像素坐标系
        double u = intrinsic_matrix(0, 0) * x_n + intrinsic_matrix(0, 2);
        double v = intrinsic_matrix(1, 1) * y_n + intrinsic_matrix(1, 2);

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

    // 定义雷达坐标系到像素坐标系的转换矩阵
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix << 0.866, -0.5, 0,
                       0.5, 0.866, 0,
                       0, 0, 1;
    Eigen::Vector3d translation_vector(1, 1, 3);

    // 定义相机的内参矩阵参数
    Eigen::Matrix3d intrinsic_matrix;
    intrinsic_matrix << 1000, 0, 320,
                        0, 1000, 240,
                        0, 0, 1;

    // 定义图像的尺寸
    // int image_width = 640;
    // int image_height = 480;

    // 转换为像素坐标
    std::vector<Eigen::Vector2d> pixel_coordinates = convertToPixelCoordinates(radar_points, rotation_matrix,translation_vector,
                                                                                intrinsic_matrix);

    // 输出像素坐标
    for (const auto& pixel_coordinate : pixel_coordinates) {
        std::cout << "Pixel Coordinate: (" << pixel_coordinate.x() << ", " << pixel_coordinate.y() << ")" << std::endl;
    }

    return 0;
}
