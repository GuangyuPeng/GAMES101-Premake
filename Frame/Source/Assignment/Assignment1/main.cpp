#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0],
                 0, 1, 0, -eye_pos[1],
                 0, 0, 1, -eye_pos[2],
                 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    float rot_rad = rotation_angle / 180.0 * MY_PI;
    Eigen::Matrix4f translate;
    translate << cos(rot_rad), -sin(rot_rad), 0, 0,
                 sin(rot_rad), cos(rot_rad), 0, 0,
                 0, 0, 1, 0,
                 0, 0, 0, 1;
    model = translate * model;

    return model;
}

Eigen::Matrix4f get_model_matrix(Vector3f axis, float angle)
{
    // Create the model matrix for rotating the triangle around any axis from zero point.
    // Then return it.
    float rot_rad = angle / 180.0 * MY_PI;
    Eigen::Matrix3f model3f = Eigen::Matrix3f::Identity();
    model3f = cos(rot_rad) * model3f + (1.0 - cos(rot_rad)) * axis * axis.transpose();
    Eigen::Matrix3f tmp3f;
    tmp3f << 0, -axis.z(), axis.y(),
             axis.z(), 0, -axis.x(),
             -axis.y(), axis.x(), 0;
    model3f += sin(rot_rad) * tmp3f;
   
    Eigen::Matrix4f model;
    model << model3f(0, 0), model3f(0, 1), model3f(0, 2), 0,
             model3f(1, 0), model3f(1, 1), model3f(1, 2), 0,
             model3f(2, 0), model3f(2, 1), model3f(2, 2), 0,
             0, 0, 0, 1;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // Create the projection matrix for the given parameters.
    // Then return it.
    float fov_rad = eye_fov / 180.0 * MY_PI;
    float tan_fov_2 = tan(fov_rad / 2.0);
    Eigen::Matrix4f translate;
    translate << -1.0 / (tan_fov_2 * aspect_ratio), 0, 0, 0,
                 0, -1.0 / tan_fov_2, 0, 0,
                 0, 0, (zNear + zFar) / (zFar - zNear), 2 * zNear * zFar / (zFar - zNear),
                 0, 0, 1, 0;
    projection = translate * projection;

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};
    Eigen::Vector3f axis = { 1, 1, 0 };

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        //r.set_model(get_model_matrix(angle));
        r.set_model(get_model_matrix(axis, angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        //r.set_model(get_model_matrix(angle));
        r.set_model(get_model_matrix(axis, angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
