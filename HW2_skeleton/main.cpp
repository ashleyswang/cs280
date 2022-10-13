#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    auto cos_a = cos(rotation_angle * MY_PI / 180.0);
    auto sin_a = sin(rotation_angle * MY_PI / 180.0);
    Eigen::Matrix4f rotation;
    rotation << cos_a, -sin_a,  0.0,  0.0,
                sin_a,  cos_a,  0.0,  0.0,
                0.0,      0.0,  1.0,  0.0,
                0.0,      0.0,  0.0,  1.0;

    return rotation * model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f persp_to_ortho, translate, scale;
    // change to negative for -z coordinates
    float n = -zNear;
    float f = -zFar;
    persp_to_ortho <<    n,  0.0,   0.0,      0.0,
                       0.0,    n,   0.0,      0.0,
                       0.0,  0.0,  n + f,  -n * f,                   
                       0.0,  0.0,   1.0,      0.0;

    // define frustum
    float t = abs(n) * tan(eye_fov * MY_PI / 180.0 / 2);
    float r = aspect_ratio * t;
    float l = -r;
    float b = -t;

    // shift box to center
    translate <<  1.0,  0.0,  0.0, -(r + l) / 2.0,
                  0.0,  1.0,  0.0, -(t + b) / 2.0,
                  0.0,  0.0,  1.0, -(n + f) / 2.0,
                  0.0,  0.0,  0.0,  1.0;

    // scale box to view
    scale <<  2.0 / (r - l),            0.0,            0.0,  0.0,
                        0.0,  2.0 / (t - b),            0.0,  0.0,
                        0.0,            0.0,  2.0 / (n - f),  0.0,
                        0.0,            0.0,            0.0,  1.0;

    projection = scale * translate * persp_to_ortho;
    return projection;
}

Eigen::Matrix4f get_rotation(Eigen::Vector3f axis, float angle)
{
    float cos_a = cos(angle * MY_PI / 180.0);
    float sin_a = sin(angle * MY_PI / 180.0);

    // convert axis to 4d vector
    Eigen::Vector4f n;
    n << axis[0], axis[1], axis[2], 0.0;
    n.normalized();

    // use Rodrigues's rotation matrix 
    Eigen::Matrix4f identity = Eigen::Matrix4f::Identity();   
    Eigen::Matrix4f rotation;
    rotation <<   0.0,  -n[2],   n[1],  0.0,
                 n[2],    0.0,  -n[0],  0.0,
                -n[1],   n[0],    0.0,  0.0,
                  0.0,    0.0,    0.0,  0.0;
    rotation = cos_a * identity + (1 - cos_a) * n * n.transpose() + sin_a * rotation;
    rotation(3,3) = 1.0;  // replace bottom right corner

    return rotation;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";
    
    // Extra Credit
    Eigen::Vector3f axis;
    bool extra_credit = false;

    if (argc >= 3) {
        std::string option = argv[1];
        if (option == "-r") {
            command_line = true;
            angle = std::stof(argv[2]); // -r by default
            if (argc != 4) return 0;
            filename = std::string(argv[3]);
        }

        // USAGE: ./Rasterizer -v x y z
        if (option == "-v") {
            if (argc != 5) return 0;
            axis << std::stof(argv[2]), std::stof(argv[3]), std::stof(argv[4]);
            extra_credit = true;
        }
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{1, 0, -2}, {0, 2, -3}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
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

        r.set_model(extra_credit ? get_rotation(axis, angle) : get_model_matrix(angle));
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
