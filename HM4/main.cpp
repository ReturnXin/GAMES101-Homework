#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4)
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
                  << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window)
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001)
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                     3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t)
{
    // TODO: Implement de Casteljau's algorithm
    if (control_points.size() == 1)
        return control_points[0];

    std::vector<cv::Point2f> new_points;
    for (int i = 0; i < control_points.size() - 1; i++)
    {
        cv::Point2f point1 = control_points[i], point2 = control_points[i + 1];
        cv::Point2f new_point(point1.x * t + point2.x * (1 - t), point1.y * t + point2.y * (1 - t));
        new_points.push_back(new_point);
    }
    return recursive_bezier(new_points, t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window)
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's
    // recursive Bezier algorithm.
    for (double i = 0; i <= 1; i += 0.0001)
    {
        auto point = recursive_bezier(control_points, i);

        cv::Point2i p0(point.x - std::floor(point.x) < 0.5 ? std::floor(point.x) - 1 : floor(point.x), //
                       point.y - std::floor(point.y) < 0.5 ? std::floor(point.y) - 1 : floor(point.y));
        std::vector<cv::Point2i> multiSample_points = {p0,                          //
                                                       cv::Point2i(p0.x, p0.y + 1), //
                                                       cv::Point2i(p0.x + 1, p0.y), //
                                                       cv::Point2i(p0.x + 1, p0.y + 1)};

        std::vector<double> distance;
        double sum_d = 0;
        for (int i = 0; i < 4; i++)
        {
            auto sample_point = multiSample_points[i];
            double d = sqrt(pow(sample_point.x + 0.5 - point.x, 2) + pow(sample_point.y + 0.5 - point.y, 2));
            distance.push_back(d);
            sum_d += d;
        }

        for (int i = 0; i < 4; i++)
        {
            auto sample_point = multiSample_points[i];
            double k = distance[i] / sum_d;
            window.at<cv::Vec3b>(sample_point.y, sample_point.x)[1] = std::min(255., window.at<cv::Vec3b>(sample_point.y, sample_point.x)[1] + k * 255.f);
        }
    }
}

int main()
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27)
    {
        for (auto &point : control_points)
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4)
        {
            // naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

    return 0;
}
