#pragma once

namespace slam
{
class RRTStar
{
public:
    RRTStar(const cv::Mat &map, const Coordinate &A, const Coordinate &B,
            int radius = 10);
    ~RRTStar() = default;

    bool pathfind(cv::Mat *canvas);

    std::vector<Coordinate> recover_path();
};

}  // namespace slam