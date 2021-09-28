#include <opencv2/opencv.hpp>

#include <array>

static const cv::Scalar CYAN(1, 1, 0);
static const cv::Scalar BLUE(1, 0, 0);
static const cv::Scalar RED(0, 0, 1);
static const cv::Scalar GREEN(0, 1, 0);
static const cv::Scalar WHITE(1, 1, 1);
static const cv::Scalar GREY(0.85, 0.85, 0.85);

namespace ploy
{
struct Coordinate
{
    int i;
    int j;

    inline bool operator==(const Coordinate &other)
    {
        return i == other.i && j == other.j;
    }

    inline bool operator!=(const Coordinate &other)
    {
        return !(*this == other);
    }
};

std::array<Coordinate, 8> adjacency_8(const Coordinate &X)
{
    const std::array<Coordinate, 8> neighbourhood = {
        Coordinate{X.i + 1, X.j + 1}, Coordinate{X.i + 1, X.j - 1},
        Coordinate{X.i - 1, X.j + 1}, Coordinate{X.i - 1, X.j - 1},
        Coordinate{X.i + 1, X.j},     Coordinate{X.i, X.j + 1},
        Coordinate{X.i - 1, X.j},     Coordinate{X.i, X.j - 1}};

    return neighbourhood;
}

std::array<Coordinate, 4> adjacency_4(const Coordinate &X)
{
    const std::array<Coordinate, 4> neighbourhood = {
        Coordinate{X.i + 1, X.j}, Coordinate{X.i, X.j + 1},
        Coordinate{X.i - 1, X.j}, Coordinate{X.i, X.j - 1}};

    return neighbourhood;
}

namespace
{
inline bool within_boundaries(const cv::Mat &map, const Coordinate &X)
{
    return X.i < map.rows && X.j < map.cols && X.i >= 0 && X.j >= 0;
}

inline int manhattan_distance(const Coordinate &A, const Coordinate &B)
{
    return std::abs(A.i - B.i) + std::abs(A.j - B.j);
}

inline int euclidean_distance_squared(const Coordinate &A, const Coordinate &B)
{
    const int idiff = A.i - B.i;
    const int jdiff = A.j - B.j;

    return idiff * idiff + jdiff * jdiff;
}

inline double euclidean_distance(const Coordinate &A, const Coordinate &B)
{
    return std::sqrt(static_cast<double>(euclidean_distance_squared(A, B)));
}

}  // namespace

}  // namespace ploy