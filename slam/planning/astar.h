#include "common.h"
#include "thirdparty/log.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <vector>

#include <opencv2/opencv.hpp>

namespace slam
{
namespace a_star
{
static constexpr unsigned MAX_QUEUE_SIZE =
    1e9 / sizeof(Coordinate);  // 1 gigabyte

template <typename T>
class AStar
{
public:
    AStar(const cv::Mat &map, const Coordinate &A, const Coordinate &B)
        : m_A(A),
          m_B(B),
          m_map(map),
          m_distances(map.size(), CV_64F,
                      cv::Scalar(std::numeric_limits<double>::max()))
    {
        m_heuristic = [&](const Coordinate &X) {
            return manhattan_distance(X, m_B);
        };

        m_comp = [&](const std::tuple<Coordinate, double> &X,
                     const std::tuple<Coordinate, double> &Y) {
            return std::get<1>(X) + m_heuristic(std::get<0>(X)) >
                   std::get<1>(Y) + m_heuristic(std::get<0>(Y));
        };

        m_q.push_back(std::tuple<Coordinate, double>{A, 0.0});
        std::push_heap(m_q.begin(), m_q.end(), m_comp);
    }

    ~AStar() = default;

    bool pathfind(cv::Mat *canvas)
    {
        if (m_used_up)
        {
            log_error("A* object must now be deallocated");
            return true;
        }

        Coordinate X{-1, -1};
        double dist;

        while (!m_q.empty())
        {
            if (m_q.size() > MAX_QUEUE_SIZE)
            {
                log_warn("A* pathfinder exceeded queue size");
                return true;
            }

            std::tie(X, dist) = m_q.front();
            std::pop_heap(m_q.begin(), m_q.end(), m_comp);
            m_q.pop_back();

            if (!within_boundaries(m_map, X)) continue;

            if (!m_map.at<T>(X.i, X.j)) continue;

            if (X == m_B)
            {
                m_success = true;
                return true;
            }

            if (dist >= m_distances.at<double>(X.i, X.j)) continue;

            m_distances.at<double>(X.i, X.j) = dist;

            for (const Coordinate &coord : adjacency_8(X))
            {
                if (canvas && within_boundaries(m_map, coord) &&
                    m_map.at<T>(coord.i, coord.j))
                {
                    cv::Vec3f &color = canvas->at<cv::Vec3f>(coord.i, coord.j);
                    color[0] = GREY[0];
                    color[1] = GREY[1];
                    color[2] = GREY[2];
                }

                double new_dist = dist;
                if (coord.i != X.i && coord.j != X.j)
                    new_dist += std::sqrt(2.0);
                else
                    new_dist += 1;

                m_q.push_back(std::tuple<Coordinate, double>(coord, new_dist));
                std::push_heap(m_q.begin(), m_q.end(), m_comp);
            }

            if (canvas)
            {
                cv::Vec3f &color = canvas->at<cv::Vec3f>(X.i, X.j);
                color[0] = GREEN[0];
                color[1] = GREEN[1];
                color[2] = GREEN[2];
            }

            return false;
        }

        m_success = false;
        m_used_up = true;

        return true;
    }

    std::vector<Coordinate> recover_path() const
    {
        if (!m_success) return {};

        std::vector<Coordinate> path;
        Coordinate next = m_B;
        while (next != m_A)
        {
            path.push_back(next);
            Coordinate next_;
            double dist = std::numeric_limits<double>::max();
            for (const Coordinate &coord : adjacency_8(next))
            {
                if (within_boundaries(m_map, coord) &&
                    m_distances.at<double>(coord.i, coord.j) < dist)
                {
                    next_ = coord;
                    dist = m_distances.at<double>(coord.i, coord.j);
                }
            }
            next = next_;
        }

        std::reverse(path.begin(), path.end());

        return path;
    }

private:
    bool m_success = false;
    bool m_used_up = false;

    Coordinate m_A;
    Coordinate m_B;
    const cv::Mat &m_map;

    cv::Mat m_distances;
    std::vector<std::tuple<Coordinate, double>> m_q;

    std::function<double(const Coordinate &X)> m_heuristic;
    std::function<bool(const std::tuple<Coordinate, double> &X,
                       const std::tuple<Coordinate, double> &Y)>
        m_comp;
};

}  // namespace a_star

}  // namespace slam