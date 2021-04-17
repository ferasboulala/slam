#include "raycast.h"
#include "thirdparty/log.h"

#include <opencv2/opencv.hpp>

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        log_error("usage : %s map_file", argv[0]);
        return -1;
    }

    cv::Mat map;
    map = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);

    return 0;
}
