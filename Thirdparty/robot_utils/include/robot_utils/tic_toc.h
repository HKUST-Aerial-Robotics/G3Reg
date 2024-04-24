/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef SRC_TIC_TOC_H
#define SRC_TIC_TOC_H

#include <chrono>

namespace robot_utils {
    class TicToc {
    public:
        TicToc() {
            tic();
            duration_ms = 0;
        }

        void tic() {
            start = std::chrono::steady_clock::now();
        }

        double toc() {
            end = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed_seconds = end - start;
            duration_ms = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
            tic();

            return duration_ms;
        }

        double time_used() {
            return duration_ms;
        }

    private:
        std::chrono::steady_clock::time_point start, end;
        double duration_ms;
    };
}


#endif //SRC_TIC_TOC_H
