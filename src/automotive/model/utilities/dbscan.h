//
// Created by carlosrisma on 14/11/24.
//

#ifndef NS3_DBSCAN_H
#define NS3_DBSCAN_H
#include <cassert>
#include <cstddef>
#include <span>
#include <vector>
#include <cstdlib>
namespace ns3 {


    struct point2
    {
        float x, y;
    };

    struct point3
    {
        float x, y, z;
    };

    auto dbscan(const std::vector< point2>& data, float eps, int min_pts) -> std::vector<std::vector<size_t>>;
    auto dbscan(const std::vector< point3>& data, float eps, int min_pts) -> std::vector<std::vector<size_t>>;



}

#endif //NS3_DBSCAN_H
