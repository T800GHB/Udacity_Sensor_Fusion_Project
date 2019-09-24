#ifndef dataStructures_h
#define dataStructures_h

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

struct LidarPoint { // single lidar point in space
    double x,y,z,r; // x,y,z in [m], r is point reflectivity
};


#endif /* dataStructures_h */
