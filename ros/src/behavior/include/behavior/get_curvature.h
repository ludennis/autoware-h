#ifndef __GET_CURVATURE_H__
#define __GET_CURVATURE_H__

#include <vector>

namespace Behavior
{
    void GetCurvature(
        const std::vector<float> & x,
        const std::vector<float> & y,
        std::vector<float> & curvature);

}/* namespace Behavior */

#endif // __GET_CURVATURE_H__
