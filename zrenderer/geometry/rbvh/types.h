#ifndef _rbvh_types_
#define _rbvh_types_

#include <zrenderer/common/mathtypes.h>

namespace zrenderer
{
    class RBVHNode;

    typedef std::shared_ptr<RBVHNode> RBVHNodePtr;

    typedef std::vector<RBVHNodePtr> RBVHNodePtrs;
}

#endif // _rbvh_types_