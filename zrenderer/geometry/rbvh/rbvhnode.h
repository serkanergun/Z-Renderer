#ifndef _rbvh_rbvhnode_h_
#define _rbvh_rbvhnode_h_

#include <zrenderer/geometry/rbvh/types.h>
#include <zrenderer/geometry/rbvh/api.h>

namespace zrenderer
{

class RBVHNode
{
public:

    enum NodeType
    {
        NT_TRIANLGES,
        NT_HEIGHTMAP,
        NT_EMPTY,
        NT_INTERNAL
    };

    ZRBVH_API RBVHNode( ConstMeshPtr mesh );

    ZRBVH_API RBVHNode();

    ZRBVH_API virtual ~RBVHNode();

    ZRBVH_API bool isLeaf() const;

    ZRBVH_API const RBVHNodePtr getLeftChild() const;

    ZRBVH_API const RBVHNodePtr getRightChild() const;

    ZRBVH_API NodeType getType() const;

private:

    struct Impl;
    std::unique_ptr<Impl> _impl;
};

}
#endif // _rbvh_rbvhnode_h_