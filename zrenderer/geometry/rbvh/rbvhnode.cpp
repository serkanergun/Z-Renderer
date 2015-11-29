
#include <zrenderer/geometry/rbvh/rbvhnode.h>
#include <zrenderer/common/mesh.h>

namespace zrenderer
{

struct RBVHTriangle
{
    AlignedBox3f bound;
    Vector3f normal;
    float surfArea;
    int index;
};

struct RBVHNode::Impl
{
    Impl( ConstMeshPtr mesh )
    {
        const Vector3fs& vertices = mesh->getVertices();
        const Vector3uis& faces = mesh->getFaces();

        RBVHNodePtrs stack;
    }

    Impl() {}
};

RBVHNode::RBVHNode( ConstMeshPtr mesh )
    : _impl( new RBVHNode::Impl( mesh ) )
{}

RBVHNode::RBVHNode()
    : _impl( new RBVHNode::Impl() )
{}

RBVHNode::~RBVHNode() {}

}