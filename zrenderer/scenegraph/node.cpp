#include <zrenderer/scenegraph/node.h>

namespace zrenderer
{

struct Node::Impl
{
    Impl()
        : variable1(0u)
        , variable2(1u)
    {}

    ~Impl() {}

    uint32_t variable1;
    uint32_t variable2;
};

Node::Node()
    : _impl( new Node::Impl() )
{}

Node::~Node() {}

}
