#include <zrenderer/scenegraph/node.h>

namespace zrenderer
{

struct Node::Impl
{
    Impl() {}
    ~Impl() {}



};

Node::Node()
    : _impl( new Node::Impl() )
{}

Node::~Node() {}

}
