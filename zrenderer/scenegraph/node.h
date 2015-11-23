#ifndef _node_h_
#define _node_h_

#include <zrenderer/scenegraph/types.h>

namespace zrenderer
{

/**
 * Node class is the base class for scenegraph nodes.
 */
class Node
{

public:

    Node() {}
    virtual ~Node() {}

private:

    struct Impl;
    std::unique_ptr<Impl> _impl;
};

}

#endif // _node_h_