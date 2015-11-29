#include <zrenderer/scenegraph/node.h>

namespace zrenderer
{

struct Node::Impl
{
    Impl( const std::string& name,
          NodeDataPtr nodeData,
          SceneGraph& sceneGraph )
        : _name( name )
        , _nodeData( nodeData )
        , sceneGraph( sceneGraph )
            {}
    ~Impl()
    {
        _sceneGraph.removeChild( _name );
    }

    ConstNodePtr getParent() const
    {
        return _sceneGraph.getParent( *this );
    }

    void addChild( NodePtr node )
    {
        _sceneGraph.addChild( *this, *node );
    }

    void removeChild( NodePtr node )
    {
        _sceneGraph.removeChild( *this );
    }

    ConstNodePtrs getChildren( const Filter& filter ) const;
    NodePtrs getChildren( const Filter& filter );

    SceneGraph& _sceneGraph;
    const std::string name;
    NodeDataPtr _nodeData;
};

Node::Node( const std::string& name,
            NodeDataPtr nodeData,
            SceneGraph& sceneGraph )
    : _impl( new Node::Impl( name,
                             nodeData,
                             sceneGraph ))
{

}

Node::~Node() {}

NodeDataPtr Node::getNodeData()
{
    return _impl->_nodeData;
}

ConstNodeDataPtr Node::getNodeData() const
{
    return _impl->_nodeData;
}


}
