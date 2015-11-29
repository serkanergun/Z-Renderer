/* Copyright (c) 2015, Zombie Rendering
 *                     ahmetbilgili@gmail.com
 *
 * This file is part of Z-Renderer <https://github.com/ZombieRendering/Z-Renderer>
 *
 * This library is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License version 3.0 as published
 * by the Free Software Foundation.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <zrenderer/scenegraph/types.h>

#include <boost/graph/adjacency_list.hpp>

namespace zrenderer
{



typedef boost::adjacency_list< boost::listS,
                               boost::vecS,
                               boost::undirectedS,
                               boost::no_property,
                               NodePtr > Graph;

typedef Graph::vertex_descriptor NodeDescriptor;
typedef std::unordered_map< std::string, NodeDescriptor > NodeMap;


struct SceneGraph::Impl
{
    Impl( SceneGraph& sceneGraph )
        : _rootNode( new Node( ROOT_NODE,
                               NodeDataPtr(),
                               sceneGraph ))
    {
        const NodeDescriptor& nd =
                boost::add_vertex( _rootNode, _graph );
        _nodeMap[ ROOT_NODE ] = nd;
    }

    ~Impl() {}

    bool nodeExists( const std::string& name ) const
    {
        ReadLock readLock( _mutex );
        return _nodeMap.count( name ) == 1;
    }

    NodePtr createNode( const std::string& name,
                        NodeDataPtr nodeData )
    {
        if( nodeExists( name ) )
            return NodePtr();

        WriteLock writeLock( _mutex );
        NodePtr node( new Node( name,
                                nodeData,
                                sceneGraph ));

        _nodeMap[ name ] =
                boost::add_vertex( node, _graph );
        return node;
    }

    NodePtr findNode( const std::string& name ) const
    {
        ReadLock readLock( _mutex );
        NodeMap::iterator it = _nodeMap.find( name );
        if( it != _nodeMap.end( ))
            return _graph[ it->second ];

        return NodePtr();
    }

    bool removeNode( const std::string& name )
    {
        if( nodeExists( name ) )
            return false;

        WriteLock writeLock( _mutex );
        _nodeMap[ name ] =
                boost::add_vertex( node, _graph );
        boost::remove_vertex( _nodeMap[ name ] );
        _nodeMap.erase( name );
        return true;
    }

    bool addChild( const std::string& parent,
                   const std::string& child )
    {
        if( !nodeExists( parent ) ||
            !nodeExists( child ))
        {
            return false;
        }

        boost::add_edge( _nodeMap[ parent ],
                         _nodeMap[ child ] );
        return true;
    }

    NodePtr _rootNode;
    NodeMap _nodeMap;
    mutable ReadWriteMutex _mutex;
    Graph _graph;
};

ConstNodePtr getRoot() const
{
    return _impl->rootNode;
}

NodePtr getRoot()
{
    return _impl->rootNode;
}

NodePtr SceneGraph::createNode( const std::string& name,
                                NodeDataPtr nodeData )
{
    return _impl->createNode( name, nodeData );
}

NodePtr SceneGraph::findNode( const std::string& name ) const
{
    return _impl->findNode( name );
}

bool SceneGraph::removeNode( const std::string& name )
{
    return _impl->removeNode( name );
}

bool SceneGraph::addChild( const std::string& parent,
                           const std::string& child )
{
    return _impl->addChild( parent, child );
}

}

