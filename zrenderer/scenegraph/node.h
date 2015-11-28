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

#ifndef _node_h_
#define _node_h_

#include <zrenderer/scenegraph/types.h>

namespace zrenderer
{

/**
 * Node class represents the scene graph nodes. It
 * carries the corresponding data.( Material, Mesh,
 * Camera, etc ). Nodes can be created through the
 * scene graph, supplied the data.
 */
class Node
{

public:

    NodeDataPtr getNodeData();
    ConstNodeDataPtr getNodeData() const;

    ConstNodePtr getParent() const;

    void addChild( NodePtr node );
    void removeChild( NodePtr node );

    ConstNodePtrs getChildren( const Filter& filter ) const;
    NodePtrs getChildren( const Filter& filter );

private:

    friend class SceneGraph;

    Node( const std::string& name,
          NodeDataPtr nodeData,
          SceneGraph& sceneGraph );

    virtual ~Node();

    struct Impl;
    std::unique_ptr<Impl> _impl;
};

}

#endif // _node_h_
