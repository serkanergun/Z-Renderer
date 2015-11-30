/* Copyright (c) 2015, Zombie Rendering
 *                     serkan.ergun@gmail.com
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