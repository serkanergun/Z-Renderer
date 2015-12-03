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

    ZRBVH_API RBVHNode( ConstMeshPtr mesh, float rho, float alpha, 
        float upperLimit );

    ZRBVH_API RBVHNode();

    ZRBVH_API virtual ~RBVHNode();

    ZRBVH_API bool isLeaf() const;

    ZRBVH_API const ConstRBVHNodePtr& getLeftChild() const;

    ZRBVH_API const ConstRBVHNodePtr& getRightChild() const;

    ZRBVH_API NodeType getType() const;

private:

    struct Impl;
    std::unique_ptr<Impl> _impl;
};

}
#endif // _rbvh_rbvhnode_h_