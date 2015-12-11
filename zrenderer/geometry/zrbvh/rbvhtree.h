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

#ifndef _zrbvh_rbvhtree_h_
#define _zrbvh_rbvhtree_h_

#include <zrenderer/geometry/zrbvh/types.h>
#include <zrenderer/geometry/zrbvh/api.h>

namespace zrenderer
{

class RBVHTree
{
public:
    typedef uint32_t NodeID;

    enum NodeType
    {
        NT_TRIANGLES,
        NT_HEIGHTMAP,
        NT_INTERNAL
    };

    ZRBVH_API RBVHTree( ConstMeshPtr mesh, float rho, float alpha, 
        float upperLimit );

    ZRBVH_API RBVHTree( const boost::filesystem::path& filename );
    
    ZRBVH_API virtual ~RBVHTree();

    ZRBVH_API NodeID getRootNode() const;
    
    ZRBVH_API NodeID getLeftChild( NodeID node ) const;
    
    ZRBVH_API NodeID getRightChild( NodeID node ) const;
    
    ZRBVH_API bool isLeaf( NodeID node ) const;

    ZRBVH_API NodeType getNodeType( NodeID node ) const;

    ZRBVH_API const AlignedBox3h& getBoundingBox( NodeID node ) const;
        
    ZRBVH_API void getHeightmap( NodeID node ) const;
    
    ZRBVH_API const Matrix3x4f& getHeightmapTransform( NodeID node ) const;
    
    ZRBVH_API void save( const boost::filesystem::path& filename ) const;

private:

    struct Impl;
    std::unique_ptr<Impl> _impl;
};

}
#endif // _zrbvh_rbvhtree_h_