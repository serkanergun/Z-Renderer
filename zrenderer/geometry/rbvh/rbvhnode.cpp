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

#define NOMINMAX

#include <oglplus/gl.hpp>
#include <oglplus/all.hpp>

#include <oglplus/images/image_spec.hpp>
#include <oglplus/bound/texture.hpp>
#include <oglplus/bound/framebuffer.hpp>
#include <oglplus/bound/renderbuffer.hpp>

#include <oglplus/opt/resources.hpp>
#include <oglplus/opt/list_init.hpp>

namespace zrenderer
{

struct RBVHTriangle
{
    AlignedBox3f bound;
    Vector3f normal;
    float surfArea;
    uint32_t faceIindex;
};

typedef std::shared_ptr<RBVHTriangle> RBVHTrianglePtr;
typedef std::vector<RBVHTrianglePtr> RBVHTriangles;
typedef std::shared_ptr<RBVHTriangles> RBVHTrianglesPtr;

struct RBVHNode::Impl
{
    Impl( ConstMeshPtr mesh, float rho, float alpha, float upperLimit ) :
        _triangles( new RBVHTriangles() )
    {
        const Vector3fs& vertices = mesh->getVertices();
        const Vector3uis& faces = mesh->getFaces();

        _triangles->reserve( faces.size() );
        uint32_t i = 0;
        for( Vector3ui face : faces )
        {
            const Vector3f& v0 = vertices[ face.x() ];
            const Vector3f& v1 = vertices[ face.y() ];
            const Vector3f& v2 = vertices[ face.z() ];

            Vector3f e1 = v1 - v0;
            Vector3f e2 = v2 - v0;

            RBVHTrianglePtr triangle( new RBVHTriangle() );

            triangle->bound.extend( v0 );
            triangle->bound.extend( v1 );
            triangle->bound.extend( v2 );
            triangle->normal = e1.cross( e2 );
            triangle->surfArea = 0.5f * triangle->normal.norm();
            triangle->normal /= 2.0f * triangle->surfArea;
            triangle->faceIindex = i++;

            _triangles->push_back( triangle );
        }

        calcProperties( 0, (uint32_t)_triangles->size(), rho, alpha,
            upperLimit );


        std::vector<RBVHNode::Impl*> stack;
        stack.push_back(  this  );
        while( stack.size() > 0 )
        {
            RBVHNode::Impl* node = stack.back();
            stack.pop_back();

            if (node->_type == NT_INTERNAL )
            {
                node->split( rho, alpha, upperLimit );
                stack.push_back( _left->_impl.get() );
                stack.push_back( _right->_impl.get() );
            }
            else if( node->_type == NT_HEIGHTMAP )
            {
                node->rasterize();
            }
        }
    }

    Impl()
    {
        _type = NT_EMPTY;
    }

    void calcProperties( uint32_t start, uint32_t count, float rho, float alpha,
        float upperLimit )
    {
        _startIdx = start;
        _count = count;
        float _sumArea = 0;
        Vector3f _sumAreaWeightedNormal;
        _bounds.setEmpty();

        for( uint32_t i = _startIdx; i < _startIdx + _count; ++i )
        {
            const RBVHTrianglePtr& tri = _triangles->at( i );
            _bounds.extend( tri->bound );
            _sumArea += tri->surfArea;
            _sumAreaWeightedNormal += tri->normal * tri->surfArea;
        }
        _R = _sumArea * sqrtf( rho / _sumAreaWeightedNormal.norm() );
        _rate = _sumAreaWeightedNormal.norm() / _sumArea;

        if( _count <= 8 )
            _type = NT_TRIANLGES;
        else if( _rate > alpha  && _R < upperLimit )
            _type = NT_HEIGHTMAP;
        else
            _type = NT_INTERNAL;
    }

    void split( float rho, float alpha, float upperLimit )
    {
        uint32_t side = 0;
        Vector3f sz = _bounds.sizes();
        for( uint32_t i = 1; i < 3; ++i )
            if( sz[ i ] > sz[ side ] ) side = i;

        std::sort( _triangles->begin() + _startIdx,
            _triangles->end() + _startIdx + _count,
            [&side]( const RBVHTrianglePtr& a, const RBVHTrianglePtr& b )
            {
                return a->bound.center()[ side ] < b->bound.center()[ side ];
            }
        );

        _left = RBVHNodePtr( new RBVHNode() );
        _right = RBVHNodePtr( new RBVHNode() );

        _left->_impl->_triangles = _triangles;
        _right->_impl->_triangles = _triangles;

        // Median Cut, other split strategies like SAH can be added
        uint32_t leftCount = _count / 2; 

        _left->_impl->calcProperties( _startIdx, leftCount, rho, alpha, 
            upperLimit );
        _right->_impl->calcProperties( _startIdx + leftCount, 
            _count  - leftCount, rho, alpha, upperLimit );
    }

    void rasterize()
    {
        oglplus::Context gl;
        oglplus::Texture tex;
        oglplus::Renderbuffer rbo;
        oglplus::Framebuffer fbo;

        int sz = 256;

        gl.Bound( oglplus::Texture::Target::Rectangle, tex )
            .MinFilter( oglplus::TextureMinFilter::Linear )
            .MagFilter( oglplus::TextureMagFilter::Linear )
            .WrapS( oglplus::TextureWrap::ClampToEdge )
            .WrapT( oglplus::TextureWrap::ClampToEdge )
            .Image2D( 0, oglplus::InternalFormat::R32F, sz, sz, 0,
                oglplus::Format::Red, oglplus::PixelDataType::Float, nullptr );


        gl.Bound( oglplus::Renderbuffer::Target::Renderbuffer, rbo )
            .Storage( oglplus::InternalFormat::DepthComponent32, sz, sz );

        gl.Bound( oglplus::Framebuffer::Target::Draw, fbo )
            .AttachTexture( oglplus::FramebufferAttachment::Color, tex, 0 )
            .AttachRenderbuffer( oglplus::FramebufferAttachment::Depth, rbo );

        (void)gl;
    }

    RBVHTrianglesPtr _triangles;
    uint32_t _startIdx, _count;
    float _R, _rate;
    AlignedBox3f _bounds;
    NodeType _type;
    RBVHNodePtr _left, _right;
};

RBVHNode::RBVHNode( ConstMeshPtr mesh, float rho, float alpha, float upperLimit )
    : _impl( new RBVHNode::Impl( mesh, rho, alpha, upperLimit ) )
{}

RBVHNode::RBVHNode()
    : _impl( new RBVHNode::Impl() )
{}

RBVHNode::~RBVHNode() {}

bool RBVHNode::isLeaf() const
{
    return getType() != NT_INTERNAL;
}

const ConstRBVHNodePtr& RBVHNode::getLeftChild() const
{
    return _impl->_left;
}

const ConstRBVHNodePtr& RBVHNode::getRightChild() const
{
    return _impl->_right;
}

RBVHNode::NodeType RBVHNode::getType() const
{
    return _impl->_type;
}

}