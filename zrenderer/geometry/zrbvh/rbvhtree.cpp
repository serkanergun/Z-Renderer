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

#include <zrenderer/zcommon/mesh.h>
#include <zrenderer/zcommon/utils.h>
#include <zrenderer/geometry/zrbvh/rbvhtree.h>

#include <zrenderer/geometry/zrbvh/shaders/fragRaster.glsl.h>
#include <zrenderer/geometry/zrbvh/shaders/vertRaster.glsl.h>

#define NOMINMAX
#include <boost/filesystem/fstream.hpp>
#include <oglplus/gl.hpp>
#include <oglplus/all.hpp>
#include <oglplus/bound/framebuffer.hpp>
#include <oglplus/bound/renderbuffer.hpp>
#include <FreeImage.h>

namespace zrenderer
{

struct RBVHTriangle
{
    AlignedBox3f bound;
    Vector3f normal;
    float surfArea;
};

typedef std::vector<RBVHTriangle> RBVHTriangles;
typedef std::shared_ptr<RBVHTriangles> RBVHTrianglesPtr;
typedef std::vector<uint32_t> RBVHTriangleIndices;
typedef std::shared_ptr<RBVHTriangleIndices> RBVHTriangleIndicesPtr;

class RBVHNode
{
public:
    RBVHNode( ConstMeshPtr mesh, MeshPtr& triMesh, float rho, float alpha,
        float upperLimit )
        : _triangles( new RBVHTriangles() )
        , _triangleIndices( new RBVHTriangleIndices() )
    {
        const Vector3fs& vertices = mesh->getVertices();
        const Vector3uis& faces = mesh->getFaces();

        _triangles->reserve( faces.size() );
        _triangleIndices->reserve( faces.size() );

        uint32_t i = 0;
        for( Vector3ui face : faces )
        {
            const Vector3f& v0 = vertices[ face.x() ];
            const Vector3f& v1 = vertices[ face.y() ];
            const Vector3f& v2 = vertices[ face.z() ];

            Vector3f e1 = v1 - v0;
            Vector3f e2 = v2 - v0;

            _triangles->push_back( RBVHTriangle() );
            _triangleIndices->push_back( i++ );
            RBVHTriangle& triangle = _triangles->back();

            triangle.bound.extend( v0 );
            triangle.bound.extend( v1 );
            triangle.bound.extend( v2 );
            triangle.normal = e1.cross( e2 );
            triangle.surfArea = 0.5f * triangle.normal.norm();
            triangle.normal /= 2.0f * triangle.surfArea;
        }

        calcProperties( 0, (uint32_t)_triangles->size(), rho, alpha, upperLimit );

        i = 0;
        uint32_t j = 0;
        traverse(
            [&i, &j, &rho, &alpha, &upperLimit]( RBVHNode* node )
            {
                node->_nodeIdx = i++;
                switch( node->_type )
                {
                case RBVHTree::NT_INTERNAL:
                    node->split( rho, alpha, upperLimit );
                    break;
                case RBVHTree::NT_HEIGHTMAP:
                    node->_heightIdx = j++;
                    node->calcTransform();
                    break;
                }
            } );

        MeshPtr heightMesh;
        createMeshes( triMesh, heightMesh, vertices, faces );

        oglplus::VertexShader vertShader;
        oglplus::FragmentShader fragShader;
        oglplus::Program prog;
        oglplus::Buffer meshVertices, meshIndices;

        fragShader.Source( fragRaster_glsl ).Compile();
        vertShader.Source( vertRaster_glsl ).Compile();
        prog.AttachShader( vertShader ).AttachShader( fragShader );
        prog.Link();
        prog.Use();

        oglplus::Uniform<oglplus::Mat4x3f> projMatrixParam( prog, "projMatrix" );

        meshVertices.Bind( oglplus::Buffer::Target::Array );
        {
            oglplus::Buffer::Data( oglplus::Buffer::Target::Array, 
                heightMesh->getVertices() );

            oglplus::VertexArrayAttrib attr( prog, "vertPos" );
            attr.Setup<GLfloat>( 3 );
            attr.Enable();
        }

        meshIndices.Bind( oglplus::Buffer::Target::ElementArray );
        {
            oglplus::Buffer::Data( oglplus::Buffer::Target::ElementArray,
                heightMesh->getFaces() );
        }

        traverse(
            [&projMatrixParam]( RBVHNode* node )
            {
                if( node->_type == RBVHTree::NT_HEIGHTMAP )
                {
                    node->rasterize( projMatrixParam );
                }
            } );
    }

    ~RBVHNode() 
    {
        if ( _left ) delete _left;
        if ( _right ) delete _right;
    }

    void traverse( 
        const std::function< void( RBVHNode* ) >& lambda )
    {
        std::vector<RBVHNode*> stack;
        stack.push_back( this );

        while( stack.size() > 0 )
        {
            RBVHNode* node = stack.back();
            stack.pop_back();
            lambda( node );
            if( node->_type == RBVHTree::NT_INTERNAL )
            {
                stack.push_back( node->_right );
                stack.push_back( node->_left );
            }
        }
    }

    AlignedBox3f _bounds;
    RBVHTree::NodeType _type;
    RBVHNode *_left, *_right;
    uint32_t _startIdx, _count;
    uint32_t _nodeIdx, _heightIdx;
    Matrix3x4f _transform;
    std::vector<float> _heightmap;

private:
    RBVHNode( RBVHTrianglesPtr triangles, 
        RBVHTriangleIndicesPtr triangleIndices, uint32_t start, uint32_t count,
        float rho, float alpha, float upperLimit )
        : _left( nullptr ), _right( nullptr ), _triangles( triangles ), 
        _triangleIndices( triangleIndices )
    {
        calcProperties( start, count, rho, alpha, upperLimit );
    }

    void calcProperties( uint32_t start, uint32_t count, float rho, float alpha,
        float upperLimit )
    {
        _startIdx = start;
        _count = count;
        float _sumArea = 0;
        ;
        _bounds.setEmpty();
        _sumAreaWeightedNormal.setZero();

        for( uint32_t i = _startIdx; i < _startIdx + _count; ++i )
        {
            uint32_t triIdx = _triangleIndices->at( i );

            const RBVHTriangle& tri = _triangles->at( triIdx );
            _bounds.extend( tri.bound );
            _sumArea += tri.surfArea;
            _sumAreaWeightedNormal += tri.normal * tri.surfArea;
        }
        _R = _sumArea * sqrtf( rho / _sumAreaWeightedNormal.norm() );
        _rate = _sumAreaWeightedNormal.norm() / _sumArea;

        if( _count <= 8 )
            _type = RBVHTree::NT_TRIANGLES;
        else if( _rate > alpha  && _R < upperLimit )
            _type = RBVHTree::NT_HEIGHTMAP;
        else
            _type = RBVHTree::NT_INTERNAL;
    }

    void split( float rho, float alpha, float upperLimit )
    {
        uint32_t side = 0;
        Vector3f sz = _bounds.sizes();
        for( uint32_t i = 1; i < 3; ++i )
            if( sz[ i ] > sz[ side ] ) side = i;

        std::sort( _triangleIndices->begin() + _startIdx,
            _triangleIndices->begin() + _startIdx + _count,
            [&side, this]( const uint32_t& a, const uint32_t& b )
            {
                return _triangles->at( a ).bound.center()[ side ]
                    < _triangles->at( b ).bound.center()[ side ];
            }
        );

        // Median Cut, other split strategies like SAH can be added
        uint32_t leftCount = _count / 2;

        _left = new RBVHNode( _triangles, _triangleIndices, 
            _startIdx, leftCount, rho, alpha, upperLimit );

        _right = new RBVHNode( _triangles, _triangleIndices,
            _startIdx + leftCount, _count - leftCount, rho, alpha, 
            upperLimit );
    }

    void createMeshes(
        MeshPtr& triMesh,
        MeshPtr& heightMesh,
        const Vector3fs& vertices,
        const Vector3uis& faces )
    {
        std::set<uint32_t> usedVertices[2];
        std::map<uint32_t, uint32_t> indexMapping[2];
        uint32_t totalFaces[ 2 ] = {0, 0};

        traverse(
            [&usedVertices, &faces, &totalFaces, this]( RBVHNode* node )
            {
                uint32_t meshIdx = 0;
                switch( node->_type )
                {
                case RBVHTree::NT_TRIANGLES:
                    meshIdx = 0;
                    break;
                case RBVHTree::NT_HEIGHTMAP:
                    meshIdx = 1;
                    break;
                case RBVHTree::NT_INTERNAL:
                    return;
                }
                
                RBVHTriangleIndices::iterator
                    it1 = _triangleIndices->begin() + node->_startIdx,
                    it2 = _triangleIndices->begin() + node->_startIdx
                    + node->_count;

                totalFaces[ meshIdx ] += node->_count;
                for( auto it = it1; it != it2; ++it )
                {
                    uint32_t triangleIdx = *it;
                    const Vector3ui& triangle = faces[ triangleIdx ];
                    usedVertices[ meshIdx ].insert( triangle.x() );
                    usedVertices[ meshIdx ].insert( triangle.y() );
                    usedVertices[ meshIdx ].insert( triangle.z() );
                }
            } );

        MeshPtr meshes[ 2 ] = {MeshPtr( new Mesh() ), MeshPtr( new Mesh() )};
        for( uint32_t j = 0; j < 2; j++ )
        {
            uint32_t i = 0;
            meshes[ j ]->reserveVertices( (uint32_t)usedVertices[ j ].size() );
            for( uint32_t v : usedVertices[j] )
            {
                meshes[ j ]->addVertex( vertices[ v ] );
                indexMapping[ j ][ v ] = i++;
            }

            meshes[ j ]->reserveFaces( totalFaces[j] );
        }

        uint32_t lastIdx[ 2 ] = {0, 0};
        traverse(
            [&indexMapping, &faces, &lastIdx, &meshes, this]( RBVHNode* node )
        {
            uint32_t meshIdx = 0;
            switch( node->_type )
            {
            case RBVHTree::NT_TRIANGLES:
                meshIdx = 0;
                break;
            case RBVHTree::NT_HEIGHTMAP:
                meshIdx = 1;
                break;
            case RBVHTree::NT_INTERNAL:
                return;
            }

            RBVHTriangleIndices::iterator
                it1 = _triangleIndices->begin() + node->_startIdx,
                it2 = _triangleIndices->begin() + node->_startIdx
                + node->_count;

            for( auto it = it1; it != it2; ++it )
            {
                uint32_t triangleIdx = *it;
                Vector3ui triangle = faces[ triangleIdx ];
                triangle.x() = indexMapping[ meshIdx ][ triangle.x() ];
                triangle.y() = indexMapping[ meshIdx ][ triangle.y() ];
                triangle.z() = indexMapping[ meshIdx ][ triangle.z() ];
                meshes[ meshIdx ]->addFace( triangle );
            }
            node->_startIdx = lastIdx[ meshIdx ];
            lastIdx[ meshIdx ] += node->_count;
        } );

        triMesh = meshes[ 0 ];
        heightMesh = meshes[ 1 ];
    }

    void calcTransform()
    {
        // Calculate view Matrix
        Vector3f forward = _sumAreaWeightedNormal.normalized();
        Matrix4f viewMatrix = Utils::lookAt( Vector3f( 0.0f, 0.0f, 0.0f ), forward );


        // Calculate projection Matrix
        AlignedBox3f transformed;
        transformed.setEmpty();
        for( uint32_t i = 0; i < 8; ++i )
        {
            Vector3f corner =
                _bounds.corner( (AlignedBox3f::CornerType)i );
            Vector3f tCorner = (viewMatrix * corner.homogeneous())
                .eval().hnormalized();
            transformed.extend( tCorner );
        }
        float l = transformed.min().x(), r = transformed.max().x();
        float b = transformed.min().y(), t = transformed.max().y();
        float n = transformed.min().z(), f = transformed.max().z();

        Matrix4f projMatrix = Utils::orthoProjection( l, r, t, b, -n, -f );
        Matrix4f m = projMatrix * viewMatrix;
        _transform = m.block( 0, 0, 3, 4 );
    }

    void rasterize( oglplus::Uniform<oglplus::Mat4x3f>& proj )
    {
        oglplus::Renderbuffer heightmap;
        oglplus::Renderbuffer depth;
        oglplus::Framebuffer fbo;
        oglplus::Context gl;

        uint32_t sz = (uint32_t)_R;

        gl.Bound( oglplus::Renderbuffer::Target::Renderbuffer, heightmap )
            .Storage( oglplus::InternalFormat::R32F, sz, sz );


        gl.Bound( oglplus::Renderbuffer::Target::Renderbuffer, depth )
            .Storage( oglplus::InternalFormat::DepthComponent32, sz, sz );

        gl.Bound( oglplus::Framebuffer::Target::Draw, fbo )
            .AttachRenderbuffer( oglplus::FramebufferAttachment::Color, heightmap )
            .AttachRenderbuffer( oglplus::FramebufferAttachment::Depth, depth )
            .Complete();

        proj.Set( oglplus::Mat4x3f( _transform.data(), 12 ) );

        gl.Viewport( sz, sz );
        gl.ClearColor( 0.5, 0.5f, 0.5f, 0.5f );
        gl.Clear().ColorBuffer().DepthBuffer();
        gl.DrawElements(
            oglplus::PrimitiveType::Triangles,
            _count * 3,
            (uint32_t*)nullptr + _startIdx*3);

        gl.Bound( oglplus::Framebuffer::Target::Read, fbo );
        gl.ReadBuffer( oglplus::FramebufferColorAttachment::_0 );

        _heightmap.resize( sz*sz );
        gl.ReadPixels( 0, 0, sz, sz, oglplus::PixelDataFormat::Red,
            oglplus::PixelDataType::Float, _heightmap.data() );
        gl;
    }

    RBVHTrianglesPtr _triangles;
    RBVHTriangleIndicesPtr _triangleIndices;
    float _R, _rate;
    Vector3f _sumAreaWeightedNormal;
};

struct RBVHNodeData
{
    uint32_t _data;            // 4 bytes: 3 bytes Idx, 6 bits triangle_count, 2 bits type
    AlignedBox3h _boundingBox; // 12 bytes
};

struct RBVHTree::Impl
{
    Impl( ConstMeshPtr mesh, float rho, float alpha, float upperLimit )
    {
        MeshPtr trimesh;
        RBVHNode nodes( mesh, trimesh, rho, alpha, upperLimit );

        nodes.traverse([this]( RBVHNode* node )
            {
                _nodes.push_back( getNodeData( node ) );
                if( node->_type == NT_HEIGHTMAP )
                {
                    _transforms.push_back( node->_transform );
                    _heightmaps.push_back( node->_heightmap );
                }
            } );

        _vertices = std::move( trimesh->getVertices() );
        _indices = std::move( trimesh->getFaces() );

    }

    RBVHNodeData getNodeData( RBVHNode* node )
    {
        uint32_t data = node->_type;

        switch( node->_type )
        {
        case NT_TRIANGLES:
            data |= ((node->_startIdx << 8) & 0xFFFFFF00);
            data |= ((node->_count << 3) & 0xFC);
            break;
        case NT_HEIGHTMAP:
            data |= ((node->_heightIdx << 8) & 0xFFFFFF00);
            break;
        case NT_INTERNAL:
            data |= ((node->_right->_nodeIdx << 8) & 0xFFFFFF00);
            break;
        }
        return{
            data,
            Utils::halfBoundingBox( node->_bounds )};
    }

    Impl( const boost::filesystem::path& filename )
    {
        //TODO: fill data
    }

    void save( const boost::filesystem::path& filename )
    {
        boost::filesystem::ofstream
            out( filename, boost::filesystem::ofstream::binary );

        uint32_t nodeSize = (uint32_t)_nodes.size();
        uint32_t vertSize = (uint32_t)_vertices.size();
        uint32_t indxSize = (uint32_t)_indices.size();
        uint32_t tranSize = (uint32_t)_transforms.size();

        out.write( (char*)&nodeSize, sizeof( uint32_t ) );
        out.write( (char*)&vertSize, sizeof( uint32_t ) );
        out.write( (char*)&indxSize, sizeof( uint32_t ) );
        out.write( (char*)&tranSize, sizeof( uint32_t ) );

        out.write( (char*)_nodes.data(), sizeof( RBVHNodeData ) * nodeSize );
        out.write( (char*)_vertices.data(), sizeof( Vector3f ) * vertSize );
        out.write( (char*)_indices.data(), sizeof( Vector3ui ) * indxSize );
        out.write( (char*)_transforms.data(), sizeof( Matrix3x4f ) * tranSize );

        for( std::vector<float>& h : _heightmaps )
        {
            uint32_t sz2 = (uint32_t)h.size();
            out.write( (char*)&sz2, sizeof( uint32_t ) );
            out.write( (char*)h.data(), sizeof( float ) * sz2 );
        }

        out.close();
    }

    void saveHeightmaps( const std::string& filename )
    {
        uint32_t i = 0;
        FreeImage_Initialise();
        for( std::vector<float>& h : _heightmaps )
        {
            std::stringstream ss;
            ss << filename << i++ << ".pfm";

            uint32_t sz = sqrt( h.size() );
            FIBITMAP* bitmap = FreeImage_AllocateT( FIT_FLOAT, sz, sz );
            memcpy( FreeImage_GetBits( bitmap ), h.data(),
                sizeof( float ) * h.size() );
            FreeImage_Save( FIF_PFM, bitmap, ss.str().c_str(), 0 );
            FreeImage_Unload( bitmap );
        }
        FreeImage_DeInitialise();
    }

    std::vector<RBVHNodeData> _nodes;
    Vector3fs _vertices;
    Vector3uis _indices;
    std::vector<Matrix3x4f> _transforms;
    std::vector<std::vector<float>> _heightmaps;
};

RBVHTree::RBVHTree( ConstMeshPtr mesh, float rho, float alpha, float upperLimit )
    : _impl( new RBVHTree::Impl( mesh, rho, alpha, upperLimit ) )
{}

RBVHTree::RBVHTree( const boost::filesystem::path& filename )
    : _impl( new RBVHTree::Impl( filename ) )
{}

RBVHTree::~RBVHTree() {}

RBVHTree::NodeID RBVHTree::getRootNode() const
{
    return 0;
}

RBVHTree::NodeID RBVHTree::getLeftChild( RBVHTree::NodeID node ) const
{
    assert( getNodeType( node ) == NT_INTERNAL );
    return node + 1;
}

RBVHTree::NodeID RBVHTree::getRightChild( RBVHTree::NodeID node ) const
{
    assert( getNodeType( node ) == NT_INTERNAL );
    return (_impl->_nodes[ node ]._data >> 8) & (0xFFFFFF);
}

bool RBVHTree::isLeaf( RBVHTree::NodeID node ) const
{
    return getNodeType( node ) != NT_INTERNAL;
}

RBVHTree::NodeType RBVHTree::getNodeType( RBVHTree::NodeID node ) const
{
    return (NodeType)(_impl->_nodes[ node ]._data && 0x3);
}

const AlignedBox3h& RBVHTree::getBoundingBox( RBVHTree::NodeID node ) const
{
    return _impl->_nodes[ node ]._boundingBox;
}

void RBVHTree::getHeightmap( RBVHTree::NodeID node ) const
{
    assert( getNodeType( node ) == NT_HEIGHTMAP );
}

const Matrix3x4f& RBVHTree::getHeightmapTransform( RBVHTree::NodeID node ) const
{
    assert( getNodeType( node ) == NT_HEIGHTMAP );
    uint32_t heightIdx = (_impl->_nodes[ node ]._data >> 8) & (0xFFFFFF);
    return _impl->_transforms[ heightIdx ];
}

void RBVHTree::save( const boost::filesystem::path& filename ) const
{
    _impl->save( filename );
}

}