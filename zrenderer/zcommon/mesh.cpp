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

#include <zrenderer/zcommon/mathtypes.h>
#include <zrenderer/zcommon/mesh.h>
#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>


namespace zrenderer
{

MeshPtrs importMesh( const boost::filesystem::path& inputfile )
{
    aiLogStream stream = aiGetPredefinedLogStream( aiDefaultLogStream_STDOUT,
        nullptr );
    const aiScene* scene = aiImportFile( inputfile.generic_string().c_str(),
        aiProcessPreset_TargetRealtime_MaxQuality |
        aiProcess_PreTransformVertices );

    MeshPtrs meshes;

    if( scene == nullptr )
        return meshes;

    // traverse all meshes assigned to root node
    for( uint32_t n = 0; n < scene->mRootNode->mNumMeshes; ++n )
    {
        const struct aiMesh* mesh =
            scene->mMeshes[ scene->mRootNode->mMeshes[ n ] ];

        MeshPtr outMesh( new Mesh() );
        outMesh->reserveVertices( mesh->mNumVertices );
        outMesh->reserveNormals( mesh->mNumVertices );
        outMesh->reserveFaces( mesh->mNumFaces );

        for( uint32_t i = 0; i < mesh->mNumVertices; ++i )
        {
            Vector3f vertex( &mesh->mVertices[ i ].x );
            Vector3f normal( &mesh->mNormals[ i ].x );
            outMesh->addVertex( vertex );
            outMesh->addNormal( normal );
        }

        for( uint32_t i = 0; i < mesh->mNumFaces; ++i )
        {
            const struct aiFace* face = &mesh->mFaces[ i ];

            if( face->mNumIndices != 3 )
                continue;

            Vector3ui indices( face->mIndices[ 0 ], face->mIndices[ 1 ],
                face->mIndices[ 2 ] );
            outMesh->addFace( indices );
        }

        if( outMesh->getFaces().size() > 0 )
            meshes.push_back( outMesh );
    }

    aiReleaseImport( scene );
    return meshes;
}

struct Mesh::Impl
{
    Vector3fs _vertices;
    Vector3fs _normals;
    Vector3uis _faces;
    AlignedBox3f _bound;
};

Mesh::Mesh()
    : _impl( new Mesh::Impl() )
{}

Mesh::~Mesh() {}

void Mesh::reserveVertices( uint32_t numVertices )
{
    _impl->_vertices.reserve( numVertices );
}

void Mesh::reserveFaces( uint32_t numFaces )
{
    _impl->_faces.reserve( numFaces );
}

void Mesh::reserveNormals( uint32_t numNormals )
{
    _impl->_normals.reserve( numNormals );
}

const Vector3fs& Mesh::getVertices() const
{
    return _impl->_vertices;
}

const Vector3fs& Mesh::getNormals() const
{
    return _impl->_normals;
}

const Vector3uis& Mesh::getFaces() const
{
    return _impl->_faces;
}

void Mesh::addVertex( const Vector3f& vertex )
{
    _impl->_vertices.push_back( vertex );
    _impl->_bound.extend( vertex );
}

void Mesh::addNormal( const Vector3f& normal )
{
    _impl->_normals.push_back( normal );
}

bool Mesh::addFace( const Vector3ui& face )
{
    uint32_t n = (uint32_t)_impl->_vertices.size();
    if( face[ 0 ] >= n || face[ 1 ] >= n || face[ 2 ] >= n )
        return false;

    _impl->_faces.push_back( face );
    return true;
}

const AlignedBox3f& Mesh::getBoundingBox() const
{
    return _impl->_bound;
}

}