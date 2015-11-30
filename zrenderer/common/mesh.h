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

#ifndef _mesh_h_
#define _mesh_h_

#include <zrenderer/common/mathtypes.h>
#include <zrenderer/common/api.h>

namespace zrenderer
{

ZCOMMON_API MeshPtrs importMesh(const boost::filesystem::path& inputfile);

/**
 * This class holds the vertex and
 * normal data for an indexed triangle mesh
 */
class Mesh
{

public:

    ZCOMMON_API Mesh();
    ZCOMMON_API virtual ~Mesh();

    /**
    * Reserves internal memory for data
    * @param numVertices expected number of vertices
    * @param numFaces expected number of faces
    */
    ZCOMMON_API void reserve( uint32_t numVertices, uint32_t numFaces );

    /**
    * Adds a vertex to the mesh
    * @param position position of the vertex
    * @param normal of the vertex
    */
    ZCOMMON_API void addVertex( const Vector3f& position, const Vector3f& normal );

    /**
    * Adds a triangle to the mesh if the given vertex indices are 
    * valid
    * @param face indices of the vertices that forms the triangle
    * @return true if the triangle is added. 
    */
    ZCOMMON_API bool addFace( const Vector3ui& face );

    /**
    * @return vertices of the mesh
    */
    ZCOMMON_API const Vector3fs& getVertices() const;

    /**
    * @return normals of the mesh
    */
    ZCOMMON_API const Vector3fs& getNormals() const;
    
    /**
    * @return faces of the mesh
    */
    ZCOMMON_API const Vector3uis& getFaces() const;

private:

    struct Impl;
    std::unique_ptr<Impl> _impl;
};

}

#endif // _mesh_h_
