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

#ifndef _zcommon_mesh_h_
#define _zcommon_mesh_h_

#include <zrenderer/zcommon/mathtypes.h>
#include <zrenderer/zcommon/geometry.h>
#include <zrenderer/zcommon/api.h>

namespace zrenderer
{

ZCOMMON_API MeshPtrs importMesh(const boost::filesystem::path& inputfile);

/**
 * This class holds the vertex and
 * normal data for an indexed triangle mesh
 */
class Mesh : public Geometry
{

public:

    ZCOMMON_API Mesh();
    ZCOMMON_API virtual ~Mesh();

    /**
    * Reserves internal memory for vertices
    * @param numVertices expected number of vertices
    */
    ZCOMMON_API void reserveVertices( uint32_t numVertices );

    /**
    * Reserves internal memory for normals
    * @param numNormals expected number of normals
    */
    ZCOMMON_API void reserveNormals( uint32_t numNormals );

    /**
    * Reserves internal memory for faces
    * @param numFaces expected number of faces
    */
    ZCOMMON_API void reserveFaces( uint32_t numFaces );

    /**
    * Adds a vertex to the mesh
    * @param position position of the vertex
    */
    ZCOMMON_API void addVertex( const Vector3f& position );

    /**
    * Adds normal to the mesh
    * @param normal of the vertex
    */
    ZCOMMON_API void addNormal( const Vector3f& normal );

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

    /**
    * @return bounding box of the mesh
    */
    ZCOMMON_API const AlignedBox3f& getBoundingBox() const override;

private:

    struct Impl;
    std::unique_ptr<Impl> _impl;
};

}

#endif // _zcommon_mesh_h_
