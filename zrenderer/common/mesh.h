#ifndef _mesh_h_
#define _mesh_h_

#include <zrenderer/common/mathtypes.h>
#include <zrenderer/common/api.h>

namespace zrenderer
{

ZCOMMON_API std::vector<MeshPtr> importMesh(const boost::filesystem::path& inputfile);

/**
 * This class holds the vertex and
 * normal data for a triangle mesh
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
