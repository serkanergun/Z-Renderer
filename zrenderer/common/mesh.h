#ifndef _mesh_h_
#define _mesh_h_

#include <zrenderer/common/mathtypes.h>
#include <zrenderer/common/api.h>

namespace zrenderer
{

class Mesh
{

public:

    ZCOMMON_API Mesh();
    ZCOMMON_API virtual ~Mesh();
	
    ZCOMMON_API const Vector3fs& getVertices() const;
    ZCOMMON_API const Vector3uis& getFaces() const;

private:

    struct Impl;
    std::unique_ptr<Impl> _impl;
};

}

#endif // _mesh_h_
