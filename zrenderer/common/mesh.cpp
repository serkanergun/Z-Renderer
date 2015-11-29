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

#include <zrenderer/common/mathtypes.h>
#include <zrenderer/common/mesh.h>

namespace zrenderer
{

struct Mesh::Impl
{
    Impl() {}

    ~Impl() {}

    const Vector3fs& getVertices() const
    {
        return _vertices;
    }

    const Vector3uis& getIndices() const
    {
        return _faces;
    }

    Vector3fs _vertices;
    Vector3uis _faces;
};

Mesh::Mesh()
    : _impl(new Mesh::Impl())
{}

Mesh::~Mesh() {}

const Vector3fs& Mesh::getVertices() const
{
    return _impl->getVertices();
}

const Vector3uis& Mesh::getIndices() const
{
    return _impl->getIndices();
}

}