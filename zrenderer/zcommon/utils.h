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

#ifndef _common_utils_h_
#define _common_utils_h_

#include <zrenderer/geometry/zrbvh/types.h>
#include <zrenderer/geometry/zrbvh/api.h>
#include <zrenderer/zcommon/api.h>

namespace zrenderer
{

class Utils
{
public:

    static ZCOMMON_API Matrix4f lookAt( const Vector3f& eye, const Vector3f& center,
        const Vector3f& up );

    static ZCOMMON_API Matrix4f lookAt( const Vector3f& eye, const Vector3f& center );

    static ZCOMMON_API Matrix4f orthoProjection( float l, float r,
        float t, float b, float n, float f);

    static ZCOMMON_API AlignedBox3h halfBoundingBox( AlignedBox3f& box );
};

}
#endif // _common_utils_h_