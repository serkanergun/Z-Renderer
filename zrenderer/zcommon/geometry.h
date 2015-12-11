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

#ifndef _geometry_h_
#define _geometry_h_

#include <zrenderer/zcommon/mathtypes.h>
#include <zrenderer/zcommon/api.h>

namespace zrenderer
{

class Geometry
{
public:

    ZCOMMON_API Geometry() {}
    ZCOMMON_API virtual ~Geometry() {}

    /**
    * @return bounding box of the geometry
    */
    ZCOMMON_API virtual const AlignedBox3f& getBoundingBox() const = 0;
};

}

#endif // _geometry_h_
