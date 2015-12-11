/* Copyright (c) 2015, Zombie Rendering
 *                     ahmetbilgili@gmail.com
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

#ifndef _mathtypes_h_
#define _mathtypes_h_

#include <zrenderer/zcommon/types.h>
#include <zrenderer/zcommon/half.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace zrenderer
{
using half_float::half;
using half_float::half_cast;

using Eigen::Vector2i;
using Eigen::Vector3i;
using Eigen::Vector4i;

typedef Eigen::Matrix<uint32_t, 2, 1> Vector2ui;
typedef Eigen::Matrix<uint32_t, 3, 1> Vector3ui;
typedef Eigen::Matrix<uint32_t, 4, 1> Vector4ui;

using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::Vector4f;

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;

typedef Eigen::Matrix<half, 2, 1> Vector2h;
typedef Eigen::Matrix<half, 3, 1> Vector3h;
typedef Eigen::Matrix<half, 4, 1> Vector4h;


using Eigen::Matrix4f;
typedef Eigen::Matrix<float, 3, 4> Matrix3x4f;

using Eigen::AlignedBox3f;
typedef Eigen::AlignedBox<half, 3> AlignedBox3h;

typedef std::vector<Vector3f> Vector3fs;
typedef std::vector<Vector3ui> Vector3uis;

}

#endif // _mathtypes_h_
