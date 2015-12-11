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

#include <zrenderer/zcommon/utils.h>

namespace zrenderer
{

Matrix4f lookAtHelper( const Vector3f& eye, const Vector3f& up,
    const Vector3f& z )
{
    Vector3f x = z.cross( up ); x.normalize();
    Vector3f y = x.cross( z );

    Matrix4f viewMatrix;
    viewMatrix <<
         x.x(),  x.y(),  x.z(), -x.dot( eye ),
         y.x(),  y.y(),  y.z(), -y.dot( eye ),
        -z.x(), -z.y(), -z.z(),  z.dot( eye ),
          0.0f,   0.0f,   0.0f,          1.0f;

    return viewMatrix;
}

Matrix4f Utils::lookAt( const Vector3f& eye, const Vector3f& center,
    const Vector3f& up )
{
    Vector3f z = center - eye; z.normalize();
    return lookAtHelper( eye, up, z );
}

Matrix4f Utils::lookAt( const Vector3f& eye, const Vector3f& center )
{
    Vector3f z = center - eye;  z.normalize();
    Vector3f up = (fabs( z.y() ) < 0.01) 
        ? Vector3f( 0.0f, 0.0f, 1.0f )
        : Vector3f( 0.0f, 1.0f, 0.0f );
    
    return lookAtHelper( eye, up, z );
}

Matrix4f Utils::orthoProjection( float l, float r,
    float t, float b, float n, float f )
{
    Matrix4f m;

    m <<
        2.0f / (r - l), 0.0f, 0.0f, (r + l) / (l - r),
        0.0f, 2.0f / (t - b), 0.0f, (t + b) / (b - t),
        0.0f, 0.0f, 2.0f / (n - f), (n + f) / (n - f),
        0.0f, 0.0f, 0.0f, 1.0f;
/*
    m <<
        2.0f / (r - l), 0.0f, 0.0f, (r + l) / (l - r),
        0.0f, 2.0f / (t - b), 0.0f, (t + b) / (b - t),
        0.0f, 0.0f, 1.0f / (f - n),  n / (n - f),
        0.0f, 0.0f, 0.0f, 1.0f;
*/

    return m;
}

AlignedBox3h Utils::halfBoundingBox( AlignedBox3f& box )
{
    Vector3h minV(
        half_cast<half, std::round_toward_neg_infinity>( box.min().x() ),
        half_cast<half, std::round_toward_neg_infinity>( box.min().y() ),
        half_cast<half, std::round_toward_neg_infinity>( box.min().z() ) );

    Vector3h maxV(
        half_cast<half, std::round_toward_infinity>( box.max().x() ),
        half_cast<half, std::round_toward_infinity>( box.max().y() ),
        half_cast<half, std::round_toward_infinity>( box.max().z() ) );

    return AlignedBox3h( minV, maxV );
}

}