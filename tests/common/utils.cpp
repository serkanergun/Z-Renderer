/*
* Copyright (c) 2015 <zombierendering@gmail.com>
*/

#include <string>

#define BOOST_TEST_MODULE Utils
#include <boost/test/unit_test.hpp>
#include <zrenderer/zcommon/utils.h>

using namespace zrenderer;

BOOST_AUTO_TEST_CASE(Utils_LookAt)
{
    const Vector3f pos( 3.0f, 1.0f, 2.0f );
    const Vector3f at( 4.0f, 5.0f, 6.0f );
    const Vector3f up( 0.0f, 1.0f, 0.0f );
    const float eps = 0.0001f;

    Matrix4f mat = Utils::lookAt( pos, at, up );

    Vector3f p1 = (mat * pos.homogeneous()).eval().hnormalized();
    Vector3f p2 = (mat * at.homogeneous()).eval().hnormalized();
    p2.normalize();

    BOOST_CHECK_CLOSE( p1.x(), 0.0f, eps );
    BOOST_CHECK_CLOSE( p1.y(), 0.0f, eps );
    BOOST_CHECK_CLOSE( p1.z(), 0.0f, eps );

    BOOST_CHECK_CLOSE( p2.x(), 0.0f, eps );
    BOOST_CHECK_CLOSE( p2.y(), 0.0f, eps );
    BOOST_CHECK_CLOSE( p2.z(), -1.0f, eps );
}
