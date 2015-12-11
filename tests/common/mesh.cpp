/*
* Copyright (c) 2015 <zombierendering@gmail.com>
*/

#include <string>

#define BOOST_TEST_MODULE Mesh
#include <boost/test/unit_test.hpp>
#include <zrenderer/zcommon/mesh.h>

using namespace zrenderer;

BOOST_AUTO_TEST_CASE(Mesh_constructor)
{
    Mesh mesh;
    BOOST_CHECK_EQUAL( mesh.getVertices().size(), 0 );
    BOOST_CHECK_EQUAL( mesh.getNormals().size(), 0 );
    BOOST_CHECK_EQUAL( mesh.getFaces().size(), 0 );
}

BOOST_AUTO_TEST_CASE(Mesh_reserve)
{
    Mesh mesh;
    mesh.reserveVertices( 10 );
    mesh.reserveNormals( 10 );
    mesh.reserveFaces( 10 );

    BOOST_CHECK_EQUAL( mesh.getVertices().size(), 0 );
    BOOST_CHECK_EQUAL( mesh.getVertices().capacity(), 10 );
    BOOST_CHECK_EQUAL( mesh.getNormals().size(), 0 );
    BOOST_CHECK_EQUAL( mesh.getNormals().capacity(), 0 );
    BOOST_CHECK_EQUAL( mesh.getFaces().size(), 0 );
    BOOST_CHECK_EQUAL( mesh.getFaces().capacity(), 20 );
}

BOOST_AUTO_TEST_CASE(Mesh_addVertex)
{
    Mesh mesh;
    Vector3f pos( 1, 2, 3 );
    Vector3f nrm( 4, 5, 6 );
    nrm.normalize();

    mesh.addVertex( pos );
    BOOST_CHECK_EQUAL( mesh.getVertices()[0], pos );

    mesh.addNormal( nrm );
    BOOST_CHECK_EQUAL( mesh.getNormals()[0], nrm );
}

BOOST_AUTO_TEST_CASE(Mesh_addFaceInvalid)
{
    Mesh mesh;
    const Vector3f pos( 1, 2, 3 );
    const Vector3f nrm = Vector3f( 4, 5, 6 ).normalized();

    mesh.addVertex( pos );
    mesh.addNormal( nrm );

    const Vector3ui face( 0, 1, 2 );
    BOOST_CHECK_EQUAL( mesh.addFace( face ), false );
    BOOST_CHECK_EQUAL( mesh.getFaces().size(), 0 );
}

BOOST_AUTO_TEST_CASE(Mesh_addFaceValid)
{
    Mesh mesh;
    const Vector3f pos( 1, 2, 3 );
    const Vector3f nrm = Vector3f( 4, 5, 6 ).normalized();

    mesh.addVertex( pos );
    mesh.addNormal( nrm );

    const Vector3ui face( 0, 0, 0 );
    BOOST_CHECK_EQUAL( mesh.addFace( face ), true );
    BOOST_CHECK_EQUAL( mesh.getFaces().size(), 1 );
    BOOST_CHECK_EQUAL( mesh.getFaces()[0], face );
}

BOOST_AUTO_TEST_CASE(Mesh_importFile)
{
    MeshPtrs meshes = importMesh( "tests/data/teapot.mesh" );

    BOOST_CHECK_EQUAL( meshes.size(), 1 );
    BOOST_CHECK_EQUAL( meshes[ 0 ]->getVertices().size(), 3644 );
    BOOST_CHECK_EQUAL( meshes[ 0 ]->getNormals().size(), 3644 );
    BOOST_CHECK_EQUAL( meshes[ 0 ]->getFaces().size(), 6320 );
}

BOOST_AUTO_TEST_CASE(Mesh_importFileNotExists)
{
    MeshPtrs meshes = importMesh( "tests/data/taepo.mesh" );

    BOOST_CHECK_EQUAL(meshes.size(), 0);
}