/*
* Copyright (c) 2015 <zombierendering@gmail.com>
*/

#include <string>

#define BOOST_TEST_MODULE Mesh
#include <boost/test/unit_test.hpp>
#include <zrenderer/common/mesh.h>

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
    mesh.reserve( 10, 20 );
    BOOST_CHECK_EQUAL( mesh.getVertices().size(), 0 );
    BOOST_CHECK_EQUAL( mesh.getNormals().size(), 0 );
    BOOST_CHECK_EQUAL( mesh.getFaces().size(), 0 );
    BOOST_CHECK_EQUAL( mesh.getVertices().capacity(), 10 );
    BOOST_CHECK_EQUAL( mesh.getNormals().capacity(), 10 );
    BOOST_CHECK_EQUAL( mesh.getFaces().capacity(), 20 );
}

BOOST_AUTO_TEST_CASE(Mesh_addVertex)
{
    Mesh mesh;
    Vector3f pos( 1, 2, 3 );
    Vector3f nrm( 4, 5, 6 );
    nrm.normalize();

    mesh.addVertex( pos, nrm );
    BOOST_CHECK_EQUAL( mesh.getVertices()[0], pos );
    BOOST_CHECK_EQUAL( mesh.getNormals()[0], nrm );
}

BOOST_AUTO_TEST_CASE(Mesh_addFaceInvalid)
{
    Mesh mesh;
    Vector3f pos( 1, 2, 3 );
    Vector3f nrm( 4, 5, 6 );
    nrm.normalize();

    mesh.addVertex( pos, nrm );

    Vector3ui face( 0, 1, 2 );
    BOOST_CHECK_EQUAL( mesh.addFace( face ), false );
    BOOST_CHECK_EQUAL( mesh.getFaces().size(), 0 );
}

BOOST_AUTO_TEST_CASE(Mesh_addFaceValid)
{
    Mesh mesh;
    Vector3f pos( 1, 2, 3 );
    Vector3f nrm( 4, 5, 6 );
    nrm.normalize();

    mesh.addVertex( pos, nrm );

    Vector3ui face( 0, 0, 0 );
    BOOST_CHECK_EQUAL( mesh.addFace(face), true );
    BOOST_CHECK_EQUAL( mesh.getFaces().size(), 1 );
    BOOST_CHECK_EQUAL( mesh.getFaces()[0], face );
}

BOOST_AUTO_TEST_CASE(Mesh_importFile)
{
    std::vector<MeshPtr> meshes = importMesh("tests/data/teapot.mesh");

    BOOST_CHECK_EQUAL(meshes.size(), 1);
    BOOST_CHECK_EQUAL(meshes[0]->getVertices().size(), 3644);
    BOOST_CHECK_EQUAL(meshes[0]->getNormals().size(), 3644);
    BOOST_CHECK_EQUAL(meshes[0]->getFaces().size(), 6320);
}

BOOST_AUTO_TEST_CASE(Mesh_importFileNotExists)
{
    std::vector<MeshPtr> meshes = importMesh("tests/data/taepo.mesh");

    BOOST_CHECK_EQUAL(meshes.size(), 0);
}