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

#include <zrenderer/geometry/rbvh/rbvhnode.h>
#include <zrenderer/geometry/rbvh/version.h>
#include <zrenderer/common/mesh.h>
#include <boost/program_options.hpp>
#include <iostream>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

int main( int argc, char *argv[] )
{
    // Arguments parsing
    po::variables_map vm;
    po::options_description desc( "Supported options" );
    desc.add_options()
        ("help,h", "show help message.")
        ("version,v", "Show program name/version banner and exit.")
        ("rev", "Print the git revision number")
        ("input,i", po::value<fs::path>(), "Input file");

    po::store( parse_command_line( argc, argv, desc ), vm );
    po::notify( vm );

    if( vm.count( "help" ) )
    {
        std::cout << desc << std::endl;
        return EXIT_SUCCESS;
    }

    if( vm.count( "version" ) )
    {
        std::cout << "RBVHConverter version " << ZRBVH::Version::getString()
            << std::endl
            << "Copyright (c) ZombieRendering 2015." << std::endl;
        return EXIT_SUCCESS;
    }

    if( vm.count( "rev" ) )
    {
        std::cout << "git revision: " << std::hex
            << ZRBVH::Version::getRevision() << std::endl;
        return EXIT_SUCCESS;
    }

    if( vm.count( "input" ) )
    {
        const auto path = vm[ "input" ].as<fs::path>();
        std::vector<zrenderer::MeshPtr> meshes = zrenderer::importMesh( path );
        std::cout << meshes.size() << " meshes imported." << std::endl;
        uint32_t i = 0;
        for( const zrenderer::MeshPtr& mesh : meshes )
            std::cout << "Mesh " << i++ << " Vertices: " 
            << mesh->getVertices().size() << " Faces: " 
            << mesh->getFaces().size() << std::endl;
    }

    return EXIT_SUCCESS;
}