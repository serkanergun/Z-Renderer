/* Copyright(c) ZombieRendering 2015 - 2016 serkan.ergun@gmail.com
 *
 * This file is part of Z-Renderer(https://github.com/ZombieRendering/Z-Renderer)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met :
 *
 * -Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * -Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and / or other materials provided with the distribution.
 * -Neither the name of Eyescale Software GmbH nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <zrenderer/geometry/rbvh/rbvhnode.h>
#include <zrenderer/geometry/rbvh/version.h>
#include <boost/program_options.hpp>
#include <iostream>

namespace po = boost::program_options;

int main( int argc, char *argv[] )
{
    // Arguments parsing
    po::variables_map vm;
    po::options_description desc( "Supported options" );
    desc.add_options()
        ("help,h", "show help message.")
        ("version,v", "Show program name/version banner and exit.")
        ("rev", "Print the git revision number");
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

    return EXIT_SUCCESS;
}