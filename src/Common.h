/*
Copyright (c) 2012 Tommi Tikkanen, Eero Laukkanen

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#pragma once

#include <string>
#include <vector>
#include <boost/regex.hpp>
#include <boost/filesystem.hpp>

namespace askinect
{

std::vector<std::string> getFiles(std::string directory, boost::regex filter)
{
    std::vector< std::string > allMatchingFiles;

    if (!boost::filesystem::is_directory(directory))
    {
        return allMatchingFiles;
    }

    boost::filesystem::directory_iterator end_itr; // Default ctor yields past-the-end
    for ( boost::filesystem::directory_iterator i( directory ); i != end_itr; ++i )
    {
        // Skip if not a file
        if ( !boost::filesystem::is_regular_file( i->status() ) ) continue;

        boost::smatch what;

        // Skip if no match
        if ( !boost::regex_match( i->path().filename().string(), what, filter ) ) continue;

        // File matches, store it
        allMatchingFiles.push_back( i->path().string() );
    }

    return allMatchingFiles;
}

}