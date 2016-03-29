/*
Copyright (c) 2016 Bastien Durix

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

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


/**
 *  \file BoundaryFile.cpp
 *  \brief Defines boundary file writer
 *  \author Bastien Durix
 */

#include "BoundaryFile.h"

#include <fstream>

void fileio::WriteBoundaryOBJ(const boundary::DiscreteBoundary<3>::Ptr bound, const std::string &filename)
{
	std::ofstream file(filename);

	if(file)
	{
		const std::map<unsigned int, Eigen::Vector3d> &map_pts = bound->getPoints();
		const std::map<unsigned int, Eigen::Vector3d> &map_nor = bound->getNormals();
		const std::list<std::tuple<unsigned int,unsigned int,unsigned int> > &map_fac = bound->getFaces();
		std::map<unsigned int,unsigned int> indice;
		unsigned int ind = 1;
		for(std::map<unsigned int, Eigen::Vector3d>::const_iterator it = map_pts.begin(); it != map_pts.end(); it++)
		{
			file << "v" << " " << it->second.x() << " " << it->second.y() << " " << it->second.z() << std::endl;
			indice.insert(std::pair<unsigned int,unsigned int>(it->first,ind));
			ind++;
		}

		file << std::endl << std::endl;

		for(std::map<unsigned int, Eigen::Vector3d>::const_iterator it = map_nor.begin(); it != map_nor.end(); it++)
		{
			file << "vn" << " " << it->second.x() << " " << it->second.y() << " " << it->second.z() << std::endl;
			indice[ind] = it->first;
			ind++;
		}

		file << std::endl << std::endl;

		for(std::list<std::tuple<unsigned int,unsigned int,unsigned int> >::const_iterator it = map_fac.begin(); it != map_fac.end(); it++)
		{
			
			file << "f";
			file << " " << indice[std::get<0>(*it)] << "//" << indice[std::get<0>(*it)];
			file << " " << indice[std::get<1>(*it)] << "//" << indice[std::get<1>(*it)];
			file << " " << indice[std::get<2>(*it)] << "//" << indice[std::get<2>(*it)];
			file << std::endl;
		}

		file << std::endl;

		file.close();
	}

}
