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
 *  \file ExtClickFile.cpp
 *  \brief Defines clicked extremities writer and reader
 *  \author Bastien Durix
 */

#include "ExtClickFile.h"
#include <fstream>

void fileio::WriteExtSkel(const std::vector<std::vector<unsigned int> > &vecext, const std::string &filename)
{
	std::ofstream ofs(filename);
	
	if(ofs)
	{
		ofs << vecext.size() << std::endl;
		for(unsigned int i = 0; i < vecext.size(); i++)
		{
			ofs << vecext[i].size() << std::endl;
			for(unsigned int j = 0; j < vecext[i].size(); j++)
			{
				ofs << vecext[i][j] << " ";
			}
			ofs << std::endl;
		}
		ofs.close();
	}
}

std::vector<std::vector<unsigned int> > fileio::ReadExtSkel(const std::string &filename)
{
	std::vector<std::vector<unsigned int> > vecext(0);
	std::ifstream ifs(filename);
	
	if(ifs)
	{
		unsigned int nbext;
		ifs >> nbext;
		vecext.resize(nbext);
		for(unsigned int i = 0; i < nbext; i++)
		{
			unsigned int nbskel;
			ifs >> nbskel;
			vecext[i].resize(nbskel);
			for(unsigned int j = 0; j < nbskel; j++)
			{
				ifs >> vecext[i][j];
			}
		}
		ifs.close();
	}

	return vecext;
}
