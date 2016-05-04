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
 *  \file RecSkelFile.cpp
 *  \brief Defines reconstruction skeleton writer and reader
 *  \author Bastien Durix
 */

#include "RecSkelFile.h"
#include <fstream>

void fileio::WriteRecSkel(const skeleton::ReconstructionSkeleton::Ptr recskel, const std::string &filename)
{
	std::ofstream ofs(filename);
	
	if(ofs)
	{
		std::list<unsigned int> l_edge;
		recskel->getAllEdges(l_edge);
		
		ofs << l_edge.size() << std::endl;
		
		for(std::list<unsigned int>::iterator it = l_edge.begin(); it != l_edge.end(); it++)
		{
			skeleton::ReconstructionBranch::Ptr recbr = recskel->getBranch(*it);
			std::pair<unsigned int,unsigned int> ext = recskel->getExtremities(*it);

			ofs << ext.first << " " << ext.second << std::endl;

			ofs << recbr->getIndSkel().size() << std::endl;
			for(unsigned int i = 0; i < recbr->getIndSkel().size(); i++)
			{
				ofs << recbr->getIndSkel()[i] << " ";
			}
			ofs << std::endl;
			
			ofs << recbr->getFirstExt().size() << std::endl;
			for(unsigned int i = 0; i < recbr->getFirstExt().size(); i++)
			{
				ofs << recbr->getFirstExt()[i] << " ";
			}
			ofs << std::endl;
			
			ofs << recbr->getLastExt().size() << std::endl;
			for(unsigned int i = 0; i < recbr->getLastExt().size(); i++)
			{
				ofs << recbr->getLastExt()[i] << " ";
			}
			ofs << std::endl;
			ofs << std::endl;
		}
		ofs.close();
	}
}

skeleton::ReconstructionSkeleton::Ptr fileio::ReadRecSkel(const std::string &filename)
{
	skeleton::ReconstructionSkeleton::Ptr recskel;
	std::ifstream ifs(filename);
	
	if(ifs)
	{
		recskel = skeleton::ReconstructionSkeleton::Ptr(new skeleton::ReconstructionSkeleton());
		unsigned int nbedge;

		ifs >> nbedge;
		
		for(unsigned int i = 0; i < nbedge; i++)
		{
			std::pair<unsigned int,unsigned int> ext;
			
			ifs >> ext.first >> ext.second;
			
			unsigned int nbind;
			ifs >> nbind;
			std::vector<unsigned int> vecindpr(nbind);
			for(unsigned int j = 0; j < nbind; j++)
			{
				ifs >> vecindpr[j];
			}

			unsigned int nbfirstext;
			ifs >> nbfirstext;
			std::vector<unsigned int> firstext(nbfirstext);
			for(unsigned int j = 0; j < nbfirstext; j++)
			{
				ifs >> firstext[j];
			}

			unsigned int nblastext;
			ifs >> nblastext;
			std::vector<unsigned int> lastext(nblastext);
			for(unsigned int j = 0; j < nblastext; j++)
			{
				ifs >> lastext[j];
			}
			skeleton::ReconstructionBranch::Ptr recbr(new skeleton::ReconstructionBranch(vecindpr,firstext,lastext));

			recskel->addNode(ext.first);
			recskel->addNode(ext.second);
			recskel->addEdge(ext.first,ext.second,recbr);
		}
		ifs.close();
	}

	return recskel;
}
