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
 *  \brief Provides tests for mathtools library
 *  \author Bastien Durix
 */

#include <iostream>
#include <mathtools/application/Bspline.h>


int main()
{
	Eigen::Matrix<double,1,Eigen::Dynamic> node(1,6);
	node << 0.0, 0.0, 0.0, 1.0, 1.0, 1.0;
	for(unsigned int i=0;i<11;i++)
		std::cout << "B_{3,0}(" << (double)i*0.1 <<  ") = " << mathtools::application::BsplineBasis<3,0>::eval((double)i*0.1,node)
				  << "; (1-x)^3=" << (1-(double)i*0.1)*(1-(double)i*0.1)*(1-(double)i*0.1) << std::endl;
	for(unsigned int i=0;i<11;i++)
		std::cout << "B_{3,1}(" << (double)i*0.1 <<  ") = " << mathtools::application::BsplineBasis<3,1>::eval((double)i*0.1,node)
				  << "; 3*(1-x)^2*x=" << 3*(1-(double)i*0.1)*(1-(double)i*0.1)*(double)i*0.1 << std::endl;
	for(unsigned int i=0;i<11;i++)
		std::cout << "B_{3,2}(" << (double)i*0.1 <<  ") = " << mathtools::application::BsplineBasis<3,2>::eval((double)i*0.1,node)
				  << "; 3*(1-x)*x^2=" << 3*(1-(double)i*0.1)*(double)i*0.1*(double)i*0.1 << std::endl;
	for(unsigned int i=0;i<11;i++)
		std::cout << "B_{3,3}(" << (double)i*0.1 <<  ") = " << mathtools::application::BsplineBasis<3,3>::eval((double)i*0.1,node)
				  << "; x^3=" << (double)i*0.1*(double)i*0.1*(double)i*0.1 << std::endl;
	for(unsigned int i=0;i<11;i++)
	{
		std::cout << "B_{3,0}(" << (double)i*0.1 <<  ") + B_{3,1}(" << (double)i*0.1 <<  ") + B_{3,2}(" << (double)i*0.1 <<  ") + B_{3,3}(" << (double)i*0.1 <<  ") = "
								<< mathtools::application::BsplineBasis<3,0>::eval((double)i*0.1,node) +
								   mathtools::application::BsplineBasis<3,1>::eval((double)i*0.1,node) +
								   mathtools::application::BsplineBasis<3,2>::eval((double)i*0.1,node) +
								   mathtools::application::BsplineBasis<3,3>::eval((double)i*0.1,node) << std::endl;
	}
	return 0;
}
