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
#include <math.h>

int main()
{
	unsigned int return_value = 0;

	bool bspline_test = true;
	unsigned int fraction = 100;
	
	std::cout << "Bspline test... ";
	std::vector<unsigned int> bin_coeff(6,0);
	bin_coeff[0] = 1;
	for(unsigned int degree = 1; degree < 5 && bspline_test; degree++)
	{
		// Filling node vector
		Eigen::Matrix<double,1,Eigen::Dynamic> node(1,degree*2);
		for(unsigned int i = 0; i < degree; i++)
		{
			node(0,i)        = 0.0;
			node(0,i+degree) = 1.0;
		}
		
		for(unsigned int i = degree; i > 0; i--)
		{
			bin_coeff[i] += bin_coeff[i-1];
		}
		
		for(unsigned int indice = 0; indice <= degree && bspline_test; indice++)
		{
			for(unsigned int i = 0; i < fraction && bspline_test; i++)
			{
				double t = (double)i*(1.0/(double)fraction);
				
				double res = mathtools::application::BsplineBasis(t,degree,indice,node);
				
				double ref = pow((1-t),degree-indice)*pow(t,indice)*(double)bin_coeff[indice];
				
				if(fabs(res - ref) > std::numeric_limits<double>::epsilon())
					bspline_test = false;
			}
		}
	}
	if(!bspline_test)
	{
		return_value = -1;
		std::cout << "Fail!" << std::endl;
	}
	else
		std::cout << "Success!" << std::endl;
	
	return return_value;
}
