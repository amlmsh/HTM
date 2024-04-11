#include "HTM.hpp"

#include <iostream>

int main(){

	IHTM *m;
	HTM3dTransRotX mX(2,1,2,3);
	HTM3dTransRotY mY(2,1,2,3);
	HTM3dTransRotZ mZ(2,1,2,3);

	m = new HTM3dTransRotX(2,1,2,3);
	std::cout << m->current() << std::endl;
	m = new HTM3dTransRotY(2,1,2,3);
   	std::cout << m->current() << std::endl;
   	m = new HTM3dTransRotZ(2,1,2,3);
   	std::cout << m->current() << std::endl;

   	m = factory('X',2,1,2,3);
   	std::cout << m->current() << std::endl;
   	delete m;
   	m = factory('Y',2,1,2,3);
   	std::cout << m->current() << std::endl;
   	delete m;
   	m = factory('Z',2,1,2,3);
   	std::cout << m->current() << std::endl;
   	delete m;
   	m = factory('A',2,1,2,3);
   	std::cout << m->current() << std::endl;



    return 0;
}
