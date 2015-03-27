/*
 * newserializer.h
 *
 *  Created on: 18 juin 2014
 *      Author: theveny
 */

#ifndef NEWSERIALIZER_H_
#define NEWSERIALIZER_H_

#include "serializer.h"
#include <sstream>

namespace std {

class newserializer: public serializer {
private :
	int shotCounter;
	

public:
	newserializer(ostream& st, shotfilter& filter);
	virtual ~newserializer();
	virtual void write(pointcloud* p);

};

} /* namespace std */
#endif /* NEWSERIALIZER_H_ */