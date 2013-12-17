/*
 * Encoders.h
 *
 * TO DO: use windows library and make it linux, so we could have encoders data
 * Probably needs async communication, so new thread to receive data ? (fork in linux)
 */

#ifndef ENCODERS_H_
#define ENCODERS_H_

class Encoders {
public:
	Encoders();
	virtual ~Encoders();


	int getLeftEncoder();
	int getRightEncoder();
};

#endif /* ENCODERS_H_ */
