/*
 * UnionFind.h
 *
 *  Created on: 23-10-2013
 *      Author: jachu
 */

#ifndef UNIONFIND_H_
#define UNIONFIND_H_

#include <vector>
#include <cstddef>


struct SetNode{
	int parent, rank, nsize;
	SetNode() : parent(-1), rank(0), nsize(1) {}
	SetNode(int iparent, int irank, int insize) : parent(iparent), rank(irank), nsize(insize) {}
};


class UnionFind{
	std::vector<SetNode> set;
public:
	UnionFind(int icount);
	~UnionFind();
	int findSet(int node);
	int unionSets(int node1, int node2);
	int size(int node);
};


#endif /* UNIONFIND_H_ */
