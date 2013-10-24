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
	int parent, rank;
	SetNode() : parent(-1), rank(0) {}
	SetNode(int iparent, int irank) : parent(iparent), rank(irank) {}
};


class UnionFind{
	std::vector<SetNode> set;
public:
	UnionFind(int icount);
	~UnionFind();
	int findSet(int node);
	void unionSets(int node1, int node2);
};


#endif /* UNIONFIND_H_ */
