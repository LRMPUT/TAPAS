/*
 * UnionFind.cpp
 *
 *  Created on: 23-10-2013
 *      Author: jachu
 */

#include "UnionFind.h"

UnionFind::UnionFind(int icount){
	set.resize(icount);
}

UnionFind::~UnionFind(){

}

int UnionFind::findSet(int node){
	if(set[node].parent == -1){
		return node;
	}

	set[node].parent = findSet(set[node].parent);
	return set[node].parent;
}

void UnionFind::unionSets(int node1, int node2){
	int node1Root = findSet(node1);
	int node2Root = findSet(node2);
	if(set[node1Root].rank > set[node2Root].rank){
		set[node2Root].parent = node1Root;
	}
	else if(set[node1Root].rank < set[node2Root].rank){
		set[node1Root].parent = node2Root;
	}
	else if(node1Root != node2Root){
		set[node2Root].parent = node1Root;
		set[node1Root].rank++;
	}
}
