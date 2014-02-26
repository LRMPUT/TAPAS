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

/** \brief Struktura reprezentująca węzeł w klasie UnionFind.
 *
 */
struct SetNode{
	int parent, rank, nsize;
	SetNode() : parent(-1), rank(0), nsize(1) {}
	SetNode(int iparent, int irank, int insize) : parent(iparent), rank(irank), nsize(insize) {}
};

/** \brief Klasa reprezentująca rozłączne zbiory, umożliwiająca
 * 			efektywne ich łączenie.
 */
class UnionFind{
	std::vector<SetNode> set;
public:
	UnionFind(int icount);
	~UnionFind();

	/** \brief Funkcja znajdująca id zbioru, do którego należy węzeł node.
	 *
	 */
	int findSet(int node);

	/** \brief Funkcja łącząca dwa zbiory.
	 *
	 */
	int unionSets(int node1, int node2);

	/** \brief Funkcja zwracająca rozmiar zbioru.
	 *
	 */
	int size(int node);
};


#endif /* UNIONFIND_H_ */
