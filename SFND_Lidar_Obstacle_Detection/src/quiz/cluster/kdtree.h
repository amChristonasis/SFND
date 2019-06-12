/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <cmath>
#include <iostream>


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		int n = 0;
		insertHelper(&root, point, id, n);

	}

	void insertHelper(Node **node, std::vector<float> point, int id, int n)
	{
		if (*node == NULL)
		{
			*node = new Node(point, id);
		}
		else if (n == 0)
		{
			n = 1;
			if (point[0] < (*node)->point[0])
			{
				insertHelper(&(*node)->left, point, id, n);
			}
			else
			{
				insertHelper(&(*node)->right, point, id, n);
			}
		}
		else
		{
			n = 0;
			if (point[1] < (*node)->point[1])
			{
				insertHelper(&(*node)->left, point, id, n);
			}
			else
			{
				insertHelper(&(*node)->right, point, id, n);
			}
		}
		
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(root, target, distanceTol, ids, 0);
		return ids;
	}

	void searchHelper(Node* node, std::vector<float> target, float distanceTol, std::vector<int>& ids, int depth)
	{
		if (node != NULL)
		{
			if (std::abs(node->point[0] - target[0]) <= distanceTol && std::abs(node->point[1] - target[1]) <= distanceTol)
			{
				if (std::sqrt(std::pow(node->point[0] - target[0],2) + std::pow(node->point[1] - target[1],2)) <= distanceTol)
				{
					ids.push_back(node->id);
					// std::cout << "Push back : " << node->id << std::endl;
				}
			}
		
			unsigned int n = depth%2;

			if (target[n] - distanceTol < node->point[n])
			{
				// std::cout << "Got to left : " << std::endl;
				searchHelper(node->left, target, distanceTol, ids, depth + 1);
			}
			if (target[n] + distanceTol > node->point[n])
			{
				// std::cout << "Got to right : " << std::endl;
				searchHelper(node->right, target, distanceTol, ids, depth + 1);
			}
		}
	}
	
};




