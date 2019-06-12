/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


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
		insertHelper(*root, point, id, n);

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
		return ids;
	}
	

};




