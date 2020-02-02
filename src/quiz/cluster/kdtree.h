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

	void insertHelper(Node** node, uint depth, std::vector<float> point, int id)
	{
		// Create new node if the tree is empty
		if (*node == NULL)
			*node = new Node(point,id);
		else
		{
			// Traverse the tree if tree is not empty
			// Calculate current dimension
			uint curr_depth = depth %2;

			// Compare the x-value with the tree if current depth is even, else
			// Compare the y-value with the tree if current depth is odd
			if(point[curr_depth] < (*node)->point[curr_depth])
				insertHelper(&((*node)->left), depth+1, point, id);
			else
				insertHelper(&((*node)->right), depth+1, point, id);
		}
	}


	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		
		// Recursive insert to re-assign node at the tree,
		// So passing in the memory address of the root,
		// which will be inserted at depth 0 at the KD-Tree
		insertHelper(&root, 0, point, id);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
};