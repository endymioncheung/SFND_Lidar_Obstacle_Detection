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
			uint curr_depth = depth%2;

			// Compare the x-value with the tree if current depth is even, else
			// Compare the y-value with the tree if current depth is odd
			if(point[curr_depth] < ((*node)->point[curr_depth]))
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

	void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		// Serach only if the tree is not empty
		if (node != NULL)
		{
			// Check if the (x,y) values of the point is within the target box
			if ((node->point[0] >= (target[0]-distanceTol) && node->point[0] <= (target[0]+distanceTol)) && 
				(node->point[1] >= (target[1]-distanceTol) && node->point[1] <= (target[1]+distanceTol)))
			{
				// Calculate the distance
				float distance = sqrt((node->point[0]-target[0])*(node->point[0]-target[0]) + 
									  (node->point[1]-target[1])*(node->point[1]-target[1]));
				
				// Add the node ID onto IDs
				// if the target less than the distance tolerance 
				if (distance <= distanceTol)
					ids.push_back(node->id);
			}
			
			// Check the box boundary to flow left / right of the tree
			// Search the LHS of the tree if the target is less than the node x or y value (for odd or even depth)
			if((target[depth%2]-distanceTol) < node->point[depth%2])
				searchHelper(target, node->left, depth+1, distanceTol, ids);
			// Search the RHS of the tree if the target is greater than the node x or y value (for odd or even depth)
			if((target[depth%2]+distanceTol) > node->point[depth%2])
				searchHelper(target, node->right, depth+1, distanceTol, ids);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		// Indices of the points nearby the target
		std::vector<int> ids;

		// Search the KD-Tree recursively starting at root
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
};