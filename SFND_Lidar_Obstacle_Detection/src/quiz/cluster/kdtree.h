/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
    std::shared_ptr<Node> left;
    std::shared_ptr<Node> right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(nullptr), right(nullptr)
	{}
};

struct KdTree
{
	std::shared_ptr<Node> root;

	KdTree()
	: root(nullptr)
	{}

	void insertHelper(std::shared_ptr<Node>& node ,uint32_t depth, std::vector<float> point, int32_t id) {
	    // Tree is empty
	    if (node == nullptr) {
	        node.reset(new Node(point, id));
	    } else {
	        // Calculate current dim
	        uint32_t cd = depth % 2;

	        if (point[cd] < node->point[cd]) {
	            insertHelper(node->left, depth + 1, point, id);
	        } else {
	            insertHelper(node->right, depth + 1, point, id);
	        }
	    }
	}
	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insertHelper(root, 0, point, id);

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};




