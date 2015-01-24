#include "nodes_and_edges.h"

namespace scc
{
	Edge concat(Edge const& edge1, Edge const& edge2)
	{
		assert(edge1.tgt == edge2.src);
		return Edge(c::NO_EID, edge1.src, edge2.tgt, edge1.dist + edge2.dist);
	}

	OSMEdge concat(OSMEdge const& edge1, OSMEdge const& edge2)
	{
		assert(edge1.tgt == edge2.src);
		return OSMEdge(c::NO_EID, edge1.src, edge2.tgt, edge1.dist + edge2.dist, 0, -1);
	}
}
