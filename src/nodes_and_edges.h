#ifndef _NODES_AND_EDGES_H
#define _NODES_AND_EDGES_H

#include "defs.h"

#include <fstream>
#include <sstream>
#include <vector>
#include <limits>
#include <map>
#include <string>

namespace scc
{

typedef uint NodeID;
typedef uint EdgeID;
namespace c
{
	uint const NO_NID(std::numeric_limits<NodeID>::max());
	uint const NO_EID(std::numeric_limits<EdgeID>::max());
	uint const NO_DIST(std::numeric_limits<uint>::max());
	uint const NO_LVL(std::numeric_limits<uint>::max());
}

typedef std::map<std::string, std::string> Metadata;

enum EdgeType {OUT = 0, IN = 1};
inline EdgeType operator!(EdgeType type) { return (EdgeType)(1 - type); }

/* 
 * Data required to construct or write out graphs.
 */
template <typename NodeT, typename EdgeT>
struct GraphInData {
	/* graph will "steal" data */
	std::vector<NodeT> nodes;
	std::vector<EdgeT> edges;
	Metadata meta_data;
};

template <typename NodeT, typename EdgeT>
struct GraphOutData {
	std::vector<NodeT> const& nodes;
	std::vector<EdgeT> const& edges;
	Metadata meta_data;
};

template <typename NodeT, typename EdgeT>
struct GraphCHOutData {
	std::vector<NodeT> const& nodes;
	std::vector<uint> const& node_levels;
	std::vector<EdgeT> const& edges;
	Metadata meta_data;
};


/*
 * Nodes
 */

template<typename NodeT>
struct CHNode : NodeT
{
	typedef NodeT base_node_type;

	uint lvl = c::NO_LVL;

	explicit CHNode() { }
	explicit CHNode(NodeT const& node) : NodeT(node) { } 
	explicit CHNode(NodeT const& node, uint lvl) : NodeT(node), lvl(lvl){}
};

template<typename NodeT>
inline CHNode<NodeT> makeCHNode(NodeT const& node, uint lvl) {
	return CHNode<NodeT>(node, lvl);
}

template <typename NodeT>
struct _MakeCHNode
{
	typedef CHNode<NodeT> type;
};

template <typename NodeT>
struct _MakeCHNode<CHNode<NodeT>>
{
	typedef CHNode<NodeT> type;
};

template <typename NodeT>
using MakeCHNode = typename _MakeCHNode<typename std::remove_reference<NodeT>::type>::type;
/* remove possible CHNode<> with: typename MakeCHNode<T>::base_node_type */

struct Node
{
	NodeID id = c::NO_NID;

	explicit Node() { }
	explicit Node(NodeID id) : id(id) { }

	bool operator<(Node const& node) const { return id < node.id; }
};

struct GeoNode
{
	NodeID id = c::NO_NID;
	double lat = 0;
	double lon = 0;
	int elev = 0;

	explicit GeoNode() { }
	explicit GeoNode(NodeID id, double lat, double lon, int elev)
		: id(id), lat(lat), lon(lon), elev(elev) { }

	explicit operator Node() const
	{
		return Node(id);
	}

	bool operator<(GeoNode const& other) const { return id < other.id; }
};

struct OSMNode
{
	NodeID id = c::NO_NID;
	uint osm_id = std::numeric_limits<uint>::max();
	double lat = 0;
	double lon = 0;
	int elev = 0;

	explicit operator Node() const
	{
		return Node(id);
	}

	operator GeoNode() const
	{
		return GeoNode(id, lat, lon, elev);
	}

	bool operator<(OSMNode const& other) const { return id < other.id; }
};

/*
 * Edges
 */

template<typename EdgeT>
inline NodeID otherNode(EdgeT const& edge, EdgeType edge_type) {
	switch (edge_type) {
	case OUT:
		return edge.tgt;
	case IN:
		break;
	}
	return edge.src;
}

template<typename EdgeT1, typename EdgeT2>
inline bool equalEndpoints(EdgeT1 const& edge1, EdgeT2 const& edge2) {
	return edge1.src == edge2.src && edge1.tgt == edge2.tgt;
}

template<typename EdgeT>
struct CHEdge : EdgeT
{
	typedef EdgeT base_edge_type;

	EdgeID child_edge1 = c::NO_EID;
	EdgeID child_edge2 = c::NO_EID;
	NodeID center_node = c::NO_NID;

	CHEdge() { }
	CHEdge(EdgeT const& edge): EdgeT(edge) { }
	CHEdge(EdgeT const& edge, EdgeID child_edge1, EdgeID child_edge2, NodeID center_node)
		: EdgeT(edge), child_edge1(child_edge1),
		child_edge2(child_edge2), center_node(center_node){}
};

template<typename EdgeT>
struct _MakeCHEdge
{
	typedef CHEdge<EdgeT> type;
};

template<typename EdgeT>
struct _MakeCHEdge<CHEdge<EdgeT>>
{
	typedef CHEdge<EdgeT> type;
};

template<typename EdgeT>
using MakeCHEdge = typename _MakeCHEdge<typename std::remove_reference<EdgeT>::type>::type;
/* remove possible CHEdge<> with: typename MakeCHEdge<T>::base_edge_type */

template<typename EdgeT>
MakeCHEdge<EdgeT> make_shortcut(EdgeT const& edge1, EdgeT const& edge2) {
	typedef typename MakeCHEdge<EdgeT>::base_edge_type base;
	assert(edge1.tgt == edge2.src);
	return MakeCHEdge<EdgeT>(concat(static_cast<base const&>(edge1), static_cast<base const&>(edge2)), edge1.id, edge2.id, edge1.tgt);
}

struct Edge
{
	EdgeID id = c::NO_EID;
	NodeID src = c::NO_NID;
	NodeID tgt = c::NO_NID;
	uint dist = c::NO_DIST;

	Edge() { }
	Edge(EdgeID id, NodeID src, NodeID tgt, uint dist)
		: id(id), src(src), tgt(tgt), dist(dist) { }

	uint distance() const { return dist; }
};
Edge concat(Edge const& edge1, Edge const& edge2);

template <typename EdgeT>
struct MetricEdge : EdgeT
{
	uint metric = 0;

	MetricEdge() {}
	MetricEdge(EdgeT const& edge) : EdgeT(edge) {}
	MetricEdge(EdgeT const& edge, uint metric) : EdgeT(edge), metric(metric){}

	friend MetricEdge concat(MetricEdge const& edge1, MetricEdge const& edge2) {
		return MetricEdge(concat(static_cast<EdgeT const&>(edge1), static_cast<EdgeT const&>(edge2)), edge1.metric + edge2.metric);
	}
};

struct OSMEdge
{
	EdgeID id = c::NO_EID;
	NodeID src = c::NO_NID;
	NodeID tgt = c::NO_NID;
	uint dist = c::NO_DIST;
	uint type = 0;
	int speed = -1;

	OSMEdge() { }
	OSMEdge(EdgeID id, NodeID src, NodeID tgt, uint dist, uint type, int speed)
	: id(id), src(src), tgt(tgt), dist(dist), type(type), speed(speed) { }

	uint distance() const { return dist; }

	operator Edge() const
	{
		return Edge(id, src, tgt, dist);
	}
};
OSMEdge concat(OSMEdge const& edge1, OSMEdge const& edge2);

struct OSMDistEdge : OSMEdge
{

};

/*
 * EdgeSort
 */

template <typename EdgeT>
struct EdgeSortSrc
{
	bool operator()(EdgeT const& edge1, EdgeT const& edge2) const
	{
		return edge1.src < edge2.src ||
		       (edge1.src == edge2.src && edge1.tgt < edge2.tgt);
	}
};

template <typename EdgeT>
struct EdgeSortTgt
{
	bool operator()(EdgeT const& edge1, EdgeT const& edge2) const
	{
		return edge1.tgt < edge2.tgt ||
		       (edge1.tgt == edge2.tgt && edge1.src < edge2.src);
	}
};

}

#endif
