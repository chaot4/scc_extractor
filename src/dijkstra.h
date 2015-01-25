#ifndef _DIJKSTRA_H
#define _DIJKSTRA_H

#include "graph.h"

#include <vector>
#include <queue>

namespace scc
{

namespace unit_tests
{
	void testDijkstra();
}

template <typename Node, typename Edge>
class Dijkstra
{
	private:
		struct PQElement;
		typedef std::priority_queue<
				PQElement, std::vector<PQElement>, std::greater<PQElement> > PQ;

		Graph<Node, Edge> const& _g;

		std::vector<EdgeID> _found_by;
		std::vector<uint> _dists;
		std::vector<uint> _reset_dists;

		void _reset();
		void _relaxAllEdges(PQ& pq, PQElement const& top);
	public:
		Dijkstra(Graph<Node, Edge> const& g);

		/**
		 * @brief Computes the shortest path between src and tgt.
		 *
		 * @param path The edges of the shortest path in no
		 * particular order.
		 *
		 * @return The distance of the shortest path.
		 */
		uint calcShopa(NodeID src, NodeID tgt,
				std::vector<EdgeID>& path);
};

template <typename Node, typename Edge>
struct Dijkstra<Node, Edge>::PQElement
{
	NodeID node;
	EdgeID found_by;
	uint _dist;

	PQElement(NodeID node, EdgeID found_by, uint dist)
		: node(node), found_by(found_by), _dist(dist) {}

	bool operator>(PQElement const& other) const
	{
		return _dist > other._dist;
	}

	/* make interface look similar to an edge */
	uint distance() const { return _dist; }
};

template <typename Node, typename Edge>
Dijkstra<Node,Edge>::Dijkstra(Graph<Node, Edge> const& g)
	: _g(g), _found_by(g.getNrOfNodes()),
	_dists(g.getNrOfNodes(), c::NO_DIST) {}

template <typename Node, typename Edge>
uint Dijkstra<Node,Edge>::calcShopa(NodeID src, NodeID tgt,
		std::vector<EdgeID>& path)
{
	_reset();
	path.clear();

	PQ pq;
	pq.push(PQElement(src, c::NO_EID, 0));
	_dists[src] = 0;
	_reset_dists.push_back(src);

	// Dijkstra loop
	while (!pq.empty() && pq.top().node != tgt) {
		PQElement top(pq.top());
		pq.pop();

		if (_dists[top.node] == top.distance()) {
			_found_by[top.node] = top.found_by;
			_relaxAllEdges(pq, top);
		}
	}

	if (pq.empty()) {
		Print("No path found from " << src << " to " << tgt << ".");
		return c::NO_DIST;
	}

	// Path backtracking.
	NodeID bt_node(tgt);
	_found_by[tgt] = pq.top().found_by;
	while (bt_node != src) {
		EdgeID edge_id = _found_by[bt_node];
		bt_node = _g.getEdge(edge_id).src;
		path.push_back(edge_id);
	}

	return pq.top().distance();
}

template <typename Node, typename Edge>
void Dijkstra<Node,Edge>::_relaxAllEdges(PQ& pq, PQElement const& top)
{
	for (auto const& edge: _g.nodeEdges(top.node, OUT)) {
		NodeID tgt(edge.tgt);
		uint new_dist(top.distance() + edge.distance());

		if (new_dist < _dists[tgt]) {
			if (_dists[tgt] == c::NO_DIST) {
				_reset_dists.push_back(tgt);
			}
			_dists[tgt] = new_dist;

			pq.push(PQElement(tgt, edge.id, new_dist));
		}
	}
}

template <typename Node, typename Edge>
void Dijkstra<Node,Edge>::_reset()
{
	for (uint i(0), size(_reset_dists.size()); i<size; i++) {
		_dists[_reset_dists[i]] = c::NO_DIST;
	}

	_reset_dists.clear();
}

}

#endif
