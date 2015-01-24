#ifndef _SCC_EXTRACTOR
#define _SCC_EXTRACTOR

#include "graph.h"

#include <vector>
#include <algorithm>

namespace scc
{

template <typename NodeT, typename EdgeT>
class SccExtractor
{
	public:
		struct Scc
		{
			std::vector<NodeT> nodes;
			std::vector<EdgeT> edges;
		};

		SccExtractor(Graph<NodeT,EdgeT>& g) : g(g) {}

		/* Kosaraju's algorithm is used. */
		void computeSccs();

		std::vector<Scc> const& getSccs() const;
		Scc const& getScc(uint scc_index) const;

		GraphOutData<NodeT,EdgeT> getGraphDataOfScc(uint scc_index) const;
		GraphOutData<NodeT,EdgeT> getGraphDataOfLargestScc() const;
	private:
		Graph<NodeT, EdgeT> const& g;
		std::vector<Scc> scc_vec;

		void dfsFirstPass(std::vector<NodeID>& S);
		void dfsSecondPass(std::vector<NodeID>& S);

		NodeID getFirstUnseen(std::vector<bool> seen);
		void pushUnseenAdjacentNodes(NodeID node, std::vector<NodeID>& dfs_stack,
				std::vector<bool>& seen, EdgeType type);

		void addEdgesToSccs();
		void resetNodeIDsInSccs();
};

template <typename NodeT, typename EdgeT>
void SccExtractor<NodeT, EdgeT>::computeSccs()
{
	if (g.getNrOfNodes() == 0) {
		Print("Graph is empty. Skipping SCC computation.");
		return;
	}

	std::vector<NodeID> S;
	dfsFirstPass(S);
	dfsSecondPass(S);
	addEdgesToSccs();
	resetNodeIDsInSccs();

	Print("Found the following SCCs:");
	Print("=========================");
	for (uint i(0); i<scc_vec.size(); i++) {
		Scc const& scc(scc_vec[i]);
		uint nr_of_nodes(scc.nodes.size());
		uint nr_of_edges(scc.edges.size());
		Print("Component " << i+1 << ": #nodes: " << nr_of_nodes << ", #edges: " << nr_of_edges);
	}
}

template <typename NodeT, typename EdgeT>
void SccExtractor<NodeT, EdgeT>::dfsFirstPass(std::vector<NodeID>& S)
{
	std::vector<NodeID> dfs_stack;
	std::vector<bool> seen(g.getNrOfNodes(), false);
	std::vector<bool> expanded(g.getNrOfNodes(), false);

	while (S.size() != g.getNrOfNodes()) {
		NodeID start_node(getFirstUnseen(seen));
		dfs_stack.push_back(start_node);
		seen[start_node] = true;

		while (!dfs_stack.empty()) {
			NodeID current_node(dfs_stack.back());

			if (expanded[current_node]) {
				dfs_stack.pop_back();
				S.push_back(current_node);
			}
			else {
				pushUnseenAdjacentNodes(current_node, dfs_stack, seen, OUT);
				expanded[current_node] = true;
			}
		}
	}
}

template <typename NodeT, typename EdgeT>
NodeID SccExtractor<NodeT, EdgeT>::getFirstUnseen(std::vector<bool> seen)
{
	NodeID i(0);
	while (seen[i]) { // assumes that there's at least one unseen
		i++;
	}

	assert(i < g.getNrOfNodes());
	return i;
}

template <typename NodeT, typename EdgeT>
void SccExtractor<NodeT, EdgeT>::pushUnseenAdjacentNodes(NodeID node, std::vector<NodeID>& dfs_stack,
		std::vector<bool>& seen, EdgeType type)
{
	for (EdgeT const& edge: g.nodeEdges(node, type)) {
		NodeID other(otherNode(edge, type));
		if (!seen[other]) {
			dfs_stack.push_back(other);
			seen[other] = true;
		}
	}
}

template <typename NodeT, typename EdgeT>
void SccExtractor<NodeT, EdgeT>::dfsSecondPass(std::vector<NodeID>& S)
{
	std::vector<NodeID> dfs_stack;
	std::vector<bool> seen(g.getNrOfNodes(), false);

	while (!S.empty()) {
		NodeID start_node(S.back());
		dfs_stack.push_back(start_node);
		seen[start_node] = true;

		/* create new scc */
		scc_vec.push_back(Scc());

		/* dfs */
		while (!dfs_stack.empty()) {
			NodeID current_node(dfs_stack.back());
			dfs_stack.pop_back();

			scc_vec.back().nodes.push_back(g.getNode(current_node));

			pushUnseenAdjacentNodes(current_node, dfs_stack, seen, IN);
		}

		/* clean stack */
		while (!S.empty() && seen[S.back()]) {
			S.pop_back();
		}
	}
}

template <typename NodeT, typename EdgeT>
void SccExtractor<NodeT, EdgeT>::addEdgesToSccs()
{
	for (Scc& scc: scc_vec) {
		std::vector<bool> in_scc(g.getNrOfNodes(), false);

		for (NodeT const& node: scc.nodes) {
			in_scc[node.id] = true;
		}

		for (NodeT const& node: scc.nodes) {
			for (EdgeT const& edge: g.nodeEdges(node.id, OUT)) {
				if (in_scc[edge.tgt]) {
					scc.edges.push_back(edge);
				}
			}
		}
	}
}

template <typename NodeT, typename EdgeT>
void SccExtractor<NodeT, EdgeT>::resetNodeIDsInSccs()
{
	for (Scc& scc: scc_vec) {
		std::sort(scc.nodes.begin(), scc.nodes.end());

		std::vector<NodeID> new_id(g.getNrOfNodes(), c::NO_NID);
		for (uint i(0); i<scc.nodes.size(); i++) {
			NodeT& node(scc.nodes[i]);
			new_id[node.id] = i;
			node.id = i;
		}

		for (EdgeT& edge: scc.edges) {
			edge.src = new_id[edge.src];
			edge.tgt = new_id[edge.tgt];
		}

		std::sort(scc.edges.begin(), scc.edges.end(), EdgeSortSrc<EdgeT>());
	}
}

template <typename NodeT, typename EdgeT>
auto SccExtractor<NodeT, EdgeT>::getSccs() const -> std::vector<Scc> const&
{
	return scc_vec;
}

template <typename NodeT, typename EdgeT>
auto SccExtractor<NodeT, EdgeT>::getScc(uint scc_index) const -> Scc const&
{
	return scc_vec[scc_index];
}

template <typename NodeT, typename EdgeT>
GraphOutData<NodeT,EdgeT> SccExtractor<NodeT, EdgeT>::getGraphDataOfScc(uint scc_index) const
{
	return GraphOutData<NodeT,EdgeT>{scc_vec[scc_index].nodes, scc_vec[scc_index].edges, g.getMetadata()};
}

template <typename NodeT, typename EdgeT>
GraphOutData<NodeT,EdgeT> SccExtractor<NodeT, EdgeT>::getGraphDataOfLargestScc() const
{
	uint max_size(0);
	uint max_scc_index(0);
	for (uint i(0); i<scc_vec.size(); i++) {
		if (scc_vec[i].nodes.size() > max_size) {
			max_size = scc_vec[i].nodes.size();
			max_scc_index = i;
		}
	}

	return getGraphDataOfScc(max_scc_index);
}

}

#endif
