#ifndef _SCC_EXTRACTOR
#define _SCC_EXTRACTOR

#include "graph.h"
#include "track_time.h"

#include <vector>
#include <algorithm>

namespace scc
{

namespace unit_tests
{
	void testSccExtractor();
}

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

		void pushUnexpandedAdjacentNodes(NodeID node, std::vector<NodeID>& dfs_stack,
				std::vector<bool>& expanded, EdgeType type);

		void addEdgesToSccs();
		void resetNodeIDsInSccs();

		friend void unit_tests::testSccExtractor();
};

template <typename NodeT, typename EdgeT>
void SccExtractor<NodeT, EdgeT>::computeSccs()
{
	if (g.getNrOfNodes() == 0) {
		Print("Graph is empty. Skipping SCC computation.");
		return;
	}

	Print("Starting the computation of the SCCs.");
	std::vector<NodeID> S;
	S.reserve(g.getNrOfNodes());

#ifndef NVERBOSE
	TrackTime tt(std::cout);
#else
	TrackTime tt;
#endif

	dfsFirstPass(S);
	tt.track("first pass of dfs", false);
	dfsSecondPass(S);
	tt.track("second pass of dfs", false);
	addEdgesToSccs();
	tt.track("edges post processing", false);
	resetNodeIDsInSccs();
	tt.track("node id reset", false);

	tt.summary();

	Print("\nFound " << scc_vec.size() << " SCC(s):");
	Print("====================");
	for (uint i(0); i<scc_vec.size(); i++) {
		Print("Component " << i << ": #nodes: " << scc_vec[i].nodes.size()
				<<", #edges: " << scc_vec[i].edges.size());
	}
}

template <typename NodeT, typename EdgeT>
void SccExtractor<NodeT, EdgeT>::dfsFirstPass(std::vector<NodeID>& S)
{
	std::vector<NodeID> dfs_stack;
	std::vector<bool> settled(g.getNrOfNodes(), false);
	std::vector<bool> expanded(g.getNrOfNodes(), false);
	NodeID start_node(0);

	while (S.size() != g.getNrOfNodes()) {
		while (settled[start_node]) { start_node++; }
		dfs_stack.push_back(start_node);
		assert(start_node < g.getNrOfNodes());

		while (!dfs_stack.empty()) {
			NodeID current_node(dfs_stack.back());

			if (expanded[current_node]) {
				if (!settled[current_node]) {
					S.push_back(current_node);
					settled[current_node] = true;
				}
				dfs_stack.pop_back();
			}
			else {
				pushUnexpandedAdjacentNodes(current_node, dfs_stack, expanded, OUT);
				expanded[current_node] = true;
			}
		}
	}
}

template <typename NodeT, typename EdgeT>
void SccExtractor<NodeT, EdgeT>::pushUnexpandedAdjacentNodes(NodeID node, std::vector<NodeID>& dfs_stack,
		std::vector<bool>& expanded, EdgeType type)
{
	for (EdgeT const& edge: g.nodeEdges(node, type)) {
		NodeID other(otherNode(edge, type));
		if (!expanded[other]) {
			dfs_stack.push_back(other);
		}
	}
}

template <typename NodeT, typename EdgeT>
void SccExtractor<NodeT, EdgeT>::dfsSecondPass(std::vector<NodeID>& S)
{
	std::vector<NodeID> dfs_stack;
	std::vector<bool> expanded(g.getNrOfNodes(), false);

	while (!S.empty()) {
		NodeID start_node(S.back());
		dfs_stack.push_back(start_node);

		/* create new scc */
		scc_vec.push_back(Scc());

		/* dfs */
		while (!dfs_stack.empty()) {
			NodeID current_node(dfs_stack.back());
			dfs_stack.pop_back();

			if (!expanded[current_node]) {
				scc_vec.back().nodes.push_back(g.getNode(current_node));
				pushUnexpandedAdjacentNodes(current_node, dfs_stack, expanded, IN);
				expanded[current_node] = true;
			}
		}

		/* clean stack */
		while (!S.empty() && expanded[S.back()]) {
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
	std::vector<NodeID> new_id(g.getNrOfNodes(), c::NO_NID); // assumes every node is only in one scc

	for (Scc& scc: scc_vec) {
		std::sort(scc.nodes.begin(), scc.nodes.end());

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
