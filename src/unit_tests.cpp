/* always Print() stuff */
#ifdef NVERBOSE
# undef NVERBOSE
#endif

#include "unit_tests.h"

#include "nodes_and_edges.h"
#include "graph.h"
#include "file_formats.h"
#include "dijkstra.h"
#include "scc_extractor.h"

namespace scc
{

void unit_tests::testAll()
{
	unit_tests::testNodesAndEdges();
	unit_tests::testGraph();
	unit_tests::testDijkstra();
	unit_tests::testSccExtractor();
}

void unit_tests::testNodesAndEdges()
{
	Print("\n=================================");
	Print("TEST: Start Nodes And Edges test.");
	Print("=================================\n");

	typedef MetricEdge<Edge> MetricEdge;
	typedef CHEdge<MetricEdge> CHEdge;

	CHNode<Node> node0(Node(0), 0);
	CHNode<Node> node1(Node(1), 1);
	CHNode<Node> node2(Node(2), 2);

	MetricEdge edge0(Edge(0, node0.id, node1.id, 42), 23);
	MetricEdge edge1(Edge(1, node1.id, node2.id, 24), 32);
	CHEdge ch_edge(make_shortcut(edge0, edge1));

	Test(otherNode(edge0, IN) == 0);
	Test(otherNode(ch_edge, OUT) == 2);

	Print("\n======================================");
	Print("TEST: Nodes and edges test successful.");
	Print("======================================\n");
}

void unit_tests::testGraph()
{
	Print("\n=======================");
	Print("TEST: Start Graph test.");
	Print("=======================\n");

	Graph<OSMNode, OSMEdge> g;
	g.init(FormatFMI::Reader::readGraph<OSMNode, OSMEdge>("../test_data/15kSZHK_fmi.txt"));

	/* Test the normal iterator. */
	for (NodeID node_id(0); node_id<g.getNrOfNodes(); node_id++) {
		Test(g.getNode(node_id).id == node_id);

		/* Find for every out_edge the corresponding in edge. */
		for (auto const& out_edge: g.nodeEdges(node_id, OUT)) {
			bool found(false);

			for (auto const& in_edge: g.nodeEdges(out_edge.tgt, IN)) {
				if (equalEndpoints(in_edge, out_edge)) {
					found = true;
					break;
				}
			}

			Test(found);
		}
	}

	Print("\n============================");
	Print("TEST: Graph test successful.");
	Print("============================\n");
}

void unit_tests::testDijkstra()
{
	Print("\n============================");
	Print("TEST: Start Dijkstra test.");
	Print("============================\n");

	Graph<OSMNode, Edge> g;
	g.init(FormatSTD::Reader::readGraph<OSMNode, Edge>("../test_data/test"));

	Dijkstra<OSMNode, Edge> dij(g);
	std::vector<EdgeID> path;
	NodeID tgt(g.getNrOfNodes() - 1);
	uint dist = dij.calcShopa(0, tgt, path);

	Print("Dist of Dijkstra from 0 to " << tgt << ": " << dist);
	Test(dist == 18);

	Print("Shortest path from 0 to " << tgt << ":");
	for (uint i(0); i<path.size(); i++) {
		Edge const& edge(g.getEdge(path[i]));
		Print("EdgeID: " << path[i] << ", src: " << edge.src << ", tgt: " << edge.tgt);
	}

	Print("Test if shortest paths are the same from both sides for the 'test' graph.");
	for (NodeID src(0); src<g.getNrOfNodes(); src++) {
		for (NodeID tgt(src); tgt<g.getNrOfNodes(); tgt++) {
			Test(dij.calcShopa(src, tgt, path) == dij.calcShopa(tgt, src, path));
		}
	}

	Print("\n=================================");
	Print("TEST: Dijkstra test successful.");
	Print("=================================\n");
}

void unit_tests::testSccExtractor()
{
	Print("\n==============================");
	Print("TEST: Start SccExtractor test.");
	Print("==============================\n");

	Graph<OSMNode, OSMEdge> g;
	g.init(readGraph<OSMNode, OSMEdge>(FMI_DIST, "../test_data/15kSZHK_fmi.txt"));

	/* Manually constrcut sccs */

	/* NOTE: normally computeSccs() should be used but it's not suited
	 * for testing purposes. When computeSccs() is changed also this
	 * test should be adapted! */
	SccExtractor<OSMNode, OSMEdge> scc_extractor(g);

	std::vector<NodeID> S;
	S.reserve(g.getNrOfNodes());

	scc_extractor.dfsFirstPass(S);
	scc_extractor.dfsSecondPass(S);
	scc_extractor.addEdgesToSccs();

	/* Now test if they really are sccs */
	auto const& scc_vec(scc_extractor.scc_vec);
	std::vector<uint> comp_num(g.getNrOfNodes());

	for (uint i(0); i<scc_vec.size(); i++) {
		for (auto const& node: scc_vec[i].nodes) {
			comp_num[node.id] = i;
		}
	}

	/* 1) test if single components are strongly connected */
	std::vector<bool> found(g.getNrOfNodes(), false);
	for (uint i(0); i<scc_vec.size(); i++) {
		NodeID start_node(scc_vec[i].nodes[0].id);
		std::vector<NodeID> dfs_stack = { start_node };
		found[start_node] = true;
		uint found_nodes_count(1);

		while (!dfs_stack.empty()) {
			NodeID current_node(dfs_stack.back());
			dfs_stack.pop_back();

			for (auto const& edge: g.nodeEdges(current_node, OUT)) {
				NodeID tgt(edge.tgt);
				if (comp_num[tgt] == i && !found[tgt]) {
					dfs_stack.push_back(tgt);
					found[tgt] = true;
					found_nodes_count++;
				}
			}
		}

		Test(found_nodes_count == scc_vec[i].nodes.size());
	}

	/* 2) test if components are pairwise not strongly connected */
	Dijkstra<OSMNode, OSMEdge> dij(g);
	std::vector<EdgeID> path;

	for (uint i(0); i<scc_vec.size(); i++) {
		for (uint j(i+1); j<scc_vec.size(); j++) {
			NodeID node1(scc_vec[i].nodes[0].id);
			NodeID node2(scc_vec[j].nodes[0].id);
			Test(dij.calcShopa(node1, node2, path) == c::NO_DIST
					|| dij.calcShopa(node2, node1, path) == c::NO_DIST);
		}
	}

	scc_extractor.resetNodeIDsInSccs();
	writeGraphFile(FMI, "../test_data/15kSZHK_fmi_szhk.txt", scc_extractor.getGraphDataOfLargestScc());

	Print("\n===================================");
	Print("TEST: SccExtractor test successful.");
	Print("===================================\n");

}

}