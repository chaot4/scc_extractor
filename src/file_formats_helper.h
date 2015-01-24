#ifndef _FILE_FORMATS_HELPER_H
#define _FILE_FORMATS_HELPER_H

#include "nodes_and_edges.h"
#include "function_traits.h"

namespace scc {
	template<typename Writer, typename NodeT, typename EdgeT>
	struct writer_can_write
	{
		typedef CHNode< typename std::decay<NodeT>::type > d_node;
		typedef typename std::decay<EdgeT>::type d_edge;
		typedef typename is_static_castable<d_node, typename Writer::node_type>::type support_node;
		typedef typename is_static_castable<d_edge, typename Writer::edge_type>::type support_edge;

		static constexpr bool value = support_node::value && support_edge::value;
	};


	template<typename Implementation>
	struct SimpleReader
	{
		typedef typename std::remove_reference<decltype(result_of(&Implementation::readNode))>::type node_type;
		typedef typename std::remove_reference<decltype(result_of(&Implementation::readEdge))>::type edge_type;
		typedef MakeCHEdge<edge_type> chedge_type;

		template<typename NodeT = node_type, typename EdgeT = chedge_type>
		struct can_read
		{
			typedef is_static_castable_t<node_type, NodeT> support_node;
			typedef is_static_castable_t<edge_type, EdgeT> support_edge;

			static constexpr bool value = support_node::value && support_edge::value;
		};

		template<typename NodeT = node_type, typename EdgeT = chedge_type, typename std::enable_if<!can_read<NodeT, EdgeT>::value>::type* = nullptr>
		static GraphInData<NodeT, EdgeT> readGraph(std::istream& is)
		{
			Print("Can't read nodes / edges in this format");
			std::abort();
		}

		template<typename NodeT = node_type, typename EdgeT = chedge_type, typename std::enable_if<can_read<NodeT, EdgeT>::value>::type* = nullptr>
		static GraphInData<NodeT, EdgeT> readGraph(std::istream& is)
		{
			Implementation impl(is);
			NodeID nr_of_nodes = 0;
			EdgeID nr_of_edges = 0;
			GraphInData<NodeT, EdgeT> result;
			impl.readHeader(nr_of_nodes, nr_of_edges, result.meta_data);

			result.nodes.reserve(nr_of_nodes);
			result.edges.reserve(nr_of_edges);

			Print("Number of nodes: " << nr_of_nodes);
			Print("Number of edges: " << nr_of_edges);

			for (NodeID i = 0; i < nr_of_nodes; ++i) {
				result.nodes.push_back(static_cast<NodeT>(impl.readNode((NodeID) i)));
			}
			Print("Read all the nodes.");

			for (EdgeID i = 0; i < nr_of_edges; ++i) {
				auto edge = static_cast<EdgeT>(impl.readEdge((EdgeID) i));
				if (edge.src == edge.tgt) {
					std::cerr << "WARNING: input contained loop edge (@" << i << "), dropped edge.\n";
					--i; --nr_of_edges;
					continue;
				}
				result.edges.push_back(std::move(edge));
			}
			Print("Read all the edges.");

			return result;
		}

		template<typename NodeT = node_type, typename EdgeT = chedge_type>
		static GraphInData<NodeT, EdgeT> readGraph(std::string const& filename)
		{
			std::ifstream is(filename);
			if (!is.is_open()) {
				std::cerr << "FATAL_ERROR: Couldn't open graph file \'" <<
					filename << "\'. Exiting." << std::endl;
				std::abort();
			}
			return readGraph<NodeT, EdgeT>(is);
		}
	};


	template<typename Implementation>
	struct SimpleWriter
	{
		typedef decay_tuple_element<0, decltype(return_args(&Implementation::writeNode))> node_type;
		typedef decay_tuple_element<0, decltype(return_args(&Implementation::writeEdge))> edge_type;


		template<typename NodeT, typename EdgeT>
		using can_write = writer_can_write<SimpleWriter, NodeT, EdgeT>;

		template<typename NodeT, typename EdgeT, typename std::enable_if<!can_write<NodeT, EdgeT>::value>::type* = nullptr>
		static void writeGraph(std::ostream&, GraphOutData<NodeT, EdgeT> const&)
		{
			Print("Can't export nodes / edges in this format");
			std::abort();
		}

		template<typename NodeT, typename EdgeT, typename std::enable_if<can_write<NodeT, EdgeT>::value>::type* = nullptr>
		static void writeGraph(std::ostream& os, GraphOutData<NodeT, EdgeT> const& data)
		{
			NodeID nr_of_nodes(data.nodes.size());
			EdgeID nr_of_edges(data.edges.size());

			Print("Exporting " << nr_of_nodes << " nodes and " << nr_of_edges << " edges");

			Implementation impl(os);

			impl.writeHeader(nr_of_nodes, nr_of_edges, data.meta_data);

			NodeID node_id = 0;
			for (auto const& node: data.nodes) {
				impl.writeNode(static_cast<node_type>(node), (NodeID) node_id);
				++node_id;
			}
			Print("Exported all nodes.");

			EdgeID edge_id = 0;
			for (auto const& edge: data.edges) {
				impl.writeEdge(static_cast<edge_type>(edge), (EdgeID) edge_id);
				++edge_id;
			}
			Print("Exported all edges.");
		}

		template<typename NodeT, typename EdgeT, typename std::enable_if<!can_write<NodeT, EdgeT>::value>::type* = nullptr>
		static void writeCHGraph(std::ostream&, GraphCHOutData<NodeT, EdgeT> const&)
		{
			Print("Can't export nodes / edges in this format");
			std::abort();
		}

		template<typename NodeT, typename EdgeT, typename std::enable_if<can_write<NodeT, EdgeT>::value>::type* = nullptr>
		static void writeCHGraph(std::ostream& os, GraphCHOutData<NodeT, EdgeT> const& data)
		{
			NodeID nr_of_nodes(data.nodes.size());
			EdgeID nr_of_edges(data.edges.size());

			Print("Exporting " << nr_of_nodes << " nodes and " << nr_of_edges << " edges");

			Implementation impl(os);

			impl.writeHeader(nr_of_nodes, nr_of_edges, data.meta_data);

			NodeID node_id = 0;
			for (auto const& node: data.nodes) {
				impl.writeNode(static_cast<node_type>(makeCHNode(node, data.node_levels[node_id])), (NodeID) node_id);
				++node_id;
			}
			Print("Exported all nodes.");

			EdgeID edge_id = 0;
			for (auto const& edge: data.edges) {
				impl.writeEdge(static_cast<edge_type>(edge), (EdgeID) edge_id);
				++edge_id;
			}
			Print("Exported all edges.");
		}
	};

}


#endif
