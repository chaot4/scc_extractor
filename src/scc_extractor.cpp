#ifdef NVERBOSE
# undef NVERBOSE
#endif

#include "graph.h"
#include "file_formats.h"
#include "scc_extractor.h"

using namespace scc;

int main()
{
	std::string in_file("../test_data/15kSZHK_fmi.txt");
	std::string out_file("../test_data/15kSZHK_fmi_szhk.txt");

	Graph<OSMNode, OSMEdge> g;
	g.init(FormatFMI_DIST::Reader::readGraph<OSMNode, OSMEdge>(in_file));

	SccExtractor<OSMNode, OSMEdge> scc_extractor(g);
	scc_extractor.computeSccs();
	writeGraphFile<FormatFMI::Writer>(out_file, scc_extractor.getGraphDataOfLargestScc());

	return 0;
}
