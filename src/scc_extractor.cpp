#include "graph.h"
#include "file_formats.h"
#include "scc_extractor.h"

#include <getopt.h>

using namespace scc;

void printHelp()
{
	std::cout
		<< "Usage: ./scc_extractor [ARGUMENTS]\n"
		<< "Mandatory arguments are:\n"
		<< "  -i, --infile <path>        Read graph from <path>\n"
		<< "Optional arguments are:\n"
		<< "  -f, --informat <format>    Expects infile in <format> (SIMPLE, STD, FMI, FMI_DIST - default FMI_DIST)\n"
		<< "  -o, --outfile <path>       Write graph to <path> (default: scc_out.graph)\n"
		<< "  -g, --outformat <format>   Writes outfile in <format> (SIMPLE, STD, FMI - default FMI)\n";
}

int main(int argc, char* argv[])
{
	/*
	 * Containers for arguments.
	 */

	std::string infile("");
	FileFormat informat(FMI_DIST);
	std::string outfile("scc_out.graph");
	FileFormat outformat(FMI);

	/*
	 * Getopt argument parsing.
	 */

	const struct option longopts[] = {
		{"help",	no_argument,        0, 'h'},
		{"infile",	required_argument,  0, 'i'},
		{"informat",	required_argument,  0, 'f'},
		{"outfile",     required_argument,  0, 'o'},
		{"outformat",   required_argument,  0, 'g'},
		{0,0,0,0},
	};

	int index(0);
	int iarg(0);
	opterr = 1;

	while((iarg = getopt_long(argc, argv, "hi:f:o:g:", longopts, &index)) != -1) {
		switch (iarg) {
			case 'h':
				printHelp();
				return 0;
				break;
			case 'i':
				infile = optarg;
				break;
			case 'f':
				informat = toFileFormat(optarg);
				break;
			case 'o':
				outfile = optarg;
				break;
			case 'g':
				outformat = toFileFormat(optarg);
				break;
			default:
				printHelp();
				return 1;
				break;
		}
	}

	if (infile == "") {
		std::cerr << "No input file specified! Exiting.\n";
		std::cerr << "Use ./scc_extractor --help to print the usage.\n";
		return 1;
	}

	Graph<OSMNode, OSMEdge> g;
	g.init(readGraph<OSMNode, OSMEdge>(informat, infile));

	SccExtractor<OSMNode, OSMEdge> scc_extractor(g);
	scc_extractor.computeSccs();

	writeGraphFile(outformat, outfile, scc_extractor.getGraphDataOfLargestScc());

	return 0;
}
