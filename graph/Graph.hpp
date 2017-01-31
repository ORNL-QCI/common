/***********************************************************************************
 * Copyright (c) 2016, UT-Battelle
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the <organization> nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Contributors:
 *   Initial API and implementation - Alex McCaskey
 *
 **********************************************************************************/
#ifndef XACC_UTILS_IGRAPH_HPP_
#define XACC_UTILS_IGRAPH_HPP_

#include <boost/graph/adjacency_list.hpp>
#include <type_traits>
#include <boost/graph/exterior_property.hpp>
#include <boost/graph/floyd_warshall_shortest.hpp>
#include <boost/graph/eccentricity.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/fusion/include/for_each.hpp>
#include <numeric>
#include "QCIError.hpp"
#include <utility>

using namespace boost;

namespace qci {
namespace common {

/**
 * Utility structs to help determine if
 * we have been given valid Vertices.
 */
template<typename T, typename = void>
struct is_valid_vertex: std::false_type {
};
template<typename T>
struct is_valid_vertex<T, decltype(std::declval<T>().properties, void())> : std::true_type {
};

/**
 * The base class of all QCI Vertices for the
 * QCI Common Graph class. All Vertices must keep
 * track of a set of properties, stored as a tuple.
 */
template<typename ... Properties>
class QCIVertex {
public:
	std::tuple<Properties...> properties;
	std::vector<std::string> propertyNames;
	QCIVertex() : propertyNames(sizeof...(Properties)) {}
	std::string getPropertyName(const int index) {
		return propertyNames[index];
	}
};

/**
 * For now, we only allow Edges with weight property.
 */
struct DefaultEdge {
	double weight = 0.0;
};

enum GraphType {
	Undirected, Directed
};

// Overload stream operator for vector
template<class T>
std::ostream& operator << (std::ostream& os, const std::vector<T>& v) {
	os << "[";
	for (std::size_t i = 0; i < v.size(); i++) {
		os << v[i] << (i == v.size() - 1 ? "" : " ");
	}
	os << "]";
	return os;
}

template<std::size_t> struct int_{};

template <class Tuple, size_t Pos>
std::ostream& print_tuple(std::ostream& out, const Tuple& t, int_<Pos> ) {
  out << std::get< std::tuple_size<Tuple>::value-Pos >(t) << ',';
  return print_tuple(out, t, int_<Pos-1>());
}

template <class Tuple>
std::ostream& print_tuple(std::ostream& out, const Tuple& t, int_<1> ) {
  return out << std::get<std::tuple_size<Tuple>::value-1>(t);
}

template <class... Args>
std::ostream& operator<<(std::ostream& out, const std::tuple<Args...>& t) {
  out << std::string(" [label=\"");
  print_tuple(out, t, int_<sizeof...(Args)>());
  return out << "\"]";
}

/**
 * The Graph class provides a generic data structure modeling
 * mathematical graph structures. It is templated on the vertex
 * type, allowing for graphs with a wide variety of graph nodes
 * (for example, in quantum computing - graph of tensors, graph of
 * Ising parameters, etc.)
 *
 * All provided Vertex types must be a subclass of the
 * QCIVertex in order to properly provide a tuple of vertex
 * properties.
 * s
 */
template<typename Vertex, GraphType type = Undirected>
class Graph {

	// Make sure we've been given a valid Vertex
	static_assert(is_valid_vertex<Vertex>::value, "QCI Graph can only take Vertices that are derived from QCIVertex, or have a tuple properties member.");

	using graph_type = typename std::conditional<(type == GraphType::Undirected), undirectedS, directedS>::type;

	// Setup some easy to use aliases
	using adj_list = adjacency_list<vecS, vecS, graph_type, Vertex, DefaultEdge>;
	using BoostGraph = std::shared_ptr<adj_list>;
	using vertex_type = typename boost::graph_traits<adjacency_list<vecS, vecS, undirectedS, Vertex, DefaultEdge>>::vertex_descriptor;
	using edge_type = typename boost::graph_traits<adjacency_list<vecS, vecS, undirectedS, Vertex, DefaultEdge>>::edge_descriptor;

protected:

	/**
	 * This is a custom utility class for writing
	 * QCIVertices with user-defined properties.
	 */
	class QCIVertexPropertiesWriter {
	protected:
		adj_list graph;
	public:
		QCIVertexPropertiesWriter(adj_list& list) :
				graph(list) {
		}
		template<class BoostVertex>
		void operator()(std::ostream& out, const BoostVertex& v) const {
			auto node = vertex(v, graph);
			std::stringstream ss;
			ss << graph[node].properties;

			// Now we have a string of comma separated parameters
			std::string result;
			auto vertexString = ss.str();
			boost::trim(vertexString);
			std::vector<std::string> splitVec;
			boost::split(splitVec, vertexString, boost::is_any_of(","));

			int counter = 0;
			for (std::size_t i = 0; i < splitVec.size(); i++) {
				if (!graph[node].propertyNames[counter].empty()) {
					auto s = splitVec[i];
					if (i == 0) {
						s.insert(8, graph[node].propertyNames[counter] + "=");
					} else {
						s.insert(0, graph[node].propertyNames[counter] + "=");
					}
					counter++;
					result += s + ",";
				}
			}

			result = result.substr(0,result.size() - 1);
			if (result.substr(result.size() - 2, result.size()) != "\"]") {
				result += "\"]";
			}
			out << " " << result;
		}
	};

	/**
	 * The actual graph data structure we are
	 * delegating to.
	 */
	BoostGraph _graph;

public:

	/**
	 * The nullary constructor
	 */
	Graph() {
		_graph = std::make_shared<adj_list>();
	}

	/**
	 * The constructor, constructs a graph with
	 * specified number of vertices.
	 *
	 * @param numberOfVertices The number of vertices
	 */
	Graph(const int numberOfVertices) {
		_graph = std::make_shared<adj_list>(
				numberOfVertices);
	}

	/**
	 * Add an edge between the vertices with given provided
	 * indices and edge weight.
	 *
	 * @param srcIndex Index of the starting vertex
	 * @param tgtIndex Index of the ending vertex
	 * @param edgeWeight The edge weight
	 */
	void addEdge(const int srcIndex, const int tgtIndex, const double edgeWeight) {
		auto edgeBoolPair = add_edge(vertex(srcIndex, *_graph.get()),
				vertex(tgtIndex, *_graph.get()), *_graph.get());
		if (!edgeBoolPair.second) {

		}
		(*_graph.get())[edgeBoolPair.first].weight = edgeWeight;
	}

	/**
	 * Add an edge with default edge weight between the
	 * vertices at the provided indices.
	 *
	 * @param srcIndex Index of the starting vertex
	 * @param tgtIndex Index of the ending vertex
	 */
	void addEdge(const int srcIndex, const int tgtIndex) {
		add_edge(vertex(srcIndex, *_graph.get()),
				vertex(tgtIndex, *_graph.get()), *_graph.get());
	}

	/**
	 * Add a vertex to this Graph.
	 */
	void addVertex() {
		add_vertex(*_graph.get());
	}

	/**
	 * Add a vertex to this graph with the provided properties.
	 * s
	 * @param properties
	 */
	template<typename ... Properties>
	void addVertex(Properties ... properties) {
		auto v = add_vertex(*_graph.get());
		(*_graph.get())[v].properties = std::make_tuple(properties...);
	}

	void addVertex(Vertex& vertex) {
		auto v = add_vertex(*_graph.get());
		(*_graph.get())[v].properties = vertex.properties;
	}

	/**
	 * Set an existing vertices properties.
	 *
	 * @param index The index of the vertex
	 * @param properties The new properties for the vertex
	 */
	template<typename... Properties>
	void setVertexProperties(const int index, Properties... properties) {
		auto v = vertex(index, *_graph.get());
		(*_graph.get())[v].properties = std::make_tuple(properties...);
	}

	/**
	 * Set a specific vertex property for the vertex at given index.
	 *
	 * @param index The index of the vertex
	 * @param prop The property to set.
	 */
	template<const int PropertyIndex>
	void setVertexProperty(const int index,
			decltype(std::get<PropertyIndex>(std::declval<Vertex>().properties)) prop) {
		auto v = vertex(index, *_graph.get());
		std::get<PropertyIndex>((*_graph.get())[v].properties) = prop;
		return;
	}

	auto getVertexProperties(const int index) -> decltype((*_graph.get())[index].properties) {
		return (*_graph.get())[index].properties;
	}

	/**
	 * Return the vertex property of the vertex
	 * at the given index and at the provided
	 * valid vertex property template index.
	 *
	 * @param index The index of the vertex
	 * @return property The property value.
	 */
	template<const int PropertyIndex>
	decltype(std::get<PropertyIndex>(std::declval<Vertex>().properties))& getVertexProperty(
			const int index) {
		auto v = vertex(index, *_graph.get());
		return std::get<PropertyIndex>((*_graph.get())[v].properties);
	}

	/**
	 * Set the weight on the edge between the vertices at the
	 * provided indices.
	 *
	 * @param srcIndex The starting vertex index
	 * @param tgtIndex The ending vertex index
	 * @param weight The weight to set.
	 */
	void setEdgeWeight(const int srcIndex, const int tgtIndex, const double weight) {
		auto e = edge(vertex(srcIndex, *_graph.get()), vertex(tgtIndex, *_graph.get()), *_graph.get());
		(*_graph.get())[e.first].weight = weight;
	}

	/**
	 * Return the edge weight at the edge between
	 * the provided vertices.
	 *
	 * @param srcIndex The starting vertex index
	 * @param tgtIndex The ending vertex index
	 * @return The edge weight
	 */
	double getEdgeWeight(const int srcIndex, const int tgtIndex) {
		auto e = edge(vertex(srcIndex, *_graph.get()), vertex(tgtIndex, *_graph.get()), *_graph.get());
		return (*_graph.get())[e.first].weight;
	}

	/**
	 * Return true if there is an edge between the
	 * two vertices at the given vertex indices.
	 *
	 * @param srcIndex The starting vertex index
	 * @param tgtIndex The ending vertex index
	 * @return exists Boolean indicating if edge exists or not
	 */
	bool edgeExists(const int srcIndex, const int tgtIndex) {
		return edge(vertex(srcIndex, *_graph.get()),
				vertex(tgtIndex, *_graph.get()), *_graph.get()).second;
	}

	/**
	 * Return the vertex degree at the given vertex index.
	 *
	 * @param index The index of the vertex
	 * @return degree The degree of the vertex
	 */
	int degree(const int index) {
		return boost::degree(vertex(index, *_graph.get()), *_graph.get());
	}

	/**
	 * Return the diameter of this Graph.
	 *
	 * @return diameter The graph diameter
	 */
	int diameter() {
		// Set some aliases we'll need
		using DistanceProperty = boost::exterior_vertex_property<adj_list, int>;
		using DistanceMatrix = typename DistanceProperty::matrix_type;
		using DistanceMatrixMap = typename DistanceProperty::matrix_map_type;
		using EccentricityProperty = boost::exterior_vertex_property<adj_list, int>;
		using EccentricityContainer = typename EccentricityProperty::container_type;
		using EccentricityMap = typename EccentricityProperty::map_type;

		// Construct the distance mapping
		DistanceMatrix distances(order());
		DistanceMatrixMap dm(distances, *_graph.get());
		constant_property_map<edge_type, int> wm(1);

		// Compute the shortest paths
		floyd_warshall_all_pairs_shortest_paths(*_graph.get(), dm, weight_map(wm));

		// Now get diameter information
		EccentricityContainer eccs(order());
		EccentricityMap em(eccs, *_graph.get());

		// Return the diameter
		return all_eccentricities(*_graph.get(), dm, em).second;
	}

	/**
	 * Return the number of edges.
	 * @return nEdges The number of edges.
	 */
	int size() {
		return num_edges(*_graph.get());
	}

	/**
	 * Return the number of vertices in this graph
	 *
	 * @return nVerts The number of vertices.
	 */
	int order() {
		return num_vertices(*_graph.get());
	}

	/**
	 * Write this graph in a graphviz dot format to the
	 * provided ostream.
	 *
	 * @param stream
	 */
	void write(std::ostream& stream) {
		std::stringstream ss;
		QCIVertexPropertiesWriter writer(*_graph.get());
		boost::write_graphviz(ss, *_graph.get(), writer);
		auto str = ss.str();
		// Modify the style...
		str = str.insert(9, "\n{\nnode [shape=box style=filled]");

		std::vector<std::string> splitVec;
		boost::split(splitVec, str, boost::is_any_of("\n"));
		splitVec.insert(splitVec.begin() + 3 + order(), "}");

		std::stringstream combine;
		std::for_each(splitVec.begin(), splitVec.end(), [&](const std::string& elem) { combine << elem << "\n"; });
		stream << combine.str().substr(0, combine.str().size() - 2);
	}

	/**
	 *
	 * @param stream
	 */
	void read(std::istream& stream) {

		std::string content { std::istreambuf_iterator<char>(stream),
				std::istreambuf_iterator<char>() };

		std::vector<std::string> lines, sections;
		boost::split(sections, content, boost::is_any_of("}"));

		// Sections should be size 2 for a valid dot file
		boost::split(lines, sections[0], boost::is_any_of("\n"));
		for (auto line : lines) {
			if (boost::contains(line, "label")) {
				Vertex v;
				std::vector<std::string> labelLineSplit, attrSplit;
				boost::split(labelLineSplit, line, boost::is_any_of("="));
				auto attributes = labelLineSplit[1].substr(1, labelLineSplit.size()-3);
				boost::split(attrSplit, attributes, boost::is_any_of(","));

				decltype(declval<Vertex>().properties) tuple;

				std::map<std::string, std::string> attrMap;
				for (auto a : attrSplit) {
//					std::vector<std::string> eqsplit;
//					boost::split(eqsplit, a, boost::is_any_of("="));
//					if (std::count_if(eqsplit[1].begin(), eqsplit[1].end(), ::isdigit) == eqsplit[1].size()) {
//
//					} else if (boost::contains(eqsplit[1], "[")) {
//						// then this is a vector
//						auto elementsStr = eqsplit[1].substr(1,eqsplit[1].size()-1);
////						std::vector<int> elements
//					}
//
//					attrMap.insert(std::make_pair(eqsplit[0], eqsplit[1]));
				}

			}
		}
	}
};

}
}

#endif
