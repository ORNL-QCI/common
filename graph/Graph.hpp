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
};

/**
 * For now, we only allow Edges with weight property.
 */
struct DefaultEdge {
	double weight = 0.0;
};

/**
 *
 */
template<typename Vertex>
class Graph {
	static_assert(is_valid_vertex<Vertex>::value, "QCI Graph can only take Vertices that are derived from QCIVertex, or have a tuple properties member.");

	using adj_list = adjacency_list<vecS, vecS, undirectedS, Vertex, DefaultEdge>;
	using BoostGraph = std::shared_ptr<adj_list>;
	using vertex_type = typename boost::graph_traits<adjacency_list<vecS, vecS, undirectedS, Vertex, DefaultEdge>>::vertex_descriptor;
	using edge_type = typename boost::graph_traits<adjacency_list<vecS, vecS, undirectedS, Vertex, DefaultEdge>>::edge_descriptor;

protected:

	BoostGraph _graph;

public:

	Graph() {
		_graph = std::make_shared<adj_list>();
	}

	Graph(const int numberOfVertices) {
		_graph = std::make_shared<adj_list>(
				numberOfVertices);
	}

	void addEdge(const int srcIndex, const int tgtIndex, const double edgeProperty) {
		auto edgeBoolPair = add_edge(vertex(srcIndex, *_graph.get()),
				vertex(tgtIndex, *_graph.get()), *_graph.get());
		if (!edgeBoolPair.second) {

		}
		(*_graph.get())[edgeBoolPair.first].weight = edgeProperty;
	}

	void addEdge(const int srcIndex, const int tgtIndex) {
		add_edge(vertex(srcIndex, *_graph.get()),
				vertex(tgtIndex, *_graph.get()), *_graph.get());
	}

	void addVertex() {
		add_vertex(*_graph.get());
	}

	template<typename ... Properties>
	void addVertex(Properties ... properties) {
		auto v = add_vertex(*_graph.get());
		(*_graph.get())[v].properties = std::make_tuple(properties...);
	}

	template<typename... Properties>
	void setVertexProperties(const int index, Properties... properties) {

	}

	template<const int PropertyIndex>
	void setVertexProperty(const int index,
			decltype(std::get<PropertyIndex>(std::declval<Vertex>().properties)) prop) {
		auto v = vertex(index, *_graph.get());
		std::get<PropertyIndex>((*_graph.get())[v].properties) = prop;
		return;
	}

	template<const int PropertyIndex>
	decltype(std::get<PropertyIndex>(std::declval<Vertex>().properties))& getVertexProperty(
			const int index) {
		auto v = vertex(index, *_graph.get());
		return std::get<PropertyIndex>((*_graph.get())[v].properties);
	}


	void setEdgeWeight(const int srcIndex, const int tgtIndex, const double weight) {
		auto e = edge(vertex(srcIndex, *_graph.get()), vertex(tgtIndex, *_graph.get()), *_graph.get());
		(*_graph.get())[e.first].weight = weight;
	}

	double getEdgeWeight(const int srcIndex, const int tgtIndex) {
		auto e = edge(vertex(srcIndex, *_graph.get()), vertex(tgtIndex, *_graph.get()), *_graph.get());
		return (*_graph.get())[e.first].weight;
	}

	bool edgeExists(const int srcIndex, const int tgtIndex) {
		return edge(vertex(srcIndex, *_graph.get()),
				vertex(tgtIndex, *_graph.get()), *_graph.get()).second;
	}

	int degree(const int index) {
		return boost::degree(vertex(index, *_graph.get()), *_graph.get());
	}

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

	int size() {
		return num_edges(*_graph.get());
	}

	int order() {
		return num_vertices(*_graph.get());
	}
};

}
}

#endif
