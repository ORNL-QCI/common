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
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE GraphTests

#include <boost/test/included/unit_test.hpp>
#include "Graph.hpp"

using namespace boost;
using namespace qci::common;

class BadVertex {

};

class FakeTensorVertex : public QCIVertex<std::string> {
};

class FakeBiasVertex : public QCIVertex<double> {
public:
	FakeBiasVertex() : QCIVertex() {
		propertyNames[0] = "bias";
	}
};

class FakeVertexFourProperties : public QCIVertex<std::string, double, int, float> {
public:
	FakeVertexFourProperties() : QCIVertex() {
		propertyNames[0] = "prop1";
		propertyNames[1] = "prop2";
		propertyNames[2] = "prop3";
		propertyNames[3] = "prop4";
	}
};

class FakeVertexFourPropertiesPrint2 : public QCIVertex<std::string, double, int, float> {
public:
	FakeVertexFourPropertiesPrint2() : QCIVertex() {
		propertyNames[0] = "prop1";
		propertyNames[1] = "prop2";
	}
};

class FakeVertexWithVector : public QCIVertex<std::vector<int>> {
public:
	FakeVertexWithVector() : QCIVertex() {
		propertyNames[0] = "prop1";
	}
};
BOOST_AUTO_TEST_CASE(checkConstruction) {

	// Check our valid vertex functions...
	BOOST_VERIFY(!is_valid_vertex<BadVertex>::value);
	BOOST_VERIFY(is_valid_vertex<FakeTensorVertex>::value);

	// Create a blank Graph with good vertices
	Graph<FakeTensorVertex> g1;
	g1.addVertex();
	BOOST_VERIFY(g1.order() == 1);

	// Create with 5 vertices
	Graph<FakeTensorVertex> g2(5);
	BOOST_VERIFY(g2.order() == 5);

	// Create a 3 node graph
	Graph<FakeBiasVertex> graph(3);

	// Verify it's size
	BOOST_VERIFY(3 == graph.order());

	// Create a complete graph with
	// the given edge weights.
	graph.addEdge(0, 1, 2.0);
	graph.addEdge(1, 2, 3.0);
	graph.addEdge(2, 0, 1.0);

	BOOST_VERIFY(2.0 == graph.getEdgeWeight(0, 1));

	// Test that we can change a weight
	graph.setEdgeWeight(0, 1, 22.0);
	BOOST_VERIFY(22.0 == graph.getEdgeWeight(0, 1));

	// Verify that we can set vertex bias values
	BOOST_VERIFY(0.0 == graph.getVertexProperty<0>(0));
	BOOST_VERIFY(0.0 == graph.getVertexProperty<0>(1));
	graph.setVertexProperty<0>(0, 3.3);
	BOOST_VERIFY(3.3 == graph.getVertexProperty<0>(0));
	graph.setVertexProperty<0>(1, 33.3);
	BOOST_VERIFY(33.3 == graph.getVertexProperty<0>(1));
}

BOOST_AUTO_TEST_CASE(checkAddVertexWithProperties) {
	Graph<FakeBiasVertex> graph;
	graph.addVertex(22.2);
	BOOST_VERIFY(graph.getVertexProperty<0>(0) == 22.2);
}

BOOST_AUTO_TEST_CASE(checkEdgeExists) {
	// Create a 2 node graph
	Graph<FakeBiasVertex> graph(3);

	// Verify it's size
	BOOST_REQUIRE_EQUAL(3, graph.order());

	graph.addEdge(0, 1, 2.0);
	graph.addEdge(1, 2, 3.0);
	graph.addEdge(2, 0, 1.0);

	BOOST_REQUIRE_EQUAL(3, graph.size());

	BOOST_VERIFY(graph.edgeExists(0,1));
	BOOST_VERIFY(graph.edgeExists(1,0));
	BOOST_VERIFY(graph.edgeExists(1,2));
	BOOST_VERIFY(graph.edgeExists(2,1));
	BOOST_VERIFY(graph.edgeExists(2,0));
	BOOST_VERIFY(graph.edgeExists(0,2));
}

BOOST_AUTO_TEST_CASE(checkDegree) {
	Graph<FakeBiasVertex> complete5(5);
	complete5.addEdge(0, 1);
	complete5.addEdge(0, 2);
	complete5.addEdge(0, 3);
	complete5.addEdge(0, 4);
	complete5.addEdge(1, 2);
	complete5.addEdge(1, 3);
	complete5.addEdge(1, 4);
	complete5.addEdge(2, 3);
	complete5.addEdge(2, 4);
	complete5.addEdge(3, 4);

	for (int i = 0; i < 5; i++) {
		BOOST_VERIFY(4 == complete5.degree(i));
	}
}

BOOST_AUTO_TEST_CASE(checkDiameter) {
	// Create a Complete 5 node graph
	Graph<FakeBiasVertex> complete5(5);
	complete5.addEdge(0, 1);
	complete5.addEdge(0, 2);
	complete5.addEdge(0, 3);
	complete5.addEdge(0, 4);
	complete5.addEdge(1, 2);
	complete5.addEdge(1, 3);
	complete5.addEdge(1, 4);
	complete5.addEdge(2, 3);
	complete5.addEdge(2, 4);
	complete5.addEdge(3, 4);
	BOOST_REQUIRE_EQUAL(1, complete5.diameter());
}

BOOST_AUTO_TEST_CASE(checkWrite) {
	Graph<FakeBiasVertex> complete5(5);

	std::string expected =
			"graph G {\n"
			"{\n"
			"node [shape=box style=filled]\n"
			"0 [label=\"bias=1\"];\n"
			"1 [label=\"bias=2\"];\n"
			"2 [label=\"bias=3\"];\n"
			"3 [label=\"bias=4\"];\n"
			"4 [label=\"bias=5\"];\n"
			"}\n"
			"0--1 ;\n"
			"0--2 ;\n"
			"0--3 ;\n"
			"0--4 ;\n"
			"1--2 ;\n"
			"1--3 ;\n"
			"1--4 ;\n"
			"2--3 ;\n"
			"2--4 ;\n"
			"3--4 ;\n"
			"}";
	complete5.setVertexProperty<0>(0, 1.0);
	complete5.setVertexProperty<0>(1, 2.0);
	complete5.setVertexProperty<0>(2, 3.0);
	complete5.setVertexProperty<0>(3, 4.0);
	complete5.setVertexProperty<0>(4, 5.0);

	complete5.addEdge(0, 1);
	complete5.addEdge(0, 2);
	complete5.addEdge(0, 3);
	complete5.addEdge(0, 4);
	complete5.addEdge(1, 2);
	complete5.addEdge(1, 3);
	complete5.addEdge(1, 4);
	complete5.addEdge(2, 3);
	complete5.addEdge(2, 4);
	complete5.addEdge(3, 4);

	std::stringstream ss;
	complete5.write(ss);
	BOOST_VERIFY(ss.str() == expected);

	Graph<FakeVertexFourProperties> complete5_4props(5);

	expected =
				"graph G {\n"
				"{\n"
				"node [shape=box style=filled]\n"
				"0 [label=\"prop1=val1,prop2=1,prop3=1,prop4=1\"];\n"
				"1 [label=\"prop1=val2,prop2=2,prop3=2,prop4=2\"];\n"
				"2 [label=\"prop1=val3,prop2=3,prop3=3,prop4=3\"];\n"
				"3 [label=\"prop1=val4,prop2=4,prop3=4,prop4=4\"];\n"
				"4 [label=\"prop1=val5,prop2=5,prop3=5,prop4=5\"];\n"
				"}\n"
				"0--1 ;\n"
				"0--2 ;\n"
				"0--3 ;\n"
				"0--4 ;\n"
				"1--2 ;\n"
				"1--3 ;\n"
				"1--4 ;\n"
				"2--3 ;\n"
				"2--4 ;\n"
				"3--4 ;\n"
				"}";
	complete5_4props.setVertexProperty<0>(0, "val1");
	complete5_4props.setVertexProperty<0>(1, "val2");
	complete5_4props.setVertexProperty<0>(2, "val3");
	complete5_4props.setVertexProperty<0>(3, "val4");
	complete5_4props.setVertexProperty<0>(4, "val5");

	complete5_4props.setVertexProperty<1>(0, 1.0);
	complete5_4props.setVertexProperty<1>(1, 2.0);
	complete5_4props.setVertexProperty<1>(2, 3.0);
	complete5_4props.setVertexProperty<1>(3, 4.0);
	complete5_4props.setVertexProperty<1>(4, 5.0);

	complete5_4props.setVertexProperty<2>(0, 1);
	complete5_4props.setVertexProperty<2>(1, 2);
	complete5_4props.setVertexProperty<2>(2, 3);
	complete5_4props.setVertexProperty<2>(3, 4);
	complete5_4props.setVertexProperty<2>(4, 5);

	complete5_4props.setVertexProperty<3>(0, 1.0);
	complete5_4props.setVertexProperty<3>(1, 2.0);
	complete5_4props.setVertexProperty<3>(2, 3.0);
	complete5_4props.setVertexProperty<3>(3, 4.0);
	complete5_4props.setVertexProperty<3>(4, 5.0);

	complete5_4props.addEdge(0, 1);
	complete5_4props.addEdge(0, 2);
	complete5_4props.addEdge(0, 3);
	complete5_4props.addEdge(0, 4);
	complete5_4props.addEdge(1, 2);
	complete5_4props.addEdge(1, 3);
	complete5_4props.addEdge(1, 4);
	complete5_4props.addEdge(2, 3);
	complete5_4props.addEdge(2, 4);
	complete5_4props.addEdge(3, 4);

	std::stringstream ss2;
	complete5_4props.write(ss2);
	BOOST_VERIFY(ss2.str() == expected);


	// Create a 3 node graph
	Graph<FakeVertexFourPropertiesPrint2> graph(3);

	// Create a complete graph with
	// the given edge weights.
	graph.addEdge(0, 1, 2.0);
	graph.addEdge(1, 2, 3.0);
	graph.addEdge(2, 0, 1.0);

	graph.setVertexProperty<0>(0, "val1");
	graph.setVertexProperty<0>(1, "val2");
	graph.setVertexProperty<0>(2, "val3");

	graph.setVertexProperty<1>(0, 1.0);
	graph.setVertexProperty<1>(1, 2.0);
	graph.setVertexProperty<1>(2, 3.0);

	expected = "graph G {\n"
			"{\n"
			"node [shape=box style=filled]\n"
			"0 [label=\"prop1=val1,prop2=1\"];\n"
			"1 [label=\"prop1=val2,prop2=2\"];\n"
			"2 [label=\"prop1=val3,prop2=3\"];\n"
			"}\n"
			"0--1 ;\n"
			"1--2 ;\n"
			"2--0 ;\n"
			"}";

	std::stringstream ss3;
	graph.write(ss3);
	BOOST_VERIFY(ss3.str() == expected);

	// Create a 3 node graph
	Graph<FakeVertexWithVector> graph2(3);

	// Create a complete graph with
	// the given edge weights.
	graph2.addEdge(0, 1, 2.0);
	graph2.addEdge(1, 2, 3.0);
	graph2.addEdge(2, 0, 1.0);

	std::vector<int> hello {1,2};
	graph2.setVertexProperty<0>(0, std::vector<int>{1,2});
	graph2.setVertexProperty<0>(1, std::vector<int>{1,2});
	graph2.setVertexProperty<0>(2, std::vector<int>{1,2});


	expected = "graph G {\n"
			"{\n"
			"node [shape=box style=filled]\n"
			"0 [label=\"prop1=[1 2]\"];\n"
			"1 [label=\"prop1=[1 2]\"];\n"
			"2 [label=\"prop1=[1 2]\"];\n"
			"}\n"
			"0--1 ;\n"
			"1--2 ;\n"
			"2--0 ;\n"
			"}";

	std::stringstream ss4;
	graph2.write(ss4);
	BOOST_VERIFY(ss4.str() == expected);

}
