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


