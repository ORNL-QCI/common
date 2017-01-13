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
#define BOOST_TEST_MODULE AbstractFactory

#include <boost/test/included/unit_test.hpp>
#include "AbstractFactory.hpp"

class Nothing : public qci::common::QCIObject {
public:
	Nothing() {}
};

class ExampleXACCCompiler : public qci::common::QCIObject {

public:
	ExampleXACCCompiler() {}
};

class TestClass : public ExampleXACCCompiler {

public:

	TestClass() { name = "HELLO WORLD"; }
};

class AnotherTestClass : public ExampleXACCCompiler {
public:
	AnotherTestClass() {name = "another one"; }
};

REGISTER_QCIOBJECT(Nothing, "NothingTest");
REGISTER_QCIOBJECT_WITH_QCITYPE(TestClass, "compiler", "TestClass");
REGISTER_QCIOBJECT_WITH_QCITYPE(AnotherTestClass, "compiler", "AnotherTestClass");

using namespace boost;

BOOST_AUTO_TEST_CASE(checkAbstractFactory) {
	ExampleXACCCompiler * obj = qci::common::AbstractFactory::createAndCast<
			TestClass>("TestClass");
	BOOST_VERIFY(obj);
	BOOST_VERIFY(obj->name == "HELLO WORLD");

	Nothing * n = qci::common::AbstractFactory::createAndCast<Nothing>("NothingTest");
	BOOST_VERIFY(n);

	delete obj;
	delete n;
}

BOOST_AUTO_TEST_CASE(checkListAll) {
	qci::common::AbstractFactory::listTypes();
}

BOOST_AUTO_TEST_CASE(checkReturnAllofType) {
	auto objs = qci::common::AbstractFactory::getAllofType<ExampleXACCCompiler>("compiler");
	BOOST_VERIFY(objs.size() == 2);
}
