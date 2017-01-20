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
#ifndef COMMON_FACTORY_ABSTRACTFACTORY_HPP_
#define COMMON_FACTORY_ABSTRACTFACTORY_HPP_

#include <map>
#include <boost/core/demangle.hpp>
#include <boost/assign/list_of.hpp>
#include "QCIError.hpp"
#include <vector>

namespace qci {

namespace common {

/**
 *
 */
class QCIObject {
public:

	std::string name = "";

	std::string description = "";

	virtual ~QCIObject() {
	}
};

/**
 * The AbstractFactory serves as a dynamically registered factory
 * pattern for general QCIObjects.
 */
class AbstractFactory {

public:

	static QCIObject * create(const std::string& id) {
		for (auto t = getConstructors().begin(); t != getConstructors().end();
						++t) {
			if (t->first.second == id) {
				return (*t->second)();
			}
		}

		return nullptr;
	}

	template <typename B>
	static B * createAndCast (const std::string& id) {
		return dynamic_cast<B*>(create(id));
	}

	/** Create and return a new QCIObject corresponding to
	 * the given string Id.
	 */
	static QCIObject * create(const std::string& type, const std::string& id) {
		const ConstructorMap::const_iterator iter = getConstructors().find(std::make_pair(type, id));
		return iter == getConstructors().end() ? 0 : (*iter->second)();
	}

	/**
	 * This is a helper function for creating a new QCIObject
	 * and casting it immediately to its known base class.
	 */
	template <typename B>
	static B * createAndCast (const std::string& type, const std::string& id) {
		return dynamic_cast<B*>(create(type, id));
	}

	/**
	 *
	 */
	static void listTypes() {
		listTypes(std::cout);
	}

	static void listTypes(std::ostream& stream) {
		for (auto t = getConstructors().begin(); t != getConstructors().end();
				++t) {
			stream << boost::core::demangle(typeid(*(t->second)()).name())
					<< " of type " << t->first.first << "\n";
		}
	}

	template<typename B>
	static std::vector<B*> getAllofType(const std::string& type) {
		std::vector<B*> retTypes;
		for (auto t = getConstructors().begin(); t != getConstructors().end();
						++t) {
			if (t->first.first == type) {
				retTypes.push_back(createAndCast<B>(type, t->first.second));
			}
		}

		return retTypes;
	}

private:

	enum qci_types {
		triggerFailure,
		unknown,
		compiler,
		accelerator,
		qpu,
		npu
	};

	// Create a typedef for QCIObject constructors
	typedef QCIObject * QCIObjectConstructor();

	typedef std::pair<std::string, std::string> TypeIdPair;

	// Create a typedef for the mapping between string ids and
	// QCIObject constructors.
	typedef std::map<TypeIdPair, QCIObjectConstructor*> ConstructorMap;

	/**
	 * Return the static Constructor mapping
	 */
	static ConstructorMap& getConstructors() {
		static ConstructorMap constructors;
		return constructors;
	}

	/**
	 * This class handles dynamic registration of
	 * new QCIObjects.
	 */
	template<class T = int>
	struct DynamicRegister {

		static QCIObject* create() {
			return new T();
		}

		static QCIObjectConstructor * initialize(const std::string& type,
				const std::string& id) {
			std::map<std::string, qci_types> tmpMap =
						boost::assign::map_list_of("unknown",unknown)("compiler", compiler)(
								"accelerator", accelerator)("qpu", qpu)("npu", npu);
			if (tmpMap[type] == triggerFailure) {
				QCIError(type + " is not a valid QCI Type "
								"(must be 'compiler', 'accelerator', "
								"'qpu', 'npu'). Exiting.\n\n");
			}

			return getConstructors()[std::make_pair(type, id)] = create;
		}

		static QCIObjectConstructor * initialize(const std::string& id) {
			return getConstructors()[std::make_pair("unknown", id)] = create;
		}

		static QCIObjectConstructor * creator;
	};
};

#define REGISTER_QCIOBJECT_WITH_QCITYPE(T, QCITYPE, STR) template<> qci::common::AbstractFactory::QCIObjectConstructor* \
	qci::common::AbstractFactory::DynamicRegister<T>::creator = \
	qci::common::AbstractFactory::DynamicRegister<T>::initialize(QCITYPE, STR)

#define REGISTER_QCIOBJECT(T, STR) template<> qci::common::AbstractFactory::QCIObjectConstructor* \
	qci::common::AbstractFactory::DynamicRegister<T>::creator = \
	qci::common::AbstractFactory::DynamicRegister<T>::initialize(STR)

}

}

#endif
