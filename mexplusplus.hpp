/**     MexPlusPlus
 *
 *      Copyright (c) 2020, Ilmari Ayres
 *      All rights reserved.
 *      
 *      Redistribution and use in source and binary forms, with or without
 *      modification, are permitted provided that the following conditions are met:
 *      
 *      1. Redistributions of source code must retain the above copyright notice, this
 *         list of conditions and the following disclaimer.
 *      
 *      2. Redistributions in binary form must reproduce the above copyright notice,
 *         this list of conditions and the following disclaimer in the documentation
 *         and/or other materials provided with the distribution.
 *      
 *      3. Neither the name of the copyright holder nor the names of its
 *         contributors may be used to endorse or promote products derived from
 *         this software without specific prior written permission.
 *      
 *      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *      AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *      IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *      DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *      FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *      DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *      SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *      CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *      OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __MEXPLUSPLUS_H__
#define __MEXPLUSPLUS_H__

#include <MatlabDataArray.hpp>
#include <mex.hpp>

#include <map>

// matlab::mex::MexIORange::operator[] must be declared for this header to work.
// Class definition can be found in cppmex/detail/mexIOAdapterImpl.hpp.

template <typename T>
T extract(const matlab::data::Array &arr)
{
	switch (arr.getType())
	{
	case matlab::data::ArrayType::DOUBLE:
	{
		matlab::data::TypedArray<double> typed = arr;
		return typed[0];
	}
	case matlab::data::ArrayType::SINGLE:
	{
		matlab::data::TypedArray<float> typed = arr;
		return typed[0];
	}
	case matlab::data::ArrayType::INT8:
	{
		matlab::data::TypedArray<int8_t> typed = arr;
		return typed[0];
	}
	case matlab::data::ArrayType::UINT8:
	{
		matlab::data::TypedArray<uint8_t> typed = arr;
		return typed[0];
	}
	case matlab::data::ArrayType::INT16:
	{
		matlab::data::TypedArray<int16_t> typed = arr;
		return typed[0];
	}
	case matlab::data::ArrayType::UINT16:
	{
		matlab::data::TypedArray<uint16_t> typed = arr;
		return typed[0];
	}
	case matlab::data::ArrayType::INT32:
	{
		matlab::data::TypedArray<int32_t> typed = arr;
		return typed[0];
	}
	case matlab::data::ArrayType::UINT32:
	{
		matlab::data::TypedArray<uint32_t> typed = arr;
		return typed[0];
	}
	case matlab::data::ArrayType::INT64:
	{
		matlab::data::TypedArray<int64_t> typed = arr;
		return typed[0];
	}
	case matlab::data::ArrayType::UINT64:
	{
		matlab::data::TypedArray<uint64_t> typed = arr;
		return typed[0];
	}
	case matlab::data::ArrayType::CHAR:
	{
		matlab::data::TypedArray<char16_t> typed = arr;
		return typed[0];
	}
	case matlab::data::ArrayType::LOGICAL:
	{
		matlab::data::TypedArray<bool> typed = arr;
		return typed[0];
	}
	}

	T t = 0;
	return t;
};

template <>
std::string extract<std::string>(const matlab::data::Array& arr);

class Args
{
public:

    matlab::mex::ArgumentList *raw;
        
    Args(matlab::mex::ArgumentList *args);
        
    template <typename T>
	T get(size_t index) { return extract<T>((*raw)[index + 1]); };
    
    template <typename T>
	matlab::data::TypedArray<T> getArray(size_t index) { matlab::data::TypedArray<T> t = (*raw)[index+1]; return t; };

	template <typename T>
	std::vector<T> getVector(size_t index)
	{
		matlab::data::Array list = (*raw)[index + 1];

		std::vector<T> ret;
		for (int i = 0; i < list.getNumberOfElements(); i++)
			ret.push_back(extract<T>(list[i]));

		return std::move(ret);
	};
        
	template <typename T>
	void set(size_t index, T &value) { (*raw)[index] = value; };
};

typedef void (*MexFunc)(Args& output, Args& input);

class MexFunction : public matlab::mex::Function
{
public:

    static std::map<std::string, MexFunc> funcs;

	void operator()(matlab::mex::ArgumentList output, matlab::mex::ArgumentList input);
};

class RegisterMexFunc
{
public:
	RegisterMexFunc(std::string name, MexFunc f);
};

class SessionBase
{
protected:
	static size_t session_variable_count;
public:
	static size_t getSessionVariableCount();
};

template<class T>
class Session : protected SessionBase
{
	static std::map<intptr_t, std::shared_ptr<T>> instances;

public:

	static intptr_t create(T* instance)
	{
		intptr_t id = reinterpret_cast<intptr_t>(instance);

		if (id == 0)
			return 0;

		if (!exist(id))
			session_variable_count++;

		instances[id] = std::shared_ptr<T>(instance);

		return id;
	}

	static void destroy(intptr_t id)
	{
		if (exist(id))
			session_variable_count--;

		instances.erase(id);
	}

	static T* get(intptr_t id)
	{
		return instances[id].get();
	}

	static const T& getConst(intptr_t id)
	{
		return *get(id);
	}

	static bool exist(intptr_t id)
	{
		return instances[id] ? true : false;
	}

	static void clear()
	{
		session_variable_count -= instances.size();
		instances.clear();
	}
};

#define MEX_DEFINE(name) \
	void mexfunc_##name (Args &output, Args &input); \
	RegisterMexFunc register_##name(#name, mexfunc_##name); \
	void mexfunc_##name (Args &output, Args &input)

#define MEX_NARGIN(nargin) \
	if(input.size() <= nargin) \
		throw matlab::engine::Exception("Not enough input arguments. " +  \
		                                std::to_string(input.size() - 1) + " given, " + \
		                                std::to_string(nargin) + " required.");

template<class T>
std::map<intptr_t, std::shared_ptr<T>> Session<T>::instances = {};

extern matlab::data::ArrayFactory factory;
extern MexFunction *instance;

#define MEX_DISPATCH \
	std::map<std::string, MexFunc> MexFunction::funcs = {}; \
	matlab::data::ArrayFactory factory; \
	MexFunction *instance; \
	size_t SessionBase::session_variable_count = 0; \
\
	size_t SessionBase::getSessionVariableCount() \
	{ \
		return session_variable_count; \
	} \
\
	Args::Args(matlab::mex::ArgumentList *args) { raw = args; } \
\
    void MexFunction::operator()(matlab::mex::ArgumentList output, matlab::mex::ArgumentList input) \
	{ \
		instance = this; \
		MexFunction::mexLock(); \
\
		std::string funcName = extract<std::string>(input[0]); \
\
		MexFunc f = funcs[std::string(funcName)]; \
		if (f) \
		{ \
			Args o(&output), i(&input); \
			f(o, i); \
		} \
		if(SessionBase::getSessionVariableCount() == 0) \
			MexFunction::mexUnlock(); \
	} \
\
	std::string extract<std::string>(const matlab::data::Array& arr) \
	{ \
		switch (arr.getType()) \
		{ \
		case matlab::data::ArrayType::MATLAB_STRING: \
		{ \
			matlab::data::TypedArray<matlab::data::MATLABString> str = arr; \
			return str[0]; \
		} \
		case matlab::data::ArrayType::CHAR: \
		{ \
			matlab::data::CharArray str = arr; \
			return str.toAscii(); \
		} \
		default: \
			return ""; \
		} \
	} \
	RegisterMexFunc::RegisterMexFunc(std::string name, MexFunc f) { MexFunction::funcs[name] = f; }; \

#endif // __MEXPLUSPLUS_H__

