#ifndef STORE_H
#define STORE_H

#include "math/Math.h"
#include <unordered_map>
#include <memory>
#include <functional>

using namespace std;

namespace Positional
{
	template <typename T>
	struct Store
	{
		template <typename>
		friend struct Ref;

		struct Handle
		{
			friend struct Ref<T>;

			UInt64 id;
			UInt64 index;
			Store *store;

			Handle(const UInt64 &_id, const UInt64 &_index, Store *_store) : id(_id), index(_index), store(_store) {}
		};

		Store() : m_nextId(0) {}

		~Store()
		{
			for (const auto &[key, ref] : m_refs)
			{
				ref->store = NULL;
			}
		}

		inline UInt64 count() const { return m_data.size(); }
		inline T &operator[](const UInt64 &i)
		{
			return m_data[i];
		}

		Ref<T> store(const T &element)
		{
			m_data.push_back(element);

			auto id = m_nextId++;
			auto idx = m_data.size() - 1;

			auto ref = make_shared<Handle>(id, idx, this);
			m_refs.insert_or_assign(id, ref);

			return Ref<T>(ref);
		}

		bool erase(const Ref<T> &ref)
		{
			if (!ref.valid())
			{
				return false;
			}

			auto shPtr = ref.m_ptr.lock();
			if (shPtr->store != this)
			{
				return false;
			}

			shPtr->store = NULL;
			m_refs.erase(shPtr->id);
			m_data.erase(m_data.begin() + shPtr->index);

			for (const auto &[key, r] : m_refs)
			{
				if (r->index > shPtr->index)
				{
					r->index--;
				}
			}

			return true;
		}

		void erase(const function<bool(const Ref<T> &elRef)> &predicate)
		{
			for (const auto &[key, ref] : m_refs)
			{
				if (predicate(ref))
				{
					m_data.erase(m_data.begin() + ref->index);
					m_refs.erase(key);
				}
			}
		}

		void forEach(const function<void(const Ref<T> &elRef)> &callback)
		{
			for (const auto &[key, ref] : m_refs)
			{
				callback(ref);
			}
		}

	private:
		UInt64 m_nextId;
		unordered_map<UInt64, shared_ptr<Handle>> m_refs;
		vector<T> m_data;
	};

	template <typename T>
	struct Ref
	{
		friend struct Store<T>;

		Ref()
		{
			m_ptr.reset();
		}

		Ref(const Ref &other)
		{
			m_ptr = other.m_ptr;
		}

		Ref &operator=(const Ref &other)
		{
			m_ptr = other.m_ptr;
			return *this;
		}

		inline void reset()
		{
			m_ptr.reset();
		}

		inline bool valid() const
		{
			return !m_ptr.expired() && m_ptr.lock().get()->store != NULL;
		}

		inline UInt64 id() const
		{
			assert(!m_ptr.expired);
			auto shPtr = m_ptr.lock();
			assert(shPtr->store != NULL);
			return shPtr->id;
		}

		inline T &get() const
		{
			assert(!m_ptr.expired);
			auto shPtr = m_ptr.lock();
			assert(shPtr && shPtr->store != NULL);
			return shPtr->store->m_data[shPtr->index];
		}

		inline bool operator==(Ref other) const
		{
			return m_ptr.lock().get() == other.m_ptr.lock().get();
		}

		inline bool operator!=(Ref other) const
		{
			return m_ptr.lock().get() != other.m_ptr.lock().get();
		}

	private:
		Ref(shared_ptr<Store<T>::Handle> ref)
		{
			m_ptr = weak_ptr<Store<T>::Handle>(ref);
		}

		weak_ptr<Store<T>::Handle> m_ptr;
	};
}
#endif // STORE_H