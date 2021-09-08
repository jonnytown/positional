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
		friend struct Ptr;
		struct Ref
		{
			UInt64 id;
			UInt64 index;
			Store *store;

			Ref(const UInt64 &_id, const UInt64 &_index, Store *_store) : id(_id), index(_index), store(_store) {}
		};

		struct Ptr
		{
			friend struct Store;
			Ptr()
			{
				m_ptr.reset();
			}

			Ptr(const Ptr &other)
			{
				m_ptr = other.m_ptr;
			}

			Ptr &operator=(const Ptr &other)
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

			inline T* get() const
			{
				assert(!m_ptr.expired);
				auto shPtr = m_ptr.lock();
				assert(shPtr && shPtr->store != NULL);
				return &(*(shPtr->store->m_data.begin() + shPtr->index));
			}

			inline bool operator==(Ptr other) const
			{
				return m_ptr.lock().get() == other.m_ptr.lock().get();
			}

			inline bool operator!=(Ptr other) const
			{
				return m_ptr.lock().get() != other.m_ptr.lock().get();
			}

		private:
			Ptr(shared_ptr<Ref> ref)
			{
				m_ptr = weak_ptr<Store::Ref>(ref);
			}

			weak_ptr<Ref> m_ptr;
		};

		~Store()
		{
			for (const auto &[key, ref] : m_refs)
			{
				ref->store = NULL;
			}
		}

		Ptr store(const T &element)
		{
			m_data.push_back(element);

			auto id = m_nextId++;
			auto idx = m_data.size() - 1;

			auto ref = make_shared<Ref>(id, idx, this);
			m_refs.insert_or_assign(id, ref);

			return Ptr(ref);
		}

		void erase(Ptr ptr)
		{
			assert(!ptr.m_ptr.expired);
			auto shPtr = ptr.m_ptr.lock();
			assert(shPtr->store == this);

			shPtr->store = NULL;
			m_refs.erase(shPtr->id);
			m_data.erase(m_data.begin() + shPtr->index);

			for (const auto &[key, ref] : m_refs)
			{
				if (ref->index > shPtr->index)
				{
					ref->index--;
				}
			}
		}

		void erase(function<bool(const Ptr &elPtr)> predicate)
		{
			for (const auto &[key, ref] : m_refs)
			{
				if (predicate(Ptr(ref)))
				{
					m_data.erase(m_data.begin() + ref->index);
					m_refs.erase(key);
				}
			}
		}

	private:
		UInt64 m_nextId;
		unordered_map<UInt64, shared_ptr<Ref>> m_refs;
		vector<T> m_data;
	};
}
#endif // STORE_H