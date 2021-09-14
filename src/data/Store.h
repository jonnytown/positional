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
		friend struct Ref;
		struct Handle;
		struct Ref
		{
			friend struct Store;
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
			Ref(shared_ptr<Store::Handle> ref)
			{
				m_ptr = weak_ptr<Store::Handle>(ref);
			}

			weak_ptr<Store::Handle> m_ptr;
		};

		~Store()
		{
			for (const auto &[key, ref] : m_refs)
			{
				ref->store = NULL;
			}
		}

		Ref store(const T &element)
		{
			m_data.push_back(element);

			auto id = m_nextId++;
			auto idx = m_data.size() - 1;

			auto ref = make_shared<Handle>(id, idx, this);
			m_refs.insert_or_assign(id, ref);

			return Ref(ref);
		}

		void erase(Ref ref)
		{
			assert(!ref.m_ptr.expired);
			auto shPtr = ref.m_ptr.lock();
			assert(shPtr->store == this);

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
		}

		void erase(function<bool(const Ref &elRef)> predicate)
		{
			for (const auto &[key, ref] : m_refs)
			{
				if (predicate(Ref(ref)))
				{
					m_data.erase(m_data.begin() + ref->index);
					m_refs.erase(key);
				}
			}
		}

	private:
		struct Handle
		{
			UInt64 id;
			UInt64 index;
			Store *store;

			Handle(const UInt64 &_id, const UInt64 &_index, Store *_store) : id(_id), index(_index), store(_store) {}
		};

		UInt64 m_nextId;
		unordered_map<UInt64, shared_ptr<Handle>> m_refs;
		vector<T> m_data;
	};
}
#endif // STORE_H