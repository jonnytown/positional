#ifndef STORE_H
#define STORE_H

#include "math/Math.h"
#include "ABody.h"
#include <unordered_map>
#include <memory>

using namespace std;

namespace Positional
{
	template <typename T>
	class Store
	{
	friend class StoreRef;
	public:
		class Ref
		{
			friend class Store;

		private:
			UInt64 m_id;
			UInt64 m_index;
			Store *m_store;

		public:
			InternalRef(UInt64 id, UInt64 index, Store *store) : m_valid(true), m_id(id), m_index(index), m_store(store) {}

			bool valid() const { return m_store != NULL; }

			T &value() const
			{
				assert(m_store != NULL);
				return m_store->m_data[m_index];
			}
		};
		typedef shared_ptr<Ref> RefPtr;
	private:
		UInt64 m_nextId;
		unordered_map<UInt64, RefPtr> m_refs;
		vector<T> m_data;

	public:
		~Store()
		{
			for ( const auto &[key, ref] : m_refs)
			{
				ref->m_store = NULL;
			}
		}

		RefPtr createElement()
		{
			T datum = T();
			m_data.push_back(datum);

			auto id = m_nextId++;
			auto idx = m_data.size() - 1;

			auto ref = make_shared<REf>(id, idx, this);
			m_refs.insert_or_assign(id, ref);

			return ref;
		}

		void destroyElement(RefPtr ref)
		{
			assert(ref->m_store == this);
			auto idx = ref->m_idx;
			ref->m_store = NULL;
			m_refs.erase(ref->m_id);
			m_data.erase(m_data.begin() + idx);

			for (const auto &[key, ref] : m_refs)
			{
				(ref->m_idx > idx)
				{
					ref->m_idx--;
				}
			}
		}
	};
}
#endif // STORE_H