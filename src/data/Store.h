#ifndef STORE_H
#define STORE_H

#include "math/Math.h"
#include <unordered_map>
#include <memory>
#include <functional>

using namespace std;

namespace Positional
{
	template <typename>
	struct Handle;

	template <typename T>
	struct Store
	{
	private:
		inline vector<UInt64> getKeys()
		{
			vector<UInt64> retval;
			retval.reserve(m_handles.size());
			for (auto const &element : m_handles)
			{
				retval.push_back(element.first);
			}
			return retval;
		}

		inline void fastErase(const UInt64 &id, const UInt64 &index)
		{
			m_handles.erase(id);

			const UInt64 size = m_data.size();
			const UInt64 last = size - 1;
			if (size > 1 && index < last)
			{
				m_data[index] = m_data[last];
				for (const auto &pair : m_handles)
				{
					if (pair.second->index == last)
					{
						pair.second->index = index;
						break;
					}
				}
			}
			m_data.pop_back();
		}

	public:
		template <typename>
		friend struct Ref;

		Store() : m_nextId(0) {}

		~Store()
		{
			for (const auto &[key, handle] : m_handles)
			{
				handle->store = NULL;
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

			auto handle = make_shared<Handle<T>>(id, idx, this);
			m_handles.insert_or_assign(id, handle);

			return Ref<T>(handle);
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

			const UInt64 index = shPtr->index;
			shPtr->store = NULL;
			fastErase(shPtr->id, shPtr->index);

			return true;
		}

		void erase(const function<bool(const Ref<T> &elRef)> &predicate)
		{
			const auto keys = getKeys();
			for (const auto &key : keys)
			{
				auto handle = m_handles.at(key);
				Ref<T> ref(handle);
				if (predicate(ref))
				{
					fastErase(handle->id, handle->index);
				}
			}
		}

		void forEach(const function<void(const Ref<T> &elRef)> &callback)
		{
			for (const auto &[key, handle] : m_handles)
			{
				Ref<T> ref(handle);
				callback(ref);
			}
		}

	private:
		UInt64 m_nextId;
		unordered_map<UInt64, shared_ptr<Handle<T>>> m_handles;
		vector<T> m_data;
	};

	template <typename T>
	struct Handle
	{
		UInt64 id;
		UInt64 index;
		Store<T> *store;

		Handle(const UInt64 &_id, const UInt64 &_index, Store<T> *_store) : id(_id), index(_index), store(_store) {}
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
		Ref(shared_ptr<Handle<T>> handle)
		{
			m_ptr = weak_ptr<Handle<T>>(handle);
		}

		weak_ptr<Handle<T>> m_ptr;
	};
}
#endif // STORE_H