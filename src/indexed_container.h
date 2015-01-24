#ifndef _INDEXED_CONTAINER_H
#define _INDEXED_CONTAINER_H

#include <algorithm>
#include <functional>
#include <vector>

namespace scc {
	template<typename Iterator>
	struct range
	{
		typedef Iterator iterator;

		iterator const first;
		iterator const last;

		template<typename FirstIterator, typename LastIterator>
		explicit range(FirstIterator&& first, LastIterator&& last)
		: first(std::forward<FirstIterator>(first)), last(std::forward<LastIterator>(last)) { }

		iterator begin() const { return first; }
		iterator end() const { return last; }

		friend iterator begin(range const& i) { return i.first; }
		friend iterator end(range const& i) { return i.last; }

		auto size() const -> decltype(last - first) { return last - first; }
	};

	template<typename Iterator>
	class counting_iterator : public std::iterator<std::forward_iterator_tag, Iterator>
	{
		typedef std::iterator<std::forward_iterator_tag, Iterator> base_iterator;
	private:
		Iterator m_it;

	public:
		using typename base_iterator::value_type;
		using typename base_iterator::difference_type;
		using typename base_iterator::reference;


		typedef Iterator const& const_reference;
		typedef Iterator const* const_pointer;

		counting_iterator() { }
		explicit counting_iterator(Iterator it) : m_it(it) { }

		const_pointer operator->() const { return &m_it; }
		const_reference operator*() const { return m_it; }
		counting_iterator& operator++() { ++m_it; return *this; }
		counting_iterator operator++(int) { return counting_iterator(m_it++); }
		counting_iterator& operator--() { --m_it; return *this;}
		counting_iterator operator--(int) { return counting_iterator(m_it--); }
		bool operator==(const counting_iterator& rhs) const { return m_it == rhs.m_it; }
		bool operator!=(const counting_iterator& rhs) const { return m_it != rhs.m_it; }

		auto operator-(counting_iterator const& rhs) const -> decltype(m_it - rhs.m_it) { return m_it - rhs.m_it; }
	};

	/* only supports container with begin() and end() members; ADL begin() and end() not supported */
	/* usage: for (auto const& it: counting_iteration(container)) */
	template<typename Container>
	auto counting_iteration(Container&& container) -> range<counting_iterator<decltype(container.begin())>>
	{
		typedef decltype(container.begin()) inner_iterator;
		typedef counting_iterator<inner_iterator> iterator;
		return range<iterator>(iterator(container.begin()), iterator(container.end()));
	}

	template<typename T, typename Container, typename Compare, typename Index = std::size_t>
	struct index_compare : private Compare {
		Container const& container;

		explicit index_compare(Container const& container)
		: container(container) { }

		explicit index_compare(Container const& container, Compare const& compare)
		: Compare(compare), container(container) { }

		explicit index_compare(Container const& container, Compare&& compare)
		: Compare(std::move(compare)), container(container) { }

		bool operator()(Index a, Index b)
		{
			return Compare::operator()(container[a], container[b]);
		}
	};

	template<typename T, typename UnaryPredicate>
	inline void erase_if(std::vector<T>& v, UnaryPredicate&& p)
	{
		v.erase(std::remove_if(v.begin(), v.end(), p), v.end());
	}

	template<typename T, typename Container, typename Index = std::size_t>
	struct index_vector {
		typedef std::vector<Index> indices_type;

		Container const& container;
		indices_type indices;
		Index container_last_sync_size = 0; // don't re-add removed indices elements in sync

		index_vector(Container const& container)
		: container(container) {
			sync();
		}

		template<typename UnaryPredicate>
		void erase_if(UnaryPredicate&& p)
		{
			erase_if(indices, [this,&p](Index ndx) { return p(container[ndx]); });
		}

		template<typename Compare>
		using compare_t = index_compare<T, Container, Compare, Index>;

		template<typename Compare>
		compare_t<typename std::decay<Compare>::type> compare_with(Compare&& compare)
		{
			return compare_t<typename std::decay<Compare>::type>(container, std::forward<Compare>(compare));
		}

		template<typename Compare = std::less<T>>
		compare_t<Compare> compare()
		{
			return compare_t<Compare>(container);
		}

		template<typename Compare = std::less<T>>
		void sort(Compare&& comp = Compare()) {
			auto index_comp = compare_with(std::forward<Compare>(comp));
			std::sort(indices.begin(), indices.end(), index_comp);
		}

		void sync() {
			auto const top = container.size();
			auto const have = indices.size();
			if (top > container_last_sync_size) {
				// new elements: add new indices at back
				indices.resize(have + (top - container_last_sync_size));
				auto middle = indices.begin() + have;
				std::iota(middle, indices.end(), container_last_sync_size);
			} else if (top < container_last_sync_size) {
				// remove elements with too large index
				indices.erase(std::remove_if(indices.begin(), indices.end(), [top](Index ndx) { return ndx >= top; }), indices.end());
			}
			container_last_sync_size = top;
		}

		template<typename Compare = std::less<T>>
		void sync_sorted(Compare&& comp = Compare()) {
			auto const top = container.size();
			auto const have = indices.size();
			if (top > container_last_sync_size) {
				// new elements: add new indices at back
				indices.resize(have + (top - container_last_sync_size));
				auto middle = indices.begin() + have;
				std::iota(middle, indices.end(), container_last_sync_size);

				// sort new elements
				auto index_comp = compare_with(std::forward<Compare>(comp));
				std::sort(middle, indices.end(), index_comp);

				// merge
				std::inplace_merge(indices.begin(), middle, indices.end(), index_comp);
			} else if (top < container_last_sync_size) {
				// remove elements with too large index
				indices.erase(std::remove_if(indices.begin(), indices.end(), [top](Index ndx) { return ndx >= top; }), indices.end());
			}
			container_last_sync_size = top;
		}

		void reset() {
			indices.clear();
			container_last_sync_size = 0;
			sync();
		}

		template<typename Compare = std::less<T>>
		void reset_sorted(Compare&& comp = Compare()) {
			indices.clear();
			container_last_sync_size = 0;
			sync();
			sort(std::forward<Compare>(comp));
		}

		class const_iterator : public std::iterator<std::random_access_iterator_tag, T const>
		{
			typedef std::iterator<std::random_access_iterator_tag, T const> base_iterator;

			Container const* container;
		public:
			using typename base_iterator::value_type;
			using typename base_iterator::difference_type;
			using typename base_iterator::reference;

			typedef typename indices_type::const_iterator index_iterator;
			index_iterator pos;

			typedef value_type const& const_reference;
			typedef value_type const* const_pointer;

			explicit const_iterator(Container const* container, index_iterator const& pos)
			: container(container), pos(pos) { }
			explicit const_iterator(Container const* container, index_iterator&& pos)
			: container(container), pos(std::move(pos)) { }

			const_iterator& operator++() { ++pos; return *this; }
			const_iterator operator++(int) { return const_iterator(container, pos++); }
			const_iterator operator+(difference_type seek) { return const_iterator(container, pos + seek); }
			const_iterator& operator+=(difference_type seek) { pos += seek; return *this; }
			const_iterator operator-(difference_type seek) { return const_iterator(container, pos - seek); }
			const_iterator& operator-=(difference_type seek) { pos -= seek; return *this; }
			const_iterator& operator--() { --pos; return *this; }
			const_iterator operator--(int) { return const_iterator(container, pos--); }
			difference_type operator-(const const_iterator& rhs) const { return pos - rhs.pos; }
			bool operator==(const const_iterator& rhs) const { return pos == rhs.pos; }
			bool operator!=(const const_iterator& rhs) const { return pos != rhs.pos; }
			bool operator <(const const_iterator& rhs) const { return pos  < rhs.pos; }
			bool operator<=(const const_iterator& rhs) const { return pos <= rhs.pos; }
			bool operator >(const const_iterator& rhs) const { return pos  > rhs.pos; }
			bool operator>=(const const_iterator& rhs) const { return pos >= rhs.pos; }
			const_reference operator[](difference_type seek) { return *const_iterator(container, pos + seek); }
			const_pointer operator->() const { return &(*container)[*pos]; }
			const_reference operator*() const { return (*container)[*pos]; }
		};

		typedef const_iterator iterator;

		const_iterator begin() const { return const_iterator(&container, indices.begin()); }
		const_iterator end() const { return const_iterator(&container, indices.end()); }
	};

}

#endif
