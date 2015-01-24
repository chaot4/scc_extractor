#ifndef _TRACK_TIME_H
#define _TRACK_TIME_H

/* doesn't depend on any ch_constructor stuff, simple "drop in" implementation;
 * everything inlined
 */

#include <chrono>
#include <vector>
#include <string>
#include <ostream>

class TrackTime {
private:
	struct entry {
		std::string title;
		std::chrono::duration<double> span;
		entry(std::string &&title, std::chrono::duration<double> &&span)
		: title(std::move(title)), span(std::move(span)) { }
	};
	std::vector<entry> record;
	std::ostream* os = nullptr;
	std::chrono::steady_clock::time_point last;

	void log(entry const& e) {
		if (!os) return;
		(*os) << "Took " << e.span.count() << " seconds: " << e.title << "\n";
	}

public:
	TrackTime(std::ostream& os) : os(&os), last(std::chrono::steady_clock::now()) { }
	TrackTime() : last(std::chrono::steady_clock::now()) { }

	void track(std::string&& title, bool log_entry = true)
	{
		using namespace std::chrono;
		auto old = last;
		last = steady_clock::now();
		record.emplace_back(std::move(title), duration_cast<duration<double>>(last - old));
		if (log_entry && os) {
			log(record.back());
			(*os) << "\n";
		}
	}

	void track(std::string const& title, bool log_entry = true)
	{
		track(std::string(title), log_entry);
	}

	void summary()
	{
		if (!os) return;
		(*os) << "\nTimeTrack summary:\n";
		for (auto const &entry: record) log(entry);
	}

};

#endif
