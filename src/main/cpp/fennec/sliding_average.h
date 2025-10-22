#pragma once

template<typename T, int Width>
struct SlidingAvg {
	T history[Width] { };
};

template<typename T, int Width>
inline T eval(SlidingAvg<T, Width> avg)
{
	T mean = { };
	for (int i=0; i<Width; ++i)
		mean = mean + avg.history[i];

	return mean / Width;
}

template<typename T, int Width>
inline void update(SlidingAvg<T, Width>* avg, T new_data)
{
	for (int i=1; i<Width; ++i)
		avg->history[i - 1] = avg->history[i];

	avg->history[Width - 1] = new_data;
}