#include <vector>

// for testing
//#include <iostream>

using namespace std;

class SimpleMovingAverage
{
	public:
		SimpleMovingAverage();
		SimpleMovingAverage(int windowSize);
		~SimpleMovingAverage();
		
		void addSample(float value);
		float getAverage();
		float getAverage(float newValue);
		// this will give just 0 if the new value is 0
		float getAverageExceptZero(float newValueWhichCouldBeZero);
		
		// for testing only
		void printValues();
	private:
		vector<float> m_accumulator;
		int m_windowSize;
		void populateVector(int windowSize);
		float calculateAverage();
};
