#include <vector>

// for testing
#include <iostream>

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
		
		// for testing
		void printValues();
	private:
		vector<float> m_accumulator;
		int m_windowSize;
		void populateVector(int windowSize);
};

SimpleMovingAverage::SimpleMovingAverage(){
	// default window size = 3 samples
	m_windowSize = 3;
	SimpleMovingAverage::populateVector(m_windowSize);
}

SimpleMovingAverage::SimpleMovingAverage(int windowSize){
	m_windowSize = windowSize;
	SimpleMovingAverage::populateVector(m_windowSize);
}

SimpleMovingAverage::~SimpleMovingAverage(){
	
}

void SimpleMovingAverage::addSample(float value){
	m_accumulator.push_back(value);
	m_accumulator.erase(m_accumulator.begin());
}

float SimpleMovingAverage::getAverage(){
	float sum = 0;
	for (int i = 0; i < m_windowSize; i++)
		sum += m_accumulator.at(i);
	
	return sum/m_windowSize;
}

float SimpleMovingAverage::getAverage(float newValue){
	SimpleMovingAverage::addSample(newValue);
	
	float sum = 0;
	for (int i = 0; i < m_windowSize; i++)
		sum += m_accumulator.at(i);
	
	return sum/m_windowSize;
}

void SimpleMovingAverage::populateVector(int windowSize){
	for (int i = 0; i < windowSize; i++)
		m_accumulator.push_back(0);
}

void SimpleMovingAverage::printValues(){
	for (int i = 0; i < m_windowSize; i++)
		cout<<"vec["<<i<<"] = "<<m_accumulator.at(i)<<endl;
}
