#include "simpleMovingAverage.h"

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
	return SimpleMovingAverage::calculateAverage();
}

float SimpleMovingAverage::getAverage(float newValue){
	SimpleMovingAverage::addSample(newValue);
		
	return SimpleMovingAverage::calculateAverage();
}

float SimpleMovingAverage::getAverageExceptZero(float newValueWhichCouldBeZero){
	if (newValueWhichCouldBeZero == 0) {
		SimpleMovingAverage::addSample(newValueWhichCouldBeZero);
		// consider this zero for the average, but give no output
		return 0;
	} else {
		SimpleMovingAverage::addSample(newValueWhichCouldBeZero);
		return SimpleMovingAverage::calculateAverage();
	}
}

void SimpleMovingAverage::populateVector(int windowSize){
	for (int i = 0; i < windowSize; i++)
		m_accumulator.push_back(0);
}

float SimpleMovingAverage::calculateAverage(){
	float sum = 0;
	for (int i = 0; i < m_windowSize; i++)
		sum += m_accumulator.at(i);
	
	return sum/m_windowSize;
}


void SimpleMovingAverage::printValues(){
	//for (int i = 0; i < m_windowSize; i++)
		//cout<<"vec["<<i<<"] = "<<m_accumulator.at(i)<<endl;
}
