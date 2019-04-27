#pragma once
#include "Filtersystem.h"
#include "Plotter.h"
#include <string>

// set xy limits, legend fontsize, legend entries - the uncertainty as the values?
// threads?
// subset of states?

class FilterPlotter {
private:
	unsigned int iID;
	FilterSystem& filter_;
	cvplot::Plotter plotter;
	float t;
public:
	enum Type {STATE, OUTPUT, DISTURBANCE, NOISE};
private:
	Type type_;
public:
	FilterPlotter(FilterSystem& filter, Type type,
		cvplot::Rect pose = cvplot::Rect(30,30,300,150)) :
		filter_(filter),
		type_(type),
		plotter("debugger",cvplot::Offset(pose.x,pose.y),cvplot::Size(pose.width,pose.height)),
		t(0)
	{ // todo: állapotnevek?...
		// Gen ID for callback identifying
		static unsigned int IDcounter = 0;
		iID = IDcounter;
		IDcounter++;
		// Register callback
		filter.AddCallback([this](FilterData data,
			FilterEvents type_) {
			this->_addData(std::move(data), std::move(type_)); },iID);
		// Create plots, set number of serieses		
		if (type == STATE) {
			Eigen::Index nx = filter.GetState().Length();
			for (Eigen::Index i = 0; i < nx; i++) {
				std::vector<std::string> names = std::vector<std::string>();
				names.push_back("ground truth");
				names.push_back("filtered");
				names.push_back("prediction");
				plotter.addPlot("state " + std::to_string(i), 3, names); //0: ground truth, 1: filtered, 2: filtered 1 sigma, 3: predicted, 4: predicted 1 sigma 
				// set linetypes
				for (unsigned int n = 1; n < 3; n++)
					plotter.setLineType(i, n, cvplot::Dots);
			}
		}
		// Output / measurements

		// noises/disturbances
		

		//Plot the actual values?
		if (type == STATE) {
			StatisticValue x0 = filter.GetState();
			_plotValueAt(2, x0);
			_plotUncertaintyAt(2, x0);
			Eigen::Index nx = x0.Length();
			for (Eigen::Index i = 0; i < nx; i++)
				plotter.updatePlot(i);
		}
	}

	~FilterPlotter() {
		filter_.DeleteCallback(iID);
	}

	void _addData(const FilterData& v, FilterEvents type) {
		// Update timestamp
		auto it = v.find(DT);
		if (it != v.end())
			t += it->second.vector(0);
		it = v.find(T);
		// Update state plots
		if (type == STATE) {
			if (it != v.end())
				t = it->second.vector(0);
			it = v.find(GROUND_TRUTH_STATE);
			if (it != v.end())
				_plotValueAt(0, it->second);
			it = v.find(FILTERED_STATE);
			if (it != v.end()) {
				_plotValueAt(1, it->second);
				_plotUncertaintyAt(1, it->second);
			}
			it = v.find(PREDICTED_STATE);
			if (it != v.end()) {
				_plotValueAt(2, it->second);
				_plotUncertaintyAt(2, it->second);
			}
		}
		

		// todo: update plots
	}

private:
	void _plotValueAt(unsigned int index, const StatisticValue& value) {
		for (unsigned int i = 0; i < plotter.getNumOfPlots(); i++)
			plotter.addValue(i, index, value.vector(i), t);
	}

	void _plotUncertaintyAt(unsigned int index, const StatisticValue& value) {
		for (unsigned int i = 0; i < plotter.getNumOfPlots(); i++) {
			float sigma = sqrtf(value.variance(i, i));
			plotter.addValue(i, index, value.vector(i)+sigma, t);
			plotter.addValue(i, index, value.vector(i)-sigma, t);
		}
	}

};

