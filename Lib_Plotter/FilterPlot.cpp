#include "FilterPlot.h"

std::vector<FilterPlot*> FilterPlot::toUpdate = std::vector<FilterPlot*>();

void FilterPlot::Callback(const FilterCallData & data) {
	if (data.ptr == ptr && data.type == valueType) {
		unsigned int index = _index(valueType, data.callType);
		_plotValueAt(index, data.value, data.t);
		
	}
}

std::string SystemValueType2String(SystemValueType type) {
	switch (type)
	{
	case NOISE:
		return "Noise";
	case DISTURBANCE:
		return "Disturbance";
	case STATE:
		return "State";
	case OUTPUT:
		return "Output";
	default:
		return "";
	}
}

FilterPlot::FilterPlot(SystemManager & filter, System::SystemPtr sys, SystemValueType type_, cvplot::Rect pose) :
	FilterLog(filter), valueType(type_), ptr(sys), nViews(sys->getNumOf(type_)),
	plotter(sys->getName() + ": " + SystemValueType2String(type_),
	cvplot::Offset(pose.x, pose.y), cvplot::Size(pose.width, pose.height)) {
	// Create plots, set number of serieses	
	std::vector<std::string> viewNames = ptr->getNames(valueType);
	unsigned int nSeries = _num(type_);
	std::vector<std::string> seriesNames = std::vector<std::string>();
	for (unsigned int i = 0; i < nSeries; i++)
		seriesNames.push_back(_callTypeToString(_eventType(type_, i)));
	for (unsigned int i = 0; i < nViews; i++) {
		plotter.addPlot(viewNames[i], nSeries, seriesNames, i == 0);
		// set linetypes
		for (unsigned int j = 0; j < nSeries; j++) {
			plotter.setLineType(i, j, cvplot::Dots);
			plotter.setLineColor(i, j, _color(j));
		}
		plotter.updatePlot(i);
	}
	toUpdate.push_back(this);
}

void FilterPlot::Update() {
	for (unsigned int i = 0; i < nViews; i++)
		plotter.updatePlot(i);
	plotter.updateWindow();
}

void FilterPlot::_plotValueAt(unsigned int index, const StatisticValue & value, double t) {
	for (unsigned int i = 0; i < plotter.getNumOfPlots(); i++) {
		plotter.addValue(i, index, (float)t, (float)value.vector(i));
		double sigma = sqrt(value.variance(i, i));
		plotter.addValue(i, index, (float)t, (float)(value.vector(i) + sigma));
		plotter.addValue(i, index, (float)t, (float)(value.vector(i) - sigma));
	}
}

unsigned int FilterPlot::_num(SystemValueType valueType) const {
	if (valueType == STATE || valueType == OUTPUT)
		return 3;
	else return 3;
}

std::string FilterPlot::_callTypeToString(FilterCallData::FilterCallType callType) const {
	switch (callType) {
	case FilterCallData::FILTERING:
		return "filt.";
	case FilterCallData::MEASUREMENT:
		return "meas.";
	case FilterCallData::PREDICTION:
		return "pred.";
	case FilterCallData::ESTIMATION:
		return "est.";
	case FilterCallData::GROUNDTRUTH:
		return "truth";
	}
	throw std::runtime_error(std::string("System::_callTypeToString(): Unhandled argument!\n"));
}

unsigned int FilterPlot::_index(SystemValueType valueType, FilterCallData::FilterCallType callType) const {
	switch (valueType) {
	case STATE:
		switch (callType) {
		case FilterCallData::FILTERING:
			return 0;
		case FilterCallData::PREDICTION:
			return 1;
		case FilterCallData::GROUNDTRUTH:
			return 2;
		}
		break;
	case OUTPUT:
		switch (callType) {
		case FilterCallData::MEASUREMENT:
			return 0;
		case FilterCallData::PREDICTION:
			return 1;
		case FilterCallData::GROUNDTRUTH:
			return 2;
		}
		break;
	case NOISE:
	case DISTURBANCE:
		switch (callType) {
		case FilterCallData::ESTIMATION:
			return 0;
		case FilterCallData::PREDICTION:
			return 1;
		case FilterCallData::GROUNDTRUTH:
			return 2;
		}
	}
	throw std::runtime_error(std::string("System::_index(): Unhandled argument!\n"));
}

FilterCallData::FilterCallType FilterPlot::_eventType(SystemValueType valueType, unsigned int index) const {
	switch (valueType) {
	case STATE:
		switch (index) {
		case 0:
			return FilterCallData::FILTERING;
		case 1:
			return FilterCallData::PREDICTION;
		case 2:
			return FilterCallData::GROUNDTRUTH;
		}
		break;
	case OUTPUT:
		switch (index) {
		case 0:
			return FilterCallData::MEASUREMENT;
		case 1:
			return FilterCallData::PREDICTION;
		case 2:
			return FilterCallData::GROUNDTRUTH;
		}
		break;
	case NOISE:
	case DISTURBANCE:
		return FilterCallData::PREDICTION;
	}
	throw std::runtime_error(std::string("System::_eventType(): Unhandled argument!\n"));
}

cvplot::Color FilterPlot::_color(unsigned int index) const {
	switch (index) {
	case 0:
		return cvplot::Blue;
	case 1:
		return cvplot::Black;
	case 2:
		return cvplot::Red;
	case 3:
		return cvplot::Green;
	default:
		return cvplot::Magenta;
	}
}

FilterPlot::~FilterPlot() {
	for (auto i = toUpdate.begin(); i != toUpdate.end(); i++)
		if (*i == this) {
			toUpdate.erase(i);
			return;
		}
}

void FilterPlot::UpdateWindows() {
	for (size_t i = 0; i < toUpdate.size(); i++)
		toUpdate[i]->Update();
}
