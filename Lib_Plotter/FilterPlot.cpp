#include "FilterPlot.h"

std::vector<FilterPlot*> FilterPlot::toUpdate = std::vector<FilterPlot*>();

void FilterPlot::Callback(const DataMsg & data) {
	if (ID == 0 && data.GetDataType() == valueType) { // TODO - sourceID
		unsigned int index = _index(valueType, data.GetDataSourceType());
		_plotValueAt(index, data.GetValue(), double(data.GetTimeInUs())*1e-6);
	}
}

std::string SystemValueType2String(DataType type) {
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

FilterPlot::FilterPlot(SystemManager & filter, System::SystemPtr sys, DataType type_, cvplot::Rect pose) :
	FilterLog(filter), valueType(type_), ID(sys->getID()), nViews(sys->getNumOf(type_)),
	plotter(sys->getName() + ": " + SystemValueType2String(type_),
	cvplot::Offset(pose.x, pose.y), cvplot::Size(pose.width, pose.height)) {
	// Create plots, set number of serieses	
	std::vector<std::string> viewNames = sys->getNames(valueType);
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

unsigned int FilterPlot::_num(DataType valueType) const {
	if (valueType == STATE || valueType == OUTPUT)
		return 3;
	else return 3;
}

std::string FilterPlot::_callTypeToString(OperationType callType) const {
	switch (callType) {
	case FILTER_MEAS_UPDATE:
		return "filt.";
	case SENSOR:
		return "meas.";
	case FILTER_TIME_UPDATE:
		return "pred.";
	case FILTER_PARAM_ESTIMATION:
		return "est.";
	case GROUND_TRUTH:
		return "truth";
	}
	throw std::runtime_error(std::string("System::_callTypeToString(): Unhandled argument!\n"));
}

unsigned int FilterPlot::_index(DataType valueType, OperationType callType) const {
	switch (valueType) {
	case STATE:
		switch (callType) {
		case FILTER_MEAS_UPDATE:
			return 0;
		case FILTER_TIME_UPDATE:
			return 1;
		case GROUND_TRUTH:
			return 2;
		}
		break;
	case OUTPUT:
		switch (callType) {
		case SENSOR:
			return 0;
		case FILTER_TIME_UPDATE:
			return 1;
		case GROUND_TRUTH:
			return 2;
		}
		break;
	case NOISE:
	case DISTURBANCE:
		switch (callType) {
		case FILTER_PARAM_ESTIMATION:
			return 0;
		case FILTER_TIME_UPDATE:
			return 1;
		case GROUND_TRUTH:
			return 2;
		}
	}
	throw std::runtime_error(std::string("System::_index(): Unhandled argument!\n"));
}

OperationType FilterPlot::_eventType(DataType valueType, unsigned int index) const {
	switch (valueType) {
	case STATE:
		switch (index) {
		case 0:
			return FILTER_MEAS_UPDATE;
		case 1:
			return FILTER_TIME_UPDATE;
		case 2:
			return GROUND_TRUTH;
		}
		break;
	case OUTPUT:
		switch (index) {
		case 0:
			return SENSOR;
		case 1:
			return FILTER_TIME_UPDATE;
		case 2:
			return GROUND_TRUTH;
		}
		break;
	case NOISE:
	case DISTURBANCE:
		return FILTER_TIME_UPDATE;
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
