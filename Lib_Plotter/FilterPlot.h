#pragma once
#include "Plotter.h"
#include "DataMsg.h"
#include <string>

/*! \brief Class for fast presentation of the data comes form sensors and the filter
*
* Usage: the type of msg-es to be visualized must be chosen in the constructor
*
* Then 
* - static void UpdateWindows(); updates all of the plots,
* - static void AddData(const DataMsg& data); forwards data to them
*/
class FilterPlot {
public:
	FilterPlot(unsigned int systemID, std::string systemName, std::vector<std::string> valueNames, DataType type_,
		cvplot::Rect pose = cvplot::Rect(60, 30, 500, 150));
	/*!< Constructor: the filter will plot only msg-s from "systemID", with type "type_", the name of the window will contain the systemName,
	the name of the plots will be set according to the valueNames */

	~FilterPlot(); /*!< Destructor */

	void AddDataToThis(const DataMsg& data); /*!< Add data to this window */

	void UpdateThis(); /*!< Update this window */

	static void UpdateWindows(); /*!< Update every window */

	static void AddDataToWindows(const DataMsg& data);  /*!< Send data to every window */

private:
	cvplot::Plotter plotter;

	DataType valueType;

	unsigned int ID;

	unsigned int nViews;

	static std::vector<FilterPlot*> toUpdate;

	void _plotValueAt(unsigned int index, const StatisticValue& value, double t);

	unsigned int _num(DataType valueType) const;

	std::string _callTypeToString(OperationType callType) const;

	unsigned int _index(DataType valueType, OperationType callType) const;

	OperationType _eventType(DataType valueType, unsigned int index) const;

	cvplot::Color _color(unsigned int index) const;
};