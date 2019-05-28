#pragma once

// set x/y interval
// ne csak zöld pacát toljon ki, ha nincs érték...

#include "window.h"
#include "figure.h"
#include "cvplot.h"
#include <string>
#include <map>

namespace cvplot {
	const int MAX_NUM_OF_ROWS = 6;

	struct SeriesID  {
		unsigned int plotindex;
		unsigned int seriesIndex;
		SeriesID(unsigned int plotID, unsigned int seriesID) : plotindex(plotID), seriesIndex(seriesID) {};
		bool operator<(SeriesID id2) const {
			if (plotindex < id2.plotindex)
				return true;
			if (plotindex == id2.plotindex && seriesIndex < id2.seriesIndex)
				return true;
			return false;
		};
	};

	class Plotter
	{
		cvplot::Window W;
		unsigned int numOfPlots;
		std::string name;

		Size plotSize;
		Offset offset;

		std::map<unsigned int, Figure::FigurePtr> figureMap;
		std::map<unsigned int, View::ViewPtr> viewMap;
		std::map<SeriesID, Series::SeriesPtr> seriesMap;
		
		Figure::FigurePtr _getNthFigure(unsigned int n);
		
		View::ViewPtr _getNthView(unsigned int n);
		
		Series::SeriesPtr _getNthSeries(unsigned int plotindex, unsigned int seriesindex);
		
		void _setNthFigure(unsigned int n, Figure::FigurePtr figure);

		void _setNthView(unsigned int n, View::ViewPtr view);

		void _setNthSeries(unsigned int plotindex, unsigned int seriesindex, Series::SeriesPtr series);
		
	public:
		~Plotter() {}

		Plotter(std::string name_, Offset o = Offset(60, 0), Size plotsize = Size(600, 200));

		void addPlot(std::string plotName, unsigned int numofsignals,
			std::vector<std::string> names = std::vector<std::string>(), bool showLegend=false);

		void setLineType(unsigned int plotindex, unsigned int seriesindex, Type type);

		void setLineColor(unsigned int plotindex, unsigned int seriesindex, Color color);

		void addValue(unsigned int plotindex, unsigned int seriesindex, float t, float value);

		void updatePlot(unsigned int plotindex);

		void updateWindow();

		unsigned int getNumOfPlots() const { return numOfPlots; }
	};

}