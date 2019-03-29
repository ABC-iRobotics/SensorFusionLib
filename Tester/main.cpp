// SensorFusion.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>

#include "youBotSystem.h"

#include "IMUSensor.h"

#include <vector>

#include "FilterSystem.h"

#include "FunctionMerge.h"

#include "cvplot.h"

#include <windows.h>

#include <chrono>

#include <opencv2/opencv.hpp>

/*
void example() {
	cvplot::figure("myplot").series("myline").addValue({ 1.f, 3.f, 2.f, 5.f, 4.f });
	cvplot::figure("myplot").show();
}
*/
/*
void demo() {
	std::vector<std::pair<float, float>> data;
	std::vector<float> values;

	cvplot::Window::current("cvplot demo").offset({ 60, 100 });

	{
		auto name = "simple";
		cvplot::setWindowTitle(name, "line and histogram");
		cvplot::moveWindow(name, 0, 0);
		cvplot::resizeWindow(name, 300, 300);
		auto &figure = cvplot::figure(name);
		figure.series("line")
			.setValue({ 1.f, 2.f, 3.f, 4.f, 5.f })
			.type(cvplot::DotLine)
			.color(cvplot::Blue);
		figure.series("histogram")
			.setValue({ 1.f, 2.f, 3.f, 4.f, 5.f })
			.type(cvplot::Histogram)
			.color(cvplot::Red);
		figure.show(false);
	}

	{
		auto name = "math";
		cvplot::setWindowTitle(name, "math curves");
		cvplot::moveWindow(name, 300, 0);
		cvplot::resizeWindow(name, 300, 300);
		auto &figure = cvplot::figure(name);
		values.clear();
		for (auto i = 0; i <= 10; i++) {
			values.push_back((i - 4) * (i - 4) - 6);
		}
		figure.series("parabola")
			.setValue(values)
			.type(cvplot::DotLine)
			.color(cvplot::Green);
		values.clear();
		for (auto i = 0; i <= 10; i++) {
			values.push_back(sin(i / 1.5f) * 5);
		}
		figure.series("sine")
			.setValue(values)
			.type(cvplot::DotLine)
			.color(cvplot::Blue);
		values.clear();
		values.push_back(15);
		figure.series("threshold")
			.setValue(values)
			.type(cvplot::Horizontal)
			.color(cvplot::Red);
		figure.show(false);
	}

	{
		auto name = "scatter";
		cvplot::setWindowTitle(name, "scatter plots");
		cvplot::moveWindow(name, 600, 0);
		cvplot::resizeWindow(name, 300, 300);
		auto &figure = cvplot::figure(name);
		data.clear();
		for (auto i = 0; i <= 100; i++) {
			data.push_back({ (rand() % 100) / 10.f, (rand() % 100) / 10.f });
		}
		figure.series("uniform").set(data).type(cvplot::Dots).color(cvplot::Orange);
		data.clear();
		for (auto i = 0; i <= 100; i++) {
			data.push_back(
				{ exp((rand() % 100) / 30.f) - 1, exp((rand() % 100) / 30.f) - 1 });
		}
		figure.series("exponential")
			.set(data)
			.type(cvplot::Dots)
			.color(cvplot::Magenta);
		figure.show(false);
	}

	{
		auto name = "histograms";
		cvplot::setWindowTitle(name, "multiple histograms");
		cvplot::moveWindow(name, 0, 300);
		cvplot::resizeWindow(name, 300, 300);
		auto &figure = cvplot::figure(name);
		figure.series("1")
			.setValue({ 1.f, 2.f, 3.f, 4.f, 5.f })
			.type(cvplot::Histogram)
			.color(cvplot::Blue.alpha(201));
		figure.series("2")
			.setValue({ 6.f, 5.f, 4.f, 3.f, 2.f, 1.f })
			.type(cvplot::Histogram)
			.color(cvplot::Green.alpha(201));
		figure.series("3")
			.setValue({ 3.f, 1.f, -1.f, 1.f, 3.f, 7.f })
			.type(cvplot::Histogram)
			.color(cvplot::Red.alpha(201));
		figure.show(false);
	}

	{
		auto name = "parametric";
		cvplot::setWindowTitle(name, "parametric plots");
		cvplot::moveWindow(name, 0, 600);
		cvplot::resizeWindow(name, 300, 300);
		auto &figure = cvplot::figure(name);
		figure.square(true);
		data.clear();
		for (auto i = 0; i <= 100; i++) {
			data.push_back({ cos(i * .0628f + 4) * 2, sin(i * .0628f + 4) * 2 });
		}
		figure.series("circle").add(data);
		data.clear();
		for (auto i = 0; i <= 100; i++) {
			data.push_back({ cos(i * .2513f + 1), sin(i * .0628f + 4) });
		}
		figure.series("lissajous").add(data);
		figure.show(false);
	}

	{
		auto name = "no-axis";
		cvplot::setWindowTitle(name, "hidden axis");
		cvplot::moveWindow(name, 600, 600);
		cvplot::resizeWindow(name, 300, 300);
		auto &figure = cvplot::figure(name);
		figure.origin(false, false);
		figure.series("histogram")
			.setValue({ 4.f, 5.f, 7.f, 6.f })
			.type(cvplot::Vistogram)
			.color(cvplot::Blue);
		figure.series("min")
			.setValue(4.f)
			.type(cvplot::Vertical)
			.color(cvplot::Pink);
		figure.series("max")
			.setValue(7.f)
			.type(cvplot::Vertical)
			.color(cvplot::Purple);
		figure.show(false);
	}

	{
		auto name = "colors";
		cvplot::setWindowTitle(name, "auto color");
		cvplot::moveWindow(name, 900, 0);
		cvplot::resizeWindow(name, 300, 300);
		auto &figure = cvplot::figure(name);
		figure.series("color")
			.dynamicColor(true)
			.type(cvplot::Vistogram)
			.legend(false);
		for (auto i = 0; i < 16; i++) {
			figure.series("color").addValue(6, cvplot::Color::index(i).hue());
		}
		figure.show(false);
	}

	{
		auto name = "fill";
		cvplot::setWindowTitle(name, "filled line");
		cvplot::moveWindow(name, 900, 0);
		cvplot::resizeWindow(name, 300, 300);
		auto &figure = cvplot::figure(name);
		figure.gridSize(20);
		figure.series("fossil").type(cvplot::FillLine).color(cvplot::Orange);
		figure.series("electric")
			.type(cvplot::FillLine)
			.color(cvplot::Green.gamma(.5f));
		for (auto i = 0; i < 16; i++) {
			figure.series("fossil").addValue(10 - i + 10.f * rand() / RAND_MAX);
			figure.series("electric").addValue(i - 10 + 10.f * rand() / RAND_MAX);
		}
		figure.show(false);
	}

	{
		auto name = "range";
		cvplot::setWindowTitle(name, "range plot");
		cvplot::moveWindow(name, 900, 300);
		cvplot::resizeWindow(name, 300, 300);
		auto &figure = cvplot::figure(name);
		values.clear();
		figure.series("apples").type(cvplot::RangeLine).color(cvplot::Orange);
		figure.series("pears").type(cvplot::RangeLine).color(cvplot::Sky);
		for (auto i = 0; i <= 10; i++) {
			float v = (i - 4) * (i - 4) - 6;
			figure.series("apples").addValue(v + 10.f + 5.f * rand() / RAND_MAX,
				v + 5.f * rand() / RAND_MAX,
				v + 20.f + 5.f * rand() / RAND_MAX);
			v = -(i - 6) * (i - 6) + 30;
			figure.series("pears").addValue(v + 10.f + 5.f * rand() / RAND_MAX,
				v + 5.f * rand() / RAND_MAX,
				v + 20.f + 5.f * rand() / RAND_MAX);
		}
		figure.show(false);
	}

	{
		auto name = "balls";
		cvplot::setWindowTitle(name, "transparent circles");
		cvplot::moveWindow(name, 300, 600);
		cvplot::resizeWindow(name, 300, 300);
		auto &figure = cvplot::figure(name);
		figure.series("purple")
			.type(cvplot::Circle)
			.color(cvplot::Purple.alpha(192));
		figure.series("aqua").type(cvplot::Circle).color(cvplot::Aqua.alpha(193));
		for (auto i = 0; i <= 20; i++) {
			figure.series("purple").add(
				(rand() % 100) / 10.f, { (rand() % 100) / 10.f, (rand() % 100) / 5.f });
			figure.series("aqua").add((rand() % 100) / 10.f,
				{ (rand() % 100) / 10.f, (rand() % 100) / 5.f });
		}
		figure.show(false);
	}
	/*
	{
		auto name = "image";
		cvplot::setWindowTitle(name, "image and text");
		cvplot::moveWindow(name, 900, 600);
		cvplot::resizeWindow(name, 300, 300);
		auto &view = cvplot::Window::current().view(name);
		auto image = cv::imread("res/demo.jpg");
		cv::copyMakeBorder(image, image, 100, 100, 0, 0, cv::BORDER_REPLICATE);
		view.drawImage(&image);
		view.drawText("..and text", { 13, 273 }, cvplot::Black.alpha(127));
		view.finish();
	}
	
	{
		auto name = "dynamic";
		cvplot::setWindowTitle(name, "dynamic plotting");
		cvplot::moveWindow(name, 300, 300);
		cvplot::resizeWindow(name, 600, 300);
		auto &view = cvplot::Window::current().view(name);
		auto &figure = cvplot::figure(name);
		figure.square(true);
		figure.origin(false, false);
		srand(clock());
		auto x = 0.f, y = 0.f, dx = 1.f, dy = 0.f, f = 0.f, df = 0.f;
		figure.series("random").dynamicColor(true).legend(false);
		clock_t time = 0;
		for (int i = 0; i < 1000; i++) {
			auto fps = CLOCKS_PER_SEC / (float)(clock() - time);
			time = clock();
			auto l = sqrt((dx * dx + dy * dy) * (f * f + 1)) * 10;
			dx = (dx + f * dy) / l;
			dy = (dy - f * dx) / l;
			f = (f + df) * 0.8f;
			df = (df + rand() % 11 / 100.f - .05f) * 0.8f;
			figure.series("random").add(x += dx, { y += dy, i / 10.f });
			figure.show(false);
			auto string = std::to_string(fps).substr(0, 4) + " fps  " +
				std::to_string(i / 10.f).substr(0, 4) + "%";
			view.drawText(string, { 480, 277 }, cvplot::Gray);
			view.finish();
			view.flush();
		}
	}
}
*/
void mouse_callback(int event, int x, int y, int flags, void *param) {
	auto &view = *(cvplot::View *)param;
	std::ostringstream stream;
	if (event == cv::EVENT_MOUSEMOVE) stream << "mousemove";
	if (event == cv::EVENT_LBUTTONDOWN) stream << "lbuttondown";
	if (event == cv::EVENT_RBUTTONDOWN) stream << "rbuttondown";
	if (event == cv::EVENT_MBUTTONDOWN) stream << "mbuttondown";
	if (event == cv::EVENT_LBUTTONUP) stream << "lbuttonup";
	if (event == cv::EVENT_RBUTTONUP) stream << "rbuttonup";
	if (event == cv::EVENT_MBUTTONUP) stream << "mbuttonup";
	if (event == cv::EVENT_LBUTTONDBLCLK) stream << "lbuttondblclk";
	if (event == cv::EVENT_RBUTTONDBLCLK) stream << "rbuttondblclk";
	if (event == cv::EVENT_MBUTTONDBLCLK) stream << "mbuttondblclk";
#if CV_MAJOR_VERSION > 2
	if (event == cv::EVENT_MOUSEWHEEL) stream << "mousewheel";
	if (event == cv::EVENT_MOUSEHWHEEL) stream << "mousehwheel";
#endif
	if (flags & cv::EVENT_FLAG_LBUTTON) stream << " lbutton";
	if (flags & cv::EVENT_FLAG_RBUTTON) stream << " rbutton";
	if (flags & cv::EVENT_FLAG_MBUTTON) stream << " mbutton";
	if (flags & cv::EVENT_FLAG_CTRLKEY) stream << " ctrlkey";
	if (flags & cv::EVENT_FLAG_SHIFTKEY) stream << " shiftkey";
	if (flags & cv::EVENT_FLAG_ALTKEY) stream << " altkey";
	stream << "  " << x << "," << y;
	view.drawRect({ 10, 24, 280, 12 }, cvplot::Light);
	view.drawText(stream.str(), { 10, 25 }, cvplot::Black);
}
/*
void transparency() {
	std::vector<std::pair<float, float>> data;
	std::vector<float> values;

	cvplot::Window::current("cvplot transparency").offset({ 30, 70 }).cursor(true);

	{
		auto name = "opaque";
		cvplot::setWindowTitle(name, "opaque");
		cvplot::moveWindow(name, 0, 0);
		cvplot::resizeWindow(name, 300, 300);
		cvplot::setMouseCallback(name, mouse_callback);
		cvplot::Window::current().view(name).frameColor(cvplot::Sky);
		auto &figure = cvplot::figure(name);
		figure.series("histogram")
			.setValue({ 1.f, 2.f, 3.f, 4.f, 5.f })
			.type(cvplot::Histogram)
			.color(cvplot::Red)
			.legend(false);
		figure.show(false);
	}

	{
		auto name = "transparent";
		auto alpha = 200;
		cvplot::setWindowTitle(name, "transparent");
		cvplot::moveWindow(name, 100, 100);
		cvplot::resizeWindow(name, 300, 300);
		cvplot::setMouseCallback(name, mouse_callback);
		cvplot::Window::current().view(name).frameColor(cvplot::Sky).alpha(alpha);
		auto &figure = cvplot::figure(name);
		figure.series("histogram")
			.setValue({ 5.f, 4.f, 3.f, 2.f, 1.f })
			.type(cvplot::Histogram)
			.color(cvplot::Blue.alpha(alpha))
			.legend(false);
		figure.alpha(alpha).show(true);
	}
}
*/
#include <string>
#include "plotter.h"

int main()
{
	//example();
	//transparency();
	//demo();
	
	cvplot::Plotter f("States");
	f.addPlot("vx",2); f.updatePlot(0);
	f.addPlot("vy",3);
	f.addPlot("vz", 4, {"a","b","c"});

	f.addValue(0, 1, 20, 10000);
	f.addValue(0, 1, 21, 10020);

	f.addValue(2, 1, 20, 10000);
	f.addValue(2, 1, 21, 10020);

	f.updatePlot(2);

	f.setLineType(2, 1, cvplot::Type::Dots);

	f.addValue(2, 1, 25, 800);
	f.addValue(2, 1, 21, 10020);

	f.updatePlot(2);

	unsigned int ifff;
	std::cin >> ifff;

	
	return 0;

	
	BaseSystem::BaseSystemPtr youBot = std::make_shared<youBotSystem>(0.002, 0.4, 0.25, 0.05);

	// TODO: now different Ts can be in the systems - include into the update/output functions, set centrally??
	// what happens if it is uncertain?

	FilterSystem sys(youBot);

	Sensor::SensorPtr  imu = std::make_shared<IMUSensor>();
	sys.AddSensor(imu);

	FunctionMerge merger(youBot->getUpdateMapping(0.001));
	merger.AddFunction(imu->getUpdateMapping(0.001));
	

	bool exit = false;


	auto start = std::chrono::steady_clock::now();
	/*
	while (exit == false)
	{
		auto actual = std::chrono::steady_clock::now();


		if (GetAsyncKeyState(VK_ESCAPE))
		{
			exit = true;
		}
		cvplot::figure("myplot").series("myline")
			.addValue((float)std::chrono::duration_cast<std::chrono::milliseconds>(actual-start).count());
		cvplot::figure("myplot").show();
		start = actual;
	}
	*/
	

	unsigned int i;
	std::cin >> i;

	return 0;


	/*
	Eigen::MatrixXd CWnl = merger.GetWSelectorNl();
	Eigen::MatrixXd SWnl = CWnl * S * CWnl.transpose();
	unsigned int n = Snl.cols();
	Eigen::LLT<Eigen::MatrixXd> chol(Snl);
	Eigen::MatrixXd sqrtSnl = chol.matrixL();
	*/




	//std::cout << S << std::endl << Snl << std::endl;




	
	

	

	//std::cout << sqrtSnl << std::endl << sqrtSnl* sqrtSnl.transpose() << std::endl;




	



	return 0;

	/*
	Eigen::VectorXd x = Eigen::VectorXd::Ones(12);
	Eigen::VectorXd w = Eigen::VectorXd::Ones(4);

	Eigen::VectorXd v0 = Eigen::VectorXd::Ones(2);

	Eigen::MatrixXd m(2, 0);
	Eigen::VectorXd v(0);

	std::cout << (v0+m*v) << std::endl;

	std::cout << merger.Eval(x,w) << std::endl;





	return 0;

	std::cout << sys.GetStateVector() << std::endl;

	std::cout << sys.GetVarianceMatrix() << std::endl;
	std::cout << "1\n";

	youBot->SetValues(StatisticValue(4),System::ValueType::DISTURBANCE);

	imu->MeasurementDone(Eigen::VectorXd(3));
	std::cout << "2\n";
	imu->MeasurementDone(Eigen::VectorXd(3));
	std::cout << "3\n";*/
}