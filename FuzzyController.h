/**
	Author: Oliver Kamperis
	Date of last edit: 14.04.2018
	Version: 0.7.1 - BETA - TESTED STABLE
	Usage: This code is made available free to use for educational purposes.
	This is a prototype version that is liable to change without notice and may potentially contain bugs.
	If a bug is found please contact: Oliver Kamperis via email at olliekampo@gmail.com.
	
	BSD 3-Clause License

	Copyright (c) 2018, Oliver Kamperis
	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:

	* Redistributions of source code must retain the above copyright notice, this
	  list of conditions and the following disclaimer.

	* Redistributions in binary form must reproduce the above copyright notice,
	  this list of conditions and the following disclaimer in the documentation
	  and/or other materials provided with the distribution.

	* Neither the name of the copyright holder nor the names of its
	  contributors may be used to endorse or promote products derived from
	  this software without specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
	AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
	IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
	DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
	FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
	DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
	SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
	OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
	OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <thread>
#include <chrono>

/**
	@author Oliver Kamperis
	@since 0.1.0
	@date 05.02.2018
	The FuzzyController class.
	Defines a Takagi-Seguno style fuzzy inference engine for control engineering purposes.
	Contains inbuilt functionality for the following:
		Proportional, integral and differential input variables
		Triangular, trapezoidal, rectangular, piecewise linear, singleton and gaussian membership functions
		Trapezoidal resolution hedges
		Multiple antecendant aggregate statement rules
		Rule modules
		Dynamic importance degrees
*/
template<class T>
class FuzzyController {
public:
	/**
		@author Oliver Kamperis
		@since 0.1.0
		@date 05.02.2018
		The templated abstract Variable class represents proportional input variables to the fuzzy controller.
		Requires a template argument of the same class type as the fuzzy controller it is to be assigned to.
		Requires overriding of the pure virtual function: virtual double getInput() const = 0;
	*/
	template<class S = T>
	class Variable {
		friend class FuzzyController<T>;
	public:
		const S* const system;
		const std::string name;
		const double maxVal;
		const double minVal;
		const double gain;
	protected:
		double value = 0.5;

	public:
		/**
			Constructs a proportional input variable.
			@template S - Class type of the system to control
			@param system - The system to control
			@param name - The arbitrary name of the variable
			@param maxVal - The maxima of the input range of the variable
			@param minVal - The maxima of the input range of the variable
			@param gain - The input gain to the variable
			@throws std::invalid_argument exception if the minimum value is greater than or equal to the maximum value
		*/
		Variable(S* const system, std::string name, double maxVal, double minVal, double gain = 1.0)
			: system(system), name(name), maxVal(maxVal), minVal(minVal), gain(gain) {
			if (minVal >= maxVal) {
				throw std::invalid_argument("Minimum value for fuzzy input variables must be less than the maximum value, i.e. the range of possible values must be greater than zero.");
			}
		}

		~Variable() {

		}

	protected:
		/**
			The user must override this function when defining new input variables.
			@return The raw value of the input variable from the system
		*/
		virtual double getInput() const = 0;

		/**
			Calculates the value of the input variable from getInput() multiplied by the input gain and then normalised around the input bounds.
			The user may override this function when defining new types of input variable.
		*/
		virtual void calculateValue() {
			this->value = ((this->gain * this->getInput()) - this->minVal) / (this->maxVal - this->minVal);
		}

	public:
		/**
			@return the current value of the normalised input as a double
		*/
		double getValue() const {
			return this->value;
		}

		bool operator==(const Variable<S>& v) const {
			return this->name == v.name;
		}
	};

	/**
		@author Oliver Kamperis
		@since 0.1.2
		@date 08.02.2018
		The templated abstract Variable class represents integral input variables to the fuzzy controller.
		Requires a template argument of the same class type as the fuzzy controller it is to be assigned to.
		Requires overriding of the pure virtual function: virtual double getInput() const = 0;
	*/
	template<class R = T>
	class IntegralVariable : public Variable<R> {
	public:
		const double setPoint;
	private:
		double integralSum = 0.0;
		std::chrono::time_point<std::chrono::system_clock> timeLast = std::chrono::system_clock::now();

	public:
		/**
			Constructs an integral input variable.
			@template R - Class type of the system to control
			@param system - The system to control
			@param name - The arbitrary name of the variable
			@param maxVal - The maxima of the input range of the variable
			@param minVal - The maxima of the input range of the variable
			@param gain - The input gain to the variable, defining the integral ramping factor
		*/
		IntegralVariable(R* const system, std::string name, double maxVal, double minVal, double gain = 1.0)
			: Variable(system, name, maxVal, minVal, gain), setPoint((maxVal + minVal) / 2.0) {

		}

		~IntegralVariable() {

		}

	protected:
		/**
			Estimates the normalised integral error from getInput().
		*/
		virtual void calculateValue() override final {
			std::chrono::time_point<std::chrono::system_clock> timeCurrent = std::chrono::system_clock::now();
			std::chrono::duration<double> deltaTime = timeCurrent - this->timeLast;
			this->timeLast = timeCurrent;
			this->integralSum += (this->getInput() - this->setPoint) * deltaTime.count();
			this->value = (((this->integralSum * this->gain) + this->setPoint) - this->minVal) / (this->maxVal - this->minVal);
		}
	};

	/**
		@author Oliver Kamperis
		@since 0.1.2
		@date 08.02.2018
		The templated abstract Variable class represents differential (derivative) input variables to the fuzzy controller.
		Requires a template argument of the same class type as the fuzzy controller it is to be assigned to.
		Requires overriding of the pure virtual function: virtual double getInput() const = 0;
	*/
	template<class Q = T>
	class DifferentialVariable : public Variable<Q> {
	private:
		double inputLast = 0.0;
		std::chrono::time_point<std::chrono::system_clock> timeLast = std::chrono::system_clock::now();

	public:
		/**
			Constructs a differential input variable.
			@template Q - Class type of the system to control
			@param system - The system to control
			@param name - The arbitrary name of the variable
			@param maxVal - The maxima of the input range of the variable
			@param minVal - The maxima of the input range of the variable
			@param gain - The input gain to the variable, defining the damping factor
		*/
		DifferentialVariable(Q* const system, std::string name, double maxVal, double minVal, double gain = 1.0)
			: Variable(system, name, maxVal, minVal, gain) {

		}

		~DifferentialVariable() {

		}

	protected:
		/**
			Estimates the normalised rate of change of error from getInput().
		*/
		virtual void calculateValue() override final {
			std::chrono::time_point<std::chrono::system_clock> timeCurrent = std::chrono::system_clock::now();
			std::chrono::duration<double> deltaTime = timeCurrent - this->timeLast;
			this->timeLast = timeCurrent;
			double temp = this->inputLast;
			this->inputLast = this->getInput();
			this->value = ((this->gain * ((this->inputLast - temp) / deltaTime.count())) - this->minVal) / (this->maxVal - this->minVal);
		}
	};

	/**
		@author Oliver Kamperis
		@since 0.1.0
		@date 05.02.2018
		Defines a resolution hedge.
	*/
	class Hedge final {
	public:
		const std::string name;
		const double start;
		const double firstPeek;
		const double secondPeek;
		const double end;

		/**
			Constructs a trapezoidal resolution hedge.
			@param name - The arbitrary name of the hedge
			@param start - The start value of the hedge as a double
			@param firstPeek - The first peek value of the hedge as a double
			@param secondPeek - The second peek value of the hedge as a double
			@param end - The end value of the hedge as a double
			@throws std::invalid_argument exception if the start value is greater than the first peek value or the first peek value is greater than the second peek value or the second peek value is greater than the end value
		*/
		Hedge(std::string name, double start, double firstPeek, double secondPeek, double end)
			: name(name), start(start), firstPeek(firstPeek), secondPeek(secondPeek), end(end) {
			if (firstPeek < start) {
				throw std::invalid_argument("Start value for fuzzy hedges must be less than or equal to the first peek value.");
			}
			else if (secondPeek < firstPeek) {
				throw std::invalid_argument("First peek value for fuzzy hedges must be less than or equal to the second peek value.");
			}
			else if (end < secondPeek) {
				throw std::invalid_argument("Second peek value for fuzzy hedges must be less than or equal to the end value.");
			}
		}

		~Hedge() {

		}

		/**
			Fuzzifies the input argument to determine the degree of membership of the input within the fuzzy set as defined by the hedge.
			@param x - Input value as a double
			@return degree of membership of the input value within the fuzzy set defined by this hedge as a double
		*/
		virtual double fuzzify(double x) const {
			if (x >= this->firstPeek && x <= this->secondPeek) {
				return 1.0;
			}
			else if (x < this->start || x > this->end) {
				return 0.0;
			}
			else if (x < this->firstPeek) {
				return (x - this->start) / (this->firstPeek - this->start);
			}
			else {
				return 1.0 - ((x - this->secondPeek) / (this->end - this->secondPeek));
			}
		}

		bool operator==(const Hedge& h) const {
			return this->name == h.name;
		}
	};

	/**
		@author Oliver Kamperis
		@since 0.1.0
		@date 05.02.2018
		Base class for membership functions.
	*/
	class Function {
	public:
		const std::string name;

		/**
			Constructs a membership function.
			@param name - The arbitrary name of the function
		*/
		Function(std::string name)
			: name(name) {
		}

		~Function() {

		}

		/**
			Fuzzifies the input argument to determine the degree of membership of the input within the fuzzy set defined by the membership function.
			The user must override this function when defining new membership functions.
			@param x - Input value as a double
			@return degree of membership of the input value as a double
		*/
		virtual double fuzzify(double x) const = 0;

		/**
			Normalises the input around the bounds of the membership function for fuzzification of a resolution hedge.
			@param x - Input value as a double
			@return normalised input around the bounds of the membership function as a double
		*/
		virtual double normaliseInputForHedge(double x) const = 0;

		bool operator==(const Function& f) const {
			return this->name == f.name;
		}
	};

	/**
		@author Oliver Kamperis
		@since 0.1.0
		@date 05.02.2018
		Defines a triangular membership function.
	*/
	class Triangular final : public Function {
	public:
		const double start;
		const double peek;
		const double end;

		/**
			Constructs a triangular membership function.
			@param name - The arbitrary name of the function
			@param start - The start value of the membership function as a double
			@param peek - The peek value of the membership function as a double
			@param end - The end value of the membership function as a double
			@throws std::invalid_argument exception if the start value is greater than the peek value or the peek value is greater than the end value
		*/
		Triangular(std::string name, double start, double peek, double end)
			: Function(name), start(start), peek(peek), end(end) {
			if (peek < start) {
				throw std::invalid_argument("Start value for triangular functions must be less than or equal to the peek value.");
			}
			else if (end < peek) {
				throw std::invalid_argument("Peek value for triangular functions must be less than or equal to the end value.");
			}
		}

		~Triangular() {

		}

		/**
			Fuzzifies the input argument to determine the degree of membership of the input within the fuzzy set defined by the membership function.
			@param x - Input value as a double
			@return degree of membership of the input value as a double
		*/
		virtual double fuzzify(double x) const override {
			if (x < this->start || x > this->end) {
				return 0.0;
			}
			else if (x < this->peek) {
				return (x - this->start) / (this->peek - this->start);
			}
			else {
				return 1.0 - ((x - this->peek) / (this->end - this->peek));
			}
		}

		/**
			Normalises the input around the bounds of the membership function for fuzzification of a resolution hedge.
			@param x - Input value as a double
			@return normalised input around the bounds of the membership function as a double
		*/
		virtual double normaliseInputForHedge(double x) const override {
			return (x - this->start) / (this->end - this->start);
		}
	};

	/**
		@author Oliver Kamperis
		@since 0.1.0
		@date 05.02.2018
		Defines a trapezoidal memberhsip function.
	*/
	class Trapezoidal final : public Function {
	public:
		const double start;
		const double firstPeek;
		const double secondPeek;
		const double end;

		/**
			Constructs a trapezoidal membership function.
			@param name - The arbitrary name of the membership function
			@param start - The start value of the membership function as a double
			@param firstPeek - The first peek value of the membership function as a double
			@param secondPeek - The second peek value of the membership function as a double
			@param end - The end value of the membership function as a double
			@throws std::invalid_argument exception if the start value is greater than the first peek value or the first peek value is greater than the second peek value or the second peek value is greater than the end value
		*/
		Trapezoidal(std::string name, double start, double firstPeek, double secondPeek, double end)
			: Function(name), start(start), firstPeek(firstPeek), secondPeek(secondPeek), end(end) {
			if (firstPeek < start) {
				throw std::invalid_argument("Start value for trapezoidal functions must be less than or equal to the first peek value.");
			}
			else if (secondPeek < firstPeek) {
				throw std::invalid_argument("First peek value for trapezoidal functions must be less than or equal to the second peek value.");
			}
			else if (end < secondPeek) {
				throw std::invalid_argument("Second peek value for trapezoidal functions must be less than or equal to the end value.");
			}
		}

		~Trapezoidal() {

		}

		/**
			Fuzzifies the input argument to determine the degree of membership of the input within the fuzzy set defined by the membership function.
			@param x - Input value as a double
			@return degree of membership of the input value as a double
		*/
		virtual double fuzzify(double x) const override {
			if (x >= this->firstPeek && x <= this->secondPeek) {
				return 1.0;
			}
			else if (x < this->start || x > this->end) {
				return 0.0;
			}
			else if (x < this->firstPeek) {
				return (x - this->start) / (this->firstPeek - this->start);
			}
			else {
				return 1.0 - ((x - this->secondPeek) / (this->end - this->secondPeek));
			}
		}

		/**
			Normalises the input around the bounds of the membership function for fuzzification of a resolution hedge.
			@param x - Input value as a double
			@return normalised input around the bounds of the membership function as a double
		*/
		virtual double normaliseInputForHedge(double x) const override {
			return (x - this->start) / (this->end - this->start);
		}
	};

	/**
		@author Oliver Kamperis
		@since 0.4.1
		@date 14.02.2018
		Defines a rectangular membership function.
	*/
	class Rectangular final : public Function {
	public:
		const double start;
		const double end;

		/**
			Constructs a rectangular membership function.
			@param name - The arbitrary name of the membership function
			@param start - The start value of the membership function as a double
			@param end - The end value of the membership function as a double
			@throws std::invalid_argument exception if the start value is greater than or equal to the end value
		*/
		Rectangular(std::string name, double start, double end)
			: Function(name), start(start), end(end) {
			if (end <= start) {
				throw std::invalid_argument("Start value for rectangular functions must be less than the end value.");
			}
		}

		~Rectangular() {

		}

		/**
			Fuzzifies the input argument to determine the degree of membership of the input within the fuzzy set defined by the membership function.
			@param x - Input value as a double
			@return degree of membership of the input value as a double
		*/
		virtual double fuzzify(double x) const override {
			if (x >= this->start && x <= this->end) {
				return 1.0;
			}
			else {
				return 0.0;
			}
		}

		/**
			Normalises the input around the bounds of the membership function for fuzzification of a resolution hedge.
			@param x - Input value as a double
			@return normalised input around the bounds of the membership function as a double
		*/
		virtual double normaliseInputForHedge(double x) const override {
			return (x - this->start) / (this->end - this->start);
		}
	};

	/**
		@author Oliver Kamperis
		@since 0.6.1
		@date 20.02.2018
		Defines a piecewise linear membership function.
	*/
	class PiecewiseLinear final : public Function {
	public:
		const std::vector<double> xPoints;
		const std::vector<double> yPoints;

		/**
			Constructs a piecewise linear membership function.
			@param name - The arbitrary name of the membership function
			@param xPoints - A list of the x values of each point on the piecewise linear function as doubles sorted in ascending order
			@param yPoints - A list of the y values of each point on the piecewise linear function as doubles
			@throws std::invalid_argument exception if the points lists are not equal in length
			@throws std::invalid_argument exception if any value in either list is less than 0 and greater than 1
			@throws std::invalid_argument exception if the x points list is not sorted in ascending order
		*/
		PiecewiseLinear(std::string name, std::vector<double> xPoints, std::vector<double> yPoints)
			: Function(name), xPoints(xPoints), yPoints(yPoints) {
			if (xPoints.size() != yPoints.size()) {
				throw std::invalid_argument("Points lists for PiecewiseLinear functions must be equal length.");
			}
			for (int i = 0; i < xPoints.size(); i++) {
				if (xPoints[i] > 1.0 || xPoints[i] < 0.0 || yPoints[i] > 1.0 || yPoints[i] < 0.0) {
					throw std::invalid_argument("All points for PiecewiseLinear functions must be greater than or equal to 0 and less than or equal to 1.");
				}
			}
			for (int i = 0; i < xPoints.size() - 1; i++) {
				if (xPoints[i] > xPoints[i + 1]) {
					throw std::invalid_argument("X points list for PiecewiseLinear functions must be sorted in ascending order.");
				}
			}
		}

		~PiecewiseLinear() {

		}

		/**
			Fuzzifies the input argument to determine the degree of membership of the input within the fuzzy set defined by the membership function.
			@param x - Input value as a double
			@return degree of membership of the input value as a double
		*/
		virtual double fuzzify(double x) const override {
			for (int i = 0; i < this->xPoints.size() - 1; i++) {
				if (x >= this->xPoints[i] && x <= this->xPoints[i + 1]) {
					if (this->yPoints[i] < this->yPoints[i + 1]) {
						return this->yPoints[i] + (((x - this->xPoints[i]) / (this->xPoints[i + 1] - this->xPoints[i])) * (this->yPoints[i + 1] - this->yPoints[i]));
					}
					else {
						return ((x - this->xPoints[i]) / (this->xPoints[i + 1] - this->xPoints[i])) * (this->yPoints[i] - this->yPoints[i + 1]);
					}
				}
			}
		}

		/**
			Normalises the input around the bounds of the membership function for fuzzification of a resolution hedge.
			@param x - Input value as a double
			@return normalised input around the bounds of the membership function as a double
		*/
		virtual double normaliseInputForHedge(double x) const override {
			return (x - this->xPoints[0]) / (this->xPoints[this->xPoints.size() - 1] - this->xPoints[0]);
		}
	};

	/**
		@author Oliver Kamperis
		@since 0.6.1
		@date 20.02.2018
		Defines a Gaussian membership function.
	*/
	class Gaussian final : public Function {
	public:
		const double mean;
		const double variance;
		const double lowerBound;
		const double upperBound;

		/**
			Constructs a Gaussian membership function.
			@param name - The arbitrary name of the membership function
			@param mean - The mean of the Gaussian function
			@param variance - The variance of the Gaussian function
			@param lowerBound - The lower bound of the function's bounds
			@param upperBound - The upper bound of the function's bounds
			@throws std::invalid_argument exception if - Variance is less than 0.0
			@throws std::invalid_argument exception if - Mean is less than 0.0 or greater than 1.0
			@throws std::invalid_argument exception if - Upper bound is less than the lower bound
		*/
		Gaussian(std::string name, double mean, double variance, double lowerBound = 0.0, double upperBound = 1.0)
			: Function(name), mean(mean), variance(variance), lowerBound(lowerBound), upperBound(upperBound) {
			if (variance <= 0.0) {
				throw std::invalid_argument("Variance for Gaussian functions must be greater than 0.");
			}
			else if (mean < 0.0 || mean > 1.0) {
				throw std::invalid_argument("Mean for Gaussian functions must be greater than or equal to 0 and less than or equal to 1.");
			}
			else if (upperBound <= lowerBound) {
				throw std::invalid_argument("Upper bound Gaussian functions must be greater than the lower bound.");
			}
		}

		~Gaussian() {

		}

		/**
			Fuzzifies the input argument to determine the degree of membership of the input within the fuzzy set defined by the membership function.
			@param x - Input value as a double
			@return degree of membership of the input value as a double
		*/
		virtual double fuzzify(double x) const override {
			if (x >= this->lowerBound && x <= this->upperBound) {
				return (1.0 / std::sqrt(2.0 * 3.14159265358979323846 * this->variance)) * exp(-(std::pow(((x * 10.0) - 5.0) - ((this->mean * 10.0) - 5.0), 2.0) / (2.0 * this->variance)));
			}
			else {
				return 0.0;
			}
		}

		/**
			Normalises the input around the bounds of the membership function for fuzzification of a resolution hedge.
			@param x - Input value as a double
			@return normalised input around the bounds of the membership function as a double
		*/
		virtual double normaliseInputForHedge(double x) const override {
			return (x - this->lowerBound) / (this->upperBound - this->lowerBound);
		}
	};

	/**
		@author Oliver Kamperis
		@since 0.4.1
		@date 14.02.2018
		Defines a singleton membership function.
	*/
	class Singleton final : public Function {
	public:
		const double center;
		const double range;

		/**
			Constructs a singleton membership function.
			@param name - The arbitrary name of the membership function
			@param center - The central point of the function
			@param range - The total range of the function
			@throws std::invalid_argument exception if - center - (range / 2.0) is less than 0.0 or center + (range / 2.0) is greater than 1.0.
		*/
		Singleton(std::string name, double center, double range)
			: Function(name), center(center) {
			if (center - (range / 2.0) < 0.0 || center + (range / 2.0) > 1.0) {
				throw std::invalid_argument("For Singleton functions, center - (range / 2.0) must be greater than or equal to 0 and center + (range / 2.0) must be less than or equal to 1.");
			}
		}

		~Singleton() {

		}

		/**
			Fuzzifies the input argument to determine the degree of membership of the input within the fuzzy set defined by the membership function.
			@param x - Input value as a double
			@return degree of membership of the input value as a double
		*/
		virtual double fuzzify(double x) const override {
			if (x >= this->center - (this->range / 2.0) && x <= this->center + (this->range / 2.0)) {
				return 1.0;
			}
			else {
				return 0.0;
			}
		}

		/**
			Singleton membership functions cannot have resolution hedges applied to them, as such this function always returns -1.0.
			@param x - Input value as a double
			@return -1.0
		*/
		virtual double normaliseInputForHedge(double x) const override {
			return -1.0;
		}
	};

	/**
		@author Oliver Kamperis
		@since 0.3.0
		@date 12.02.2018
		Defines a max-saturated membership function.
	*/
	class MaxSaturated final : public Function {
	public:
		/**
			Constructs a max-saturated membership function.
			@param name - The arbitrary name of the membership function
		*/
		MaxSaturated(std::string name = "Max saturated")
			: Function(name) {

		}

		~MaxSaturated() {

		}

		/**
			Fuzzifies the input argument to determine the degree of membership of the input within the fuzzy set defined by the membership function.
			Returns 1.0 if x > 1.0 else returns 0.0.
			@param x - Input value as a double
			@return degree of membership of the input value as a double
		*/
		virtual double fuzzify(double x) const override {
			return x > 1.0 ? 1.0 : 0.0;
		}

		/**
			MaxSaturated membership functions cannot have resolution hedges applied to them, as such this function always returns -1.0.
			@param x - Input value as a double
			@return -1.0
		*/
		virtual double normaliseInputForHedge(double x) const override {
			return -1.0;
		}
	};

	/**
		@author Oliver Kamperis
		@since 0.3.0
		@date 12.02.2018
		Defines a min-saturated membership function.
	*/
	class MinSaturated final : public Function {
	public:
		/**
			Constructs a min-saturated membership function.
			@param name - The arbitrary name of the membership function
		*/
		MinSaturated(std::string name = "Min saturated")
			: Function(name) {

		}

		~MinSaturated() {

		}

		/**
			Fuzzifies the input argument to determine the degree of membership of the input within the fuzzy set defined by the membership function.
			Returns 1.0 if x < 0.0 else returns 0.0.
			@param x - Input value as a double
			@return degree of membership of the input value as a double
		*/
		virtual double fuzzify(double x) const override {
			return x < 0.0 ? 1.0 : 0.0;
		}

		/**
			MinSaturated membership functions cannot have resolution hedges applied to them, as such this function always returns -1.0.
			@param x - Input value as a double
			@return -1.0
		*/
		virtual double normaliseInputForHedge(double x) const override {
			return -1.0;
		}
	};

	/**
		@author Oliver Kamperis
		@since 0.1.1
		@date 06.02.2018
		Defines a Takagi-Seguno style fuzzy rule.
	*/
	class Rule final {
		friend class FuzzyController<T>;
	public:
		const Variable<T>* const var;
		const Function* const func;
		const Hedge* const hedge;
		const Rule* const aggregate;
		const double outVal;

	private:
		/**
			Contructs a fuzzy rule from a input variable, hedge, membership function, aggregate rule and output value.
			@param var - pointer to the Variable object
			@param hedge - pointer to the Hedge
			@param func - pointer to the Function
			@param aggregate - pointer to the Rule object
			@param outVal - output value of the rule as a double
		*/
		Rule(const Variable<T>* const var, const Hedge* const hedge, const Function* const func, const Rule* const aggregate = nullptr, double outVal = 0.0)
			: var(var), func(func), hedge(hedge), aggregate(aggregate), outVal(outVal) {

		}

	public:
		~Rule() {

		}

		/**
			Determines the degree of thruth of the premise of the rule.
			@return The degree of truth as a double
		*/
		double fuzzyIf() const {
			if (this->hedge != nullptr) {
				double f = this->func->fuzzify(this->var->getValue());
				double h = this->hedge->fuzzify(this->func->normaliseInputForHedge(this->var->getValue()));
				double temp = f < h ? f : h;
				if (this->aggregate == nullptr || temp == 0.0) {
					return temp;
				}
				else {
					double agg = this->aggregate->fuzzyIf();
					return temp < agg ? temp : agg;
				}
			}
			else {
				double temp = this->func->fuzzify(this->var->getValue());
				if (this->aggregate == nullptr || temp == 0.0) {
					return temp;
				}
				else {
					double agg = this->aggregate->fuzzyIf();
					return temp < agg ? temp : agg;
				}
			}
		}

		bool operator==(const Rule& r) const {
			return *(this->var) == *(r.var) && (this->hedge != nullptr && r.hedge != nullptr ? *(this->hedge) == *(r.hedge) : this->hedge == nullptr && r.hedge == nullptr) && *(this->func) == *(r.func) && (this->aggregate != nullptr && r.aggregate != nullptr ? *(this->aggregate) == *(r.aggregate) : this->aggregate == nullptr && r.aggregate == nullptr);
		}
	};

	/**
		@author Oliver Kamperis
		@since 0.4.0
		@date 13.02.2018
		Defines the builder class for a fuzzy controller.
		Input variables, membership functions and resolution hedges can be added to the builder sequentially.
	*/
	template<class U = T>
	class FuzzyControllerBuilder final {
		friend class FuzzyController<U>;
	private:
		U* const system;
		std::vector<Variable<U>*> variables;
		std::vector<std::vector<Function*>> functions;
		std::vector<Hedge*> hedges;

	public:
		/**
			Contructs a fuzzy controller builder.
			@template U - Class type of the system to control
			@param system - Pointer to the system to control
		*/
		FuzzyControllerBuilder(U* const system)
			: system(system), variables(std::vector<Variable<U>*>()), functions(std::vector<std::vector<Function*>>()), hedges(std::vector<Hedge*>()) {

		}

		~FuzzyControllerBuilder() {

		}

		/**
			Adds an input variable to the builder.
			@param v - Pointer to the Variable object to add
			@return Reference to the current fuzzy controller builder
			@throws std::invalid_argument exception, if a variable with the same name already exists in the builder
		*/
		FuzzyControllerBuilder<U>& addVariable(Variable<U>* const v) {
			for (int i = 0; i < this->variables.size(); i++) {
				if (this->variables[i]->name == v->name) {
					throw std::invalid_argument("Variable with name: " + v->name + ", has already been defined.");
				}
			}
			this->variables.push_back(v);
			this->functions.push_back(std::vector<Function*>());
			return *this;
		}

		/**
			Adds a membership function to the builder.
			@param f - Pointer to the Function object to add
			@return Reference to the current fuzzy controller builder
			@throws std::invalid_argument exception, if a function with the same name already exists in the builder
		*/
		FuzzyControllerBuilder<U>& addFunction(Function* const f, std::string var = "All") {
			if (var == "All") {
				for (int i = 0; i < this->variables.size(); i++) {
					for (int j = 0; j < this->functions[i].size(); j++) {
						if (this->functions[i][j]->name == f->name) {
							throw std::invalid_argument("Function with name: " + f->name + ", has already been defined for Variable with name: " + var);
						}
					}
					this->functions[i].push_back(f);
				}
				return *this;
			}
			else {
				for (int i = 0; i < this->variables.size(); i++) {
					if (this->variables[i]->name == var) {
						for (int j = 0; j < this->functions[i].size(); j++) {
							if (this->functions[i][j]->name == f->name) {
								throw std::invalid_argument("Function with name: " + f->name + ", has already been defined for Variable with name: " + var);
							}
						}
						this->functions[i].push_back(f);
						return *this;
					}
				}
			}
			throw std::invalid_argument("Variable with name: " + var + ", does not exist.");
		}

		/**
			Adds a default membership function from the list of:
			Name: "Huge", Type: Trapezoidal, Start: 0.72727272, First peek: 0.81818181, Second peek: 1.0, End: 1.0
			Name: "Large", Type: Trapezoidal, Start: 0.54545454, First peek: 0.63636363, Second peek: 0.72727272, End: 0.81818181
			Name: "Big", Type: Triangular, Start: 0.45454545, Peek: 0.54545454, End: 0.63636363
			Name: "Small", Type: Triangular, Start: 0.36363636, Peek: 0.45454545, End: 0.54545454
			Name: "Little", Type: Trapezoidal, Start: 0.18181818, First peek: 0.27272727, Second peek: 0.36363636, End: 0.45454545
			Name: "Tiny", Type: Trapezoidal, Start: 0.0, First peek: 0.0, Second peek: 0.18181818, End: 0.27272727
			@param functionName - The name of the default function to add to the builder
			@return Reference to the current fuzzy controller builder
			@throws std::invalid_argument exception, if the parameter name is not one of the default functions or a function with the same name already exists in the builder
		*/
		FuzzyControllerBuilder<U>& addDefaultFunction(std::string functionName, std::string var = "All") {
			Function* f = nullptr;
			if (functionName == "Huge") {
				f = new Trapezoidal("Huge", 0.72727272, 0.81818181, 1.0, 1.0);
			}
			else if (functionName == "Large") {
				f = new Trapezoidal("Large", 0.54545454, 0.63636363, 0.72727272, 0.81818181);
			}
			else if (functionName == "Big") {
				f = new Triangular("Big", 0.45454545, 0.54545454, 0.63636363);
			}
			else if (functionName == "Small") {
				f = new Triangular("Small", 0.36363636, 0.45454545, 0.54545454);
			}
			else if (functionName == "Little") {
				f = new Trapezoidal("Little", 0.18181818, 0.27272727, 0.36363636, 0.45454545);
			}
			else if (functionName == "Tiny") {
				f = new Trapezoidal("Tiny", 0.0, 0.0, 0.18181818, 0.27272727);
			}
			else {
				throw std::invalid_argument("Default function with name: " + functionName + ", is undefined");
			}
			if (var == "All") {
				for (int i = 0; i < this->variables.size(); i++) {
					for (int j = 0; j < this->functions[i].size(); j++) {
						if (this->functions[i][j]->name == f->name) {
							throw std::invalid_argument("Function with name: " + f->name + ", has already been defined for Variable with name: " + var);
						}
					}
					this->functions[i].push_back(f);
				}
				return *this;
			}
			else {
				for (int i = 0; i < this->variables.size(); i++) {
					if (this->variables[i]->name == var) {
						for (int j = 0; j < this->functions[i].size(); j++) {
							if (this->functions[i][j]->name == f->name) {
								throw std::invalid_argument("Function with name: " + f->name + ", has already been defined for Variable with name: " + var);
							}
						}
						this->functions[i][j].push_back(f);
						return *this;
					}
				}
			}
			throw std::invalid_argument("Variable with name: " + var + ", does not exist.");
		}

		/**
			Adds all default membership functions from the list of:
			Name: "Huge", Type: Trapezoidal, Start: 0.72727272, First peek: 0.81818181, Second peek: 1.0, End: 1.0
			Name: "Large", Type: Trapezoidal, Start: 0.54545454, First peek: 0.63636363, Second peek: 0.72727272, End: 0.81818181
			Name: "Big", Type: Triangular, Start: 0.45454545, Peek: 0.54545454, End: 0.63636363
			Name: "Small", Type: Triangular, Start: 0.36363636, Peek: 0.45454545, End: 0.54545454
			Name: "Little", Type: Trapezoidal, Start: 0.18181818, First peek: 0.27272727, Second peek: 0.36363636, End: 0.45454545
			Name: "Tiny", Type: Trapezoidal, Start: 0.0, First peek: 0.0, Second peek: 0.18181818, End: 0.27272727
			@return Reference to the current fuzzy controller builder
			@throws std::invalid_argument exception, if the a function with the same name already exists in the builder
		*/
		FuzzyControllerBuilder<U>& addAllDefaultFunctions(std::string var = "All") {
			std::vector<Function*> fs = std::vector<Function*>();
			fs.push_back(new Trapezoidal("Huge", 0.72727272, 0.81818181, 1.0, 1.0));
			fs.push_back(new Trapezoidal("Large", 0.54545454, 0.63636363, 0.72727272, 0.81818181));
			fs.push_back(new Triangular("Big", 0.45454545, 0.54545454, 0.63636363));
			fs.push_back(new Triangular("Small", 0.36363636, 0.45454545, 0.54545454));
			fs.push_back(new Trapezoidal("Little", 0.18181818, 0.27272727, 0.36363636, 0.45454545));
			fs.push_back(new Trapezoidal("Tiny", 0.0, 0.0, 0.18181818, 0.27272727));
			if (var == "All") {
				for (int k = 0; k < this->variables.size(); k++) {
					for (int i = 0; i < fs.size(); i++) {
						for (int j = 0; j < this->functions[k].size(); j++) {
							if (this->functions[k][j]->name == fs[i]->name) {
								throw std::invalid_argument("Function with name: " + fs[i]->name + ", has already been defined for Variable with name: " + var);
							}
						}
						this->functions[k].push_back(fs[i]);
					}
				}
				return *this;
			}
			else {
				for (int k = 0; k < this->variables.size(); k++) {
					if (this->variables[k]->name == var) {
						for (int i = 0; i < fs.size(); i++) {
							for (int j = 0; j < this->functions[k].size(); j++) {
								if (this->functions[k][j]->name == fs[i]->name) {
									throw std::invalid_argument("Function with name: " + fs[i]->name + ", has already been defined for Variable with name: " + var);
								}
							}
							this->functions[k].push_back(fs[i]);
						}
						return *this;
					}
				}
			}
			throw std::invalid_argument("Variable with name: " + var + ", does not exist.");
		}

		/**
			Adds a hedge to the builder.
			@param h - Pointer to the Hedge object to add
			@return Reference to the current fuzzy controller builder
			@throws std::invalid_argument exception, if a hedge with the same name already exists in the builder
		*/
		FuzzyControllerBuilder<U>& addHedge(Hedge* const h) {
			for (int i = 0; i < this->hedges.size(); i++) {
				if (this->hedges[i]->name == h->name || h->name == "Any") {
					throw std::invalid_argument("Hedge with name: " + h->name + ", has already been defined.");
				}
			}
			this->hedges.push_back(h);
			return *this;
		}

		/**
			Adds a default hedge from the list of:
			Name: "Extremely", Type: Trapezoidal, Start: 0.72727272, First peek: 0.81818181, Second peek: 1.0, End: 1.0
			Name: "Very", Type: Trapezoidal, Start: 0.54545454, First peek: 0.63636363, Second peek: 0.72727272, End: 0.81818181
			Name: "Medium", Type: Trapezoidal, Start: 0.36363636, First peek: 0.45454545, Second peek: 0.54545454, End: 0.63636363
			Name: "Somewhat", Type: Trapezoidal, Start: 0.18181818, First peek: 0.27272727, Second peek: 0.36363636, End: 0.45454545
			Name: "Slightly", Type: Trapezoidal, Start: 0.0, First peek: 0.0, Second peek: 0.18181818, End: 0.27272727
			Name: "Absolute maximum", Type: Singleton, Center: 0.995, Range: 0.01
			Name: "Absolute median", Type: Singleton, Center: 0.5, Range: 0.01
			Name: "Absolute minimum", Type: Singleton, Center: 0.005, Range: 0.01
			@param hedgeName - The name of the default hedge to add to the builder
			@return Reference to the current fuzzy controller builder
			@throws std::invalid_argument exception, if the parameter name is not one of the default hedges or a hedge with the same name already exists in the builder
		*/
		FuzzyControllerBuilder<U>& addDefaultHedge(std::string hedgeName) {
			Hedge* h = nullptr;
			if (hedgeName == "Extremely") {
				h = new Hedge("Extremely", 0.72727272, 0.81818181, 1.0, 1.0);
			}
			else if (hedgeName == "Very") {
				h = new Hedge("Very", 0.54545454, 0.63636363, 0.72727272, 0.81818181);
			}
			else if (hedgeName == "Medium") {
				h = new Hedge("Medium", 0.36363636, 0.45454545, 0.54545454, 0.63636363);
			}
			else if (hedgeName == "Somewhat") {
				h = new Hedge("Somewhat", 0.18181818, 0.27272727, 0.36363636, 0.45454545);
			}
			else if (hedgeName == "Slightly") {
				h = new Hedge("Slightly", 0.0, 0.0, 0.18181818, 0.27272727);
			}
			else if (hedgeName == "Absolute maximum") {
				h = new Hedge("Absolute maximum", 0.99, 0.99, 1.0, 1.0);
			}
			else if (hedgeName == "Absolute median") {
				h = new Hedge("Absolute median", 0.495, 0.495, 0.505, 0.505);
			}
			else if (hedgeName == "Absolute minimum") {
				h = new Hedge("Absolute minimum", 0.0, 0.0, 0.01, 0.01);
			}
			else if (h == nullptr) {
				throw std::invalid_argument("Default hedge with name: " + hedgeName + ", is undefined");
			}
			for (int i = 0; i < this->hedges.size(); i++) {
				if (this->hedges[i]->name == h->name) {
					throw std::invalid_argument("Hedge with name: " + h->name + ", has already been defined.");
				}
			}
			this->hedges.push_back(h);
			return *this;
		}

		/**
			Adds all default hedges from the list of:
			Name: "Extremely", Type: Trapezoidal, Start: 0.72727272, First peek: 0.81818181, Second peek: 1.0, End: 1.0
			Name: "Very", Type: Trapezoidal, Start: 0.54545454, First peek: 0.63636363, Second peek: 0.72727272, End: 0.81818181
			Name: "Medium", Type: Trapezoidal, Start: 0.36363636, First peek: 0.45454545, Second peek: 0.54545454, End: 0.63636363
			Name: "Somewhat", Type: Trapezoidal, Start: 0.18181818, First peek: 0.27272727, Second peek: 0.36363636, End: 0.45454545
			Name: "Slightly", Type: Trapezoidal, Start: 0.0, First peek: 0.0, Second peek: 0.18181818, End: 0.27272727
			Name: "Absolute maximum", Type: Singleton, Center: 0.995, Range: 0.01
			Name: "Absolute median", Type: Singleton, Center: 0.5, Range: 0.01
			Name: "Absolute minimum", Type: Singleton, Center: 0.005, Range: 0.01
			@return Reference to the current fuzzy controller builder
			@throws std::invalid_argument exception - if a hedge with the same name already exists in the builder
		*/
		FuzzyControllerBuilder<U>& addAllDefaultHedges() {
			std::vector<Hedge*> hs = std::vector<Hedge*>();
			hs.push_back(new Hedge("Extremely", 0.72727272, 0.81818181, 1.0, 1.0));
			hs.push_back(new Hedge("Very", 0.54545454, 0.63636363, 0.72727272, 0.81818181));
			hs.push_back(new Hedge("Medium", 0.36363636, 0.45454545, 0.54545454, 0.63636363));
			hs.push_back(new Hedge("Somewhat", 0.18181818, 0.27272727, 0.36363636, 0.45454545));
			hs.push_back(new Hedge("Slightly", 0.0, 0.0, 0.18181818, 0.27272727));
			hs.push_back(new Hedge("Absolute maximum", 0.99, 0.99, 1.0, 1.0));
			hs.push_back(new Hedge("Absolute median", 0.495, 0.495, 0.505, 0.505));
			hs.push_back(new Hedge("Absolute minimum", 0.0, 0.0, 0.01, 0.01));
			for (int i = 0; i < hs.size(); i++) {
				for (int j = 0; j < this->hedges.size(); j++) {
					if (this->hedges[j]->name == hs[i]->name) {
						throw std::invalid_argument("Hedge with name: " + hs[i]->name + ", has already been defined.");
					}
				}
				this->hedges.push_back(hs[i]);
			}
			return *this;
		}
	};

private:
	const std::vector<Variable<T>*> variables;
	const std::vector<std::vector<Function*>> functions;
	const std::vector<Hedge*> hedges;

	std::vector<std::vector<Rule*>> ruleModules;
	std::vector<double> moduleGains;
	std::vector<bool> moduleModes;

	Variable<T>* priorityVariable;
	int priorityVariableIndex;
	std::vector<std::vector<Rule*>> dynamicImportanceDegreeRuleModules;
	bool weightedAverageMode;
	double outputGain;

protected:
	T* const system;

	/**
		Sets the controller output to the system.
		The user must override this function when defining a new fuzzy controller.
	*/
	virtual void setOutput(double x) = 0;

public:
	/**
		Constructs a fuzzy controller.
		@param fcb - The fuzzy controller builder to construct the controller with
		@param outputGain - The output gain of the fuzzy controller
	*/
	FuzzyController(const FuzzyControllerBuilder<T>& fcb, double outputGain = 1.0)
		: system(fcb.system), variables(fcb.variables), functions(fcb.functions), hedges(fcb.hedges),
		ruleModules(std::vector<std::vector<Rule*>>()), moduleGains(std::vector<double>()), moduleModes(std::vector<bool>()),
		priorityVariable(nullptr), dynamicImportanceDegreeRuleModules(std::vector<std::vector<Rule*>>()), weightedAverageMode(false), outputGain(outputGain) {

	}

	~FuzzyController() {

	}

	/**
		Creates a new rule module in the controller.
		@param moduleGain - the output gain of the module
		@param weightedAverageMode - true sets the module output to be a weighted average of the degrees of activation of all fired rules, otherwise false sets the module output to be a weighted sum of the degrees of activation of all fired rules
	*/
	void addRuleModule(double moduleGain = 1.0, bool weightedAverageMode = true) {
		this->ruleModules.push_back(std::vector<Rule*>());
		this->moduleGains.push_back(moduleGain);
		this->moduleModes.push_back(weightedAverageMode);
		this->dynamicImportanceDegreeRuleModules.push_back(std::vector<Rule*>());
	}

	/**
		Creates a rule in the specified rule module within the controller
		@param ruleModule - the rule to add the rule to
		@param var - name of the input variable for the antecendant
		@param hedge - name of the hedge for the antecendant
		@param func - name of the membership function for the antecendant
		@param outVal - the raw output value of the rule
		@param aggregate - optional aggregate rule statement
		@throws std::invalid_argument exception if - the specified variable, function or hedge does not exist
		@throws std::invalid_argument exception if - the specified rule module does not exist
		@throws std::invalid_argument exception if - the same rule antecendant already exists in the same rule module
	*/
	void addRule(int ruleModule, std::string var, std::string hedge, std::string func, double outVal, Rule* aggregateRule = nullptr) {
		Variable<T>* v = nullptr;
		Function* f = nullptr;
		Hedge* h = nullptr;
		for (int i = 0; i < this->variables.size(); i++) {
			if (this->variables[i]->name == var) {
				v = this->variables[i];
				for (int j = 0; j < this->functions[i].size(); j++) {
					if (this->functions[i][j]->name == func) {
						f = this->functions[i][j];
						break;
					}
				}
				break;
			}
		}
		if (hedge != "Any") {
			for (int i = 0; i < this->hedges.size(); i++) {
				if (this->hedges[i]->name == hedge) {
					h = this->hedges[i];
				}
			}
		}
		if (v == nullptr || f == nullptr || (h == nullptr && (hedge != "Any"))) {
			throw std::invalid_argument("Either the specified variable, function or hedge does not exist.");
		}
		if (ruleModule < 1 || ruleModule > this->ruleModules.size()) {
			throw std::invalid_argument("Rule module: " + std::to_string(ruleModule) + ", does not exist.");
		}
		Rule* rule = new Rule(v, h, f, aggregateRule, outVal);
		for (int i = 0; i < this->ruleModules[ruleModule - 1].size(); i++) {
			if (*(this->ruleModules[ruleModule - 1][i]) == *(rule)) {
				throw std::invalid_argument("The same rule antecendant cannot exist twice in the same rule module.");
			}
		}
		this->ruleModules[ruleModule - 1].push_back(rule);
	}

	/**
		Creates an aggregate rule statement.
		@param var - name of the input variable for the antecendant
		@param hedge - name of the hedge for the antecendant
		@param func - name of the membership function for the antecendant
		@param aggregate - optional aggregate rule statement
		@return aggregate Rule object
		@throws std::invalid_argument exception if - if the specified variable, function or hedge does not exist
	*/
	Rule* aggregate(std::string var, std::string hedge, std::string func, Rule* aggregateRule = nullptr) {
		Variable<T>* v = nullptr;
		Function* f = nullptr;
		Hedge* h = nullptr;
		for (int i = 0; i < this->variables.size(); i++) {
			if (this->variables[i]->name == var) {
				v = this->variables[i];
				for (int j = 0; j < this->functions[i].size(); j++) {
					if (this->functions[i][j]->name == func) {
						f = this->functions[i][j];
						break;
					}
				}
				break;
			}
		}
		if (hedge != "Any") {
			for (int i = 0; i < this->hedges.size(); i++) {
				if (this->hedges[i]->name == hedge) {
					h = this->hedges[i];
					break;
				}
			}
		}
		if (v == nullptr || f == nullptr || (h == nullptr && (hedge != "Any"))) {
			throw std::invalid_argument("Either the specified variable, function or hedge does not exist.");
		}
		return new Rule(v, h, f, aggregateRule);
	}

	/**
		Defines the priority variable to be used for dynamic importance degrees.
		@param var - Name of the priority input variable for the antecendant
		@param weightedAverageMode - true sets the controller output to be a weighted average of the output all rule modules, otherwise false sets the controller output to be a weighted sum of the output of all rule modules
		@throws std::invalid_argument exception if - if the specified variable, function or hedge does not exist
	*/
	void definePriorityVariable(std::string var, bool weightedAverageMode = false) {
		if (this->priorityVariable != nullptr) {
			throw std::invalid_argument("Priority variable cannot be redefined.");
		}
		this->weightedAverageMode = weightedAverageMode;
		for (int i = 0; i < this->variables.size(); i++) {
			if (this->variables[i]->name == var) {
				this->priorityVariableIndex = i;
				return;
			}
		}
		throw std::invalid_argument("The specified variable does not exist.");
	}
	
	/**
		Creates a rule in the specified dynamic importance degree rule module within the controller.
		@param ruleModule - the rule to add the dynamic importance degree rule to
		@param hedge - name of the hedge for the antecendant
		@param func - name of the membership function for the antecendant
		@param importanceDegree - the importance degree to apply to the rule module
		@throws std::invalid_argument exception if - the priority variable has not been defined
		@throws std::invalid_argument exception if - the specified variable, function or hedge does not exist
		@throws std::invalid_argument exception if - the specified rule module does not exist
		@throws std::invalid_argument exception if - the same rule antecendant already exists in the same dynamic importance degree rule module
	*/
	void addImportanceDegreeRule(int ruleModule, std::string hedge, std::string func, double importanceDegree) {
		if (this->priorityVariable == nullptr) {
			throw std::invalid_argument("Priority variable must be defined to add importance degree rules.");
		}
		if (ruleModule < 1 || ruleModule > this->ruleModules.size()) {
			throw std::invalid_argument("Rule module: " + std::to_string(ruleModule) + ", does not exist.");
		}
		Function* f = nullptr;
		Hedge* h = nullptr;
		for (int i = 0; i < this->functions[this->priorityVariableIndex].size(); i++) {
			if (this->functions[this->priorityVariableIndex][i]->name == func) {
				f = this->functions[this->priorityVariableIndex][i];
				break;
			}
		}
		if (hedge != "Any") {
			for (int i = 0; i < this->hedges.size(); i++) {
				if (this->hedges[i]->name == hedge) {
					h = this->hedges[i];
					break;
				}
			}
		}
		if (f == nullptr || (h == nullptr && (hedge != "Any"))) {
			throw std::invalid_argument("Either the specified variable, function or hedge does not exist.");
		}
		Rule* rule = new Rule(this->variables[this->priorityVariableIndex], h, f, nullptr, importanceDegree);
		for (int i = 0; i < this->dynamicImportanceDegreeRuleModules[ruleModule - 1].size(); i++) {
			if (*(this->dynamicImportanceDegreeRuleModules[ruleModule - 1][i]) == *(rule)) {
				throw std::invalid_argument("The same rule premise cannot exist twice in the same dynamic importance degree rule module.");
			}
		}
		this->dynamicImportanceDegreeRuleModules[ruleModule - 1].push_back(rule);
	}

	/**
		Ticks the fuzzy controller.
		Runs through all rules and calculates the total controller output based on the current inputs from the system.
	*/
	void tick() {
		for (int i = 0; i < this->variables.size(); i++) {
			this->variables[i]->calculateValue();
		}
		double importanceSum = 0.0;
		if (this->priorityVariable != nullptr) {
			for (int i = 0; i < this->dynamicImportanceDegreeRuleModules.size(); i++) {
				double activation = 0.0, truth = 0.0, temp = 0.0;
				for (int j = 0; j < this->dynamicImportanceDegreeRuleModules[i].size(); j++) {
					temp = this->dynamicImportanceDegreeRuleModules[i][j]->fuzzyIf();
					activation += temp * this->dynamicImportanceDegreeRuleModules[i][j]->outVal;
					truth += temp;
				}
				if (isfinite<double>(activation / truth)) {
					this->moduleGains[i] = activation / truth;
					importanceSum += activation / truth;
				}
			}
		}
		double rawOutput = 0.0;
		for (int i = 0; i < this->ruleModules.size(); i++) {
			double activation = 0.0, truth = 0.0, temp = 0.0;
			for (int j = 0; j < this->ruleModules[i].size(); j++) {
				temp = this->ruleModules[i][j]->fuzzyIf();
				activation += temp * this->ruleModules[i][j]->outVal;
				truth += temp;
			}
			if (this->moduleModes[i] && isfinite<double>(activation / truth)) {
				rawOutput += this->moduleGains[i] * (activation / truth);
			}
			else if (isfinite<double>(activation)) {
				rawOutput += this->moduleGains[i] * activation;
			}
		}
		if (this->weightedAverageMode && isfinite<double>(rawOutput / importanceSum)) {
			this->setOutput(this->outputGain * (rawOutput / importanceSum));
		}
		else if (isfinite<double>(rawOutput)) {
			this->setOutput(this->outputGain * rawOutput);
		}
	}
};
