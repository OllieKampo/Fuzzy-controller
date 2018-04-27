#include "FuzzyController.h"
#include <thread>
#include <chrono>
#include <math.h>

int main() {
	try {
		class Pendulum {
		public:
			const bool ignoreCart = true;
			const double steadyStateError = 10.0;
			const double cartMass = 1.0;
			const double pendulumLength = 1.0;
			const double gravity = 9.80665;
			double force = 0.0;
			double pendulumAngle = 45.0;
			double pendulumAngularVelocity = 0.0;
			double pendulumAngularAcceleration = 0.0;
			double cartPosition = 90.0;
			double cartVelocity = 0.0;
			double cartAcceleration = 0.0;
			std::chrono::time_point<std::chrono::system_clock> timeLast = std::chrono::system_clock::now();

			Pendulum() { }

			void updateForce(double x) {
				this->force = x;
			}

			void tick() {
				std::chrono::time_point<std::chrono::system_clock> timeCurrent = std::chrono::system_clock::now();
				std::chrono::duration<double> deltaTime = timeCurrent - timeLast;
				timeLast = timeCurrent;
				cartAcceleration = (force - steadyStateError) / cartMass;
				double v0 = cartVelocity;
				cartVelocity += cartAcceleration * deltaTime.count();
				cartPosition += ((v0 + cartVelocity) / 2.0) * deltaTime.count();
				if (!ignoreCart) {
					if (cartPosition <= 0.0) {
						cartPosition = 0.0;
						cartVelocity = 0.0;
						cartAcceleration = 0.0;
						pendulumAngularVelocity -= (cartVelocity / pendulumLength) * sin(pendulumAngle * (3.14159265 / 180.0));
					}
					else if (cartPosition >= 200.0) {
						cartPosition = 200.0;
						cartVelocity = 0.0;
						cartAcceleration = 0.0;
						pendulumAngularVelocity -= (cartVelocity / pendulumLength) * sin(pendulumAngle * (3.14159265 / 180.0));
					}
				}
				pendulumAngularAcceleration = ((cartAcceleration / pendulumLength) * sin(pendulumAngle * (3.14159265 / 180.0))) - ((gravity / pendulumLength) * cos(pendulumAngle * (3.14159265 / 180.0)));
				double v1 = pendulumAngularVelocity;
				pendulumAngularVelocity += pendulumAngularAcceleration * deltaTime.count();
				pendulumAngle += ((v1 + pendulumAngularVelocity) / 2.0) * deltaTime.count();
				if (pendulumAngle <= 30.0) {
					pendulumAngle = 30.0;
					pendulumAngularVelocity = 0.0;
					pendulumAngularAcceleration = 0.0;
				}
				else if (pendulumAngle >= 150.0) {
					pendulumAngle = 150.0;
					pendulumAngularVelocity = 0.0;
					pendulumAngularAcceleration = 0.0;
				}
			}
		}* pendulum = new Pendulum();

		class PendulumAngle : public FuzzyController<Pendulum>::Variable<Pendulum> {
		public:
			using Variable::Variable;

			virtual double getInput() const override final {
				return system->pendulumAngle;
			}
		}* pa = new PendulumAngle(pendulum, "pendulum angle", 150.0, 30.0);

		class PendulumIntegralError : public FuzzyController<Pendulum>::IntegralVariable<Pendulum> {
		public:
			using IntegralVariable::IntegralVariable;

			virtual double getInput() const override final {
				return system->pendulumAngle;
			}
		}* pie = new PendulumIntegralError(pendulum, "pendulum integral error", 150.0, 30.0, 0.25);

		class PendulumAngularVelocity : public FuzzyController<Pendulum>::DifferentialVariable<Pendulum> {
		public:
			using DifferentialVariable::DifferentialVariable;

			virtual double getInput() const override final {
				return system->pendulumAngle;
			}
		}* pav = new PendulumAngularVelocity(pendulum, "pendulum angular velocity", 5.0, -5.0);

		class CartPosition : public FuzzyController<Pendulum>::Variable<Pendulum> {
		public:
			using Variable::Variable;

			virtual double getInput() const override final {
				return system->cartPosition;
			}
		}* cp = new CartPosition(pendulum, "cart position", 200.0, 0.0);

		class CartVelocity : public FuzzyController<Pendulum>::DifferentialVariable<Pendulum> {
		public:
			using DifferentialVariable::DifferentialVariable;

			virtual double getInput() const override final {
				return system->cartVelocity;
			}
		}* cv = new CartVelocity(pendulum, "cart velocity", 5.0, -5.0);

		FuzzyController<Pendulum>::FuzzyControllerBuilder<Pendulum> builder = FuzzyController<Pendulum>::FuzzyControllerBuilder<Pendulum>(pendulum);

		builder.addVariable(pa)
			.addVariable(pie)
			.addVariable(pav)
			.addVariable(cp)
			.addVariable(cv)
			.addAllDefaultFunctions()
			.addFunction(new FuzzyController<Pendulum>::MaxSaturated())
			.addFunction(new FuzzyController<Pendulum>::MinSaturated())
			.addAllDefaultHedges();

		class PendulumController : public FuzzyController<Pendulum> {
		public:
			using FuzzyController<Pendulum>::FuzzyController;

			virtual void setOutput(double x) override {
				system->updateForce(x);
			}
		}* f = new PendulumController(builder);

		f->addRuleModule(1.0, true);
		f->addRuleModule(1.0, true);
		f->addRuleModule(1.0, true);
		f->addRuleModule(1.0, true);
		f->addRuleModule(1.0, true);
		/*
		f->addRule(1, "pendulum angle", "Any", "Huge", -50.0);
		f->addRule(1, "pendulum angle", "Any", "Large", -25.0);
		f->addRule(1, "pendulum angle", "Any", "Big", -10.0);
		f->addRule(1, "pendulum angle", "Any", "Small", 10.0);
		f->addRule(1, "pendulum angle", "Any", "Little", 25.0);
		f->addRule(1, "pendulum angle", "Any", "Tiny", 50.0);
		*/
		
		f->addRule(1, "pendulum angle", "Any", "Huge", -50.0);
		f->addRule(1, "pendulum angle", "Any", "Large", -25.0);
		f->addRule(1, "pendulum angle", "Extremely", "Big", -25.0);
		f->addRule(1, "pendulum angle", "Very", "Big", -25.0);
		f->addRule(1, "pendulum angle", "Medium", "Big", -17.5);
		f->addRule(1, "pendulum angle", "Somewhat", "Big", -10.0);
		f->addRule(1, "pendulum angle", "Very", "Small", 10.0);
		f->addRule(1, "pendulum angle", "Medium", "Small", 17.5);
		f->addRule(1, "pendulum angle", "Somewhat", "Small", 25.0);
		f->addRule(1, "pendulum angle", "Slightly", "Small", 25.0);
		f->addRule(1, "pendulum angle", "Any", "Little", 25.0);
		f->addRule(1, "pendulum angle", "Any", "Tiny", 50.0);
		
		
		f->addRule(2, "pendulum integral error", "Any", "Max saturated", -20.0);
		f->addRule(2, "pendulum integral error", "Any", "Huge", -15.0);
		f->addRule(2, "pendulum integral error", "Any", "Large", -10.0);
		f->addRule(2, "pendulum integral error", "Any", "Big", -5.0);
		f->addRule(2, "pendulum integral error", "Any", "Small", 5.0);
		f->addRule(2, "pendulum integral error", "Any", "Little", 10.0);
		f->addRule(2, "pendulum integral error", "Any", "Tiny", 15.0);
		f->addRule(2, "pendulum integral error", "Any", "Min saturated", 20.0);
		
		
		f->addRule(3, "pendulum angular velocity", "Any", "Max saturated", -25.0);
		f->addRule(3, "pendulum angular velocity", "Any", "Huge", -17.5);
		f->addRule(3, "pendulum angular velocity", "Any", "Large", -10.0);
		f->addRule(3, "pendulum angular velocity", "Any", "Big", -5.0);
		f->addRule(3, "pendulum angular velocity", "Any", "Small", 5.0);
		f->addRule(3, "pendulum angular velocity", "Any", "Little", 10.0);
		f->addRule(3, "pendulum angular velocity", "Any", "Tiny", 17.5);
		f->addRule(3, "pendulum angular velocity", "Any", "Min saturated", 25.0);
		
		/*
		f->addRule(4, "cart position", "Any", "Huge", -75.0);
		f->addRule(4, "cart position", "Any", "Large", -50.0);
		f->addRule(4, "cart position", "Any", "Big", -25.0);
		f->addRule(4, "cart position", "Any", "Small", 25.0);
		f->addRule(4, "cart position", "Any", "Little", 50.0);
		f->addRule(4, "cart position", "Any", "Tiny", 75.0);
		*/
		/*
		f->addRule(1, "cart position", "Any", "Huge", -1000.0, f->aggregate("pendulum angle", "Any", "Huge"));
		f->addRule(1, "cart position", "Any", "Huge", -125.0, f->aggregate("pendulum angle", "Any", "Large"));
		f->addRule(1, "cart position", "Any", "Huge", -5.0, f->aggregate("pendulum angle", "Any", "Big"));
		f->addRule(1, "cart position", "Any", "Huge", 5.0, f->aggregate("pendulum angle", "Any", "Small"));
		f->addRule(1, "cart position", "Any", "Huge", -2500.0, f->aggregate("pendulum angle", "Any", "Little"));
		f->addRule(1, "cart position", "Any", "Huge", -2500.0, f->aggregate("pendulum angle", "Any", "Tiny"));

		f->addRule(1, "cart position", "Any", "Large", -1000.0, f->aggregate("pendulum angle", "Any", "Huge"));
		f->addRule(1, "cart position", "Any", "Large", -25.0, f->aggregate("pendulum angle", "Any", "Large"));
		f->addRule(1, "cart position", "Any", "Large", -5.0, f->aggregate("pendulum angle", "Any", "Big"));
		f->addRule(1, "cart position", "Any", "Large", 5.0, f->aggregate("pendulum angle", "Any", "Small"));
		f->addRule(1, "cart position", "Any", "Large", -2500.0, f->aggregate("pendulum angle", "Any", "Little"));
		f->addRule(1, "cart position", "Any", "Large", -2500.0, f->aggregate("pendulum angle", "Any", "Tiny"));

		f->addRule(1, "cart position", "Any", "Big", 2500.0, f->aggregate("pendulum angle", "Any", "Huge"));
		f->addRule(1, "cart position", "Any", "Big", -5.0, f->aggregate("pendulum angle", "Any", "Large"));
		f->addRule(1, "cart position", "Any", "Big", -2.5, f->aggregate("pendulum angle", "Any", "Big"));
		f->addRule(1, "cart position", "Any", "Big", 2.5, f->aggregate("pendulum angle", "Any", "Small"));
		f->addRule(1, "cart position", "Any", "Big", -2500.0, f->aggregate("pendulum angle", "Any", "Little"));
		f->addRule(1, "cart position", "Any", "Big", -2500.0, f->aggregate("pendulum angle", "Any", "Tiny"));

		f->addRule(1, "cart position", "Any", "Small", 2500.0, f->aggregate("pendulum angle", "Any", "Huge"));
		f->addRule(1, "cart position", "Any", "Small", 2500.0, f->aggregate("pendulum angle", "Any", "Large"));
		f->addRule(1, "cart position", "Any", "Small", -2.5, f->aggregate("pendulum angle", "Any", "Big"));
		f->addRule(1, "cart position", "Any", "Small", 2.5, f->aggregate("pendulum angle", "Any", "Small"));
		f->addRule(1, "cart position", "Any", "Small", 5.0, f->aggregate("pendulum angle", "Any", "Little"));
		f->addRule(1, "cart position", "Any", "Small", -2500.0, f->aggregate("pendulum angle", "Any", "Tiny"));

		f->addRule(1, "cart position", "Any", "Little", 2500.0, f->aggregate("pendulum angle", "Any", "Huge"));
		f->addRule(1, "cart position", "Any", "Little", 2500.0, f->aggregate("pendulum angle", "Any", "Large"));
		f->addRule(1, "cart position", "Any", "Little", -5.0, f->aggregate("pendulum angle", "Any", "Big"));
		f->addRule(1, "cart position", "Any", "Little", 5.0, f->aggregate("pendulum angle", "Any", "Small"));
		f->addRule(1, "cart position", "Any", "Little", 25.0, f->aggregate("pendulum angle", "Any", "Little"));
		f->addRule(1, "cart position", "Any", "Little", 500.0, f->aggregate("pendulum angle", "Any", "Tiny"));

		f->addRule(1, "cart position", "Any", "Tiny", 2500.0, f->aggregate("pendulum angle", "Any", "Huge"));
		f->addRule(1, "cart position", "Any", "Tiny", 2500.0, f->aggregate("pendulum angle", "Any", "Large"));
		f->addRule(1, "cart position", "Any", "Tiny", -5.0, f->aggregate("pendulum angle", "Any", "Big"));
		f->addRule(1, "cart position", "Any", "Tiny", 5.0, f->aggregate("pendulum angle", "Any", "Small"));
		f->addRule(1, "cart position", "Any", "Tiny", 125.0, f->aggregate("pendulum angle", "Any", "Little"));
		f->addRule(1, "cart position", "Any", "Tiny", 1000.0, f->aggregate("pendulum angle", "Any", "Tiny"));
		*/
		/*
		f->definePriorityVariable("pendulum angle", false);
		
		f->addImportanceDegreeRule(1, "Any", "Huge", 1.0);
		f->addImportanceDegreeRule(1, "Any", "Large", 1.0);
		f->addImportanceDegreeRule(1, "Any", "Big", 1.0);
		f->addImportanceDegreeRule(1, "Any", "Small", 1.0);
		f->addImportanceDegreeRule(1, "Any", "Little", 1.0);
		f->addImportanceDegreeRule(1, "Any", "Tiny", 1.0);

		f->addImportanceDegreeRule(2, "Any", "Huge", 1.0);
		f->addImportanceDegreeRule(2, "Any", "Large", 1.0);
		f->addImportanceDegreeRule(2, "Any", "Big", 1.0);
		f->addImportanceDegreeRule(2, "Any", "Small", 1.0);
		f->addImportanceDegreeRule(2, "Any", "Little", 1.0);
		f->addImportanceDegreeRule(2, "Any", "Tiny", 1.0);

		f->addImportanceDegreeRule(3, "Any", "Huge", 1.0);
		f->addImportanceDegreeRule(3, "Any", "Large", 1.0);
		f->addImportanceDegreeRule(3, "Any", "Big", 1.0);
		f->addImportanceDegreeRule(3, "Any", "Small", 1.0);
		f->addImportanceDegreeRule(3, "Any", "Little", 1.0);
		f->addImportanceDegreeRule(3, "Any", "Tiny", 1.0);
		*/

		for (int i = 0, j = 0; true; i++) {
			f->tick();
			if (i % 100 == 0) {
				if (pendulum->ignoreCart) {
					std::cout << "Angle: " << pendulum->pendulumAngle << " Controller output: " << pendulum->force << std::endl;
				}
				else {
					std::cout << "Pendulum angle: " << pendulum->pendulumAngle << ", Cart position: " << pendulum->cartPosition << " Controller output force: " << pendulum->force << std::endl;
				}
			}
			pendulum->tick();
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			if (pendulum->pendulumAngle >= 89.999 && pendulum->pendulumAngle <= 90.001 && (!pendulum->ignoreCart ? pendulum->cartPosition >= 99.9999 && pendulum->cartPosition <= 100.0001 : true)) {
				j++;
			}
			else if (j != 0) {
				j = 0;
			}
			if (j >= 1000) {
				std::cout << "Pendulum system has reached stability in: " << std::to_string((double)i / 1000.0) << " seconds!";
				break;
			}
		}
	}
	catch (std::exception e) {
		std::cout << e.what();
	}
	std::cin.ignore();
}