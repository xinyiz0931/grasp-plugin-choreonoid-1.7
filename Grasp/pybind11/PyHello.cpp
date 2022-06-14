#include <iostream>

#include <cnoid/PyBase>
#include <cnoid/BodyItem>
#include <cnoid/MessageView>

#include "../PlanBase.h"
#include "../PlanInterface.h"

using namespace cnoid;
using namespace grasp;
namespace py = pybind11;

class Animal {
public:
    virtual ~Animal() { }
    virtual std::string go(int n_times) = 0;
};

class Dog : public Animal {
public:
    std::string go(int n_times) override {
        std::string result;
        for (int i=0; i<n_times; ++i)
            result += "woof! ";
        return result;
    }
}