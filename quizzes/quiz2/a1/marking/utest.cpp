#include "gtest/gtest.h"
#include <iostream> //Not needed usually, here just for debugging
#include <vector>
#include <algorithm>

//header files needed from our libraries
#include "../analysis.h"
using namespace std;

TEST (AnalysisTest, DragRace) {
    std::vector<CarInterface*> cars;
    cars.push_back(new Car("merc", "c180",20,1.45,1.77,143,0.29,1200));
    cars.push_back(new Car("bugatti", "veyron",31,1.19,2.00,1200,0.35,2200));
    cars.push_back(new Car("toyota", "yaris",15,1.19,1.87,420,0.30,1190));

    //We create a pointer to the Radar, will use a shared pointer here
    std::shared_ptr<Analysis> analysisPtr(new Analysis(cars));

    std::vector<unsigned int> order = analysisPtr->dragRace(150.0);

    ASSERT_EQ(order.size(),cars.size());
    EXPECT_EQ(order.at(0),31);
    EXPECT_EQ(order.at(1),15);
    EXPECT_EQ(order.at(2),20);

}

TEST (AnalysisTest, StopAllCars) {
    std::vector<CarInterface*> cars;
    cars.push_back(new Car("merc", "c180",77,1.45,1.77,143,0.29,1200));
    cars.push_back(new Car("bugatti", "veyron",45,1.19,2.00,1200,0.35,2200));
    cars.push_back(new Car("toyota", "yaris",7,1.19,1.87,420,0.30,1190));

    //We create a pointer to the Radar, will use a shared pointer here
    std::shared_ptr<Analysis> analysisPtr(new Analysis(cars));

    std::vector<unsigned int> order = analysisPtr->dragRace(10.0);
    analysisPtr->stopAllCars();

    EXPECT_LT(cars.at(0)->getCurrentSpeed(),1e-1);
    EXPECT_LT(cars.at(1)->getCurrentSpeed(),1e-1);
    EXPECT_LT(cars.at(2)->getCurrentSpeed(),1e-1);

}

TEST (AnalysisTest, zeroTopZeroRace) {
    std::vector<CarInterface*> cars;
    cars.push_back(new Car("merc", "c180",23,1.45,1.77,143,0.29,1200));
    cars.push_back(new Car("bugatti", "veyron",4,1.19,2.00,1200,0.35,2200));
    cars.push_back(new Car("toyota", "yaris",7,1.19,1.87,420,0.30,1190));

    std::shared_ptr<Analysis> analysisPtr(new Analysis(cars));

    std::vector<unsigned int> order = analysisPtr->zeroTopZeroRace();

    ASSERT_EQ(order.size(),cars.size());
    EXPECT_EQ(order.at(0),7);
    EXPECT_EQ(order.at(1),23);
    EXPECT_EQ(order.at(2),4);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
