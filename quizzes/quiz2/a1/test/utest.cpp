#include "gtest/gtest.h"
#include <iostream> //Not needed usually, here just for debugging
#include <vector>
#include <algorithm>

//header files needed from our libraries
#include "../analysis.h"
using namespace std;

TEST (AnalysisTest, DragRace) {
    std::vector<CarInterface*> cars;
    cars.push_back(new Car("merc", "c180",10,1.45,1.77,143,0.29,1200));
    cars.push_back(new Car("bugatti", "veyron",11,1.19,2.00,1200,0.35,2200));
    cars.push_back(new Car("toyota", "yaris",15,1.19,1.87,420,0.30,1190));

    std::shared_ptr<Analysis> analysisPtr(new Analysis(cars));

    std::vector<unsigned int> order = analysisPtr->dragRace(100.0);

    ASSERT_EQ(order.size(),cars.size());
    EXPECT_EQ(order.at(0),11);
    EXPECT_EQ(order.at(1),15);
    EXPECT_EQ(order.at(2),10);

}

TEST (AnalysisTest, StopAllCars) {
    std::vector<CarInterface*> cars;
    cars.push_back(new Car("merc", "c180",77,1.45,1.77,143,0.29,1200));
    cars.push_back(new Car("bugatti", "veyron",45,1.19,2.00,1200,0.35,2200));
    cars.push_back(new Car("toyota", "yaris",7,1.19,1.87,420,0.30,1190));

    std::shared_ptr<Analysis> analysisPtr(new Analysis(cars));

    std::vector<unsigned int> order = analysisPtr->dragRace(10.0);
    //At this point the cars should be moving
    EXPECT_GT(cars.at(0)->getCurrentSpeed(),1e-1);
    EXPECT_GT(cars.at(1)->getCurrentSpeed(),1e-1);
    EXPECT_GT(cars.at(2)->getCurrentSpeed(),1e-1);

    analysisPtr->stopAllCars();
    //At this point the cars should be stopped
    EXPECT_NEAR(cars.at(0)->getCurrentSpeed(),0,1e-1);
    EXPECT_NEAR(cars.at(1)->getCurrentSpeed(),0,1e-1);
    EXPECT_NEAR(cars.at(2)->getCurrentSpeed(),0,1e-1);

}

TEST (AnalysisTest, zeroTopZeroRace) {
    std::vector<CarInterface*> cars;
    cars.push_back(new Car("merc", "c180",23,1.45,1.77,143,0.29,1200));
    cars.push_back(new Car("bugatti", "veyron",44,1.19,2.00,1200,0.35,2200));
    cars.push_back(new Car("toyota", "yaris",7,1.19,1.87,420,0.30,1190));

    std::shared_ptr<Analysis> analysisPtr(new Analysis(cars));

    std::vector<unsigned int> order = analysisPtr->zeroTopZeroRace();

    ASSERT_EQ(order.size(),cars.size());
    EXPECT_EQ(order.at(0),7);
    EXPECT_EQ(order.at(1),23);
    EXPECT_EQ(order.at(2),44);

}

TEST (AnalysisTest, sortByOdometry) {
    std::vector<CarInterface*> cars;
    cars.push_back(new Car("merc", "c180",1,1.45,1.77,143,0.29,1200));
    cars.push_back(new Car("bugatti", "veyron",2,1.19,2.00,1200,0.35,2200));
    cars.push_back(new Car("toyota", "yaris",3,1.19,1.87,420,0.30,1190));

    std::shared_ptr<Analysis> analysisPtr(new Analysis(cars));

    std::vector<unsigned int> order = analysisPtr->sortByOdometry();

    ASSERT_EQ(order.size(),cars.size());
    EXPECT_EQ(order.at(0),7);
    EXPECT_EQ(order.at(1),23);
    EXPECT_EQ(order.at(2),44);

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
