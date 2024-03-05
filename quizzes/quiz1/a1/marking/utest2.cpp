#include "gtest/gtest.h"
#include <iostream> //Not needed usually, here just for debugging
#include <vector>
#include <algorithm>

//header files needed from our libraries
#include "../person.h"
#include "../processing.h"
using namespace std;



TEST (FunctionTest, DetectVaccineEligibility) {
    std::vector<Person> crowd;
    crowd.push_back(Person("Alice",22,true));
    crowd.push_back(Person("Bob",60,false));
    crowd.push_back(Person("Carol",85,false));
    crowd.push_back(Person("John",82,true));
    crowd.push_back(Person("Richard",61,false));
    crowd.push_back(Person("Tom",62,false));


    unsigned int ageCutOff=50;

    std::vector<Person> personsToVaccinate;
    personsToVaccinate = eligibleForVaccine(crowd,ageCutOff);

    //As an age cutoff is often inclusive of the age such as:
    // 18+ for entry into establishments that serve alcohol means 18 and above
    // So for us cutoff of 62 means 62 and above and not vaccinated
    // Ordered from oldest to youngest
    ASSERT_EQ(personsToVaccinate.size(),4);

    EXPECT_EQ(personsToVaccinate.at(0).getName(), "Carol");
    EXPECT_EQ(personsToVaccinate.at(1).getName(), "Tom");
    EXPECT_EQ(personsToVaccinate.at(2).getName(), "Richard");
    EXPECT_EQ(personsToVaccinate.at(3).getName(), "Bob");

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

















//std::cout << "******************************" << std::endl;
//for (auto p : oldest){
//    std::cout << p.getName() << " " << p.getAge() << std::endl;
//}
