#include "processing.h"
#include "person.h"
// You can add additional STL headers needed here, such as <algortithm>
#include <limits>
#include <iostream>
#include <algorithm>

//3) TASK Implement function according to specification in header file
std::vector<Person> oldestPerson(std::vector<Person> crowd){
    std::vector<Person> oldest;
    int oldestAge = 0;
    for(int i=0; i<crowd.size(); i++){
        if (crowd.at(i).getAge()>oldestAge){
            oldestAge = crowd.at(i).getAge();
            oldest.clear();
            oldest.push_back(crowd.at(i));
        }
        else if (crowd.at(i).getAge()==oldestAge){
            oldest.push_back(crowd.at(i));
        }
    }
    return oldest;
}

//4) TASK Implement function according to specification in header file
std::vector<Person> eligibleForVaccine(std::vector<Person> crowd, unsigned int ageCutoff){
    std::vector<Person> eligible;
    for(int i=0; i<crowd.size(); i++){
        if (crowd.at(i).getVaccinatedStatus()==false && crowd.at(i).getAge()>=ageCutoff){
            eligible.push_back(crowd.at(i));
        }
    }
    std::sort(eligible.begin(), eligible.end(),[]( Person& a, Person& b){return a.getAge() > b.getAge();});

    return eligible;
}
